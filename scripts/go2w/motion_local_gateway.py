#!/usr/bin/env python3
"""Bridge a network-independent ROS action stack to a Unix socket.

The Foxy ROS participant that owns the motion action is intentionally kept
local to the Jetson.  This gateway is its local ROS client and exposes only a
small request/reply Unix socket to the network-facing proxy.  It never opens a
Unitree SDK channel and never sends a low-level command.
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import signal
import socket
import stat
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from go2w_motion_interfaces.action import MotionCommand
from std_srvs.srv import SetBool, Trigger


MAX_REQUEST_BYTES = 8192


@dataclass
class Request:
    payload: dict[str, Any]
    event: threading.Event = field(default_factory=threading.Event)
    response: dict[str, Any] | None = None

    def finish(self, response: dict[str, Any]) -> None:
        self.response = response
        self.event.set()


def _read_line(connection: socket.socket) -> bytes:
    chunks: list[bytes] = []
    size = 0
    while size <= MAX_REQUEST_BYTES:
        chunk = connection.recv(min(4096, MAX_REQUEST_BYTES + 1 - size))
        if not chunk:
            break
        if b"\n" in chunk:
            chunks.append(chunk.split(b"\n", 1)[0])
            break
        chunks.append(chunk)
        size += len(chunk)
    return b"".join(chunks)


class MotionLocalGateway(Node):
    """Local ROS side of the action/service proxy."""

    def __init__(self, socket_path: Path) -> None:
        super().__init__("go2w_motion_local_gateway")
        self._socket_path = socket_path
        self._requests: queue.Queue[Request] = queue.Queue()
        self._stop = threading.Event()
        self._current_lock = threading.Lock()
        self._current: dict[str, Any] | None = None
        self._motion_client = ActionClient(self, MotionCommand, "/go2w/local_motion")
        self._arm_client = self.create_client(SetBool, "/go2w/local_arm")
        self._stop_client = self.create_client(Trigger, "/go2w/local_emergency_stop")
        self._timer = self.create_timer(0.02, self._dispatch)
        self._server_thread = threading.Thread(
            target=self._serve_socket,
            name="motion-local-gateway-socket",
            daemon=True,
        )
        self._server_thread.start()
        self.get_logger().info(
            f"local motion gateway ready on {self._socket_path}"
        )

    def _safe_unlink_socket(self) -> None:
        try:
            mode = self._socket_path.lstat().st_mode
        except FileNotFoundError:
            return
        if not stat.S_ISSOCK(mode):
            raise RuntimeError(f"refusing to replace non-socket: {self._socket_path}")
        self._socket_path.unlink()

    def _serve_socket(self) -> None:
        self._socket_path.parent.mkdir(parents=True, exist_ok=True)
        self._safe_unlink_socket()
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.bind(str(self._socket_path))
        os.chmod(self._socket_path, 0o600)
        server.listen(8)
        server.settimeout(0.2)
        try:
            while not self._stop.is_set():
                try:
                    connection, _ = server.accept()
                except socket.timeout:
                    continue
                threading.Thread(
                    target=self._handle_connection,
                    args=(connection,),
                    name="motion-local-gateway-request",
                    daemon=True,
                ).start()
        finally:
            server.close()
            try:
                self._socket_path.unlink()
            except FileNotFoundError:
                pass

    def _handle_connection(self, connection: socket.socket) -> None:
        with connection:
            connection.settimeout(125.0)
            try:
                payload = json.loads(_read_line(connection).decode("utf-8"))
                if not isinstance(payload, dict):
                    raise ValueError("request must be a JSON object")
                request = Request(payload)
                self._requests.put(request)
                if not request.event.wait(120.0):
                    response = {"ok": False, "error": "local gateway timeout"}
                else:
                    response = request.response or {
                        "ok": False,
                        "error": "local gateway returned no response",
                    }
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                response = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
            try:
                connection.sendall(
                    json.dumps(response, separators=(",", ":")).encode("utf-8")
                    + b"\n"
                )
            except OSError:
                pass

    def _dispatch(self) -> None:
        try:
            request = self._requests.get_nowait()
        except queue.Empty:
            return
        operation = str(request.payload.get("op", ""))
        if operation == "ping":
            request.finish({"ok": True, "gateway": "local_ros"})
        elif operation == "arm":
            self._start_arm(request)
        elif operation == "emergency_stop":
            self._start_emergency_stop(request)
        elif operation == "motion":
            self._start_motion(request)
        elif operation == "cancel":
            self._cancel_motion(request)
        else:
            request.finish({"ok": False, "error": f"unknown operation: {operation}"})

    @staticmethod
    def _result_payload(result: MotionCommand.Result) -> dict[str, Any]:
        return {
            "success": bool(result.success),
            "error_code": int(result.error_code),
            "message": str(result.message),
            "elapsed_sec": float(result.elapsed_sec),
            "estimated_distance_m": float(result.estimated_distance_m),
            "actual_relative_yaw_deg": float(result.actual_relative_yaw_deg),
            "last_move_status_code": int(result.last_move_status_code),
            "last_stop_status_code": int(result.last_stop_status_code),
        }

    def _start_arm(self, request: Request) -> None:
        value = bool((request.payload.get("request") or {}).get("data", False))
        if not self._arm_client.service_is_ready():
            request.finish({"ok": False, "error": "local /go2w/local_arm service unavailable"})
            return
        service_request = SetBool.Request()
        service_request.data = value
        future = self._arm_client.call_async(service_request)

        def done(completed) -> None:
            try:
                response = completed.result()
                request.finish({
                    "ok": True,
                    "success": bool(response.success),
                    "message": str(response.message),
                })
            except Exception as exc:  # ROS service boundary
                request.finish({"ok": False, "error": f"{type(exc).__name__}: {exc}"})

        future.add_done_callback(done)

    def _start_emergency_stop(self, request: Request) -> None:
        if not self._stop_client.service_is_ready():
            request.finish({
                "ok": False,
                "error": "local /go2w/local_emergency_stop service unavailable",
            })
            return
        future = self._stop_client.call_async(Trigger.Request())

        def done(completed) -> None:
            try:
                response = completed.result()
                request.finish({
                    "ok": True,
                    "success": bool(response.success),
                    "message": str(response.message),
                })
            except Exception as exc:  # ROS service boundary
                request.finish({"ok": False, "error": f"{type(exc).__name__}: {exc}"})

        future.add_done_callback(done)

    def _start_motion(self, request: Request) -> None:
        with self._current_lock:
            if self._current is not None:
                request.finish({"ok": False, "error": "local motion goal already active"})
                return
            self._current = {"request": request, "goal_handle": None}
        if not self._motion_client.server_is_ready():
            self._finish_motion({"ok": False, "error": "local /go2w/local_motion action unavailable"})
            return
        source = request.payload.get("goal") or {}
        goal = MotionCommand.Goal()
        for field_name in (
            "mode", "vx", "vy", "yaw_rate", "duration_sec",
            "relative_yaw_deg", "max_yaw_rate", "timeout_sec",
        ):
            if field_name in source:
                setattr(goal, field_name, source[field_name])
        future = self._motion_client.send_goal_async(goal)

        def goal_done(completed) -> None:
            try:
                goal_handle = completed.result()
            except Exception as exc:  # ROS action boundary
                self._finish_motion({"ok": False, "error": f"{type(exc).__name__}: {exc}"})
                return
            if goal_handle is None or not goal_handle.accepted:
                self._finish_motion({"ok": False, "error": "local motion goal rejected"})
                return
            with self._current_lock:
                if self._current is not None:
                    self._current["goal_handle"] = goal_handle
            result_future = goal_handle.get_result_async()

            def result_done(result_completed) -> None:
                try:
                    response = result_completed.result()
                    result = response.result
                    self._finish_motion({"ok": True, "result": self._result_payload(result)})
                except Exception as exc:  # ROS action boundary
                    self._finish_motion({"ok": False, "error": f"{type(exc).__name__}: {exc}"})

            result_future.add_done_callback(result_done)

        future.add_done_callback(goal_done)

    def _cancel_motion(self, request: Request) -> None:
        with self._current_lock:
            current = self._current
            goal_handle = current.get("goal_handle") if current else None
        if goal_handle is None:
            request.finish({"ok": False, "error": "no active local motion goal"})
            return
        try:
            goal_handle.cancel_goal_async()
            request.finish({"ok": True, "message": "cancel requested"})
        except Exception as exc:  # ROS action boundary
            request.finish({"ok": False, "error": f"{type(exc).__name__}: {exc}"})

    def _finish_motion(self, response: dict[str, Any]) -> None:
        with self._current_lock:
            current = self._current
            self._current = None
        if current is not None:
            current["request"].finish(response)

    def close(self) -> None:
        self._stop.set()
        self._server_thread.join(timeout=2.0)
        try:
            self._socket_path.unlink()
        except FileNotFoundError:
            pass


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--socket", default="/tmp/go2w_motion_gateway.sock", type=Path)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = MotionLocalGateway(args.socket)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    def stop(_signum: int, _frame: object) -> None:
        node.close()
        executor.shutdown()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        executor.spin()
    finally:
        node.close()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
