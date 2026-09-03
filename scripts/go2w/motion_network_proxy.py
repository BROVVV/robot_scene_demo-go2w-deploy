#!/usr/bin/env python3
"""Expose the local motion gateway to the network ROS graph.

This process is deliberately a thin transport proxy.  All motion validation,
arming, lease ownership, stop verification and SDK calls remain in the local
Foxy action server and its lease holder.
"""

from __future__ import annotations

import argparse
import json
import socket
import threading
from pathlib import Path
from typing import Any

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from go2w_motion_interfaces.action import MotionCommand
from std_srvs.srv import SetBool, Trigger


def gateway_request(socket_path: Path, payload: dict[str, Any], timeout: float = 8.0) -> dict[str, Any]:
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(timeout)
        connection.connect(str(socket_path))
        connection.sendall(json.dumps(payload, separators=(",", ":")).encode("utf-8") + b"\n")
        chunks: list[bytes] = []
        while True:
            chunk = connection.recv(4096)
            if not chunk:
                break
            chunks.append(chunk)
            if b"\n" in chunk:
                break
        raw = b"".join(chunks).split(b"\n", 1)[0]
        response = json.loads(raw.decode("utf-8"))
        if not isinstance(response, dict):
            raise ValueError("gateway response must be a JSON object")
        return response


def _result_from_payload(payload: dict[str, Any]) -> MotionCommand.Result:
    result = MotionCommand.Result()
    result.success = bool(payload.get("success", False))
    result.error_code = int(payload.get("error_code", MotionCommand.Result.ERROR_INTERNAL))
    result.message = str(payload.get("message", ""))
    result.elapsed_sec = float(payload.get("elapsed_sec", 0.0))
    result.estimated_distance_m = float(payload.get("estimated_distance_m", 0.0))
    result.actual_relative_yaw_deg = float(payload.get("actual_relative_yaw_deg", 0.0))
    result.last_move_status_code = int(payload.get("last_move_status_code", 0))
    result.last_stop_status_code = int(payload.get("last_stop_status_code", 0))
    return result


class MotionNetworkProxy(Node):
    """Network ROS action/service facade backed by the local Unix gateway."""

    def __init__(self, socket_path: Path) -> None:
        super().__init__("go2w_motion_network_proxy")
        self._socket_path = socket_path
        self._action_server = ActionServer(
            self,
            MotionCommand,
            "/go2w/motion",
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.create_service(SetBool, "/go2w/arm", self._arm)
        self.create_service(Trigger, "/go2w/emergency_stop", self._emergency_stop)
        self.get_logger().info(
            f"network motion proxy ready: /go2w/motion -> {self._socket_path}"
        )

    @staticmethod
    def _goal_callback(_goal_request: MotionCommand.Goal) -> GoalResponse:
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle) -> MotionCommand.Result:
        goal = goal_handle.request
        payload = {
            "op": "motion",
            "goal": {
                "mode": int(goal.mode),
                "vx": float(goal.vx),
                "vy": float(goal.vy),
                "yaw_rate": float(goal.yaw_rate),
                "duration_sec": float(goal.duration_sec),
                "relative_yaw_deg": float(goal.relative_yaw_deg),
                "max_yaw_rate": float(goal.max_yaw_rate),
                "timeout_sec": float(goal.timeout_sec),
            },
        }
        response_holder: dict[str, Any] = {}
        response_ready = threading.Event()

        def request_local_gateway() -> None:
            try:
                response_holder["response"] = gateway_request(
                    self._socket_path, payload, 125.0
                )
            except Exception as exc:  # transport boundary
                response_holder["error"] = exc
            finally:
                response_ready.set()

        threading.Thread(
            target=request_local_gateway,
            name="motion-network-proxy-gateway-call",
            daemon=True,
        ).start()
        cancel_sent = False
        while not response_ready.wait(0.05):
            if goal_handle.is_cancel_requested and not cancel_sent:
                cancel_sent = True
                try:
                    gateway_request(self._socket_path, {"op": "cancel"}, 5.0)
                except Exception as exc:  # cancel is best effort; local STOP remains authoritative
                    self.get_logger().error(f"local motion cancel transport failed: {exc}")
        try:
            if "error" in response_holder:
                raise response_holder["error"]
            response = response_holder["response"]
            result_payload = response.get("result") if response.get("ok") else None
            if not isinstance(result_payload, dict):
                result_payload = {
                    "success": False,
                    "error_code": MotionCommand.Result.ERROR_INTERNAL,
                    "message": str(response.get("error", "local gateway failed")),
                }
        except Exception as exc:  # transport boundary
            result_payload = {
                "success": False,
                "error_code": MotionCommand.Result.ERROR_INTERNAL,
                "message": f"network proxy transport failed: {type(exc).__name__}: {exc}",
            }
        result = _result_from_payload(result_payload)
        self.get_logger().info(
            f"proxied motion result: success={result.success} "
            f"error_code={result.error_code} message={result.message!r}"
        )
        if goal_handle.is_cancel_requested or result.error_code == MotionCommand.Result.ERROR_CANCELED:
            goal_handle.canceled()
        elif result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _arm(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        try:
            result = gateway_request(
                self._socket_path,
                {"op": "arm", "request": {"data": bool(request.data)}},
                8.0,
            )
            response.success = bool(result.get("success", False))
            response.message = str(result.get("message", result.get("error", "")))
        except Exception as exc:
            response.success = False
            response.message = f"network proxy transport failed: {type(exc).__name__}: {exc}"
        return response

    def _emergency_stop(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        try:
            result = gateway_request(self._socket_path, {"op": "emergency_stop"}, 8.0)
            response.success = bool(result.get("success", False))
            response.message = str(result.get("message", result.get("error", "")))
        except Exception as exc:
            response.success = False
            response.message = f"network proxy transport failed: {type(exc).__name__}: {exc}"
        return response


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--socket", default="/tmp/go2w_motion_gateway.sock", type=Path)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = MotionNetworkProxy(args.socket)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
