#!/usr/bin/env python3
"""Bridge a network-independent ROS action stack to a Unix socket.

The Foxy ROS participant that owns the motion action is intentionally kept
local to the Jetson.  This gateway is its local ROS client and exposes only a
small request/reply Unix socket to the network-facing proxy.  It never opens a
Unitree SDK channel and never sends a low-level command.

除 ``motion`` / ``cancel`` / ``arm`` / ``emergency_stop`` 之外，这里还提供一条
只读的 ``state``：把动作服务器侧的真实锁存状态（报错码、运动租约、武装租约、
急停锁存）报给网络代理，由它发布成 ``go2w_cmd_vel_bridge`` 订阅的那五个 Bool
话题。改动的成因、以及为什么只能从这里取，见 ``motion_gate_state_core`` 的模块
文档；折算规则全在那个文件里，本文件只负责**取数**。
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
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from go2w_motion_interfaces.action import MotionCommand
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from unitree_go.msg import SportModeState


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

    def __init__(
        self,
        socket_path: Path,
        *,
        sport_state_topic: str = "/go2w/motion/local_sportmodestate",
        lease_status_dir: Path = Path("/tmp/go2w_lease_status"),
        lease_heartbeat_timeout: float = 1.5,
        arm_timeout_sec: float = 60.0,
        velocity_update_topic: str = "/go2w/motion/velocity_update",
    ) -> None:
        super().__init__("go2w_motion_local_gateway")
        self._socket_path = socket_path
        self._requests: queue.Queue[Request] = queue.Queue()
        self._stop = threading.Event()
        self._current_lock = threading.Lock()
        self._current: dict[str, Any] | None = None
        self._motion_client = ActionClient(self, MotionCommand, "/go2w/local_motion")
        self._velocity_update_pub = self.create_publisher(
            String, velocity_update_topic,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self._arm_client = self.create_client(SetBool, "/go2w/local_arm")
        self._stop_client = self.create_client(Trigger, "/go2w/local_emergency_stop")
        # ---- 下面这些是 ``op:"state"`` 要报的数。全部只读 ------------------ #
        self._lease_status_dir = lease_status_dir
        self._lease_heartbeat_timeout = lease_heartbeat_timeout
        self._arm_timeout_sec = arm_timeout_sec
        # 动作服务器的 ``armed_`` 是"60 秒空闲租约"（``arm_timeout_sec``，
        # 见 ``motion_action_server.cpp::CheckArmTimeout``）。网关是它唯一的本地
        # 客户端，每一次 arm 调用、每一个运动 goal 都从这里过，所以按同一条规则
        # 重建是**有依据的镜像**，不是猜。它可以偏乐观（服务器因别的原因提前解除
        # 武装时这里看不出来），但那不解锁任何东西：服务器自己的
        # ``GoalContext{!require_arm || armed_}`` 才是权威，乐观只会让 goal 多发
        # 一次然后被服务器拒掉。
        self._armed_until_monotonic = 0.0
        self._estop_latched = False
        self._sport_error_code: int | None = None
        self._sport_error_monotonic = 0.0
        self._sport_state_topic = sport_state_topic
        self.create_subscription(
            SportModeState, sport_state_topic, self._on_sport_state, 10
        )
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

    def _on_sport_state(self, message: SportModeState) -> None:
        self._sport_error_code = int(message.error_code)
        self._sport_error_monotonic = time.monotonic()

    def _read_lease(self) -> tuple[bool | None, float | None]:
        """按 ``lease_status_bridge.py`` 的同一条规则读租约状态文件。

        刻意读文件而不是订阅 ``/go2w/sport_lease/alive``：那个话题由
        ``lease_status_bridge.py`` 发在机器人本地图上，网关与它同图时订阅也可以，
        但读文件少一个活动部件，且与那份发布者用的是**同一个真相**（它也只是
        在转发这几个文件）。文件缺失时返回 ``(None, None)``，由折算层判为不满足。
        """
        heartbeat = self._lease_status_dir / "heartbeat"
        try:
            heartbeat_age = time.time() - heartbeat.stat().st_mtime
        except OSError:
            return None, None
        try:
            lease_id = int((self._lease_status_dir / "id").read_text("ascii").strip())
        except (OSError, ValueError):
            lease_id = 0
        try:
            alive_flag = (self._lease_status_dir / "alive").read_text("ascii").strip() == "1"
        except OSError:
            alive_flag = False
        return (
            alive_flag and lease_id > 0 and heartbeat_age <= self._lease_heartbeat_timeout,
            heartbeat_age,
        )

    def _state_response(self) -> dict[str, Any]:
        now = time.monotonic()
        lease_alive, lease_age = self._read_lease()
        error_age = (
            now - self._sport_error_monotonic
            if self._sport_error_code is not None
            else None
        )
        return {
            "ok": True,
            "state": {
                "error_code": self._sport_error_code,
                "error_age_sec": error_age,
                "lease_alive": lease_alive,
                "lease_age_sec": lease_age,
                "armed": now < self._armed_until_monotonic,
                "emergency_stop_latched": self._estop_latched,
            },
        }

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
        # 有运动 goal 在跑 = 不空闲，武装租约因此不过期。动作服务器那边是同一
        # 条规则（``CheckArmTimeout`` 在 ``active_goal_`` 非空时直接 return）。
        if self._armed_until_monotonic > 0.0 and self._current_is_active():
            self._armed_until_monotonic = time.monotonic() + self._arm_timeout_sec
        try:
            request = self._requests.get_nowait()
        except queue.Empty:
            return
        operation = str(request.payload.get("op", ""))
        if operation == "ping":
            request.finish({"ok": True, "gateway": "local_ros"})
        elif operation == "state":
            request.finish(self._state_response())
        elif operation == "arm":
            self._start_arm(request)
        elif operation == "emergency_stop":
            self._start_emergency_stop(request)
        elif operation == "motion":
            self._start_motion(request)
        elif operation == "velocity_update":
            self._publish_velocity_update(request)
        elif operation == "cancel":
            self._cancel_motion(request)
        else:
            request.finish({"ok": False, "error": f"unknown operation: {operation}"})

    def _publish_velocity_update(self, request: Request) -> None:
        raw = request.payload.get("data")
        if not isinstance(raw, str) or len(raw) > 1024:
            request.finish({"ok": False, "error": "invalid velocity update payload"})
            return
        # Publish verbatim: token/timestamp/speed validation belongs to the
        # local C++ safety server and invalid updates must not renew its TTL.
        self._velocity_update_pub.publish(String(data=raw))
        request.finish({"ok": True})

    def _current_is_active(self) -> bool:
        with self._current_lock:
            return self._current is not None

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
                if bool(response.success):
                    # 服务器只在真的成功时才会存 armed_，这里跟着它走：成功才改。
                    self._armed_until_monotonic = (
                        time.monotonic() + self._arm_timeout_sec if value else 0.0
                    )
                    # 服务器的 emergency STOP 只会解除 armed_，成功重新 arm
                    # 就是它的恢复动作。网关也必须在同一个成功响应后解除镜像锁存，
                    # 否则 /go2w/emergency_stop 永远为 true，Nav2 仲裁器无法恢复。
                    if value:
                        self._estop_latched = False
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
                # 急停是**锁存**的：动作服务器那边一按就 armed_=false 并做三次
                # STOP 验证（``HandleEmergencyStop``），没有任何东西会自动清它。
                # 这里跟着锁存，语义才一致 —— 想恢复只能重新 arm。
                if bool(response.success):
                    self._estop_latched = True
                    self._armed_until_monotonic = 0.0
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
            "velocity_stream_token",
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
    parser.add_argument(
        "--sport-state-topic", default="/go2w/motion/local_sportmodestate"
    )
    parser.add_argument(
        "--lease-status-dir",
        default=os.environ.get("GO2W_LEASE_STATUS_DIR", "/tmp/go2w_lease_status"),
        type=Path,
    )
    parser.add_argument("--lease-heartbeat-timeout", type=float, default=1.5)
    # 与 ``motion_control.yaml`` 的 ``arm_timeout_sec`` 必须一致 —— 这里是镜像
    # 规则，两边不同步的话报出去的就是错的。改一边记得改另一边。
    parser.add_argument("--arm-timeout-sec", type=float, default=60.0)
    parser.add_argument("--velocity-update-topic", default="/go2w/motion/velocity_update",
                        help="Local stream topic; default preserves the physical ROS-island wiring")
    args = parser.parse_args()
    rclpy.init(args=[])
    node = MotionLocalGateway(
        args.socket,
        sport_state_topic=args.sport_state_topic,
        lease_status_dir=args.lease_status_dir,
        lease_heartbeat_timeout=args.lease_heartbeat_timeout,
        arm_timeout_sec=args.arm_timeout_sec,
        velocity_update_topic=args.velocity_update_topic,
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    stopped = threading.Event()

    def stop(_signum: int, _frame: object) -> None:
        stopped.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok() and not stopped.is_set():
            executor.spin_once(timeout_sec=.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        # Foxy action clients are waitables, not ordinary Node clients. Release
        # them before the node handle and executor guard condition disappear.
        node._motion_client.destroy()
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
