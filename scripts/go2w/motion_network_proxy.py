#!/usr/bin/env python3
"""Expose the local motion gateway to the network ROS graph.

This process is deliberately a thin transport proxy.  All motion validation,
arming, lease ownership, stop verification and SDK calls remain in the local
Foxy action server and its lease holder.

它同时是 ``go2w_cmd_vel_bridge`` 那五道闸门的**唯一**发布者：网关侧的真实状态
经 ``op:"state"`` 报过来，这里按 ``motion_gate_state_core`` 的折算规则发成
``/go2w/software_arm`` / ``/go2w/sport_lease/alive`` / ``/go2w/robot_error_zero``
/ ``/go2w/emergency_stop`` / ``/go2w/remote_override``。为什么必须是这里：这五个
话题原先一个发布者都没有（``create_publisher`` 全仓检索），而真实状态全在机器人
本地图上，只有这个进程既看得到本地状态、又活在网络图上。
"""

from __future__ import annotations

import argparse
import json
import math
import queue
import socket
import threading
from pathlib import Path
from typing import Any

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from go2w_motion_interfaces.action import MotionCommand
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from motion_gate_state_core import GATE_TOPICS, gate_state_from_payload

#: 发现延迟：话题唯一性自检要等 DDS 把对端发布者收全了才有意义。启动瞬间
#: ``count_publishers`` 一律是 0，那时建发布者就会造出重复发布者。
DISCOVERY_GRACE_SECONDS = 3.0

#: 这四条是"锁存/慢变"状态，后来的订阅者必须能立刻拿到，所以用 TRANSIENT_LOCAL。
#: ``/go2w/robot_error_zero`` 是连续读数，用普通的 depth 10。
LATCHED_TOPICS = frozenset(
    {
        "/go2w/software_arm",
        "/go2w/sport_lease/alive",
        "/go2w/emergency_stop",
        "/go2w/remote_override",
    }
)


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


def _boolean_argument(value: str) -> bool:
    """argparse 的布尔值：只认写成字的真假，不认 Python 的 ``bool("false")``。"""
    lowered = value.strip().lower()
    if lowered in ("1", "true", "yes", "on"):
        return True
    if lowered in ("0", "false", "no", "off"):
        return False
    raise argparse.ArgumentTypeError(f"expected a boolean, got {value!r}")


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

    def __init__(
        self,
        socket_path: Path,
        *,
        state_poll_sec: float = 0.2,
        remote_override_cleared: bool = False,
    ) -> None:
        super().__init__("go2w_motion_network_proxy")
        self._socket_path = socket_path
        self._remote_override_cleared = remote_override_cleared
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
        self._gate_publishers: dict[str, Any] = {}
        self._gate_lock = threading.Lock()
        self._gate_payload: dict[str, Any] | None = None
        self._gate_poll_in_flight = False
        self._gate_last_notes: tuple[str, ...] | None = None
        self._gate_timer = self.create_timer(state_poll_sec, self._poll_gate_state)
        # The local Foxy safety server lives in a different ROS participant.
        # Action goals already cross that boundary through the Unix gateway;
        # every streaming-velocity heartbeat must cross it too.  Without this
        # relay the first goal is accepted but the local 250 ms lease expires.
        self.create_subscription(
            String,
            "/go2w/motion/velocity_update",
            self._relay_velocity_update,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self._velocity_relay_queue: queue.Queue[str] = queue.Queue(maxsize=1)
        self._velocity_relay_stop = threading.Event()
        self._velocity_relay_thread = threading.Thread(
            target=self._velocity_relay_loop,
            name="motion-proxy-velocity-relay",
            daemon=True,
        )
        self._velocity_relay_thread.start()
        # ``create_timer`` 是**周期性的**，不是一次性的：不取消的话每 3 秒重跑一次，
        # 第二次起 ``count_publishers`` 数到的是**自己刚建的**发布者，于是每 3 秒
        # 刷五条"already has 1 publisher(s)"的假冲突告警（离线冒烟实测到的）。
        # 这里在第一次回调里自取消，把它变成一次性的。
        self._gate_start_timer = self.create_timer(
            DISCOVERY_GRACE_SECONDS, self._start_gate_publishers
        )
        self.get_logger().info(
            f"network motion proxy ready: /go2w/motion -> {self._socket_path}"
        )

    def _relay_velocity_update(self, message: String) -> None:
        """Relay one fresh, structurally valid heartbeat to the local graph.

        The local action server remains authoritative for token identity,
        monotonic timestamps and configured velocity limits.  This boundary
        check only avoids spending socket work on malformed traffic.
        """
        if len(message.data) > 1024:
            return
        try:
            payload = json.loads(message.data)
            values = (
                float(payload["stamp"]), float(payload["vx"]),
                float(payload["vy"]), float(payload["yaw_rate"]),
            )
            token = payload["token"]
            if (payload.get("version") != 1 or not all(map(math.isfinite, values))
                    or not isinstance(token, str) or len(token) != 32):
                return
            # Never block a ROS executor thread on socket I/O.  Arm/disarm,
            # emergency stop and action-result callbacks share that executor.
            # Keep only the latest heartbeat; replaying an older queued command
            # would be both less responsive and rejected by the local stamp gate.
            try:
                self._velocity_relay_queue.put_nowait(message.data)
            except queue.Full:
                try:
                    self._velocity_relay_queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self._velocity_relay_queue.put_nowait(message.data)
                except queue.Full:
                    pass
        except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):
            # Missing a heartbeat is fail-closed: the local 250 ms stream TTL
            # expires and executes STOP.  Do not substitute an old command.
            return

    def _velocity_relay_loop(self) -> None:
        while not self._velocity_relay_stop.is_set():
            try:
                raw = self._velocity_relay_queue.get(timeout=.10)
            except queue.Empty:
                continue
            try:
                gateway_request(
                    self._socket_path,
                    {"op": "velocity_update", "data": raw},
                    0.20,
                )
            except (OSError, ValueError, TypeError, json.JSONDecodeError):
                # Local TTL/STOP is the authority on a lost relay.
                pass

    def close(self) -> None:
        self._velocity_relay_stop.set()
        self._velocity_relay_thread.join(timeout=.5)

    # ------------------------------------------------------------------ #
    # 五道闸门的状态发布                                                   #
    # ------------------------------------------------------------------ #

    def _start_gate_publishers(self) -> None:
        """发现期结束后建发布者，并让掉已经有主的那些话题。

        重复发布者是真实风险：``lease_status_bridge.py`` 在机器人本地图上发
        ``/go2w/sport_lease/alive``，同一个图里两个发布者各发各的会让 bridge 的
        订阅来回跳。这里不抢：已经有发布者就不建，并说清楚是哪一个话题、有几个。

        只跑一次（自取消）：``count_publishers`` 把**自己**也算进去，再跑一次就必然
        数到 1，于是把自检变成每 3 秒五条的假冲突告警。
        """
        self._gate_start_timer.cancel()
        for topic, _field in GATE_TOPICS:
            existing = self.count_publishers(topic)
            if existing > 0:
                self.get_logger().warning(
                    f"{topic} already has {existing} publisher(s); not publishing it "
                    "(the existing owner wins)"
                )
                continue
            if topic in LATCHED_TOPICS:
                qos = QoSProfile(
                    depth=1,
                    history=HistoryPolicy.KEEP_LAST,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
            else:
                qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST)
            self._gate_publishers[topic] = self.create_publisher(Bool, topic, qos)
        self.get_logger().info(
            "gate state publishers: "
            + (", ".join(sorted(self._gate_publishers)) or "(none)")
        )

    def _poll_gate_state(self) -> None:
        """取一次状态然后把上一次取到的发出去。

        取数走 socket 会阻塞，所以放在后台线程里；发布留在定时器线程上，于是
        即使一次取数卡住，也已经知道的值仍按 ``state_poll_sec`` 继续发（发的是
        上一次的读数，过期会由折算层判成不满足）。
        """
        if not self._gate_publishers:
            return
        with self._gate_lock:
            if not self._gate_poll_in_flight:
                self._gate_poll_in_flight = True
                threading.Thread(
                    target=self._fetch_gate_state,
                    name="motion-proxy-gate-state",
                    daemon=True,
                ).start()
            payload = self._gate_payload
        # 还没有过一次读数就**什么都不发**。``payload is None`` 会被折算层当成
        # "gateway unavailable" 而退回 ``GateState()`` 默认值，其中
        # ``emergency_stop=True`` —— 那是**断言"急停被按下"**，不是 fail-closed
        # 的沉默。2026-10-10 实测到它的真实代价：建好发布者后的第一次轮询
        # （后台取数还没返回，约 0.2 s）发了这么一组值，控制仲裁器那边
        # ``self._emergency = self._emergency or data`` 是**锁存**的，于是永久卡在
        # emergency_stop，只能重启进程才能解除（重启后立刻变成
        # remote_override_active_or_unverified，已实证）。
        #
        # 沉默是安全的：消费端各自的初值本来就是拦住（bridge 的
        # ``_emergency=True``、arbiter 的 ``_remote_override=True``），首发之前
        # 少发一轮不会让任何一道闸门变松。只有**读过之后**才谈得上"把上一次的
        # 读数继续发出去"（网关后来挂掉属于这一类，仍按 fail-closed 发）。
        if payload is None:
            return
        gate = gate_state_from_payload(
            payload, remote_override_cleared=self._remote_override_cleared
        )
        values = gate.publish_values()
        for topic, publisher in self._gate_publishers.items():
            publisher.publish(Bool(data=values[topic]))
        if gate.notes != self._gate_last_notes:
            self._gate_last_notes = gate.notes
            self.get_logger().info("gate state: " + "; ".join(gate.notes))

    def _fetch_gate_state(self) -> None:
        try:
            payload: dict[str, Any] = gateway_request(
                self._socket_path, {"op": "state"}, 2.0
            )
        except Exception as exc:  # transport boundary
            payload = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
        with self._gate_lock:
            self._gate_payload = payload
            self._gate_poll_in_flight = False

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
                "velocity_stream_token": goal.velocity_stream_token,
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
        # A local disarm/estop may cancel the private action while the public
        # action remains EXECUTING. Only CANCELING permits the CANCELED
        # transition; otherwise abort and preserve the actual error payload.
        if goal_handle.is_cancel_requested:
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
    parser.add_argument("--state-poll-sec", type=float, default=0.2)
    parser.add_argument(
        # 这是本文件里唯一一个"断言"而不是"测量"的输入，所以它必须是显式的、
        # 一次一条命令行的，而不是某个配置文件的默认值。``nargs="?"`` 让
        # ``--remote-override-cleared`` 与 ``--remote-override-cleared=true``
        # 两种写法都能用（launch 文件需要后者）。
        "--remote-override-cleared",
        nargs="?",
        const=True,
        default=False,
        type=_boolean_argument,
        metavar="BOOL",
        help=(
            "assert /go2w/remote_override is clear (default: do not assert, i.e. keep "
            "blocking). There is no remote-teleop input in this deployment, and "
            "'remote not pressed' cannot be told apart from 'remote not connected'; "
            "only pass this with an operator decision."
        ),
    )
    args = parser.parse_args()
    rclpy.init(args=[])
    node = MotionNetworkProxy(
        args.socket,
        state_poll_sec=args.state_poll_sec,
        remote_override_cleared=args.remote_override_cleared,
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        node._action_server.destroy()
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
