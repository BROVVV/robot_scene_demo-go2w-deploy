#!/usr/bin/env python3
"""Go2-W manual web demo ROS worker.

Runs under ``/usr/bin/python3`` with the ROS 2 environment sourced. The Web /
Conda process spawns this worker and talks to it over JSON Lines on stdin/
stdout (plan book §18–§20). stdout is reserved for protocol JSON only; every
log line goes to stderr.

Responsibilities:
  * subscribe ``/camera/front/image_raw/compressed`` and atomically write
    ``latest.jpg`` (+ ``camera_status.json``);
  * subscribe SportModeState / LowState / /go2w/safety/* / odom for the status
    snapshot;
  * execute one short pulse at a time through the existing ``/go2w/motion``
    Action, arm through ``/go2w/arm`` and stop through ``/go2w/emergency_stop``;
  * run its own watchdog: if the Web keepalive is stale for longer than
    ``MANUAL_DEMO_ROS_WORKER_DEADMAN_MS`` while a motion is in flight, cancel
    it (the second layer of the deadman, plan book §20).

No ``/lowcmd``, no direct joint control, no raw SDK velocity loop.
"""

from __future__ import annotations

import json
import os
import sys
import threading
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# Imported lazily so ROS-less syntax checks can still import this module for
# the pure ``emit`` helpers; rclpy itself is only required at runtime.
_ROS_IMPORTED = False


def emit(payload: dict) -> None:
    """Write one protocol JSON line to stdout (never logs).

    Tolerates a closed pipe: when the Web process exits it closes stdin/stdout,
    and the worker's own shutdown path may still try to emit one last message.
    Catching BrokenPipeError here lets the worker exit cleanly instead of
    printing a traceback.
    """
    try:
        sys.stdout.write(json.dumps(payload, ensure_ascii=False) + "\n")
        sys.stdout.flush()
    except (BrokenPipeError, OSError):
        pass


def log(message: str) -> None:
    """All diagnostics go to stderr so the JSONL parser stays clean."""
    sys.stderr.write(f"[manual-web-demo-worker] {message}\n")
    sys.stderr.flush()


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _jpeg_dimensions(data: bytes) -> tuple[int | None, int | None]:
    """Parse JPEG SOF marker for width/height without decoding the frame."""
    index = 2
    length = len(data)
    while index + 9 < length:
        if data[index] != 0xFF:
            index += 1
            continue
        marker = data[index + 1]
        if marker in (0xD8, 0xD9) or 0xD0 <= marker <= 0xD7:
            index += 2
            continue
        if index + 4 > length:
            return None, None
        segment_length = (data[index + 2] << 8) | data[index + 3]
        if 0xC0 <= marker <= 0xCF and marker not in (0xC4, 0xC8, 0xCC):
            if index + 9 <= length:
                height = (data[index + 5] << 8) | data[index + 6]
                width = (data[index + 7] << 8) | data[index + 8]
                return width, height
            return None, None
        index += 2 + segment_length
    return None, None


def _error_code_name(code: int) -> str:
    names = {
        0: "none",
        1: "not_armed",
        2: "invalid_goal",
        3: "lease_unavailable",
        4: "state_stale",
        5: "robot_error",
        6: "move_rejected",
        7: "stop_failed",
        8: "timeout",
        9: "canceled",
        10: "turn_overshoot",
        11: "stationary_verify_failed",
        12: "concurrent_goal",
        13: "direction_not_calibrated",
        255: "internal",
    }
    return names.get(code, f"error_{code}")


def _parse_worker_command(line: str) -> dict:
    text = line.strip()
    if not text:
        return {}
    try:
        payload = json.loads(text)
    except json.JSONDecodeError:
        log(f"dropped malformed command line: {text[:120]!r}")
        return {}
    if not isinstance(payload, dict):
        log("dropped non-object command")
        return {}
    return payload


class ManualWebDemoWorker:
    """ROS-side worker. Created only when rclpy is available."""

    def __init__(self) -> None:
        import rclpy  # noqa: PLC0415
        from rclpy.action import ActionClient  # noqa: PLC0415
        from rclpy.executors import MultiThreadedExecutor  # noqa: PLC0415
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: PLC0415
        from sensor_msgs.msg import CompressedImage  # noqa: PLC0415
        from std_msgs.msg import Bool, Float32  # noqa: PLC0415
        from std_srvs.srv import SetBool, Trigger  # noqa: PLC0415
        from unitree_go.msg import LowState, SportModeState  # noqa: PLC0415
        from nav_msgs.msg import Odometry  # noqa: PLC0415

        # The motion interface/control workspace is an external prerequisite
        # in the README. Keep the ROS worker alive for camera/status display
        # when that workspace has not been deployed yet, but never substitute
        # a different driver or enable motion without the exact Action type.
        motion_import_error = None
        try:
            from go2w_motion_interfaces.action import MotionCommand  # noqa: PLC0415
        except Exception as exc:  # noqa: BLE001
            MotionCommand = None
            motion_import_error = f"{type(exc).__name__}: {exc}"

        rclpy.init(args=[])
        from app.manual_web_demo.config import get_manual_demo_settings  # noqa: PLC0415

        self.settings = get_manual_demo_settings()
        self.node = rclpy.create_node("manual_web_demo_ros_worker")
        self.rclpy = rclpy
        self.ActionClient = ActionClient
        self.MultiThreadedExecutor = MultiThreadedExecutor
        self.SetBool = SetBool
        self.Trigger = Trigger
        self.MotionCommand = MotionCommand
        self._motion_backend_error = motion_import_error

        self._lock = threading.Lock()
        self._motion_active = False
        self._current_direction: str | None = None
        self._goal_handle = None
        self._stop_requested = False
        self._last_keepalive = time.monotonic()

        self._camera_available = False
        self._last_camera_monotonic: float | None = None
        self._sport = None
        self._last_sport_monotonic: float | None = None
        self._last_low_monotonic: float | None = None
        self._front_clearance: float | None = None
        self._lidar_fresh: bool | None = None
        self._left_clearance: float | None = None
        self._right_clearance: float | None = None
        self._rotation_clearance_valid: bool | None = None
        self._odom_frame: str | None = None
        self._odom_pose: tuple[float, float, float] | None = None

        runtime_dir = self.settings.runtime_dir_path
        runtime_dir.mkdir(parents=True, exist_ok=True)
        self._latest_path = runtime_dir / "latest.jpg"
        self._camera_status_path = runtime_dir / "camera_status.json"
        self._worker_pid_path = runtime_dir / "worker.pid"
        self._worker_pid_path.write_text(str(os.getpid()), encoding="utf-8")

        sensor_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        state_qos = QoSProfile(
            depth=20,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.node.create_subscription(
            CompressedImage,
            "/camera/front/image_raw/compressed",
            self._on_camera,
            sensor_qos,
        )
        self.node.create_subscription(
            SportModeState, "/lf/sportmodestate", self._on_sport, state_qos
        )
        self.node.create_subscription(
            LowState, "/lf/lowstate", self._on_low, state_qos
        )
        self.node.create_subscription(
            Float32, "/go2w/safety/front_clearance", self._on_front_clearance, state_qos
        )
        self.node.create_subscription(
            Bool, "/go2w/safety/lidar_fresh", self._on_lidar_fresh, state_qos
        )
        self.node.create_subscription(
            Float32, "/go2w/safety/left_clearance", self._on_left_clearance, state_qos
        )
        self.node.create_subscription(
            Float32, "/go2w/safety/right_clearance", self._on_right_clearance, state_qos
        )
        self.node.create_subscription(
            Bool,
            "/go2w/safety/rotation_clearance_valid",
            self._on_rotation_clearance_valid,
            state_qos,
        )
        try:
            self.node.create_subscription(
                Odometry, "/go2w/odom/fused", self._on_odom, state_qos
            )
        except Exception as exc:  # noqa: BLE001
            log(f"odom subscription unavailable: {exc}")

        self._motion_client = (
            self.ActionClient(self.node, self.MotionCommand, "/go2w/motion")
            if self.MotionCommand is not None
            else None
        )
        self._arm_client = self.node.create_client(self.SetBool, "/go2w/arm")
        self._estop_client = self.node.create_client(
            self.Trigger, "/go2w/emergency_stop"
        )
        self._watchdog_timer = self.node.create_timer(
            0.1, self._watchdog_tick
        )
        # ``server_is_ready()`` is only re-queried every few seconds; the
        # status push itself must never stall the executor's camera callbacks.
        self._motion_available_cached = False
        self._motion_available_checked_at = 0.0
        self._clearance_bad_count = 0

    # NOTE: there is intentionally NO periodic worker_status push. A 1s timer
    # that calls ``server_is_ready()`` repeatedly starved the single
    # MultiThreadedExecutor of threads and made the camera callback drop
    # frames for seconds at a time. The Web process requests status on demand
    # (about every 2.5s) instead.

    # ------------------------------------------------------------------ #
    # subscriptions                                                       #
    # ------------------------------------------------------------------ #
    def _on_camera(self, msg) -> None:
        data = bytes(msg.data)
        try:
            temporary = self._latest_path.with_name("latest.tmp.jpg")
            with temporary.open("wb") as handle:
                handle.write(data)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, self._latest_path)
        except OSError as exc:
            log(f"camera write failed: {exc}")
        now = time.monotonic()
        self._camera_available = True
        self._last_camera_monotonic = now
        width, height = _jpeg_dimensions(data)
        status = {
            "type": "camera_status",
            "available": True,
            "received_at": now,
            "width": width,
            "height": height,
            "format": str(msg.format),
        }
        try:
            self._camera_status_path.write_text(
                json.dumps(status, ensure_ascii=False), encoding="utf-8"
            )
        except OSError:
            pass
        emit(status)

    def _on_sport(self, msg) -> None:
        self._sport = msg
        self._last_sport_monotonic = time.monotonic()

    def _on_low(self, msg) -> None:
        self._last_low_monotonic = time.monotonic()

    def _on_front_clearance(self, msg) -> None:
        self._front_clearance = float(msg.data)

    def _on_lidar_fresh(self, msg) -> None:
        self._lidar_fresh = bool(msg.data)

    def _on_left_clearance(self, msg) -> None:
        self._left_clearance = float(msg.data)

    def _on_right_clearance(self, msg) -> None:
        self._right_clearance = float(msg.data)

    def _on_rotation_clearance_valid(self, msg) -> None:
        self._rotation_clearance_valid = bool(msg.data)

    def _on_odom(self, msg) -> None:
        pose = msg.pose.pose
        self._odom_frame = msg.header.frame_id
        q = pose.orientation
        import math  # noqa: PLC0415

        yaw = math.atan2(2.0 * (q.z * q.w), 1.0 - 2.0 * (q.z * q.z))
        self._odom_pose = (float(pose.position.x), float(pose.position.y), yaw)

    # ------------------------------------------------------------------ #
    # status snapshot                                                     #
    # ------------------------------------------------------------------ #
    def _status_snapshot(self) -> dict:
        with self._lock:
            now = time.monotonic()
            state_fresh = bool(
                self._sport is not None
                and self._last_sport_monotonic is not None
                and self._last_low_monotonic is not None
                and now - self._last_sport_monotonic <= 1.5
                and now - self._last_low_monotonic <= 1.5
            )
            mode = int(self._sport.mode) if self._sport is not None else None
            error_code = (
                int(self._sport.error_code) if self._sport is not None else None
            )
            if now - self._motion_available_checked_at >= 5.0:
                try:
                    self._motion_available_cached = bool(
                        self._motion_client.server_is_ready()
                        if self._motion_client is not None
                        else False
                    )
                except Exception:  # noqa: BLE001 - status push must never crash
                    self._motion_available_cached = False
                self._motion_available_checked_at = now
            motion_available = self._motion_available_cached
            if self._motion_backend_error is not None:
                motion_available = False
            odom_pose = list(self._odom_pose) if self._odom_pose else None
        return {
            "type": "worker_status",
            "state": "ready",
            "motion_available": motion_available,
            "motion_backend_error": self._motion_backend_error,
            "robot_mode": mode,
            "robot_error_code": error_code,
            "state_fresh": state_fresh,
            "lease_alive": False,
            "lidar_fresh": self._lidar_fresh,
            "front_clearance_m": self._front_clearance,
            "left_clearance_m": self._left_clearance,
            "right_clearance_m": self._right_clearance,
            "rotation_clearance_valid": self._rotation_clearance_valid,
            "camera_available": self._camera_available,
            "odom_frame": self._odom_frame,
            "odom_pose": odom_pose,
        }

    # ------------------------------------------------------------------ #
    # commands                                                            #
    # ------------------------------------------------------------------ #
    def handle_pulse(self, direction: str) -> None:
        if direction not in (
            "forward",
            "backward",
            "strafe_left",
            "strafe_right",
            "turn_left",
            "turn_right",
        ):
            emit({"type": "blocked", "reason": f"unknown_direction:{direction}"})
            return
        if self._motion_backend_error is not None or self._motion_client is None:
            emit(
                {
                    "type": "blocked",
                    "reason": "motion_backend_unavailable",
                    "message": self._motion_backend_error
                    or "go2w motion Action client unavailable",
                }
            )
            return
        with self._lock:
            if self._motion_active:
                emit({"type": "blocked", "reason": "motion_already_active"})
                return
            self._motion_active = True
            self._current_direction = direction
            self._stop_requested = False
        emit({"type": "motion_started", "direction": direction})
        self._arm_then_send(direction)

    def handle_stop(self) -> None:
        with self._lock:
            self._stop_requested = True
            goal_handle = self._goal_handle
        if goal_handle is not None:
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda _f: emit({"type": "stopped", "status": "cancel_requested"})
            )
        else:
            emit({"type": "stopped", "status": "no_active_goal"})

    def handle_estop(self) -> None:
        with self._lock:
            self._stop_requested = True
        request = self.Trigger.Request()
        future = self._estop_client.call_async(request)
        future.add_done_callback(self._on_estop_done)

    def handle_keepalive(self) -> None:
        self._last_keepalive = time.monotonic()

    def handle_status(self) -> None:
        emit(self._status_snapshot())

    # ------------------------------------------------------------------ #
    # motion chain (done-callbacks driven by the executor spin thread)    #
    # ------------------------------------------------------------------ #
    def _arm_then_send(self, direction: str) -> None:
        request = self.SetBool.Request()
        request.data = True
        future = self._arm_client.call_async(request)
        future.add_done_callback(lambda f: self._on_arm_done(f, direction))

    def _on_arm_done(self, future, direction: str) -> None:
        if self._stop_requested:
            self._finish_motion(
                {
                    "success": False,
                    "direction": direction,
                    "error_code": "canceled",
                    "message": "stopped before goal arm",
                }
            )
            return
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self._finish_motion(
                {
                    "success": False,
                    "direction": direction,
                    "error_code": "arm_failed",
                    "message": str(exc),
                }
            )
            return
        if response is None or not response.success:
            self._finish_motion(
                {
                    "success": False,
                    "direction": direction,
                    "error_code": "arm_failed",
                    "message": getattr(response, "message", "arm rejected"),
                }
            )
            return
        goal = self._build_goal(direction)
        send_future = self._motion_client.send_goal_async(goal)
        send_future.add_done_callback(lambda f: self._on_goal_sent(f, direction))

    def _build_goal(self, direction: str):
        """Continuous hold goal: one timed-velocity goal kept running while the
        key is held (renewed on completion). Turns use a continuous ``yaw_rate``
        so there is no per-step angle limit; the operator cancels to stop."""
        settings = self.settings
        goal = self.MotionCommand.Goal()
        goal.mode = self.MotionCommand.Goal.MODE_TIMED_VELOCITY
        goal.duration_sec = settings.hold_duration_sec
        goal.timeout_sec = settings.hold_duration_sec + 10.0
        if direction == "forward":
            goal.vx = settings.pulse_vx
        elif direction == "backward":
            goal.vx = -settings.pulse_vx_backward
        elif direction in ("strafe_left", "strafe_right"):
            goal.vy = settings.pulse_vy * (
                1.0 if direction == "strafe_left" else -1.0
            )
        elif direction in ("turn_left", "turn_right"):
            goal.yaw_rate = settings.turn_yaw_rate * (
                1.0 if direction == "turn_left" else -1.0
            )
        return goal

    def _on_goal_sent(self, future, direction: str) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._finish_motion(
                {
                    "success": False,
                    "direction": direction,
                    "error_code": "goal_send_failed",
                    "message": str(exc),
                }
            )
            return
        if goal_handle is None or not goal_handle.accepted:
            self._finish_motion(
                {
                    "success": False,
                    "direction": direction,
                    "error_code": "goal_rejected",
                    "message": "motion goal rejected by action server",
                }
            )
            return
        with self._lock:
            self._goal_handle = goal_handle
            stop_requested = self._stop_requested
        if stop_requested:
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda _f: emit({"type": "stopped", "status": "cancel_requested"})
            )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_goal_result(f, direction)
        )

    def _on_goal_result(self, future, direction: str) -> None:
        with self._lock:
            self._goal_handle = None
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as exc:  # noqa: BLE001
            self._finish_motion(
                {
                    "success": False,
                    "direction": direction,
                    "error_code": "result_failed",
                    "message": str(exc),
                }
            )
            return
        self._finish_motion(
            {
                "success": bool(result.success),
                "direction": direction,
                "error_code": _error_code_name(result.error_code),
                "message": str(result.message),
                "elapsed_sec": float(result.elapsed_sec),
                "estimated_distance_m": float(result.estimated_distance_m),
                "actual_relative_yaw_deg": float(result.actual_relative_yaw_deg),
            }
        )

    def _finish_motion(self, payload: dict) -> None:
        with self._lock:
            self._motion_active = False
            self._current_direction = None
        emit({"type": "motion_finished", **payload})

    def _on_estop_done(self, future) -> None:
        try:
            response = future.result()
            message = (
                getattr(response, "message", "emergency stop done")
                if response is not None
                else "no response"
            )
        except Exception as exc:  # noqa: BLE001
            message = str(exc)
        emit({"type": "estop_done", "message": message})

    # ------------------------------------------------------------------ #
    # watchdog                                                            #
    # ------------------------------------------------------------------ #
    def _watchdog_tick(self) -> None:
        with self._lock:
            motion_active = self._motion_active
            goal_handle = self._goal_handle
            direction = self._current_direction
            lidar_fresh = self._lidar_fresh
            front_clearance = self._front_clearance
        if not motion_active:
            return
        # Continuous-motion forward clearance watchdog: while a forward hold is
        # running, stop if the LiDAR sees the path close in (checked every 0.1s,
        # two consecutive bad samples before canceling so noise does not jerk).
        if direction == "forward":
            blocked = (
                lidar_fresh is not True
                or front_clearance is None
                or front_clearance < self.settings.min_front_clearance_m
            )
            if blocked:
                self._clearance_bad_count += 1
                if self._clearance_bad_count >= 2:
                    self.node.get_logger().error(
                        f"forward clearance blocked ({front_clearance}); canceling"
                    )
                    with self._lock:
                        self._stop_requested = True
                    emit(
                        {
                            "type": "blocked",
                            "reason": "front_clearance_stop",
                        }
                    )
                    if goal_handle is not None:
                        cancel_future = goal_handle.cancel_goal_async()
                        cancel_future.add_done_callback(
                            lambda _f: emit(
                                {"type": "stopped", "status": "clearance_stop"}
                            )
                        )
                    return
            else:
                self._clearance_bad_count = 0
        # Keepalive watchdog (Web process died / stalled).
        if time.monotonic() - self._last_keepalive > self.settings.ros_worker_deadman_sec:
            self.node.get_logger().error(
                "worker watchdog: Web keepalive expired; stopping motion"
            )
            with self._lock:
                self._stop_requested = True
            if goal_handle is not None:
                cancel_future = goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(
                    lambda _f: emit(
                        {"type": "stopped", "status": "watchdog_cancel_requested"}
                    )
                )
            else:
                emit({"type": "blocked", "reason": "worker_watchdog_keepalive_timeout"})

    # ------------------------------------------------------------------ #
    # lifecycle                                                           #
    # ------------------------------------------------------------------ #
    def run_forever(self) -> int:
        executor = self.MultiThreadedExecutor(num_threads=4)
        executor.add_node(self.node)
        spinner = threading.Thread(target=executor.spin, daemon=True)
        spinner.start()
        self.node.get_logger().info("manual web demo ROS worker ready")
        emit({"type": "ready"})
        try:
            for line in sys.stdin:
                if line is None or line.strip() == "":
                    continue
                command = _parse_worker_command(line)
                command_type = command.get("type")
                if command_type == "pulse":
                    self.handle_pulse(str(command.get("direction") or ""))
                elif command_type == "stop":
                    self.handle_stop()
                elif command_type == "estop":
                    self.handle_estop()
                elif command_type == "keepalive":
                    self.handle_keepalive()
                elif command_type == "status":
                    self.handle_status()
                elif command_type == "shutdown":
                    break
                else:
                    log(f"ignored unknown command type {command_type!r}")
        except KeyboardInterrupt:
            pass
        finally:
            self.handle_stop()
            executor.shutdown()
            self.node.destroy_node()
            try:
                self.rclpy.shutdown()
            except RuntimeError:
                pass
            try:
                self._worker_pid_path.unlink(missing_ok=True)
            except OSError:
                pass
        return 0


def main() -> int:
    try:
        worker = ManualWebDemoWorker()
    except Exception as exc:  # noqa: BLE001
        emit(
            {
                "type": "error",
                "message": f"worker failed to initialize: "
                f"{type(exc).__name__}: {exc}",
            }
        )
        log(f"init failure: {type(exc).__name__}: {exc}")
        return 2
    return worker.run_forever()


if __name__ == "__main__":
    raise SystemExit(main())
