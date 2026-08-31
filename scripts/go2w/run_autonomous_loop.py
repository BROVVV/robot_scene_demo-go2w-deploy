#!/usr/bin/env python3
"""Autonomous small-range motion loop for the Go2-W (operator-authorized).

The robot arms itself, executes a configured pattern of forward steps and
relative turns through the audited /go2w/motion action server, verifies every
step with wheel odometry and the front-clearance gate, and finishes with a
triple STOP plus disarm. No user commands are required while it runs.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

try:
    import cv2
except Exception:  # pragma: no cover - recording is optional
    cv2 = None

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from app.live_robot.search_state_machine import SensorSnapshot
from app.live_robot.step_search_runner import (
    Detection,
    StepSearchConfig,
    StepSearchRunner,
    VerificationResult,
)
from app.detectors.siliconflow_vision_protocol import (
    SiliconFlowDaemonClient,
    VLMRequest,
)
from app.live_robot.semantic_observer import (
    LiveSemanticObserver,
    semantic_payload_from_quick_target_absence,
)
from app.config import get_settings
from app.memory.observation_memory_store import ObservationMemoryStore
from app.live_robot.motion_bounds import (
    MotionBoundaryDecision,
    evaluate_dual_lidar_rotation_gate,
    evaluate_lidar_motion_readiness,
    evaluate_motion_observation,
    evaluate_rotation_clearance,
    evaluate_step_boundary,
    position_within_boundary,
)
from app.live_robot.step_parser import (
    backward_step_distance,
    forward_step_distance,
    translation_step_distance,
)
from app.live_robot.rotation_lease import (
    build_rotation_lease_binding,
    evaluate_rotation_lease,
    evaluate_rotation_lease_step,
    load_rotation_lease,
    resolve_rotation_clearance_source,
    rotation_lease_stage2_scope_errors,
)
from app.live_robot.current_hardware import (
    geometry_hash,
    load_current_hardware_geometry,
    load_current_hardware_state,
    state_hash,
)
from app.live_robot.pandar_clock import (
    PandarClockTier,
    DEFAULT_PANDAR_CLOCK_TIER,
)
from app.live_robot.stage2_readiness import (
    compute_stage2_readiness,
)
from app.reasoning.semantic_navigation.router import SemanticSearchController
from app.reasoning.semantic_navigation.semantic_memory import SemanticSearchMemory
from app.reasoning.semantic_navigation.auxiliary_hints import (
    build_precomputed_situated_prior_hints,
    build_psg_auxiliary_hints,
)
from app.reasoning.target_profile import TargetProfileResolver

import rclpy
from go2w_motion_interfaces.action import MotionCommand
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool, Trigger
from unitree_go.msg import LowState, SportModeState


DEFAULT_PATTERN = ["f", "l20", "f", "r20", "f", "l20", "f", "r20", "f"]
GO2W_ROTATION_ENVELOPE_RADIUS_M = 0.511


class MotionGoalTimeout(RuntimeError):
    """The action transport did not settle within the goal's safety window.

    This is intentionally distinct from an ordinary rejected/failed goal: a
    late action response makes an automatic retry unsafe because the first
    command may still have reached the robot.
    """

PROMPT_MAP = {
    "手机": "phone. cellphone. mobile phone. smartphone",
    "箱子": "cardboard box. carton box",
    "瓶子": "plastic bottle. water bottle",
    "杯子": "cup. mug. glass",
    "书": "book. textbook",
    "人": "person. human",
    "书包": "gray backpack. grey backpack. rucksack",
    "灰色书包": "gray backpack. grey backpack. rucksack",
}


def strict_json_value(value):
    """Recursively replace non-finite telemetry with JSON ``null``."""

    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: strict_json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [strict_json_value(item) for item in value]
    return value


class BundleVideoRecorder:
    """Record the live camera stream with a locked target overlay.

    Runs as its own ROS 2 node on a dedicated executor thread so the camera
    feed is captured continuously while the main runner blocks on detection
    subprocesses. The CSRT tracker keeps the detection box locked onto the
    target between detector updates.
    """

    def __init__(self, output_video: str,
                 fps: float = 10.0, scale: float = 0.5,
                 camera_topic: str = "/camera/front/image_raw") -> None:
        if cv2 is None:
            raise RuntimeError("OpenCV is unavailable; video recording disabled")
        self._output = Path(output_video)
        self._fps = fps
        self._scale = scale
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._executor = None
        self._node = None
        self._writer: cv2.VideoWriter | None = None
        self._tracker = None
        self._pending: tuple[str, float, tuple[float, float, float, float]] | None = None
        self._locked_label = ""
        self._locked_score = 0.0
        self._tracker_ok = False
        self._command_text = ""
        self._cjk_font_path = self._find_cjk_font()
        self._frames_written = 0
        self._last_write_time = 0.0
        self._camera_topic = camera_topic
        self._output.parent.mkdir(parents=True, exist_ok=True)
        self._log_path = self._output.with_suffix(".jsonl")
        self._log = open(self._log_path, "a", encoding="utf-8")

    def start(self) -> None:
        if self._thread is not None:
            return
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        self._node = rclpy.create_node("go2w_video_recorder")
        self._node.create_subscription(
            Image, self._camera_topic, self._on_image,
            qos_profile_sensor_data,
        )
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(
            target=self._executor.spin, daemon=True
        )
        self._thread.start()

    def set_detection(self, label: str, score: float,
                      bbox_xyxy: tuple[float, float, float, float]) -> None:
        self._pending = (label, score, tuple(float(v) for v in bbox_xyxy))

    def clear_detection(self) -> None:
        self._pending = None
        self._tracker = None
        self._locked_label = ""
        self._locked_score = 0.0
        self._tracker_ok = False

    def set_command(self, text: str) -> None:
        self._command_text = str(text)

    @staticmethod
    def _find_cjk_font() -> str:
        candidates = [
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
            "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
            "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
        ]
        for path in candidates:
            if Path(path).is_file():
                return path
        return ""

    def _draw_cjk_text(
        self,
        frame,
        text: str,
        org: tuple[int, int],
        size: int,
        color: tuple[int, int, int],
        background: tuple[int, int, int, int] | None = None,
    ) -> None:
        """Draw text (including CJK) with PIL when a CJK font is available.

        cv2.putText cannot render CJK and produces '????'; this helper falls
        back to ASCII-only text when no CJK font is installed.
        """
        if not text:
            return
        try:
            import numpy as np
            from PIL import Image, ImageDraw, ImageFont

            if self._cjk_font_path:
                font = ImageFont.truetype(self._cjk_font_path, size)
                display_text = text
            else:
                font = ImageFont.load_default()
                display_text = text.encode("ascii", "ignore").decode("ascii")
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(rgb_frame)
            draw = ImageDraw.Draw(image)
            if background is not None:
                left, top = org
                bbox = draw.textbbox((0, 0), display_text, font=font)
                right = left + (bbox[2] - bbox[0])
                bottom = top + size + 6
                draw.rectangle(
                    (left - 4, top - 2, right + 4, bottom),
                    fill=background[:3] + (int(background[3]),)
                    if len(background) == 4 else background,
                )
            draw.text((org[0], org[1]), display_text, font=font,
                      fill=color)
            frame[:] = cv2.cvtColor(
                np.asarray(image), cv2.COLOR_RGB2BGR
            )
        except Exception:
            # Keep the recorder alive if text rendering fails for any reason.
            safe = text.encode("ascii", "ignore").decode("ascii")
            cv2.putText(frame, safe, org, cv2.FONT_HERSHEY_SIMPLEX,
                        max(0.5, size / 32.0), color, 2)

    def _on_image(self, message) -> None:
        now = time.monotonic()
        if self._frames_written > 0:
            interval = 1.0 / max(self._fps, 1.0)
            if now - self._last_write_time < interval:
                return
        import numpy as np
        frame = np.frombuffer(
            bytes(message.data), dtype=np.uint8
        ).reshape((message.height, message.width, 3))
        frame = frame.copy()
        if self._scale != 1.0:
            width = int(frame.shape[1] * self._scale)
            height = int(frame.shape[0] * self._scale)
            frame = cv2.resize(frame, (width, height),
                               interpolation=cv2.INTER_AREA)
        if self._writer is None:
            height, width = frame.shape[:2]
            for codec in ("avc1", "mp4v"):
                fourcc = cv2.VideoWriter_fourcc(*codec)
                writer = cv2.VideoWriter(
                    str(self._output), fourcc, self._fps, (width, height)
                )
                if writer.isOpened():
                    self._writer = writer
                    break
                writer.release()
            if self._writer is None:
                raise RuntimeError("no usable video codec (avc1/mp4v)")
        self._draw_overlay(frame, frame.shape[1], frame.shape[0])
        self._writer.write(frame)
        self._frames_written += 1
        self._last_write_time = now
        self._log.write(
            json.dumps({
                "stamp_ns": int(message.header.stamp.sec) * 1000000000
                            + int(message.header.stamp.nanosec),
                "frames_written": self._frames_written,
                "locked": self._tracker_ok,
                "label": self._locked_label,
                "score": self._locked_score,
                "command": self._command_text,
            }, ensure_ascii=False) + "\n"
        )
        self._log.flush()

    def _draw_overlay(self, frame, width: int, height: int):
        if self._pending is not None:
            label, score, bbox = self._pending
            x1, y1, x2, y2 = bbox
            pixel_box = (
                int(x1 * width), int(y1 * height),
                int(x2 * width), int(y2 * height),
            )
            try:
                self._tracker = cv2.TrackerCSRT_create()
                self._tracker.init(frame, pixel_box)
                self._locked_label = label
                self._locked_score = score
                self._tracker_ok = True
            except Exception:
                # Fallback: when the CSRT tracker is unavailable, draw the
                # detection box and label directly so the overlay still shows
                # the detected object and its confidence.
                cv2.rectangle(
                    frame,
                    (pixel_box[0], pixel_box[1]),
                    (pixel_box[2], pixel_box[3]),
                    (0, 255, 0), 2,
                )
                self._draw_cjk_text(
                    frame, f"{label} {score:.2f}",
                    (pixel_box[0], max(18, pixel_box[1] - 8)), 20,
                    (0, 255, 0), (0, 0, 0, 160),
                )
                self._tracker = None
            self._pending = None
        if self._tracker is not None:
            ok, box = self._tracker.update(frame)
            self._tracker_ok = bool(ok)
            if ok:
                x, y, w, h = (int(v) for v in box)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                text = f"{self._locked_label} {self._locked_score:.2f} LOCK"
                self._draw_cjk_text(
                    frame, text, (x, max(18, y - 8)), 20,
                    (0, 255, 0), (0, 0, 0, 160),
                )
            else:
                self._draw_cjk_text(
                    frame, "target lost, searching...", (12, 30), 22,
                    (0, 0, 255), (0, 0, 0, 160),
                )
        else:
            self._draw_cjk_text(
                frame, "searching...", (12, 30), 22,
                (0, 255, 255), (0, 0, 0, 160),
            )
        if self._command_text:
            self._draw_cjk_text(
                frame, f"指令: {self._command_text}",
                (12, height - 32), 22,
                (255, 255, 255), (0, 0, 0, 180),
            )

    def stop(self) -> None:
        self._stop.set()
        if self._executor is not None:
            self._executor.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        if self._node is not None:
            self._node.destroy_node()
        if self._writer is not None:
            self._writer.release()
            self._writer = None
        self._log.close()


class AutonomousLoop(Node):
    def __init__(self, pattern: list[str], output: str, forward_vx: float,
                 forward_seconds: float, max_yaw_rate: float,
                 min_clearance_m: float, mode: str, max_seconds: float,
                 wander_front_go_m: float, wander_turn_deg: float,
                 max_radius_m: float, scan_turn_deg: float,
                 scan_span: int, pre_scan_turns: int, record_video: str,
                 video_fps: float, video_scale: float,
                 scan360_steps: int, scan360_turn_deg: float,
                 odom_topic: str = "/go2w/odom/wheel") -> None:
        super().__init__("go2w_autonomous_loop")
        self._odom_topic = odom_topic
        self._pattern = pattern
        self._forward_vx = forward_vx
        self._forward_seconds = forward_seconds
        self._forward_duration_scale = 1.0
        # Real-robot logs show that ai-w reverse at 0.08 m/s only produces a
        # short breakaway twitch; 0.12 m/s is the calibrated reverse floor.
        self._backward_vx = 0.16
        self._backward_seconds = max(0.5, forward_seconds * 0.8)
        # Real Go2-W backward response is significantly slower than forward.
        # Keep the commanded distance bounded, but extend the timed-velocity
        # window so the odometry can actually confirm a short reverse move.
        self._backward_duration_scale = 1.40
        self._backward_min_step_m = 0.05
        self._backward_max_step_m = 0.12
        self._backward_max_age_sec = 8.0
        self._backward_heading_tolerance_deg = 8.0
        self._max_yaw_rate = max_yaw_rate
        self._min_clearance = min_clearance_m
        self._mode = mode
        self._max_seconds = max_seconds
        self._wander_front_go = wander_front_go_m
        self._wander_turn_deg = wander_turn_deg
        self._max_radius = max_radius_m
        self._scan_turn_deg = scan_turn_deg
        self._scan_span = scan_span
        self._pre_scan_turns = pre_scan_turns
        self._record_video = record_video
        self._video_fps = video_fps
        self._video_scale = video_scale
        self._video: BundleVideoRecorder | None = None
        self._scan360_steps = scan360_steps
        self._scan360_turn_deg = scan360_turn_deg
        self._output = open(output, "a", encoding="utf-8")
        self._start_monotonic = time.monotonic()
        self._motion_origin: tuple[float, float, float] | None = None
        self._turn_only = False
        self._front_half_plane_only = False
        self._boundary_tolerance_m = 0.05
        self._max_motion_steps = 0
        self._min_rotation_clearance_m = 0.0

        self._client = ActionClient(self, MotionCommand, "/go2w/motion")
        self._arm_client = self.create_client(SetBool, "/go2w/arm")
        self._stop_srv = self.create_client(Trigger, "/go2w/emergency_stop")

        self._sport: SportModeState | None = None
        self._low: LowState | None = None
        self._odom: Odometry | None = None
        self._odom_receive_monotonic: float | None = None
        self._odom_discontinuity_monotonic: float | None = None
        self._odom_discontinuity_reason = ""
        self._clearance: float | None = None
        self._left_clearance: float | None = None
        self._right_clearance: float | None = None
        self._lidar_fresh: bool | None = None
        self._rotation_clearance_valid: bool | None = None
        self._diagnostic_left_clearance: float | None = None
        # Dual-LiDAR safety fusion state (fail-closed while enabled).
        self._dual_lidar_enabled = False
        self._dual_lidar_fused_state: str | None = None
        self._dual_lidar_unknown_is_clear = False
        self._dual_lidar_occupied_sources: list[str] = []
        # Current-hardware binding for any pose-bound rotation lease.
        self._hardware_binding: dict | None = None
        self._hardware_geometry_hash: str | None = None
        self._hardware_state_hash: str | None = None
        # Explicit operator authorization to allow turns without a rotation
        # lease. Only relaxes the rotation-clearance/lease gate; every other
        # safety gate stays active and every motion records this flag.
        self._operator_authorized_rotation = False
        self._diagnostic_right_clearance: float | None = None
        self._diagnostic_clearance_receive_s: float | None = None
        self._rotation_lease: dict | None = None
        self._rotation_lease_path = ""
        self._rotation_lease_error = ""
        self._armed_by_runner = False
        # Breadcrumb-safe backward recovery: the most recently confirmed
        # forward corridor is the only autonomous reverse evidence available.
        self._breadcrumb: dict[str, Any] | None = None
        self._recovery_budget = {
            "consecutive": 0,
            "max_consecutive": 2,
            "total_m": 0.0,
            "max_total_m": 0.36,
        }
        qos = QoSProfile(depth=20, reliability=2)  # BEST_EFFORT
        self.create_subscription(SportModeState, "/lf/sportmodestate",
                                 self._on_sport, qos)
        self.create_subscription(LowState, "/lf/lowstate", self._on_low, qos)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom,
                                 QoSProfile(depth=50, reliability=2))
        self.create_subscription(Float32, "/go2w/safety/front_clearance",
                                 self._on_clearance, qos)
        self.create_subscription(Float32, "/go2w/safety/left_clearance",
                                 self._on_left_clearance, qos)
        self.create_subscription(Float32, "/go2w/safety/right_clearance",
                                 self._on_right_clearance, qos)
        self.create_subscription(
            Bool,
            "/go2w/safety/lidar_fresh",
            self._on_lidar_fresh,
            qos,
        )
        self.create_subscription(
            Bool,
            "/go2w/safety/rotation_clearance_valid",
            self._on_rotation_clearance_valid,
            qos,
        )
        self.create_subscription(
            Vector3Stamped,
            "/go2w/diagnostics/lidar_clearance_raw",
            self._on_diagnostic_clearance,
            qos,
        )
        # Pandar diagnostic status (only published when the diagnostic
        # preprocessor is running). Missing topic keeps the field None which
        # the fail-closed snapshot treats as not-fresh.
        try:
            from diagnostic_msgs.msg import DiagnosticArray

            self._pandar_raw_fresh: bool | None = None
            self.create_subscription(
                DiagnosticArray,
                "/go2w/hesai/status",
                self._on_pandar_status,
                qos,
            )
        except Exception:
            self._pandar_raw_fresh: bool | None = None

    def _on_pandar_status(self, msg) -> None:
        fresh = False
        for status in msg.status:
            for value in status.values:
                if value.key == "fresh":
                    fresh = str(value.value).lower() == "true"
                    break
        self._pandar_raw_fresh = fresh

    def _host_s(self) -> float:
        return round(time.monotonic(), 6)

    def _write(self, row: dict) -> None:
        self._output.write(
            json.dumps(
                strict_json_value(row),
                ensure_ascii=False,
                allow_nan=False,
            )
            + "\n"
        )
        self._output.flush()

    def _on_sport(self, msg: SportModeState) -> None:
        self._sport = msg

    def _on_low(self, msg: LowState) -> None:
        self._low = msg

    def _on_odom(self, msg: Odometry) -> None:
        now = time.monotonic()
        if self._odom is not None and self._odom_receive_monotonic is not None:
            old = self._odom.pose.pose.position
            new = msg.pose.pose.position
            elapsed = now - self._odom_receive_monotonic
            jump = math.hypot(float(new.x) - float(old.x), float(new.y) - float(old.y))
            # A Go2-W cannot translate 0.75 m between adjacent odometry
            # samples.  This most commonly means two publishers with different
            # origins are interleaving on the same topic.  Keep the last good
            # sample and fail motion closed until the stream stabilizes.
            if elapsed < 1.0 and jump > 0.75:
                self._odom_discontinuity_monotonic = now
                self._odom_discontinuity_reason = (
                    "ODOM_DISCONTINUITY: adjacent odometry samples jumped "
                    f"{jump:.3f}m in {elapsed:.3f}s; duplicate publishers or "
                    "an odometry reset are likely"
                )
                return
        self._odom = msg
        self._odom_receive_monotonic = now

    def _on_clearance(self, msg: Float32) -> None:
        self._clearance = float(msg.data)

    def _on_left_clearance(self, msg: Float32) -> None:
        self._left_clearance = float(msg.data)

    def _on_right_clearance(self, msg: Float32) -> None:
        self._right_clearance = float(msg.data)

    def _on_lidar_fresh(self, msg: Bool) -> None:
        self._lidar_fresh = bool(msg.data)

    def _on_rotation_clearance_valid(self, msg: Bool) -> None:
        self._rotation_clearance_valid = bool(msg.data)

    def _on_diagnostic_clearance(self, msg: Vector3Stamped) -> None:
        if msg.header.frame_id != "base_link":
            self._diagnostic_left_clearance = None
            self._diagnostic_right_clearance = None
            self._diagnostic_clearance_receive_s = None
            return
        self._diagnostic_left_clearance = float(msg.vector.y)
        self._diagnostic_right_clearance = float(msg.vector.z)
        self._diagnostic_clearance_receive_s = time.monotonic()

    def _yaw(self) -> float:
        if self._sport is None:
            return 0.0
        return float(self._sport.imu_state.rpy[2])

    def _wait_for(self, predicate, timeout: float, label: str) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        self.get_logger().error(f"timeout waiting for {label}")
        return False

    def _safety_ok(self, step: str | None = None) -> tuple[bool, str]:
        if (
            self._odom_discontinuity_monotonic is not None
            and time.monotonic() - self._odom_discontinuity_monotonic < 5.0
        ):
            return False, self._odom_discontinuity_reason
        if self._sport is None:
            return False, "no sport state"
        if int(self._sport.mode) != 1 or int(self._sport.error_code) != 0:
            return False, (f"robot mode={self._sport.mode} "
                           f"error={self._sport.error_code}")
        if step is not None and step.startswith("b"):
            backward = self._backward_safety_ok()
            if not backward.allowed:
                return False, backward.reason
            return True, ""
        lidar = evaluate_lidar_motion_readiness(
            lidar_fresh=self._lidar_fresh,
            front_clearance_m=self._clearance,
            minimum_clearance_m=self._min_clearance,
        )
        if not lidar.allowed:
            return False, lidar.reason
        return True, ""

    def _backward_safety_ok(self):
        """Breadcrumb-only reverse safety gate (fail closed)."""
        breadcrumb = self._breadcrumb
        if breadcrumb is None or breadcrumb.get("invalidated"):
            return MotionBoundaryDecision(
                False, "NO_VALID_REVERSE_CORRIDOR: no confirmed forward breadcrumb"
            )
        age = time.monotonic() - float(breadcrumb.get("created_monotonic", 0.0))
        if age > self._backward_max_age_sec:
            return MotionBoundaryDecision(
                False,
                "NO_VALID_REVERSE_CORRIDOR: breadcrumb expired "
                f"({age:.1f}s > {self._backward_max_age_sec:.1f}s)",
            )
        _, _, yaw = self._odom_snapshot()
        heading_error = abs(
            (yaw - float(breadcrumb.get("heading_rad", yaw)) + math.pi)
            % (2.0 * math.pi) - math.pi
        )
        heading_error_deg = math.degrees(heading_error)
        if heading_error_deg > self._backward_heading_tolerance_deg:
            return MotionBoundaryDecision(
                False,
                "NO_VALID_REVERSE_CORRIDOR: heading drift "
                f"{heading_error_deg:.1f}deg > "
                f"{self._backward_heading_tolerance_deg:.1f}deg",
            )
        if self._recovery_budget["consecutive"] >= self._recovery_budget["max_consecutive"]:
            return MotionBoundaryDecision(
                False,
                "RECOVERY_BUDGET_EXCEEDED: too many consecutive backward recoveries",
            )
        if self._recovery_budget["total_m"] >= self._recovery_budget["max_total_m"]:
            return MotionBoundaryDecision(
                False,
                "RECOVERY_BUDGET_EXCEEDED: cumulative backward recovery distance exceeded",
            )
        return MotionBoundaryDecision(True, reason="breadcrumb_safe")

    def _record_breadcrumb(
        self,
        before: tuple[float, float, float],
        after: tuple[float, float, float],
        distance_m: float,
        source_step: str,
    ) -> None:
        """Record a confirmed forward corridor as the reverse safety evidence."""
        self._breadcrumb = {
            "start_pose": list(before),
            "end_pose": list(after),
            "signed_distance_m": round(distance_m, 4),
            "heading_rad": before[2],
            "confirmed": True,
            "created_monotonic": time.monotonic(),
            "source_step": source_step,
            "invalidated": False,
        }
        self._write({
            "event": "breadcrumb_created",
            "host_s": self._host_s(),
            "start_pose": list(before),
            "end_pose": list(after),
            "distance_m": round(distance_m, 3),
            "heading_rad": before[2],
        })

    def _consume_breadcrumb(self, distance_m: float) -> None:
        """Mark the breadcrumb consumed after a successful backward recovery."""
        if self._breadcrumb is not None:
            self._breadcrumb["invalidated"] = True
        self._recovery_budget["consecutive"] += 1
        self._recovery_budget["total_m"] = round(
            self._recovery_budget["total_m"] + float(distance_m), 4
        )
        self._write({
            "event": "breadcrumb_consumed",
            "host_s": self._host_s(),
            "distance_m": round(distance_m, 3),
            "recovery_budget": dict(self._recovery_budget),
        })

    def _maybe_autonomous_reverse(
        self, index: int, step: str, failure_reason: str
    ) -> None:
        """Best-effort breadcrumb reverse after a forward failure.

        This is a low-level safety fallback for the most common recovery
        trigger (front too close / forward not confirmed).  It never retries
        the failed forward step and never executes without a valid breadcrumb.
        """
        if not step.startswith("f"):
            return
        upper = str(failure_reason or "").upper()
        if "MOTION_TIMEOUT" in upper or "EMERGENCY" in upper:
            return
        safety = self._backward_safety_ok()
        if not safety.allowed:
            self._write({
                "event": "autonomous_recovery_skipped",
                "index": index,
                "step": step,
                "reason": safety.reason,
                "host_s": self._host_s(),
            })
            return
        recovery_step = f"b{self._backward_max_step_m:.2f}"
        self._write({
            "event": "autonomous_recovery_attempt",
            "index": index,
            "step": step,
            "recovery_step": recovery_step,
            "reason": failure_reason,
            "host_s": self._host_s(),
        })
        try:
            ok, reason = self._execute_step(index + 1, recovery_step, attempts=1)
            self._write({
                "event": "autonomous_recovery_result",
                "index": index,
                "recovery_step": recovery_step,
                "ok": ok,
                "reason": reason,
                "host_s": self._host_s(),
            })
        except Exception as exc:  # noqa: BLE001 - recovery never crashes search
            self.get_logger().error(
                f"autonomous reverse recovery failed: {type(exc).__name__}: {exc}"
            )

    def _call_service(self, client, request, label: str):
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"{label} service unavailable")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            raise RuntimeError(f"{label} service timeout")
        return future.result()

    def _arm(self, value: bool) -> None:
        request = SetBool.Request()
        request.data = value
        last_error = ""
        # MotionSwitcher/lease status can be transiently stale immediately
        # after a Move/STOP cycle.  Retry briefly before failing closed; this
        # is still safe because a failed arm never sends a motion goal.
        for attempt in range(1, 4):
            try:
                response = self._call_service(
                    self._arm_client, request,
                    "arm" if value else "disarm",
                )
                if response.success:
                    self._armed_by_runner = bool(value)
                    if attempt > 1:
                        self.get_logger().warning(
                            f"arm({'on' if value else 'off'}) recovered on "
                            f"attempt {attempt}"
                        )
                    return
                last_error = f"arm response failed: {response.message}"
            except RuntimeError as exc:
                last_error = str(exc)
            if attempt < 3:
                self.get_logger().warning(
                    f"arm({'on' if value else 'off'}) attempt {attempt} "
                    f"failed ({last_error}); retrying after settle"
                )
                time.sleep(1.0)
        raise RuntimeError(last_error or "arm failed after retries")

    def _emergency_stop(self) -> None:
        response = self._call_service(self._stop_srv, Trigger.Request(),
                                      "emergency stop")
        if not response.success:
            self.get_logger().error(f"emergency stop failed: {response.message}")

    def _send_goal(self, goal) -> dict:
        if not self._client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("motion action server unavailable")
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            try:
                self._emergency_stop()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"emergency stop after goal-accept timeout failed: {exc}"
                )
            raise MotionGoalTimeout(
                "MOTION_ACCEPT_TIMEOUT: /go2w/motion did not acknowledge the "
                "goal within 10s; possible duplicate action server or DDS "
                "response loss; emergency stop requested"
            )
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("motion goal rejected")
        result_future = goal_handle.get_result_async()
        # The server already has a bounded goal.timeout_sec.  Waiting a fixed
        # 90 seconds here hid dropped/mismatched action responses from the UI
        # and, together with _execute_step's retry, could duplicate a command.
        result_timeout = min(
            45.0,
            max(10.0, float(getattr(goal, "timeout_sec", 0.0) or 0.0) + 5.0),
        )
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=result_timeout
        )
        if not result_future.done():
            # Best-effort action cancellation followed by the independent
            # emergency-stop service.  Do not wait indefinitely for either
            # response; the caller treats this exception as non-retryable.
            try:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(
                    self, cancel_future, timeout_sec=2.0
                )
            except Exception as exc:  # noqa: BLE001 - safety fallback below
                self.get_logger().error(f"motion cancel failed: {exc}")
            try:
                self._emergency_stop()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"emergency stop after timeout failed: {exc}")
            raise MotionGoalTimeout(
                "MOTION_RESULT_TIMEOUT: /go2w/motion returned no result within "
                f"{result_timeout:.1f}s (server goal timeout="
                f"{float(getattr(goal, 'timeout_sec', 0.0) or 0.0):.1f}s); "
                "goal cancelled and emergency stop requested"
            )
        wrapped_result = result_future.result()
        if wrapped_result is None or wrapped_result.result is None:
            raise RuntimeError(
                "MOTION_EMPTY_RESULT: /go2w/motion completed without a result payload"
            )
        return {
            "success": bool(wrapped_result.result.success),
            "message": str(wrapped_result.result.message),
            "yaw_deg": float(wrapped_result.result.actual_relative_yaw_deg),
            "elapsed": float(wrapped_result.result.elapsed_sec),
            "estimated_distance_m": float(
                wrapped_result.result.estimated_distance_m
            ),
        }

    def _odom_snapshot(self) -> tuple[float, float, float]:
        if self._odom is None:
            return (0.0, 0.0, 0.0)
        pose = self._odom.pose.pose
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.z * q.w), 1.0 - 2.0 * (q.z * q.z))
        return (float(pose.position.x), float(pose.position.y), yaw)

    def _describe_step(self, step: str) -> str:
        if step.startswith("f"):
            distance = forward_step_distance(
                step, self._forward_vx * self._forward_seconds
            )
            return f"前进 {distance:.2f} m"
        if step.startswith("b"):
            distance = backward_step_distance(
                step, self._backward_vx * self._backward_seconds
            )
            return f"后退 {distance:.2f} m"
        if step.startswith("l"):
            return f"左转 {step[1:]}°"
        if step.startswith("r"):
            return f"右转 {step[1:]}°"
        return step

    def _execute_step(self, index: int, step: str,
                      attempts: int = 2) -> tuple[bool, str]:
        if self._video is not None:
            self._video.set_command(self._describe_step(step))
        self._write({"event": "step_start", "index": index, "step": step,
                     "host_s": self._host_s(), "clearance": self._clearance,
                     "operator_authorized_rotation": self._operator_authorized_rotation})
        origin = self._motion_origin or self._odom_snapshot()
        current = self._odom_snapshot()
        if self._rotation_lease is not None:
            lease_step = evaluate_rotation_lease_step(step, maximum_turn_deg=30.0)
            if not lease_step.allowed:
                self._write({
                    "event": "rotation_lease_step_reject",
                    "index": index,
                    "step": step,
                    "reason": lease_step.reason,
                    "host_s": self._host_s(),
                })
                self.get_logger().error(lease_step.reason)
                return False, lease_step.reason
        elif self._operator_authorized_rotation and (
            step.startswith("l") or step.startswith("r")
        ):
            # Even without a lease, operator-authorized turns stay clamped to
            # (0, 30] degrees so the search sweep cannot command a large spin.
            lease_step = evaluate_rotation_lease_step(step, maximum_turn_deg=30.0)
            if not lease_step.allowed:
                self._write({
                    "event": "operator_rotation_step_clamped",
                    "index": index,
                    "step": step,
                    "reason": lease_step.reason,
                    "host_s": self._host_s(),
                })
                self.get_logger().warning(
                    f"operator-authorized turn clamped: {lease_step.reason}"
                )
                return False, lease_step.reason
        if step.startswith("f") or step.startswith("b"):
            _, expected_forward_m = translation_step_distance(
                step,
                default_forward_m=(
                    self._forward_vx * self._forward_seconds
                ),
                default_backward_m=(
                    self._backward_vx * self._backward_seconds
                ),
            )
        else:
            expected_forward_m = self._forward_vx * self._forward_seconds
        boundary = evaluate_step_boundary(
            step,
            origin=origin,
            current=current,
            max_radius_m=self._max_radius,
            front_half_plane_only=self._front_half_plane_only,
            turn_only=self._turn_only,
            forward_distance_m=expected_forward_m,
            tolerance_m=self._boundary_tolerance_m,
        )
        if not boundary.allowed:
            self._write({
                "event": "motion_boundary_reject",
                "index": index,
                "step": step,
                "reason": boundary.reason,
                "current": list(current),
                "predicted_position": list(boundary.predicted_position or current[:2]),
                "host_s": self._host_s(),
            })
            self.get_logger().error(
                f"motion boundary rejected step {step}: {boundary.reason}"
            )
            return False, boundary.reason
        raw_age = (
            time.monotonic() - self._diagnostic_clearance_receive_s
            if self._diagnostic_clearance_receive_s is not None
            else math.inf
        )
        rotation_inputs = resolve_rotation_clearance_source(
            formal_left_clearance_m=self._left_clearance,
            formal_right_clearance_m=self._right_clearance,
            formal_valid=self._rotation_clearance_valid,
            lease=self._rotation_lease,
            current_pose=current,
            current_frame=(self._odom.header.frame_id if self._odom is not None else ""),
            diagnostic_left_clearance_m=self._diagnostic_left_clearance,
            diagnostic_right_clearance_m=self._diagnostic_right_clearance,
            diagnostic_age_seconds=raw_age,
            lidar_fresh=self._lidar_fresh,
            now=datetime.now(timezone.utc),
            expected_binding=self._hardware_binding,
        )
        rotation_clearance = evaluate_rotation_clearance(
            step,
            left_clearance_m=rotation_inputs.left_clearance_m,
            right_clearance_m=rotation_inputs.right_clearance_m,
            minimum_clearance_m=self._min_rotation_clearance_m,
            clearance_valid=rotation_inputs.valid,
        )
        if not rotation_clearance.allowed:
            reason = rotation_inputs.reason or rotation_clearance.reason
            if self._operator_authorized_rotation:
                # Operator explicitly authorized turns without a rotation lease.
                # Every motion still passes mode/error, lidar fresh, front
                # clearance, motion bounds, turn<=30deg and the emergency stop.
                self._write({
                    "event": "operator_authorized_rotation_applied",
                    "index": index,
                    "step": step,
                    "overridden_reason": reason,
                    "left_clearance_m": rotation_inputs.left_clearance_m,
                    "right_clearance_m": rotation_inputs.right_clearance_m,
                    "rotation_clearance_valid": rotation_inputs.valid,
                    "host_s": self._host_s(),
                })
                self.get_logger().warning(
                    f"operator-authorized rotation overrides step {step}: {reason}"
                )
            else:
                self._write({
                    "event": "rotation_clearance_reject",
                    "index": index,
                    "step": step,
                    "reason": reason,
                    "left_clearance_m": rotation_inputs.left_clearance_m,
                    "right_clearance_m": rotation_inputs.right_clearance_m,
                    "rotation_clearance_valid": rotation_inputs.valid,
                    "rotation_clearance_source": rotation_inputs.source,
                    "required_clearance_m": self._min_rotation_clearance_m,
                    "host_s": self._host_s(),
                })
                self.get_logger().error(
                    f"rotation clearance rejected step {step}: "
                    f"{reason}"
                )
                return False, reason
        if self._dual_lidar_enabled:
            dual_gate = evaluate_dual_lidar_rotation_gate(
                fused_state=self._dual_lidar_fused_state,
                dual_lidar_enabled=True,
                unknown_is_clear=self._dual_lidar_unknown_is_clear,
                occupied_sources=self._dual_lidar_occupied_sources,
            )
            if not dual_gate.allowed:
                self._write({
                    "event": "dual_lidar_rotation_reject",
                    "index": index,
                    "step": step,
                    "reason": dual_gate.reason,
                    "fused_state": self._dual_lidar_fused_state,
                    "host_s": self._host_s(),
                })
                self.get_logger().error(
                    f"dual-lidar rotation gate rejected step {step}: "
                    f"{dual_gate.reason}"
                )
                return False, dual_gate.reason
        ok, reason = self._safety_ok(step)
        if not ok:
            self._write({"event": "abort", "index": index, "step": step,
                         "reason": reason, "host_s": self._host_s()})
            self.get_logger().error(f"abort before step {step}: {reason}")
            self._maybe_autonomous_reverse(index, step, reason)
            return False, reason
        for attempt in range(1, attempts + 1):
            if attempt > 1:
                time.sleep(0.8)
            try:
                # The arm state expires after arm_timeout_sec on the action
                # server; long LLM detections can exceed it, so re-arm right
                # before every motion command (idempotent).
                self._arm(True)
            except RuntimeError as exc:
                self._write({"event": "abort", "index": index, "step": step,
                             "reason": f"arm failed: {exc}",
                             "host_s": self._host_s()})
                self.get_logger().error(f"arm failed before step {step}: {exc}")
                return False, f"arm failed: {exc}"
            before = self._odom_snapshot()
            goal = MotionCommand.Goal()
            if step.startswith("f") or step.startswith("b"):
                goal.mode = MotionCommand.Goal.MODE_TIMED_VELOCITY
                goal.vx = (
                    self._forward_vx
                    if step.startswith("f")
                    else -abs(self._backward_vx)
                )
                duration_scale = (
                    self._backward_duration_scale
                    if step.startswith("b")
                    else self._forward_duration_scale
                )
                goal.duration_sec = (
                    expected_forward_m
                    / abs(goal.vx)
                    * duration_scale
                )
                goal.timeout_sec = goal.duration_sec + 8.0
            elif step.startswith("l") or step.startswith("r"):
                degrees = float(step[1:])
                if step.startswith("r"):
                    degrees = -degrees
                goal.mode = MotionCommand.Goal.MODE_RELATIVE_YAW
                goal.relative_yaw_deg = degrees
                goal.max_yaw_rate = self._max_yaw_rate
                goal.timeout_sec = max(
                    15.0, abs(degrees) / self._max_yaw_rate * 3.0 + 8.0
                )
            else:
                self.get_logger().error(f"unknown step {step}")
                return False, f"unknown step {step}"
            try:
                result = self._send_goal(goal)
            except MotionGoalTimeout as exc:
                # Never retry an indeterminate action: the request might have
                # executed even though DDS lost its response.  Retrying could
                # unexpectedly rotate/advance the robot a second time.
                reason = str(exc)
                self._write({
                    "event": "motion_timeout",
                    "index": index,
                    "step": step,
                    "attempt": attempt,
                    "reason": reason,
                    "non_retryable": True,
                    "host_s": self._host_s(),
                })
                self.get_logger().error(
                    f"step {step} timed out and was not retried: {reason}"
                )
                return False, reason
            except (RuntimeError, OSError, ValueError) as exc:
                if attempt < attempts:
                    self.get_logger().warn(
                        f"step {step} attempt {attempt} send failed "
                        f"({exc}); retrying"
                    )
                    continue
                self._write({"event": "abort", "index": index, "step": step,
                             "reason": str(exc),
                             "host_s": self._host_s()})
                self.get_logger().error(
                    f"step {step} send failed: {exc}"
                )
                return False, str(exc)
            self._write({"event": "step_result", "index": index, "step": step,
                         "attempt": attempt, "host_s": self._host_s(),
                         "result": result})
            if not result["success"]:
                # An accepted Action may have moved partially before its
                # encoder/stop verification failed.  Retrying could accumulate
                # an unknown displacement, so every returned motion failure is
                # terminal for this step.
                self._write({"event": "abort", "index": index, "step": step,
                             "reason": result["message"],
                             "non_retryable": True,
                             "host_s": self._host_s()})
                self.get_logger().error(f"step failed: {result['message']}")
                self._maybe_autonomous_reverse(index, step, result["message"])
                return False, result["message"]
            time.sleep(1.0)
            rclpy.spin_once(self, timeout_sec=0.2)
            after = self._odom_snapshot()
            actual_boundary = position_within_boundary(
                origin=origin,
                position=after[:2],
                max_radius_m=self._max_radius,
                front_half_plane_only=self._front_half_plane_only,
                tolerance_m=self._boundary_tolerance_m,
            )
            if not actual_boundary.allowed:
                self._write({
                    "event": "motion_boundary_violation",
                    "index": index,
                    "step": step,
                    "reason": actual_boundary.reason,
                    "actual": list(after),
                    "host_s": self._host_s(),
                })
                self._emergency_stop()
                return False, actual_boundary.reason
            verification = evaluate_motion_observation(
                step,
                before=before,
                after=after,
                expected_forward_m=expected_forward_m,
            )
            self._write({"event": "step_verified", "index": index, "step": step,
                         "attempt": attempt, "host_s": self._host_s(),
                         "distance_m": verification.distance_m,
                         "yaw_delta_rad": math.radians(
                             verification.yaw_delta_deg
                         ),
                         "expected_turn_deg": abs(float(step[1:]))
                         if len(step) > 1
                         and not (step.startswith("f") or step.startswith("b"))
                         else 0.0,
                         "expected_forward_m": expected_forward_m,
                         "action_estimated_distance_m": result.get(
                             "estimated_distance_m"
                         ),
                         "verification_code": verification.code,
                         "verification_reason": verification.reason,
                         "verified": bool(verification.allowed)})
            if verification.allowed:
                if step.startswith("f"):
                    self._record_breadcrumb(before, after, expected_forward_m, step)
                    # A successful forward step creates a fresh corridor; any
                    # old recovery budget from a different corridor should not
                    # count against this new one.
                    self._recovery_budget["consecutive"] = 0
                    self._recovery_budget["total_m"] = 0.0
                elif step.startswith("b"):
                    self._consume_breadcrumb(expected_forward_m)
                return True, ""
            # An RPC-success/observation-failure is indeterminate.  Retrying
            # could double an action that physically happened, so stop and
            # surface the exact reason instead of silently retrying.
            try:
                self._emergency_stop()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"emergency stop after verification failure failed: {exc}"
                )
            self.get_logger().error(verification.reason)
            self._maybe_autonomous_reverse(index, step, verification.reason)
            return False, verification.reason
        self._write({"event": "abort", "index": index, "step": step,
                     "reason": "wheel odometry did not confirm motion after retries",
                     "host_s": self._host_s()})
        self.get_logger().error("wheel odometry did not confirm motion")
        self._maybe_autonomous_reverse(
            index, step, "wheel odometry did not confirm motion after retries"
        )
        return False, "wheel odometry did not confirm motion after retries"

    def _next_wander_step(self) -> str:
        front = self._clearance if self._clearance is not None else 0.0
        left = self._left_clearance if self._left_clearance is not None else 0.0
        right = self._right_clearance if self._right_clearance is not None else 0.0
        if front > self._wander_front_go:
            return "f"
        if left > right:
            return f"l{int(self._wander_turn_deg)}"
        return f"r{int(self._wander_turn_deg)}"

    def _run_camera_guided(
        self,
        target: str,
        spool_root: str,
        score_min: float,
        align_threshold: float,
        align_yaw_max_deg: float,
        reach_area_ratio: float,
    ) -> None:
        env = self._load_detector_env()
        prompt = PROMPT_MAP.get(target.strip(), f"{target.strip()}. {target.strip()} object")
        self._write({"event": "camera_guided_start", "host_s": self._host_s(),
                     "target": target, "prompt": prompt})
        started = time.monotonic()
        index = 0
        alternate = 0
        while time.monotonic() - started < self._max_seconds:
            ok, reason = self._safety_ok()
            if not ok:
                self._write({"event": "abort", "index": index,
                             "reason": reason, "host_s": self._host_s()})
                break
            try:
                image_path, frame_id = self._latest_bundle_image(spool_root)
                self._write({"event": "camera_bundle", "index": index,
                             "frame_id": frame_id, "host_s": self._host_s()})
                objects = self._detect(image_path, prompt, env)
            except (RuntimeError, OSError, ValueError) as exc:
                self._write({"event": "detection_error", "index": index,
                             "error": str(exc), "host_s": self._host_s()})
                self.get_logger().warn(f"detection failed: {exc}")
                if self._is_stale_error(exc):
                    self._write({"event": "abort", "index": index,
                                 "reason": str(exc),
                                 "host_s": self._host_s()})
                    break
                step = "l30" if alternate % 2 == 0 else "r30"
                alternate += 1
                ok, _ = self._execute_step(index, step)
                if not ok:
                    break
                index += 1
                continue
            best = max(
                (item for item in objects if float(item.get("score", 0.0)) >= score_min),
                key=lambda item: float(item.get("score", 0.0)),
                default=None,
            )
            if best is None:
                self._write({"event": "target_not_found", "index": index,
                             "objects": len(objects),
                             "host_s": self._host_s()})
                step = "l30" if alternate % 2 == 0 else "r30"
                alternate += 1
                ok, _ = self._execute_step(index, step)
                if not ok:
                    break
                index += 1
                continue
            bbox = [float(value) for value in best.get("bbox_2d", [0.0, 0.0, 1.0, 1.0])]
            x1, y1, x2, y2 = bbox
            center_x = (x1 + x2) / 2.0
            area_ratio = max(0.0, (x2 - x1) * (y2 - y1))
            offset = center_x - 0.5
            self._write({"event": "target_found", "index": index,
                         "label": str(best.get("label", "object")),
                         "score": round(float(best.get("score", 0.0)), 3),
                         "center_x": round(center_x, 3),
                         "area_ratio": round(area_ratio, 4),
                         "host_s": self._host_s()})
            self._feed_detection(
                str(best.get("label", "object")),
                float(best.get("score", 0.0)),
                (x1, y1, x2, y2),
            )
            self.get_logger().info(
                f"target {best.get('label')} "
                f"score={float(best.get('score', 0.0)):.2f} "
                f"cx={center_x:.2f} area={area_ratio:.3f}"
            )
            if area_ratio >= reach_area_ratio:
                self._write({"event": "target_reached", "index": index,
                             "host_s": self._host_s()})
                self.get_logger().info("target reached; stopping")
                break
            if abs(offset) > align_threshold:
                degrees = max(
                    -align_yaw_max_deg,
                    min(align_yaw_max_deg, -offset * align_yaw_max_deg * 2.0),
                )
                step = (
                    f"l{int(abs(degrees))}"
                    if degrees > 0.0
                    else f"r{int(abs(degrees))}"
                )
            else:
                step = "f"
            ok, _ = self._execute_step(index, step)
            if not ok:
                break
            index += 1
        else:
            self._write({"event": "camera_guided_time_limit",
                         "host_s": self._host_s()})

    def _scan_sequence(self) -> list[str]:
        deg = int(self._scan_turn_deg)
        right = [f"r{deg}"] * self._scan_span
        left = [f"l{deg}"] * self._scan_span
        # Sweep right, return, small forward, sweep left, return, small forward.
        # Net heading is zero and the tether is not progressively twisted.
        return right + left + ["f"] + left + right + ["f"]

    def _distance_from(self, origin: tuple[float, float, float]) -> float:
        current = self._odom_snapshot()
        return math.hypot(current[0] - origin[0], current[1] - origin[1])

    def _feed_detection(
        self, label: str, score: float,
        bbox_xyxy: tuple[float, float, float, float],
    ) -> None:
        if self._video is not None:
            self._video.set_detection(label, score, bbox_xyxy)

    def _run_level_a_search(
        self,
        target: str,
        spool_root: str,
        score_min: float,
        align_threshold: float,
        align_yaw_max_deg: float,
        reach_area_ratio: float,
    ) -> None:
        env = self._load_detector_env()
        prompt = PROMPT_MAP.get(
            target.strip(), f"{target.strip()}. {target.strip()} object"
        )
        origin = self._odom_snapshot()
        scan = self._scan_sequence()
        scan_index = 0
        self._write({
            "event": "level_a_start",
            "host_s": self._host_s(),
            "target": target,
            "prompt": prompt,
            "max_radius_m": self._max_radius,
            "scan_turn_deg": self._scan_turn_deg,
            "scan_span": self._scan_span,
        })
        self.get_logger().info(
            f"Level A search start: target={target} "
            f"radius_limit={self._max_radius if self._max_radius > 0 else 'unlimited'}"
        )
        started = time.monotonic()
        index = 0
        for _ in range(self._pre_scan_turns):
            ok, reason = self._safety_ok()
            if not ok:
                self._write({"event": "abort", "index": index,
                             "reason": reason, "host_s": self._host_s()})
                break
            step = scan[scan_index % len(scan)]
            scan_index += 1
            self._write({"event": "search_step", "index": index,
                         "step": step, "phase": "PRE_SCAN",
                         "host_s": self._host_s()})
            ok, _ = self._execute_step(index, step)
            if not ok:
                break
            index += 1
        while time.monotonic() - started < self._max_seconds:
            ok, reason = self._safety_ok()
            if not ok:
                self._write({"event": "abort", "index": index,
                             "reason": reason, "host_s": self._host_s()})
                break
            distance = self._distance_from(origin)
            if self._max_radius > 0.0 and distance > self._max_radius:
                self._write({"event": "range_limit", "index": index,
                             "distance_m": round(distance, 3),
                             "host_s": self._host_s()})
                self.get_logger().warn(
                    f"range limit reached ({distance:.2f} m); stopping"
                )
                break
            try:
                image_path, frame_id = self._latest_bundle_image(spool_root)
                self._write({"event": "camera_bundle", "index": index,
                             "frame_id": frame_id, "host_s": self._host_s()})
                objects = self._detect(image_path, prompt, env)
            except (RuntimeError, OSError, ValueError) as exc:
                self._write({"event": "detection_error", "index": index,
                             "error": str(exc), "host_s": self._host_s()})
                self.get_logger().warn(f"detection failed: {exc}")
                if self._is_stale_error(exc):
                    self._write({"event": "abort", "index": index,
                                 "reason": str(exc),
                                 "host_s": self._host_s()})
                    break
                step = "r30"
                ok, _ = self._execute_step(index, step)
                if not ok:
                    break
                index += 1
                continue
            best = max(
                (item for item in objects
                 if float(item.get("score", 0.0)) >= score_min),
                key=lambda item: float(item.get("score", 0.0)),
                default=None,
            )
            if self._video is not None:
                if best is None:
                    self._video.set_command("LLM: 未发现目标，扫描中")
                else:
                    self._video.set_command(
                        f"LLM: {best.get('label')} "
                        f"score={float(best.get('score', 0.0)):.2f}"
                    )
            if best is None:
                step = scan[scan_index % len(scan)]
                scan_index += 1
                if step == "f":
                    step_estimate = (
                        self._forward_seconds * self._forward_vx * 0.6
                    )
                    if (self._max_radius > 0.0
                            and distance + step_estimate >= self._max_radius):
                        step = "r30"
                self._write({"event": "search_step", "index": index,
                             "step": step, "phase": "SEARCH",
                             "distance_m": round(distance, 3),
                             "objects": len(objects),
                             "host_s": self._host_s()})
                ok, _ = self._execute_step(index, step)
                if not ok:
                    break
                index += 1
                continue

            bbox = [
                float(value)
                for value in best.get("bbox_2d", [0.0, 0.0, 1.0, 1.0])
            ]
            x1, y1, x2, y2 = bbox
            center_x = (x1 + x2) / 2.0
            area_ratio = max(0.0, (x2 - x1) * (y2 - y1))
            offset = center_x - 0.5
            self._write({"event": "target_found", "index": index,
                         "label": str(best.get("label", "object")),
                         "score": round(float(best.get("score", 0.0)), 3),
                         "center_x": round(center_x, 3),
                         "area_ratio": round(area_ratio, 4),
                         "distance_m": round(distance, 3),
                         "host_s": self._host_s()})
            self._feed_detection(
                str(best.get("label", "object")),
                float(best.get("score", 0.0)),
                (x1, y1, x2, y2),
            )
            self.get_logger().info(
                f"DISCOVERED {best.get('label')} score="
                f"{float(best.get('score', 0.0)):.2f} "
                f"cx={center_x:.2f} area={area_ratio:.3f} "
                f"range={distance:.2f} m"
            )
            if area_ratio >= reach_area_ratio:
                try:
                    verification = self._verify_target(image_path, bbox, env)
                except (RuntimeError, OSError, ValueError) as exc:
                    verification = {
                        "object_name_zh": "复核失败",
                        "is_target": False,
                        "confidence": 0.0,
                        "reason_zh": str(exc),
                    }
                self._write({"event": "target_verification", "index": index,
                             "label": str(best.get("label", "object")),
                             "area_ratio": round(area_ratio, 4),
                             "verification": verification,
                             "host_s": self._host_s()})
                if self._video is not None:
                    verdict = "通过" if verification.get("is_target") else "拒绝"
                    self._video.set_command(
                        f"复核: {verification.get('object_name_zh', '?')} {verdict}"
                    )
                if verification.get("is_target"):
                    self._write({"event": "target_reached", "index": index,
                                 "verification": verification,
                                 "host_s": self._host_s()})
                    self.get_logger().info(
                        f"target reached and verified: "
                        f"{verification.get('object_name_zh')} "
                        f"({verification.get('reason_zh')})"
                    )
                    break
                self.get_logger().warn(
                    f"target verification rejected: "
                    f"{verification.get('object_name_zh')} - "
                    f"{verification.get('reason_zh')}"
                )
                step = "r15"
                ok, _ = self._execute_step(index, step)
                if not ok:
                    break
                index += 1
                continue
            if abs(offset) > align_threshold:
                degrees = max(
                    -align_yaw_max_deg,
                    min(align_yaw_max_deg, -offset * align_yaw_max_deg * 2.0),
                )
                step = (
                    f"l{int(abs(degrees))}"
                    if degrees > 0.0
                    else f"r{int(abs(degrees))}"
                )
            else:
                step = "f"
                step_estimate = (
                    self._forward_seconds * self._forward_vx * 0.6
                )
                if (self._max_radius > 0.0
                        and distance + step_estimate >= self._max_radius):
                    self._write({"event": "range_limit", "index": index,
                                 "phase": "APPROACH",
                                 "distance_m": round(distance, 3),
                                 "host_s": self._host_s()})
                    self.get_logger().warn(
                        "approach would exceed range limit; stopping"
                    )
                    break
            self._write({"event": "approach_step", "index": index,
                         "step": step, "phase": "APPROACH",
                         "host_s": self._host_s()})
            ok, _ = self._execute_step(index, step)
            if not ok:
                break
            index += 1
        else:
            self._write({"event": "level_a_time_limit",
                         "host_s": self._host_s()})

    def _run_scan360_approach(
        self,
        target: str,
        spool_root: str,
        score_min: float,
        align_threshold: float,
        align_yaw_max_deg: float,
        reach_area_ratio: float,
    ) -> None:
        env = self._load_detector_env()
        prompt = PROMPT_MAP.get(
            target.strip(), f"{target.strip()}. {target.strip()} object"
        )
        origin = self._odom_snapshot()
        start_yaw = origin[2]
        started = time.monotonic()
        index = 0
        best = None
        stale_aborted = False
        self._write({
            "event": "scan360_start",
            "host_s": self._host_s(),
            "target": target,
            "prompt": prompt,
            "steps": self._scan360_steps,
            "turn_deg": self._scan360_turn_deg,
            "max_radius_m": self._max_radius,
        })
        self.get_logger().info(
            f"360 scan start: target={target} "
            f"{self._scan360_steps} x {self._scan360_turn_deg} deg"
        )

        for step_index in range(self._scan360_steps):
            ok, reason = self._safety_ok()
            if not ok:
                self._write({"event": "abort", "index": index,
                             "reason": reason, "host_s": self._host_s()})
                break
            try:
                image_path, frame_id = self._latest_bundle_image(spool_root)
                objects = self._detect(image_path, prompt, env)
            except (RuntimeError, OSError, ValueError) as exc:
                self._write({"event": "detection_error", "index": index,
                             "error": str(exc), "host_s": self._host_s()})
                if self._is_stale_error(exc):
                    self._write({"event": "scan360_abort", "index": index,
                                 "reason": str(exc),
                                 "host_s": self._host_s()})
                    stale_aborted = True
                    break
                objects = []
            heading_deg = math.degrees(
                self._odom_snapshot()[2] - start_yaw
            )
            heading_deg = (heading_deg + 180.0) % 360.0 - 180.0
            candidates = [
                item for item in objects
                if float(item.get("score", 0.0)) >= score_min
            ]
            self._write({"event": "scan360_heading", "index": index,
                         "step": step_index, "heading_deg": round(heading_deg, 1),
                         "candidates": len(candidates),
                         "host_s": self._host_s()})
            if self._video is not None:
                if candidates:
                    top_candidate = candidates[0]
                    self._video.set_command(
                        f"LLM 命中: {top_candidate.get('label')} "
                        f"{float(top_candidate.get('score', 0.0)):.2f} "
                        f"@ {heading_deg:.0f}°"
                    )
                else:
                    self._video.set_command(
                        f"LLM: 未发现目标 @ {heading_deg:.0f}°"
                    )
            for item in candidates:
                score = float(item.get("score", 0.0))
                bbox = [float(v) for v in item.get("bbox_2d", [0.0, 0.0, 1.0, 1.0])]
                self._write({"event": "scan360_candidate", "index": index,
                             "heading_deg": round(heading_deg, 1),
                             "label": str(item.get("label", "object")),
                             "score": round(score, 3),
                             "host_s": self._host_s()})
                self._feed_detection(
                    str(item.get("label", "object")), score,
                    (bbox[0], bbox[1], bbox[2], bbox[3]),
                )
                if best is None or score > best["score"]:
                    best = {
                        "label": str(item.get("label", "object")),
                        "score": score,
                        "bbox": (bbox[0], bbox[1], bbox[2], bbox[3]),
                        "heading_deg": heading_deg,
                    }
            if best is not None and best["score"] >= 0.80:
                self._write({"event": "scan360_early_hit", "index": index,
                             "heading_deg": round(best["heading_deg"], 1),
                             "label": best["label"],
                             "score": round(best["score"], 3),
                             "host_s": self._host_s()})
                self.get_logger().info(
                    f"high-confidence target {best['label']} "
                    f"score={best['score']:.2f} at "
                    f"{best['heading_deg']:.0f} deg; stopping scan early"
                )
                break
            if step_index < self._scan360_steps - 1:
                step = f"r{int(self._scan360_turn_deg)}"
                ok, _ = self._execute_step(index, step)
                if not ok:
                    break
                index += 1

        if stale_aborted:
            self.get_logger().error(
                "camera went stale during 360 scan; aborting"
            )
            return

        if best is None:
            self._write({"event": "scan360_no_target",
                         "host_s": self._host_s()})
            self.get_logger().warn(
                "no target found during 360 scan; falling back to search"
            )
            self._run_level_a_search(
                target, spool_root, score_min, align_threshold,
                align_yaw_max_deg, reach_area_ratio,
            )
            return

        self._write({"event": "scan360_best", "host_s": self._host_s(),
                     "label": best["label"], "score": round(best["score"], 3),
                     "heading_deg": round(best["heading_deg"], 1)})
        self.get_logger().info(
            f"360 scan best: {best['label']} score={best['score']:.2f} "
            f"at heading {best['heading_deg']:.0f} deg"
        )
        self._feed_detection(best["label"], best["score"], best["bbox"])

        current_heading = math.degrees(self._odom_snapshot()[2] - start_yaw)
        current_heading = (current_heading + 180.0) % 360.0 - 180.0
        turn_needed = best["heading_deg"] - current_heading
        turn_needed = (turn_needed + 180.0) % 360.0 - 180.0
        if abs(turn_needed) > 2.0:
            step = (
                f"l{int(abs(turn_needed))}"
                if turn_needed > 0.0
                else f"r{int(abs(turn_needed))}"
            )
            time.sleep(1.0)
            self._write({"event": "scan360_turn_to_target", "index": index,
                         "step": step, "host_s": self._host_s()})
            ok, _ = self._execute_step(index, step)
            if not ok:
                return
            index += 1

        while time.monotonic() - started < self._max_seconds:
            ok, reason = self._safety_ok()
            if not ok:
                self._write({"event": "abort", "index": index,
                             "reason": reason, "host_s": self._host_s()})
                break
            distance = self._distance_from(origin)
            if self._max_radius > 0.0 and distance > self._max_radius:
                self._write({"event": "range_limit", "index": index,
                             "distance_m": round(distance, 3),
                             "host_s": self._host_s()})
                break
            try:
                image_path, frame_id = self._latest_bundle_image(spool_root)
                objects = self._detect(image_path, prompt, env)
            except (RuntimeError, OSError, ValueError) as exc:
                self._write({"event": "detection_error", "index": index,
                             "error": str(exc), "host_s": self._host_s()})
                if self._is_stale_error(exc):
                    self._write({"event": "abort", "index": index,
                                 "reason": str(exc),
                                 "host_s": self._host_s()})
                    break
                step = "r30"
                ok, _ = self._execute_step(index, step)
                if not ok:
                    break
                index += 1
                continue
            best2 = max(
                (item for item in objects
                 if float(item.get("score", 0.0)) >= score_min),
                key=lambda item: float(item.get("score", 0.0)),
                default=None,
            )
            if best2 is None:
                step = "r30"
                self._write({"event": "search_step", "index": index,
                             "step": step, "phase": "APPROACH_RECOVERY",
                             "host_s": self._host_s()})
                ok, _ = self._execute_step(index, step)
                if not ok:
                    break
                index += 1
                continue
            bbox = [
                float(value)
                for value in best2.get("bbox_2d", [0.0, 0.0, 1.0, 1.0])
            ]
            x1, y1, x2, y2 = bbox
            center_x = (x1 + x2) / 2.0
            area_ratio = max(0.0, (x2 - x1) * (y2 - y1))
            offset = center_x - 0.5
            self._write({"event": "target_found", "index": index,
                         "label": str(best2.get("label", "object")),
                         "score": round(float(best2.get("score", 0.0)), 3),
                         "center_x": round(center_x, 3),
                         "area_ratio": round(area_ratio, 4),
                         "distance_m": round(distance, 3),
                         "host_s": self._host_s()})
            self._feed_detection(
                str(best2.get("label", "object")),
                float(best2.get("score", 0.0)),
                (x1, y1, x2, y2),
            )
            if area_ratio >= reach_area_ratio:
                try:
                    verification = self._verify_target(image_path, bbox, env)
                except (RuntimeError, OSError, ValueError) as exc:
                    verification = {
                        "object_name_zh": "复核失败",
                        "is_target": False,
                        "confidence": 0.0,
                        "reason_zh": str(exc),
                    }
                self._write({"event": "target_verification", "index": index,
                             "label": str(best2.get("label", "object")),
                             "area_ratio": round(area_ratio, 4),
                             "verification": verification,
                             "host_s": self._host_s()})
                if self._video is not None:
                    verdict = "通过" if verification.get("is_target") else "拒绝"
                    self._video.set_command(
                        f"复核: {verification.get('object_name_zh', '?')} {verdict}"
                    )
                if verification.get("is_target"):
                    self._write({"event": "target_reached", "index": index,
                                 "verification": verification,
                                 "host_s": self._host_s()})
                    self.get_logger().info(
                        f"target reached and verified: "
                        f"{verification.get('object_name_zh')} "
                        f"({verification.get('reason_zh')})"
                    )
                    break
                self.get_logger().warn(
                    f"target verification rejected: "
                    f"{verification.get('object_name_zh')} - "
                    f"{verification.get('reason_zh')}"
                )
                step = "r15"
                ok, _ = self._execute_step(index, step)
                if not ok:
                    break
                index += 1
                continue
            if abs(offset) > align_threshold:
                degrees = max(
                    -align_yaw_max_deg,
                    min(align_yaw_max_deg, -offset * align_yaw_max_deg * 2.0),
                )
                step = (
                    f"l{int(abs(degrees))}"
                    if degrees > 0.0
                    else f"r{int(abs(degrees))}"
                )
            else:
                step = "f"
                step_estimate = (
                    self._forward_seconds * self._forward_vx * 0.6
                )
                if (self._max_radius > 0.0
                        and distance + step_estimate >= self._max_radius):
                    self._write({"event": "range_limit", "index": index,
                                 "phase": "APPROACH",
                                 "distance_m": round(distance, 3),
                                 "host_s": self._host_s()})
                    break
            self._write({"event": "approach_step", "index": index,
                         "step": step, "phase": "APPROACH",
                         "host_s": self._host_s()})
            ok, _ = self._execute_step(index, step)
            if not ok:
                break
            index += 1
        else:
            self._write({"event": "scan360_time_limit",
                         "host_s": self._host_s()})

    def _run_state_machine_search(
        self,
        target: str,
        spool_root: str,
        score_min: float,
        align_threshold: float,
        align_yaw_max_deg: float,
        reach_area_ratio: float,
    ) -> None:
        """Run the formal app-layer state machine with the LLM workers."""
        env = self._load_detector_env()
        prompt = PROMPT_MAP.get(
            target.strip(), f"{target.strip()}. {target.strip()} object"
        )
        state: dict[str, object] = {"image_path": None}
        step_index = [0]
        settings = get_settings()
        semantic_enabled = bool(getattr(self, "_semantic_reasoning", False))
        semantic_controller = None
        observation_store = (
            ObservationMemoryStore(settings=settings)
            if settings.live_search_reasoner_use_observation_memory
            else None
        )
        semantic_memory = SemanticSearchMemory(
            default_ttl_sec=settings.live_search_negative_memory_ttl_seconds,
            observation_store=observation_store,
        )
        observation_memory: list[dict] = []
        semantic_observer = None

        if semantic_enabled:
            profile = TargetProfileResolver().resolve(target, use_llm=False)
            semantic_controller = SemanticSearchController(
                profile,
                backend=str(getattr(self, "_search_reasoner", "hybrid")),
                partial_threshold=settings.live_search_graph_match_partial_threshold,
                strong_threshold=settings.live_search_graph_match_strong_threshold,
            )
            observation_memory = semantic_memory.retrieve_long_term(
                profile.canonical_name_zh,
                top_k=settings.observation_memory_retrieval_top_k,
            )
            self._write({
                "event": "semantic_memory_loaded",
                "observation_memory_enabled": bool(observation_store),
                "observation_memory_count": len(observation_memory),
                "observation_memory_ids": [
                    str(item.get("memory_id"))
                    for item in observation_memory
                    if isinstance(item, dict) and item.get("memory_id")
                ],
                "negative_memory_enabled": settings.live_search_negative_memory_enabled,
                "persistent_write_attempted": False,
                "host_s": self._host_s(),
            })

            def analyze_semantic(image_path: object, _profile: object) -> dict:
                if not isinstance(image_path, str):
                    raise RuntimeError("semantic observation has no stable image")
                quick_reuse = semantic_payload_from_quick_target_absence(
                    getattr(self, "_last_llm_detection_payload", None),
                    image_path=image_path,
                    frame_id=str(state.get("frame_id", "semantic_live")),
                )
                if quick_reuse is not None:
                    return quick_reuse
                python = env.get(
                    "SILICONFLOW_PYTHON",
                    env.get(
                        "GROUNDED_SAM_PYTHON",
                        sys.executable,
                    ),
                )
                worker = PROJECT_ROOT / "app/detectors/siliconflow_vision_worker.py"
                output_path = PROJECT_ROOT / "runtime/go2w/llm_semantic_observation.json"
                command = [
                    python, str(worker), "--image", image_path,
                    "--output", str(output_path), "--target", target,
                    "--extra-instructions",
                    "完整列出当前画面的可见物体与关系，供下一视角选择；不要确认目标。",
                    "--model", getattr(self, "_llm_model", ""),
                ]
                completed = subprocess.run(
                    command, cwd=str(PROJECT_ROOT), env=env, text=True,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    timeout=120.0, check=False,
                )
                if completed.returncode != 0:
                    raise RuntimeError(
                        "semantic observer worker failed "
                        f"rc={completed.returncode}: {completed.stderr[-600:]}"
                    )
                payload = json.loads(output_path.read_text(encoding="utf-8"))
                payload.update({
                    "image_path": image_path,
                    "frame_id": str(state.get("frame_id", "semantic_live")),
                    "source": "siliconflow_full_scene_existing_pipeline",
                })
                return payload

            semantic_observer = LiveSemanticObserver(
                analyze_semantic,
                ttl_seconds=settings.live_search_reasoner_scene_ttl_seconds,
            )

        def detect() -> list[Detection]:
            image_path, frame_id = self._latest_bundle_image(spool_root)
            state["image_path"] = image_path
            state["frame_id"] = frame_id
            self._write({"event": "camera_bundle", "frame_id": frame_id,
                         "host_s": self._host_s()})
            objects = self._detect(image_path, prompt, env)
            detections = []
            for item in objects:
                bbox = [
                    float(value)
                    for value in item.get(
                        "bbox_2d", [0.0, 0.0, 1.0, 1.0]
                    )
                ]
                detections.append(
                    Detection(
                        label=str(item.get("label", "object")),
                        score=float(item.get("score", 0.0)),
                        bbox=(bbox[0], bbox[1], bbox[2], bbox[3]),
                    )
                )
            if detections:
                best = max(detections, key=lambda item: item.score)
                self._feed_detection(best.label, best.score, best.bbox)
            return detections

        def verify(bbox: tuple[float, float, float, float]
                   ) -> VerificationResult:
            image_path = state["image_path"]
            if not isinstance(image_path, str):
                raise RuntimeError("no image available for verification")
            result = self._verify_target(image_path, list(bbox), env)
            return VerificationResult(
                object_name_zh=result.get("object_name_zh", ""),
                is_target=bool(result.get("is_target", False)),
                confidence=float(result.get("confidence", 0.0)),
                reason_zh=result.get("reason_zh", ""),
            )

        def execute_step(step: str) -> tuple[bool, str]:
            index = step_index[0]
            step_index[0] += 1
            return self._execute_step(index, step)

        def snapshot() -> SensorSnapshot:
            return SensorSnapshot(
                camera_fresh=True,
                lidar_fresh=self._lidar_fresh is True,
                robot_stationary=True,
                rotation_clearance_valid=self._rotation_clearance_valid is True,
                dual_lidar_clearance_valid=(
                    self._dual_lidar_enabled
                    and self._dual_lidar_fused_state == "clear"
                ),
                pandar_raw_fresh=self._pandar_raw_fresh is True,
            )

        def semantic_observe():
            if semantic_observer is None or semantic_controller is None:
                return None
            x, y, yaw = self._odom_snapshot()
            return semantic_observer.observe(
                target_profile=semantic_controller.target_profile,
                frame_or_bundle=state.get("image_path"),
                robot_pose={
                    "x": x, "y": y, "yaw_rad": yaw,
                    "yaw_deg": math.degrees(yaw),
                },
            )

        def reason_next_view(context):
            if semantic_controller is None:
                raise RuntimeError("semantic controller is disabled")
            context.negative_memory = semantic_memory
            if settings.live_search_negative_memory_enabled:
                semantic_memory.add_negative(
                    target_key=semantic_controller.target_profile.canonical_name_zh,
                    heading_sector=int(round(context.robot_yaw_deg / 30.0)),
                    reason="当前稳定观察未发现目标",
                    source_event_id=f"not_seen_scan_{context.scan_index:04d}",
                    observation_pose=context.robot_pose,
                    confidence=0.65,
                )
            return semantic_controller.propose(context)

        def build_auxiliary_hints(semantic):
            scene_graph = getattr(semantic, "scene_graph", None)
            if scene_graph is None and isinstance(semantic, dict):
                scene_graph = semantic.get("scene_graph")
            psg = build_psg_auxiliary_hints(
                scene_graph,
                enabled=settings.live_search_reasoner_use_psg,
                max_predicted_nodes=settings.video_psg_max_predicted_nodes,
                confidence_threshold=settings.video_psg_confidence_threshold,
            )
            situated = build_precomputed_situated_prior_hints(
                getattr(self, "_last_llm_detection_payload", None),
                enabled=settings.live_search_reasoner_use_llm_situated_prior,
            )
            return {
                "hints": [
                    *list(psg.get("hints") or []),
                    *list(situated.get("hints") or []),
                ],
                "status": {
                    "psg": psg.get("status") or {},
                    "llm_situated_prior": situated.get("status") or {},
                    "priority_contract": (
                        "observed_graph_and_negative_memory_before_auxiliary"
                    ),
                    "duplicate_network_call_started": False,
                },
            }

        config = StepSearchConfig(
            target=target,
            max_seconds=self._max_seconds,
            max_radius_m=self._max_radius,
            score_min=score_min,
            align_threshold=align_threshold,
            align_yaw_max_deg=align_yaw_max_deg,
            reach_area_ratio=reach_area_ratio,
            scan_turn_deg=self._scan_turn_deg,
            scan_span=self._scan_span,
            semantic_reasoning_enabled=semantic_enabled,
            search_reasoner_backend=str(getattr(self, "_search_reasoner", "legacy")),
            search_reasoner_mode=str(getattr(self, "_search_reasoner_mode", "shadow")),
            reasoner_min_confidence=settings.live_search_reasoner_min_confidence,
            reasoner_allow_forward=bool(getattr(self, "_semantic_allow_forward", False)),
            reasoner_max_turn_deg=min(
                30.0, settings.live_search_reasoner_max_turn_deg
            ),
            reasoner_min_replan_seconds=max(
                0.0, settings.live_search_reasoner_min_replan_seconds
            ),
            max_motion_steps=self._max_motion_steps,
        )
        runner = StepSearchRunner(
            config,
            detect=detect,
            verify=verify,
            execute_step=execute_step,
            snapshot=snapshot,
            odometry=self._odom_snapshot,
            reason_next_view=reason_next_view if semantic_enabled else None,
            semantic_observe=semantic_observe if semantic_enabled else None,
            observation_memory=observation_memory,
            negative_memory=semantic_memory if semantic_enabled else None,
            build_auxiliary_hints=(
                build_auxiliary_hints if semantic_enabled else None
            ),
        )
        result = runner.run()
        for event in result["events"]:
            self._write(event)
        self.get_logger().info(
            f"state machine search finished: {result['status']} - "
            f"{result['finish_reason']} "
            f"(steps={result['steps_executed']})"
        )

    @staticmethod
    def _load_detector_env() -> dict[str, str]:
        env_file = PROJECT_ROOT / ".env"
        values: dict[str, str] = {}
        if env_file.is_file():
            for raw_line in env_file.read_text(encoding="utf-8").splitlines():
                line = raw_line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, _, value = line.partition("=")
                values[key.strip()] = value.strip().strip("'\"")
        env = os.environ.copy()
        env["PYTHONPATH"] = values.get(
            "GROUNDED_SAM_PYTHONPATH",
            str(PROJECT_ROOT / "external/Grounded-SAM-2") + ":"
            + str(PROJECT_ROOT / "external/Grounded-SAM-2/grounding_dino"),
        )
        env["GROUNDED_SAM_ROOT"] = values.get(
            "GROUNDED_SAM_ROOT", str(PROJECT_ROOT / "external/Grounded-SAM-2")
        )
        env.setdefault(
            "SILICONFLOW_PYTHON",
            os.environ.get("GO2W_CONDA_PYTHON")
            or str(PROJECT_ROOT / ".venv/bin/python")
            or sys.executable,
        )
        return env

    def _latest_bundle_image(self, spool_root: str,
                             retries: int = 6,
                             retry_delay_seconds: float = 3.0
                             ) -> tuple[str, int]:
        """Return the latest camera bundle, tolerating brief camera stalls.

        The camera RPC stream can hiccup for a few seconds after a robot
        restart. The loop only calls this while the robot is stationary, so
        bounded waiting is safe; if the bundle stays stale beyond the retry
        window the caller aborts instead of acting on an old frame.
        """
        directory = (Path(spool_root) / "latest").resolve()
        last_error: RuntimeError | None = None
        for attempt in range(retries + 1):
            if attempt:
                time.sleep(retry_delay_seconds)
            try:
                ready = directory / "READY"
                if not ready.is_file():
                    raise RuntimeError("no READY bundle available")
                payload = json.loads(
                    (directory / "frame_bundle.json").read_text(
                        encoding="utf-8"
                    )
                )
                image_path = directory / str(payload["image_path"])
                if not image_path.is_file():
                    raise RuntimeError("bundle image missing")
                receive_ns = payload.get("image_receive_time_ns")
                if isinstance(receive_ns, (int, float)) and receive_ns > 0:
                    age_seconds = (
                        time.time_ns() - int(receive_ns)
                    ) / 1.0e9
                else:
                    age_seconds = time.time() - os.path.getmtime(image_path)
                if age_seconds > 5.0:
                    raise RuntimeError(
                        f"camera bundle stale (age={age_seconds:.1f}s); "
                        "refusing to act on an old frame"
                    )
                return str(image_path), int(payload.get("frame_id", -1))
            except RuntimeError as exc:
                last_error = exc
        if last_error is None:
            last_error = RuntimeError("camera bundle unavailable")
        raise last_error

    @staticmethod
    def _is_stale_error(exc: Exception) -> bool:
        return "stale" in str(exc).lower()

    def _detect(self, image_path: str, prompt: str, env: dict[str, str]) -> list[dict]:
        if getattr(self, "_detector", "grounded_sam") == "llm":
            return self._detect_llm(image_path, env)
        root = env["GROUNDED_SAM_ROOT"]
        python = env.get(
            "GROUNDED_SAM_PYTHON",
            sys.executable,
        )
        worker = PROJECT_ROOT / "app/detectors/grounded_sam_worker.py"
        command = [
            python,
            str(worker),
            "--image", image_path,
            "--output", str(PROJECT_ROOT / "runtime/go2w/detection_result.json"),
            "--root", root,
            "--text-prompt", prompt,
            "--grounding-config",
            env.get("GROUNDING_DINO_CONFIG",
                    "grounding_dino/groundingdino/config/"
                    "GroundingDINO_SwinT_OGC_local.py"),
            "--grounding-checkpoint",
            env.get("GROUNDING_DINO_CHECKPOINT",
                    "gdino_checkpoints/groundingdino_swint_ogc.pth"),
            "--box-threshold",
            env.get("GROUNDING_DINO_BOX_THRESHOLD", "0.12"),
            "--text-threshold",
            env.get("GROUNDING_DINO_TEXT_THRESHOLD", "0.10"),
            "--sam2-config", "configs/sam2.1/sam2.1_hiera_t.yaml",
            "--sam2-checkpoint", "checkpoints/sam2.1_hiera_tiny.pt",
            "--max-objects", "20",
            "--device", "auto",
            "--disable-sam2",
        ]
        try:
            completed = subprocess.run(
                command,
                cwd=root,
                env=env,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=150.0,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError("detector timed out") from exc
        if completed.returncode != 0:
            raise RuntimeError(
                f"detector failed rc={completed.returncode}: "
                f"{completed.stderr[-400:]}"
            )
        output_path = PROJECT_ROOT / "runtime/go2w/detection_result.json"
        payload = json.loads(output_path.read_text(encoding="utf-8"))
        return list(payload.get("objects", []))

    @staticmethod
    def _daemon_vlm_request(
        mode: str,
        image_path: str,
        target: str,
        *,
        bbox: list[float] | None = None,
        extra_instructions: str = "",
        model: str = "",
        frame_id: str = "",
        timeout: float = 120.0,
    ) -> dict | None:
        """Try the long-running VLM daemon; return None if unavailable/failed."""
        try:
            from app.detectors.siliconflow_vision_protocol import (
                SiliconFlowDaemonClient,
                VLMRequest,
            )

            socket_path = PROJECT_ROOT / "runtime/go2w/siliconflow_vlm.sock"
            client = SiliconFlowDaemonClient(str(socket_path), timeout=timeout)
            if not client.available():
                return None
            request = VLMRequest(
                request_id=f"{time.time_ns()}_{os.getpid()}",
                mode=mode,
                image_path=image_path,
                frame_id=frame_id,
                target=target,
                bbox=bbox,
                priority=(
                    "background" if mode == "semantic" else "realtime"
                ),
                extra_instructions=extra_instructions,
                model=model,
            )
            response = client.request(request)
            if not response.ok:
                return None
            return response.payload
        except Exception:
            return None

    def _detect_llm(self, image_path: str, env: dict[str, str]) -> list[dict]:
        daemon_payload = self._daemon_vlm_request(
            "quick",
            image_path,
            self._target,
            model=getattr(self, "_llm_model", ""),
            frame_id=str(getattr(self, "_latest_frame_id", "")),
            timeout=20.0,
        )
        if daemon_payload is not None:
            self._last_llm_detection_payload = daemon_payload
            return list(daemon_payload.get("objects", []))
        python = env.get(
            "SILICONFLOW_PYTHON",
            env.get(
                "GROUNDED_SAM_PYTHON",
                sys.executable,
            ),
        )
        worker = PROJECT_ROOT / "app/detectors/siliconflow_vision_worker.py"
        output_path = PROJECT_ROOT / "runtime/go2w/llm_detection_result.json"
        command = [
            python,
            str(worker),
            "--image", image_path,
            "--output", str(output_path),
            "--target", self._target,
            "--quick",
            "--model", getattr(self, "_llm_model", ""),
        ]
        try:
            completed = subprocess.run(
                command,
                cwd=str(PROJECT_ROOT),
                env=env,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=float(get_settings().vlm_runtime_quick_timeout_seconds),
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError("SiliconFlow vision API timed out") from exc
        if completed.returncode != 0:
            raise RuntimeError(
                f"SiliconFlow vision worker failed rc={completed.returncode}: "
                f"{completed.stderr[-600:]}"
            )
        payload = json.loads(output_path.read_text(encoding="utf-8"))
        self._last_llm_detection_payload = payload
        scene_summary = str(payload.get("scene_summary_zh") or "")
        if scene_summary:
            self.get_logger().info(
                f"LLM scene: {scene_summary} "
                f"(matched={len(payload.get('objects', []))}/"
                f"all={payload.get('all_objects_count', 0)})"
            )
        return list(payload.get("objects", []))

    def _verify_target(self, image_path: str, bbox: list[float],
                       env: dict[str, str]) -> dict:
        """Ask the vision LLM whether the object inside bbox is the target."""
        daemon_payload = self._daemon_vlm_request(
            "verify",
            image_path,
            self._target,
            bbox=list(bbox),
            model=getattr(self, "_llm_model", ""),
            frame_id=str(getattr(self, "_latest_frame_id", "")),
            timeout=20.0,
        )
        if daemon_payload is not None:
            return {
                "object_name_zh": str(daemon_payload.get("object_name_zh") or ""),
                "is_target": bool(daemon_payload.get("is_target", False)),
                "confidence": float(daemon_payload.get("confidence", 0.0)),
                "reason_zh": str(daemon_payload.get("reason_zh") or ""),
            }
        python = env.get(
            "SILICONFLOW_PYTHON",
            env.get(
                "GROUNDED_SAM_PYTHON",
                sys.executable,
            ),
        )
        worker = PROJECT_ROOT / "app/detectors/siliconflow_vision_worker.py"
        output_path = PROJECT_ROOT / "runtime/go2w/llm_verify_result.json"
        bbox_text = ",".join(f"{float(value):.4f}" for value in bbox)
        command = [
            python,
            str(worker),
            "--image", image_path,
            "--output", str(output_path),
            "--target", self._target,
            "--verify",
            "--bbox", bbox_text,
            "--model", getattr(self, "_llm_model", ""),
        ]
        try:
            completed = subprocess.run(
                command,
                cwd=str(PROJECT_ROOT),
                env=env,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=float(get_settings().vlm_runtime_verify_timeout_seconds),
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError("SiliconFlow verification timed out") from exc
        if completed.returncode != 0:
            raise RuntimeError(
                f"SiliconFlow verification worker failed rc={completed.returncode}: "
                f"{completed.stderr[-600:]}"
            )
        payload = json.loads(output_path.read_text(encoding="utf-8"))
        return {
            "object_name_zh": str(payload.get("object_name_zh") or ""),
            "is_target": bool(payload.get("is_target", False)),
            "confidence": float(payload.get("confidence", 0.0)),
            "reason_zh": str(payload.get("reason_zh") or ""),
        }

    def run(self) -> int:
        if not self._wait_for(lambda: self._sport is not None and self._odom is not None,
                              10.0, "sport/odom"):
            return 2
        if not self._wait_for(
            lambda: self._lidar_fresh is True and self._clearance is not None,
            5.0,
            "fresh LiDAR clearance",
        ):
            return 2
        if (
            self._mode == "state_machine_search"
            and self._turn_only
            and not self._wait_for(
                lambda: self._rotation_clearance_valid is not None,
                3.0,
                "rotation-clearance validity",
            )
        ):
            if self._operator_authorized_rotation:
                self._write({
                    "event": "rotation_clearance_wait_bypassed",
                    "reason": "operator_authorized_rotation",
                    "host_s": self._host_s(),
                })
                self.get_logger().warning(
                    "operator-authorized rotation: skipping rotation-clearance "
                    "validity wait"
                )
            else:
                return 2
        if self._rotation_lease_error:
            if self._operator_authorized_rotation:
                self._write({
                    "event": "rotation_lease_error_bypassed",
                    "reason": self._rotation_lease_error,
                    "host_s": self._host_s(),
                })
                self.get_logger().warning(
                    f"operator-authorized rotation overrides lease error: "
                    f"{self._rotation_lease_error}"
                )
            else:
                self._write({
                    "event": "rotation_lease_reject",
                    "reason": self._rotation_lease_error,
                    "host_s": self._host_s(),
                })
                self.get_logger().error(self._rotation_lease_error)
                return 2
        if self._rotation_lease is not None and not self._wait_for(
            lambda: self._diagnostic_clearance_receive_s is not None,
            3.0,
            "raw diagnostic rotation clearance",
        ):
            if not self._operator_authorized_rotation:
                return 2
        self._motion_origin = self._odom_snapshot()
        if self._rotation_lease is not None:
            lease = evaluate_rotation_lease(
                self._rotation_lease,
                current_pose=self._motion_origin,
                current_frame=(self._odom.header.frame_id if self._odom is not None else ""),
                now=datetime.now(timezone.utc),
                expected_binding=self._hardware_binding,
            )
            if not lease.allowed:
                self._write({
                    "event": "rotation_lease_reject",
                    "reason": lease.reason,
                    "motion_origin": list(self._motion_origin),
                    "host_s": self._host_s(),
                })
                if self._operator_authorized_rotation:
                    self.get_logger().warning(
                        f"operator-authorized rotation overrides lease invalidity: "
                        f"{lease.reason}"
                    )
                else:
                    self.get_logger().error(lease.reason)
                    return 2
        if self._record_video:
            try:
                self._video = BundleVideoRecorder(
                    self._record_video,
                    self._video_fps, self._video_scale,
                )
                self._video.start()
                self.get_logger().info(
                    f"recording camera to {self._record_video}"
                )
            except RuntimeError as exc:
                self.get_logger().warn(f"video recording disabled: {exc}")
        self._write({
            "event": "start",
            "host_s": self._host_s(),
            "pattern": self._pattern,
            "mode": self._mode,
            "motion_origin": list(self._motion_origin),
            "max_radius_m": self._max_radius,
            "front_half_plane_only": self._front_half_plane_only,
            "turn_only": self._turn_only,
            "max_motion_steps": self._max_motion_steps,
            "min_rotation_clearance_m": self._min_rotation_clearance_m,
            "rotation_lease_path": self._rotation_lease_path or None,
            "rotation_lease_active": self._rotation_lease is not None,
            "forward_vx": self._forward_vx,
            "forward_duration_scale": self._forward_duration_scale,
            "backward_vx": self._backward_vx,
            "backward_duration_scale": self._backward_duration_scale,
        })
        ready, reason = self._safety_ok()
        if not ready:
            self._write({
                "event": "pre_arm_safety_reject",
                "reason": reason,
                "host_s": self._host_s(),
            })
            self.get_logger().error(f"pre-arm safety rejected: {reason}")
            return 2
        if self._mode == "state_machine_search":
            # Detection and semantic reasoning can be slow, and each actual
            # step already performs boundary, rotation-clearance and fresh
            # LiDAR checks before idempotently arming.  Do not touch the
            # control service when a proposed step will be rejected by those
            # gates (the current rotation-unvalidated deployment is one such
            # case).
            self._write({
                "event": "initial_arm_deferred",
                "reason": "state_machine_steps_arm_only_after_motion_gates",
                "host_s": self._host_s(),
            })
        else:
            try:
                self._arm(True)
            except RuntimeError as exc:
                self.get_logger().error(str(exc))
                return 3

        if self._mode == "wander":
            started = time.monotonic()
            index = 0
            while time.monotonic() - started < self._max_seconds:
                ok, reason = self._safety_ok()
                if not ok:
                    self._write({"event": "abort", "index": index,
                                 "reason": reason, "host_s": self._host_s()})
                    break
                step = self._next_wander_step()
                ok, reason = self._execute_step(index, step)
                if not ok:
                    if step == "f" and "did not confirm motion" in reason:
                        left = (self._left_clearance
                                if self._left_clearance is not None else 0.0)
                        right = (self._right_clearance
                                 if self._right_clearance is not None else 0.0)
                        turn = "l90" if left >= right else "r90"
                        self._write({"event": "forward_blocked",
                                     "index": index, "turn": turn,
                                     "host_s": self._host_s(),
                                     "left": left, "right": right})
                        self.get_logger().warn(
                            f"forward blocked; turning {turn} to find a path"
                        )
                        ok, reason = self._execute_step(index, turn)
                        if not ok:
                            self._write({"event": "abort", "index": index,
                                         "reason": reason,
                                         "host_s": self._host_s()})
                            break
                    else:
                        break
                index += 1
            else:
                self._write({"event": "wander_time_limit",
                             "host_s": self._host_s()})
        elif self._mode == "camera_guided":
            self._run_camera_guided(
                self._target,
                self._spool_root,
                self._target_score_min,
                self._align_threshold,
                self._align_yaw_max_deg,
                self._reach_area_ratio,
            )
        elif self._mode == "level_a_search":
            self._run_level_a_search(
                self._target,
                self._spool_root,
                self._target_score_min,
                self._align_threshold,
                self._align_yaw_max_deg,
                self._reach_area_ratio,
            )
        elif self._mode == "scan360_approach":
            self._run_scan360_approach(
                self._target,
                self._spool_root,
                self._target_score_min,
                self._align_threshold,
                self._align_yaw_max_deg,
                self._reach_area_ratio,
            )
        elif self._mode == "state_machine_search":
            self._run_state_machine_search(
                self._target,
                self._spool_root,
                self._target_score_min,
                self._align_threshold,
                self._align_yaw_max_deg,
                self._reach_area_ratio,
            )
        else:
            for index, step in enumerate(self._pattern):
                ok, reason = self._execute_step(index, step)
                if not ok:
                    break
            else:
                self._write({"event": "pattern_complete",
                             "host_s": self._host_s()})

        if self._armed_by_runner:
            self._emergency_stop()
            try:
                self._arm(False)
            except RuntimeError as exc:
                self.get_logger().error(str(exc))
        else:
            self._write({
                "event": "control_cleanup_skipped",
                "reason": "runner_never_armed_or_sent_motion",
                "host_s": self._host_s(),
            })
        end = self._odom_snapshot()
        self._write({"event": "finish", "host_s": self._host_s(),
                     "odom": list(end), "clearance": self._clearance})
        if self._video is not None:
            self._video.stop()
            self._video = None
        self.get_logger().info(
            f"finished at ({end[0]:.3f}, {end[1]:.3f}) yaw {math.degrees(end[2]):.1f} deg"
        )
        self._output.close()
        return 0


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pattern", default=",".join(DEFAULT_PATTERN),
                        help="comma-separated steps: f or l<deg>/r<deg>")
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--odom-topic",
        default="/go2w/odom/wheel",
        help="odometry topic used for step verification "
             "(e.g. /go2w/odom/fused)",
    )
    parser.add_argument("--forward-vx", type=float, default=0.12)
    parser.add_argument("--forward-seconds", type=float, default=2.0)
    parser.add_argument(
        "--forward-duration-scale",
        type=float,
        default=1.0,
        help="calibrated duration multiplier for distance-qualified forward steps",
    )
    parser.add_argument(
        "--backward-vx",
        type=float,
        default=0.16,
        help="reverse primitive speed; values below 0.12 m/s are clamped to "
             "the measured ai-w reverse breakaway floor",
    )
    parser.add_argument(
        "--backward-duration-scale",
        type=float,
        default=1.40,
        help="calibrated duration multiplier for distance-qualified reverse steps",
    )
    parser.add_argument("--max-yaw-rate", type=float, default=0.15)
    parser.add_argument("--min-clearance", type=float, default=0.30)
    parser.add_argument(
        "--mode",
        choices=("pattern", "wander", "camera_guided", "level_a_search",
                 "scan360_approach", "state_machine_search"),
        default="pattern",
    )
    parser.add_argument("--max-seconds", type=float, default=120.0)
    parser.add_argument(
        "--detector",
        choices=("llm", "grounded_sam"),
        default="llm",
        help="detection backend: llm uses the SiliconFlow vision API "
             "(default); grounded_sam uses local GroundingDINO",
    )
    parser.add_argument(
        "--llm-model",
        default="Qwen/Qwen3-VL-30B-A3B-Instruct",
        help="SiliconFlow vision model for quick robot-loop detection "
             "(fast default; use Qwen/Qwen3-VL-8B-Instruct for higher detail)",
    )
    parser.add_argument("--wander-front-go", type=float, default=0.45)
    parser.add_argument("--wander-turn-deg", type=float, default=30.0)
    parser.add_argument("--target", default="手机")
    parser.add_argument("--spool-root", default="runtime/go2w/spool")
    parser.add_argument("--target-score-min", type=float, default=0.15)
    parser.add_argument("--align-threshold", type=float, default=0.08)
    parser.add_argument("--align-yaw-max-deg", type=float, default=25.0)
    parser.add_argument("--reach-area-ratio", type=float, default=0.15)
    parser.add_argument("--max-radius", type=float, default=1.5,
                        help="search/approach radius limit in metres; "
                             "0 disables the limit (free exploration)")
    parser.add_argument(
        "--front-half-plane-only",
        action="store_true",
        help="keep translation in the half-plane ahead of the initial pose",
    )
    parser.add_argument(
        "--turn-only",
        action="store_true",
        help="reject every forward command at the final motion executor",
    )
    parser.add_argument(
        "--max-motion-steps",
        type=int,
        default=0,
        help="stop state-machine search after this many successful motion steps; 0 is unlimited",
    )
    parser.add_argument(
        "--min-rotation-clearance",
        type=float,
        default=0.0,
        help="require both live side-clearance topics to meet this full-body rotation envelope",
    )
    parser.add_argument(
        "--rotation-clearance-evidence",
        default="",
        help="short-lived initial-pose physical cross-check JSON; enables "
             "raw side clearance only while the lease remains valid",
    )
    parser.add_argument("--scan-turn-deg", type=float, default=30.0)
    parser.add_argument("--scan-span", type=int, default=3)
    parser.add_argument(
        "--pre-scan-turns",
        type=int,
        default=0,
        help="blind scan turns before the first detection (e.g. 3 turns "
             "rotate the target out of view to demo finding an unseen object)",
    )
    parser.add_argument(
        "--record-video",
        default="",
        help="record the camera stream with locked target overlay to this "
             "MP4 path; empty disables recording",
    )
    parser.add_argument("--video-fps", type=float, default=15.0)
    parser.add_argument("--video-scale", type=float, default=0.4)
    parser.add_argument("--scan360-steps", type=int, default=8)
    parser.add_argument("--scan360-turn-deg", type=float, default=45.0)
    parser.add_argument(
        "--semantic-reasoning", action="store_true",
        help="enable event-driven SemanticNavigation-style semantic next-view reasoning",
    )
    parser.add_argument(
        "--search-reasoner", choices=("legacy", "semantic_navigation", "hybrid"),
        default="legacy",
    )
    parser.add_argument(
        "--search-reasoner-mode", choices=("shadow", "active"),
        default="shadow",
    )
    parser.add_argument(
        "--semantic-no-forward", action="store_true",
        help="force semantic forward requests off (forward is already off by default)",
    )
    parser.add_argument(
        "--semantic-allow-forward", action="store_true",
        help="allow semantic short-forward requests through all existing safety gates",
    )
    parser.add_argument(
        "--dual-lidar-safety-config",
        default="configs/go2w/dual_lidar_safety.yaml",
        help="dual-LiDAR safety policy; enabled=true is fail-closed until evidence is provided",
    )
    parser.add_argument(
        "--dual-lidar-evidence",
        default="",
        help="fused dual-LiDAR evidence JSON with a fused_state field; "
             "required when dual-lidar safety is enabled",
    )
    parser.add_argument(
        "--hardware-geometry-config",
        default="configs/go2w/current_hardware_geometry.yaml",
        help="current whole-machine geometry config (0.70 x 0.43 x 0.70 m)",
    )
    parser.add_argument(
        "--hardware-state-config",
        default="configs/go2w/current_hardware_state.yaml",
        help="current hardware state manifest",
    )
    parser.add_argument(
        "--stage2-readiness",
        default="",
        help="write a machine-readable Stage-2 readiness report to this JSON path "
             "and exit; no motion is attempted",
    )
    parser.add_argument(
        "--operator-authorized-rotation",
        action="store_true",
        help="EXPLICIT operator authorization to allow in-place turns WITHOUT a "
             "four-direction pose-bound rotation lease. The operator must confirm "
             "the swept envelope is clear and hold the remote emergency stop. All "
             "other safety gates (mode/error, lidar fresh, front clearance, motion "
             "bounds, turn<=30deg, single step, emergency stop) remain active. "
             "Every motion event records operator_authorized_rotation=true.",
    )
    args = parser.parse_args()
    pattern = [item for item in args.pattern.split(",") if item]
    rclpy.init()
    node = AutonomousLoop(pattern, args.output, args.forward_vx,
                          args.forward_seconds, args.max_yaw_rate,
                          args.min_clearance, args.mode, args.max_seconds,
                          args.wander_front_go, args.wander_turn_deg,
                          args.max_radius, args.scan_turn_deg, args.scan_span,
                          args.pre_scan_turns, args.record_video,
                          args.video_fps, args.video_scale,
                          args.scan360_steps, args.scan360_turn_deg,
                          args.odom_topic)
    node._target = args.target
    node._forward_duration_scale = max(
        0.5, min(2.0, float(args.forward_duration_scale))
    )
    node._backward_vx = max(0.12, min(0.16, abs(float(args.backward_vx))))
    node._backward_duration_scale = max(
        0.5, min(2.0, float(args.backward_duration_scale))
    )
    node._detector = args.detector
    node._llm_model = args.llm_model or get_settings().vision_model
    node._spool_root = args.spool_root
    node._target_score_min = args.target_score_min
    node._align_threshold = args.align_threshold
    node._align_yaw_max_deg = args.align_yaw_max_deg
    node._reach_area_ratio = args.reach_area_ratio
    node._semantic_reasoning = args.semantic_reasoning
    node._search_reasoner = args.search_reasoner
    node._search_reasoner_mode = args.search_reasoner_mode
    node._semantic_allow_forward = bool(
        args.semantic_allow_forward and not args.semantic_no_forward
    )
    node._front_half_plane_only = bool(args.front_half_plane_only)
    node._turn_only = bool(args.turn_only)
    node._max_motion_steps = max(0, int(args.max_motion_steps))
    node._min_rotation_clearance_m = max(0.0, float(args.min_rotation_clearance))
    # ---- Dual-LiDAR safety gate (fail-closed) ----------------------------
    hardware_binding: dict | None = None
    dual_lidar_config = None
    node._dual_lidar_enabled = False
    node._dual_lidar_unknown_is_clear = False
    node._dual_lidar_fused_state = None
    node._dual_lidar_occupied_sources = []
    try:
        if Path(args.dual_lidar_safety_config).is_file():
            sys.path.insert(
                0, str(PROJECT_ROOT / "ros2_ws" / "src" / "go2w_lidar_preprocessor")
            )
            from go2w_lidar_preprocessor.dual_lidar_config import (
                load_dual_lidar_safety_config,
            )

            dual_lidar_config = load_dual_lidar_safety_config(args.dual_lidar_safety_config)
            node._dual_lidar_enabled = bool(dual_lidar_config.get("enabled", False))
            node._dual_lidar_unknown_is_clear = bool(
                dual_lidar_config.get("unknown_is_clear", False)
            )
            if node._dual_lidar_enabled:
                if not args.dual_lidar_evidence:
                    raise ValueError(
                        "--dual-lidar-evidence is required when dual-lidar safety is enabled"
                    )
                evidence = json.loads(
                    Path(args.dual_lidar_evidence).read_text(encoding="utf-8")
                )
                node._dual_lidar_fused_state = str(evidence.get("fused_state") or "unknown")
                node._dual_lidar_occupied_sources = list(
                    evidence.get("occupied_sources") or []
                )
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        node._rotation_lease_error = f"dual-lidar safety config rejected: {exc}"

    # ---- Current hardware binding for the pose-bound lease ---------------
    try:
        geometry = load_current_hardware_geometry(args.hardware_geometry_config)
        state = load_current_hardware_state(args.hardware_state_config)
        hardware_binding = build_rotation_lease_binding(
            hardware_state_hash=state_hash(state),
            geometry_hash=geometry_hash(geometry),
            extrinsic_version="hesai_pandarxt16_extrinsics_20260813_unconfirmed",
            clock_tier=DEFAULT_PANDAR_CLOCK_TIER.value,
        )
        node._hardware_geometry_hash = geometry_hash(geometry)
        node._hardware_state_hash = state_hash(state)
        node._hardware_binding = hardware_binding
    except (OSError, ValueError, KeyError, TypeError) as exc:
        node._rotation_lease_error = f"current hardware binding rejected: {exc}"

    node._operator_authorized_rotation = bool(args.operator_authorized_rotation)
    if node._operator_authorized_rotation:
        node._write({
            "event": "operator_authorized_rotation_declared",
            "host_s": node._host_s(),
            "scope": "turn steps allowed without four-direction lease",
            "remaining_gates": [
                "sport_mode_error",
                "lidar_fresh",
                "front_clearance",
                "motion_bounds",
                "turn_le_30deg",
                "single_step",
                "emergency_stop",
            ],
        })
        node.get_logger().warning(
            "operator-authorized rotation: turns allowed without a rotation lease; "
            "all other safety gates remain active and every motion is recorded"
        )
    node._rotation_lease_path = str(args.rotation_clearance_evidence or "")
    if node._rotation_lease_path:
        lease_scope_errors = rotation_lease_stage2_scope_errors(
            mode=args.mode,
            semantic_reasoning=args.semantic_reasoning,
            search_reasoner=args.search_reasoner,
            search_reasoner_mode=args.search_reasoner_mode,
            turn_only=args.turn_only,
            front_half_plane_only=args.front_half_plane_only,
            max_motion_steps=args.max_motion_steps,
            max_radius_m=args.max_radius,
            semantic_allow_forward=args.semantic_allow_forward,
        )
        if lease_scope_errors:
            node._rotation_lease_error = (
                "pose-bound rotation evidence Stage-2 scope rejected: "
                + "; ".join(lease_scope_errors)
            )
        elif args.odom_topic != "/go2w/odom/wheel":
            node._rotation_lease_error = (
                "pose-bound rotation evidence requires "
                "--odom-topic /go2w/odom/wheel"
            )
        elif node._min_rotation_clearance_m + 1e-9 < GO2W_ROTATION_ENVELOPE_RADIUS_M:
            node._rotation_lease_error = (
                "pose-bound rotation evidence requires "
                f"--min-rotation-clearance >= {GO2W_ROTATION_ENVELOPE_RADIUS_M:.3f}"
            )
        else:
            try:
                node._rotation_lease = load_rotation_lease(
                    node._rotation_lease_path,
                    required_envelope_radius_m=GO2W_ROTATION_ENVELOPE_RADIUS_M,
                    project_root=PROJECT_ROOT,
                    expected_binding=hardware_binding,
                )
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                node._rotation_lease_error = f"rotation evidence rejected: {exc}"

    # ---- Machine-readable Stage-2 readiness (no motion attempted) ---------
    if args.stage2_readiness:
        readiness = compute_stage2_readiness(
            semantic_v1_ready=True,
            pandar_raw_ready=True,
            pandar_preprocess_ready=True,
            pandar_extrinsics_validated=False,
            current_hardware_geometry_loaded=hardware_binding is not None,
            dual_lidar_rotation_observability_valid=False,
            current_hardware_four_direction_evidence_valid=False,
            pose_bound_rotation_lease_valid=node._rotation_lease is not None
            and not node._rotation_lease_error,
            odom_fresh=True,
            mode_ok=True,
            motion_action_available=False,
            no_stage2_error=not bool(node._rotation_lease_error),
            reasons={
                "stage2": (
                    "Active turn-only is BLOCKED until the Pandar extrinsic, "
                    "dual-lidar observability, four-direction evidence and a "
                    "pose-bound lease all pass on the current hardware"
                ),
                "pandar_extrinsics_validated": "multi-scene extrinsic calibration pending",
                "dual_lidar_rotation_observability_valid": "requires validated extrinsics + self-occlusion",
                "current_hardware_four_direction_evidence_valid": "empty baseline + front/right/rear/left pending",
                "pose_bound_rotation_lease_valid": node._rotation_lease_error or "no current lease",
                "motion_action_available": "no /go2w/motion Action server on the current rig",
            },
        )
        readiness_path = Path(args.stage2_readiness)
        readiness_path.parent.mkdir(parents=True, exist_ok=True)
        readiness_path.write_text(
            json.dumps(readiness.to_dict(), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(readiness.to_dict(), ensure_ascii=False, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return
    try:
        code = node.run()
    except Exception as exc:
        node.get_logger().error(f"unhandled runner exception: {exc}")
        if node._armed_by_runner:
            try:
                node._emergency_stop()
            except Exception as stop_exc:
                node.get_logger().error(
                    f"emergency stop during cleanup failed: {stop_exc}"
                )
            try:
                node._arm(False)
            except Exception as disarm_exc:
                node.get_logger().error(
                    f"disarm during cleanup failed: {disarm_exc}"
                )
        try:
            node._output.close()
        except Exception:
            pass
        code = 4
    finally:
        # Foxy ActionClient is not owned by Node.destroy_node(); explicitly
        # destroy it before the node handle to avoid a late InvalidHandle
        # destructor and to release all Action graph entities deterministically.
        try:
            node._client.destroy()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
