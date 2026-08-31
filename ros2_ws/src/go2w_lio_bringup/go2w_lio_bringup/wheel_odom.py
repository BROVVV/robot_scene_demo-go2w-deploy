"""Wheel-encoder odometry with optional Sport+LIO heading fusion.

The Go2-W's four wheel encoders provide a monotonic, repeatable forward
translation signal, and the robot's own Sport yaw (``/lf/sportmodestate``) is
the validated heading source. Point-LIO's *yaw* is also usable while its
*translation* is not (straight-line LIO remains BLOCKED as of 2026-08-07).

This node publishes:

* ``/go2w/odom/wheel`` -- unchanged pure wheel + Sport-yaw odometry;
* ``/go2w/odom/fused`` -- wheel translation integrated along a fused heading
  (Sport yaw blended with fresh, sane LIO yaw); falls back to Sport yaw when
  LIO is stale, diverged, or inconsistent.

The fusion is deliberately conservative: LIO yaw is only used when its age,
position magnitude and per-tick delta agree with Sport within configured
bounds. Any violation disables LIO for the next tick, and persistent
inconsistency is logged once.
"""

from __future__ import annotations

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster
from unitree_go.msg import LowState, SportModeState


def unwrap_yaw(raw: float, previous: float | None, accumulator: float) -> float:
    if previous is None:
        return accumulator
    delta = raw - previous
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return accumulator + delta


def step_odometry(
    x: float,
    y: float,
    yaw: float,
    wheel_q_deltas: list[float],
    radius: float,
) -> tuple[float, float]:
    """Integrate one LowState tick of equal-wheel motion along the heading."""
    mean_delta = sum(wheel_q_deltas) / len(wheel_q_deltas)
    distance = mean_delta * radius
    return x + distance * math.cos(yaw), y + distance * math.sin(yaw)


def fuse_yaw_delta(
    sport_delta: float, lio_delta: float, lio_weight: float
) -> float:
    """Blend two unwrapped yaw deltas; weight is clamped to [0, 1]."""
    weight = max(0.0, min(1.0, float(lio_weight)))
    return sport_delta + weight * (lio_delta - sport_delta)


def heading_delta_sane(
    sport_delta: float, lio_delta: float, max_diff_rad: float
) -> bool:
    """Return True when the LIO and Sport yaw deltas agree closely."""
    diff = abs(lio_delta - sport_delta)
    if diff > math.pi:
        diff = abs(diff - 2.0 * math.pi)
    return diff <= max_diff_rad


class WheelOdomNode(Node):
    def __init__(self) -> None:
        super().__init__("go2w_wheel_odom")
        self.declare_parameter("wheel_radius_m", 0.089)
        self.declare_parameter("wheel_start_index", 12)
        self.declare_parameter("wheel_count", 4)
        self.declare_parameter("low_topic", "/lf/lowstate")
        self.declare_parameter("sport_topic", "/lf/sportmodestate")
        self.declare_parameter("odom_topic", "/go2w/odom/wheel")
        self.declare_parameter("frame_id", "odom_wheel")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("turn_yaw_rate_threshold_radps", 0.10)
        self.declare_parameter("skip_translation_during_turns", True)
        self.declare_parameter("lio_enabled", True)
        self.declare_parameter("lio_odom_topic", "/lio/odom")
        self.declare_parameter("lio_yaw_weight", 0.35)
        self.declare_parameter("lio_max_age_sec", 0.5)
        self.declare_parameter("lio_max_position_m", 5.0)
        self.declare_parameter("lio_max_yaw_step_rad", 0.05)
        self.declare_parameter("fused_odom_topic", "/go2w/odom/fused")
        self.declare_parameter("fused_frame_id", "odom_fused")

        self._radius = float(self.get_parameter("wheel_radius_m").value)
        self._start = int(self.get_parameter("wheel_start_index").value)
        self._count = int(self.get_parameter("wheel_count").value)
        low_topic = str(self.get_parameter("low_topic").value)
        sport_topic = str(self.get_parameter("sport_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        self._frame = str(self.get_parameter("frame_id").value)
        self._child = str(self.get_parameter("child_frame_id").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        self._turn_yaw_threshold = float(
            self.get_parameter("turn_yaw_rate_threshold_radps").value
        )
        self._skip_turn_translation = bool(
            self.get_parameter("skip_translation_during_turns").value
        )
        self._lio_enabled = bool(self.get_parameter("lio_enabled").value)
        lio_topic = str(self.get_parameter("lio_odom_topic").value)
        self._lio_weight = float(
            self.get_parameter("lio_yaw_weight").value
        )
        self._lio_max_age = float(
            self.get_parameter("lio_max_age_sec").value
        )
        self._lio_max_position = float(
            self.get_parameter("lio_max_position_m").value
        )
        self._lio_max_yaw_step = float(
            self.get_parameter("lio_max_yaw_step_rad").value
        )
        fused_topic = str(self.get_parameter("fused_odom_topic").value)
        self._fused_frame = str(
            self.get_parameter("fused_frame_id").value
        )
        if self._radius <= 0.0 or self._radius > 0.3:
            raise ValueError("wheel radius must be in (0, 0.3] m")
        if self._start < 0 or self._count <= 0 or self._start + self._count > 20:
            raise ValueError("wheel index range must fit LowState motor_state")
        if not 0.0 <= self._lio_weight <= 1.0:
            raise ValueError("lio_yaw_weight must be in [0, 1]")

        # Pure wheel state (Sport yaw only).
        self._wheel_x = 0.0
        self._wheel_y = 0.0
        # Fused state (Sport + LIO yaw blend).
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._sport_yaw = 0.0
        self._sport_raw: float | None = None
        self._sport_time: float | None = None
        self._yaw_rate = 0.0
        self._fused_yaw_rate = 0.0
        self._last_low_sport_yaw: float | None = None
        self._last_low_lio_yaw: float | None = None
        self._last_low_time: float | None = None
        self._last_wheel_q: list[float] | None = None

        # LIO state.
        self._lio_yaw = 0.0
        self._lio_raw: float | None = None
        self._lio_position_norm = 0.0
        self._lio_stamp_ns: int | None = None
        self._lio_violations = 0
        self._lio_warned = False

        # The official Unitree state publishers offer RELIABLE samples.  A
        # matching reader is required on this deployment's DDS path.
        reliable_state = QoSProfile(
            depth=20,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            LowState, low_topic, self._on_low, reliable_state
        )
        self.create_subscription(
            SportModeState, sport_topic, self._on_sport, reliable_state
        )
        if self._lio_enabled:
            self.create_subscription(
                Odometry, lio_topic, self._on_lio, reliable_state
            )
        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self._fused_pub = self.create_publisher(Odometry, fused_topic, 10)
        self._status_pub = self.create_publisher(
            DiagnosticArray, "/go2w/odom/fused/status", 10
        )
        self._tf = TransformBroadcaster(self) if self._publish_tf else None
        self.get_logger().info(
            f"wheel odometry ready: radius={self._radius:.4f} m "
            f"wheels={self._start}..{self._start + self._count - 1} "
            f"lio_fusion={self._lio_enabled} "
            f"lio_weight={self._lio_weight:.2f}"
        )

    def _on_sport(self, msg: SportModeState) -> None:
        raw = float(msg.imu_state.rpy[2])
        previous_yaw = self._sport_yaw
        self._sport_yaw = unwrap_yaw(raw, self._sport_raw, self._sport_yaw)
        now = self.get_clock().now().nanoseconds / 1e9
        if self._sport_time is not None:
            dt = now - self._sport_time
            if 0.0 < dt < 1.0:
                self._yaw_rate = (self._sport_yaw - previous_yaw) / dt
        self._sport_raw = raw
        self._sport_time = now

    def _on_lio(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        quat = pose.orientation
        raw_yaw = math.atan2(
            2.0 * (quat.z * quat.w), 1.0 - 2.0 * quat.z * quat.z
        )
        self._lio_yaw = unwrap_yaw(
            raw_yaw, self._lio_raw, self._lio_yaw
        )
        self._lio_raw = raw_yaw
        self._lio_position_norm = math.sqrt(
            pose.position.x ** 2
            + pose.position.y ** 2
            + pose.position.z ** 2
        )
        # Point-LIO stamps derive from the lidar clock which is offset from
        # host time (~534 s on this robot), so freshness is judged by host
        # receive time, not the message stamp.
        self._lio_stamp_ns = time.time_ns()

    def _lio_valid_now(self) -> bool:
        if not self._lio_enabled or self._lio_raw is None:
            return False
        if self._lio_stamp_ns is None:
            return False
        now_ns = time.time_ns()
        age = (now_ns - self._lio_stamp_ns) / 1e9
        if not math.isfinite(self._lio_yaw) or not math.isfinite(
            self._lio_position_norm
        ):
            return False
        return (
            age <= self._lio_max_age
            and self._lio_position_norm <= self._lio_max_position
        )

    def _on_low(self, msg: LowState) -> None:
        wheels = [
            float(msg.motor_state[i].q)
            for i in range(self._start, self._start + self._count)
        ]
        if self._last_wheel_q is None:
            self._last_wheel_q = wheels
            self._last_low_sport_yaw = self._sport_yaw
            self._last_low_lio_yaw = self._lio_yaw
            self._last_low_time = self.get_clock().now().nanoseconds / 1e9
            self._yaw = self._sport_yaw
            return
        deltas = [
            new - old
            for new, old in zip(wheels, self._last_wheel_q)
        ]
        self._last_wheel_q = wheels
        if any(abs(delta) > 1.0 for delta in deltas):
            self.get_logger().warn("wheel q jump detected; skipping integration")
            return
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - (self._last_low_time or now)
        self._last_low_time = now
        if dt <= 0.0 or dt > 1.0:
            dt = 0.05

        sport_delta = self._sport_yaw - (self._last_low_sport_yaw or 0.0)
        lio_delta = self._lio_yaw - (self._last_low_lio_yaw or 0.0)
        self._last_low_sport_yaw = self._sport_yaw
        self._last_low_lio_yaw = self._lio_yaw

        fused_delta = sport_delta
        lio_used = False
        lio_valid = self._lio_valid_now()
        if lio_valid and heading_delta_sane(
            sport_delta, lio_delta, self._lio_max_yaw_step
        ):
            fused_delta = fuse_yaw_delta(
                sport_delta, lio_delta, self._lio_weight
            )
            lio_used = True
            self._lio_violations = 0
            self._lio_warned = False
        elif lio_valid:
            self._lio_violations += 1
            if self._lio_violations >= 3 and not self._lio_warned:
                self._lio_warned = True
                self.get_logger().warn(
                    "LIO yaw delta disagrees with Sport; ignoring LIO "
                    "heading until it recovers"
                )

        self._yaw += fused_delta
        self._fused_yaw_rate = fused_delta / dt if dt > 0.0 else 0.0

        skip_turn = (
            self._skip_turn_translation
            and abs(self._yaw_rate) > self._turn_yaw_threshold
        )
        if not skip_turn:
            self._wheel_x, self._wheel_y = step_odometry(
                self._wheel_x, self._wheel_y, self._sport_yaw,
                deltas, self._radius,
            )
            self._x, self._y = step_odometry(
                self._x, self._y, self._yaw, deltas, self._radius
            )
        self._publish(msg, lio_valid=lio_valid, lio_used=lio_used)

    def _publish(self, msg: LowState, *, lio_valid: bool,
                 lio_used: bool) -> None:
        stamp = self.get_clock().now().to_msg()
        wheel_odom = Odometry()
        wheel_odom.header.stamp = stamp
        wheel_odom.header.frame_id = self._frame
        wheel_odom.child_frame_id = self._child
        wheel_odom.pose.pose.position.x = self._wheel_x
        wheel_odom.pose.pose.position.y = self._wheel_y
        wheel_odom.pose.pose.orientation = _quaternion_from_yaw(
            self._sport_yaw
        )
        wheel_odom.twist.twist.angular.z = self._yaw_rate
        self._odom_pub.publish(wheel_odom)

        fused_odom = Odometry()
        fused_odom.header.stamp = stamp
        fused_odom.header.frame_id = self._fused_frame
        fused_odom.child_frame_id = self._child
        fused_odom.pose.pose.position.x = self._x
        fused_odom.pose.pose.position.y = self._y
        fused_odom.pose.pose.orientation = _quaternion_from_yaw(self._yaw)
        fused_odom.twist.twist.angular.z = self._fused_yaw_rate
        self._fused_pub.publish(fused_odom)

        status = DiagnosticArray()
        status.header.stamp = stamp
        item = DiagnosticStatus()
        item.level = DiagnosticStatus.OK if lio_valid else DiagnosticStatus.WARN
        item.name = "go2w_wheel_odom/fused_heading"
        item.hardware_id = "go2w_wheel_encoders+sport+lio"
        item.message = (
            "LIO heading fused" if lio_used else "Sport heading only"
        )
        item.values = [
            KeyValue(key="lio_valid", value=str(lio_valid).lower()),
            KeyValue(key="lio_used", value=str(lio_used).lower()),
            KeyValue(key="lio_position_norm_m",
                     value=f"{self._lio_position_norm:.3f}"),
            KeyValue(key="lio_yaw_rad", value=f"{self._lio_yaw:.4f}"),
            KeyValue(key="sport_yaw_rad", value=f"{self._sport_yaw:.4f}"),
            KeyValue(key="fused_yaw_rad", value=f"{self._yaw:.4f}"),
            KeyValue(key="lio_violations",
                     value=str(self._lio_violations)),
        ]
        status.status = [item]
        self._status_pub.publish(status)

        if self._tf is not None:
            for frame, x, y, yaw in (
                (self._frame, self._wheel_x, self._wheel_y, self._sport_yaw),
                (self._fused_frame, self._x, self._y, self._yaw),
            ):
                transform = TransformStamped()
                transform.header.stamp = stamp
                transform.header.frame_id = frame
                transform.child_frame_id = self._child
                transform.transform.translation.x = x
                transform.transform.translation.y = y
                transform.transform.rotation = _quaternion_from_yaw(yaw)
                self._tf.sendTransform(transform)


def _quaternion_from_yaw(yaw: float) -> Quaternion:
    quaternion = Quaternion()
    quaternion.z = math.sin(yaw / 2.0)
    quaternion.w = math.cos(yaw / 2.0)
    return quaternion


def main() -> None:
    rclpy.init()
    node = WheelOdomNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
