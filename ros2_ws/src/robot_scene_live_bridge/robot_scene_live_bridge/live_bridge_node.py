from __future__ import annotations

import math
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from .bundle_writer import AtomicBundleWriter


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def quaternion_yaw(quaternion) -> float:
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny, cosy)


def finite_or_none(value: float) -> float | None:
    value = float(value)
    return value if math.isfinite(value) else None


def clearance_status(value: float, *, source_fresh: bool, valid: bool = True) -> str:
    """Preserve measured/no-return/unknown semantics in finite JSON."""

    if not source_fresh or not valid:
        return "unknown"
    value = float(value)
    if math.isnan(value):
        return "unknown"
    if math.isinf(value):
        return "no_return"
    return "measured"


def scheduled_bundle_deadline(
    next_stamp_ns: int | None, current_stamp_ns: int, period_ns: int
) -> tuple[bool, int]:
    """Rate-limit without accumulating source-frame quantization drift."""

    if period_ns <= 0:
        raise ValueError("period_ns must be positive")
    if next_stamp_ns is None:
        return True, current_stamp_ns + period_ns
    if current_stamp_ns < next_stamp_ns:
        return False, next_stamp_ns
    periods = max(1, (current_stamp_ns - next_stamp_ns) // period_ns + 1)
    return True, next_stamp_ns + periods * period_ns


class LiveBridge(Node):
    def __init__(self) -> None:
        super().__init__("robot_scene_live_bridge")
        self.declare_parameter("spool_root", "")
        self.declare_parameter("session_id", f"session_{int(time.time())}")
        self.declare_parameter("sensor_timeout_seconds", 0.3)
        self.declare_parameter("bundle_rate_hz", 1.0)
        self.declare_parameter("max_bundles_per_session", 30)
        root = str(self.get_parameter("spool_root").value).strip()
        if not root:
            raise ValueError("spool_root is required")
        bundle_rate_hz = float(self.get_parameter("bundle_rate_hz").value)
        if not math.isfinite(bundle_rate_hz) or bundle_rate_hz <= 0.0:
            raise ValueError("bundle_rate_hz must be positive and finite")
        max_bundles = int(self.get_parameter("max_bundles_per_session").value)
        if max_bundles < 1:
            raise ValueError("max_bundles_per_session must be positive")
        self._session_id = str(self.get_parameter("session_id").value)
        self._writer = AtomicBundleWriter(
            Path(root), max_bundles_per_session=max_bundles
        )
        self._bundle_period_ns = int(1e9 / bundle_rate_hz)
        self._next_bundle_stamp_ns = None
        self._bridge = CvBridge()
        self._frame_id = 0
        self._pending_image = None
        self._pending_info = None
        self._odom = None
        self._odom_receive_ns = None
        self._clearance = None
        self._clearance_receive_ns = None
        self._lidar_fresh = False
        self._lidar_fresh_receive_ns = None
        self._rotation_clearance_valid = False
        self._rotation_clearance_valid_receive_ns = None
        self._extrinsics_validated = False
        self._extrinsics_receive_ns = None
        self._rgb_lidar_overlay_ready = False
        self._rgb_lidar_overlay_receive_ns = None
        self._fusion_ready = False
        self._fusion_receive_ns = None
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(
            Image, "/camera/front/image_raw", self._image, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            "/camera/front/camera_info",
            self._camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(Odometry, "/lio/odom", self._on_odom, 10)
        self.create_subscription(
            Vector3Stamped, "/go2w/lidar/clearance", self._on_clearance, 10
        )
        self.create_subscription(
            Bool, "/go2w/safety/lidar_fresh", self._on_lidar_fresh, 10
        )
        self.create_subscription(
            Bool,
            "/go2w/safety/rotation_clearance_valid",
            self._on_rotation_clearance_valid,
            10,
        )
        self.create_subscription(
            Bool,
            "/perception/rgb_lidar_extrinsics_validated",
            self._on_extrinsics_validated,
            10,
        )
        self.create_subscription(
            Bool,
            "/perception/rgb_lidar_overlay_ready",
            self._on_rgb_lidar_overlay_ready,
            10,
        )
        self.create_subscription(
            Bool, "/perception/fusion_ready", self._on_fusion_ready, 10
        )

    def _image(self, message: Image) -> None:
        self._pending_image = message
        self._try_write()

    def _camera_info(self, message: CameraInfo) -> None:
        self._pending_info = message
        self._try_write()

    def _on_odom(self, message: Odometry) -> None:
        self._odom = message
        self._odom_receive_ns = self.get_clock().now().nanoseconds

    def _on_clearance(self, message: Vector3Stamped) -> None:
        self._clearance = message
        self._clearance_receive_ns = self.get_clock().now().nanoseconds

    def _on_lidar_fresh(self, message: Bool) -> None:
        self._lidar_fresh = bool(message.data)
        self._lidar_fresh_receive_ns = self.get_clock().now().nanoseconds

    def _on_rotation_clearance_valid(self, message: Bool) -> None:
        self._rotation_clearance_valid = bool(message.data)
        self._rotation_clearance_valid_receive_ns = (
            self.get_clock().now().nanoseconds
        )

    def _on_extrinsics_validated(self, message: Bool) -> None:
        self._extrinsics_validated = bool(message.data)
        self._extrinsics_receive_ns = self.get_clock().now().nanoseconds

    def _on_rgb_lidar_overlay_ready(self, message: Bool) -> None:
        self._rgb_lidar_overlay_ready = bool(message.data)
        self._rgb_lidar_overlay_receive_ns = self.get_clock().now().nanoseconds

    def _on_fusion_ready(self, message: Bool) -> None:
        self._fusion_ready = bool(message.data)
        self._fusion_receive_ns = self.get_clock().now().nanoseconds

    def _try_write(self) -> None:
        image = self._pending_image
        info = self._pending_info
        if image is None or info is None or stamp_ns(image) != stamp_ns(info):
            return
        self._pending_image = None
        self._pending_info = None
        current_stamp_ns = stamp_ns(image)
        allowed, next_deadline_ns = scheduled_bundle_deadline(
            self._next_bundle_stamp_ns,
            current_stamp_ns,
            self._bundle_period_ns,
        )
        if not allowed:
            return
        cv_image = self._bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        success, encoded = cv2.imencode(".jpg", cv_image, [cv2.IMWRITE_JPEG_QUALITY, 92])
        if not success:
            self.get_logger().error("failed to encode frame bundle JPEG")
            return
        self._frame_id += 1
        payload = self._bundle_payload(image, info)
        try:
            path = self._writer.write(encoded.tobytes(), payload)
            self._next_bundle_stamp_ns = next_deadline_ns
            self.get_logger().info(
                f"frame bundle {self._frame_id} ready at {path}",
                throttle_duration_sec=5.0,
            )
        except Exception as exc:
            self.get_logger().error(f"frame bundle write failed: {exc}")

    def _bundle_payload(self, image: Image, info: CameraInfo) -> dict:
        now_ns = self.get_clock().now().nanoseconds
        timeout_ns = int(float(self.get_parameter("sensor_timeout_seconds").value) * 1e9)
        lio_fresh = (
            self._odom is not None
            and self._odom_receive_ns is not None
            and 0 <= now_ns - self._odom_receive_ns <= timeout_ns
        )
        clearance_fresh = (
            self._clearance is not None
            and self._clearance_receive_ns is not None
            and 0 <= now_ns - self._clearance_receive_ns <= timeout_ns
            and self._lidar_fresh
            and self._lidar_fresh_receive_ns is not None
            and 0 <= now_ns - self._lidar_fresh_receive_ns <= timeout_ns
        )
        rotation_clearance_valid = bool(
            clearance_fresh
            and self._rotation_clearance_valid
            and self._rotation_clearance_valid_receive_ns is not None
            and 0
            <= now_ns - self._rotation_clearance_valid_receive_ns
            <= timeout_ns
        )
        front_clearance = (
            float(self._clearance.vector.x) if self._clearance is not None else math.nan
        )
        left_clearance = (
            float(self._clearance.vector.y) if self._clearance is not None else math.nan
        )
        right_clearance = (
            float(self._clearance.vector.z) if self._clearance is not None else math.nan
        )
        extrinsics_fresh = (
            self._extrinsics_validated
            and self._extrinsics_receive_ns is not None
            and 0 <= now_ns - self._extrinsics_receive_ns <= timeout_ns
        )
        rgb_lidar_overlay_fresh = (
            self._rgb_lidar_overlay_ready
            and self._rgb_lidar_overlay_receive_ns is not None
            and 0 <= now_ns - self._rgb_lidar_overlay_receive_ns <= timeout_ns
        )
        fusion_fresh = (
            self._fusion_ready
            and self._fusion_receive_ns is not None
            and 0 <= now_ns - self._fusion_receive_ns <= timeout_ns
            and extrinsics_fresh
        )
        pose = self._pose_payload(lio_fresh)
        tf_ok = self._tf_available()
        calibrated = bool(
            len(info.k) == 9
            and info.k[0] > 0.0
            and info.k[4] > 0.0
            and len(info.p) == 12
            and info.p[0] > 0.0
            and info.p[5] > 0.0
        )
        return {
            "schema_version": "1.0",
            "session_id": self._session_id,
            "frame_id": self._frame_id,
            "image_path": "image.jpg",
            "image_receive_time_ns": stamp_ns(image),
            "image_capture_time_trusted": False,
            "camera_frame": image.header.frame_id,
            "camera_info": {
                "width": int(info.width),
                "height": int(info.height),
                "k": list(info.k),
                "d": list(info.d),
                "distortion_model": info.distortion_model,
                "calibrated": calibrated,
            },
            "robot_pose": pose,
            "clearance": {
                "front_m": finite_or_none(self._clearance.vector.x)
                if clearance_fresh
                else None,
                "left_m": finite_or_none(self._clearance.vector.y)
                if clearance_fresh
                else None,
                "right_m": finite_or_none(self._clearance.vector.z)
                if clearance_fresh
                else None,
                "front_status": clearance_status(
                    front_clearance, source_fresh=clearance_fresh
                ),
                "left_status": clearance_status(
                    left_clearance,
                    source_fresh=clearance_fresh,
                    valid=rotation_clearance_valid,
                ),
                "right_status": clearance_status(
                    right_clearance,
                    source_fresh=clearance_fresh,
                    valid=rotation_clearance_valid,
                ),
                "lidar_fresh": clearance_fresh,
                "rotation_clearance_valid": rotation_clearance_valid,
            },
            "sensor_health": {
                "camera": True,
                "camera_info_calibrated": calibrated,
                "rgb_lidar_overlay": rgb_lidar_overlay_fresh,
                "rgb_lidar_extrinsics": extrinsics_fresh,
                "rgb_lidar_fusion": fusion_fresh,
                "lidar": clearance_fresh,
                "lio": lio_fresh,
                "tf": tf_ok,
            },
            "motion_state": {
                "commanded_by_bridge": False,
                "observation_mode": "stationary_required",
            },
        }

    def _pose_payload(self, fresh: bool) -> dict:
        if not fresh:
            return {
                "available": False,
                "frame": "odom",
                "x": None,
                "y": None,
                "z": None,
                "yaw": None,
                "source": None,
                "fresh": False,
            }
        pose = self._odom.pose.pose
        return {
            "available": True,
            "frame": self._odom.header.frame_id,
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "yaw": quaternion_yaw(pose.orientation),
            "source": "/lio/odom",
            "fresh": True,
        }

    def _tf_available(self) -> bool:
        return bool(
            self._tf_buffer.can_transform(
                "base_link",
                "front_camera_optical_frame",
                Time(),
                timeout=Duration(seconds=0.0),
            )
            and self._tf_buffer.can_transform(
                "base_link",
                "utlidar_lidar",
                Time(),
                timeout=Duration(seconds=0.0),
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LiveBridge()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
