"""Calibration-gated ROS 2 RGB-LiDAR target fusion node."""

from __future__ import annotations

from collections import deque
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray, Detection3D, Detection3DArray

from .config_gate import (
    load_diagnostic_fusion_gate,
    load_extrinsics_gate,
    load_fusion_gate,
)
from .fusion_core import localize_mask_points
from .overlay_core import (
    load_confirmed_transform,
    load_diagnostic_transform,
    transform_lidar_to_camera,
)


def message_stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def quaternion_matrix(quaternion) -> np.ndarray:
    values = np.asarray(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w], dtype=np.float64
    )
    norm = float(np.linalg.norm(values))
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("transform quaternion is invalid")
    x, y, z, w = values / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def transform_points(points: np.ndarray, transform) -> np.ndarray:
    rotation = quaternion_matrix(transform.rotation)
    translation = np.asarray(
        [transform.translation.x, transform.translation.y, transform.translation.z],
        dtype=np.float64,
    )
    return np.asarray(points, dtype=np.float64) @ rotation.T + translation


def transform_camera_to_lidar(points: np.ndarray, lidar_to_camera) -> np.ndarray:
    """Invert the calibrated row-vector LiDAR-to-camera transform."""

    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError("camera points must have shape Nx3")
    return (values - lidar_to_camera.translation.reshape(1, 3)) @ (
        lidar_to_camera.rotation
    )


def cloud_xyz(message: PointCloud2) -> np.ndarray:
    raw = point_cloud2.read_points(
        message, field_names=("x", "y", "z"), skip_nans=True
    )
    if getattr(raw.dtype, "names", None):
        points = np.column_stack([raw[name] for name in ("x", "y", "z")])
    else:
        points = np.asarray(raw).reshape(-1, 3)
    return np.asarray(points, dtype=np.float64)


def nearest(messages: deque, stamp_ns: int, maximum_delta_ns: int):
    if not messages:
        return None
    candidate = min(messages, key=lambda item: abs(message_stamp_ns(item) - stamp_ns))
    return (
        candidate
        if abs(message_stamp_ns(candidate) - stamp_ns) <= maximum_delta_ns
        else None
    )


class FusionNode(Node):
    def __init__(self) -> None:
        super().__init__("go2w_rgb_lidar_fusion")
        self.declare_parameter("fusion_config", "")
        self.declare_parameter("camera_config", "")
        self.declare_parameter("extrinsics_config", "")
        self.declare_parameter("camera_frame", "front_camera_optical_frame")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("cloud_topic", "/go2w/lidar/cloud_filtered")
        self._bridge = CvBridge()
        self._tf = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf, self)
        self._images = deque(maxlen=5)
        self._infos = deque(maxlen=5)
        self._masks = deque(maxlen=5)
        self._clouds = deque(maxlen=10)
        self._diagnostic_enabled = False
        self._enabled = False
        self._extrinsics_validated = False
        self._blocker = "fusion configuration has not been evaluated"
        self._extrinsics_blocker = "extrinsics configuration has not been evaluated"
        # Do not use ``_parameters`` here: that name belongs to rclpy.Node and
        # stores every declared ROS parameter.
        self._fusion_parameters = None
        self._lidar_to_camera = None
        fusion_config = str(self.get_parameter("fusion_config").value or "")
        camera_config = str(self.get_parameter("camera_config").value or "")
        extrinsics_config = str(self.get_parameter("extrinsics_config").value or "")
        try:
            _, self._fusion_parameters = load_diagnostic_fusion_gate(
                fusion_config,
                camera_config,
                extrinsics_config,
            )
            self._lidar_to_camera = load_diagnostic_transform(extrinsics_config)
            self._diagnostic_enabled = True
        except Exception as exc:
            self._blocker = f"diagnostic overlay unavailable: {exc}"
        try:
            load_extrinsics_gate(camera_config, extrinsics_config)
            self._extrinsics_validated = True
            self._extrinsics_blocker = ""
        except Exception as exc:
            self._extrinsics_blocker = str(exc)
        try:
            _, self._fusion_parameters = load_fusion_gate(
                fusion_config,
                camera_config,
                extrinsics_config,
            )
            self._lidar_to_camera = load_confirmed_transform(extrinsics_config)
            self._diagnostic_enabled = True
            self._enabled = True
            self._blocker = ""
        except Exception as exc:
            self._blocker = str(exc)

        self._detection3d_pub = self.create_publisher(
            Detection3DArray, "/perception/detections_3d", 10
        )
        self._relative_pose_pub = self.create_publisher(
            PoseStamped, "/perception/target_pose_relative", 10
        )
        self._odom_pose_pub = self.create_publisher(
            PoseStamped, "/perception/target_pose_odom", 10
        )
        self._debug_pub = self.create_publisher(
            Image, "/perception/fusion_debug_image", qos_profile_sensor_data
        )
        self._status_pub = self.create_publisher(
            DiagnosticArray, "/perception/fusion_status", 10
        )
        self._ready_pub = self.create_publisher(Bool, "/perception/fusion_ready", 10)
        self._extrinsics_pub = self.create_publisher(
            Bool, "/perception/rgb_lidar_extrinsics_validated", 10
        )
        self._overlay_ready_pub = self.create_publisher(
            Bool, "/perception/rgb_lidar_overlay_ready", 10
        )
        self.create_subscription(
            Image, "/camera/front/image_raw", self._images.append, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            "/camera/front/camera_info",
            self._infos.append,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, "/perception/target_mask", self._masks.append, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("cloud_topic").value),
            self._clouds.append,
            QoSProfile(
                depth=5,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self.create_subscription(
            Detection2DArray, "/perception/detections_2d", self._detection, 10
        )
        self.create_timer(0.2, self._heartbeat)

    def _heartbeat(self) -> None:
        self._ready_pub.publish(Bool(data=self._enabled))
        self._extrinsics_pub.publish(Bool(data=self._extrinsics_validated))
        self._overlay_ready_pub.publish(Bool(data=self._diagnostic_enabled))
        level = (
            DiagnosticStatus.OK
            if self._enabled
            else DiagnosticStatus.WARN
            if self._diagnostic_enabled
            else DiagnosticStatus.ERROR
        )
        message = (
            "navigation-grade fusion gate open"
            if self._enabled
            else "diagnostic overlay ready; metric 3D gate closed"
            if self._diagnostic_enabled
            else "fusion gate closed"
        )
        self._diagnostic(
            level,
            message,
            {
                "blocker": self._blocker,
                "extrinsics_blocker": self._extrinsics_blocker,
                "diagnostic_overlay_ready": str(self._diagnostic_enabled).lower(),
                "authorizes_3d_output": str(self._enabled).lower(),
                "authorizes_motion": "false",
            },
        )

    def _detection(self, detections: Detection2DArray) -> None:
        if (
            not self._diagnostic_enabled
            or self._fusion_parameters is None
            or self._lidar_to_camera is None
        ):
            return
        if not detections.detections:
            self._diagnostic(DiagnosticStatus.WARN, "no 2D detection", {})
            return
        stamp = message_stamp_ns(detections)
        maximum_delta_ns = int(
            self._fusion_parameters.maximum_timestamp_delta_ms * 1e6
        )
        image = nearest(self._images, stamp, maximum_delta_ns)
        info = nearest(self._infos, stamp, maximum_delta_ns)
        mask = nearest(self._masks, stamp, maximum_delta_ns)
        cloud = nearest(self._clouds, stamp, maximum_delta_ns)
        if any(value is None for value in (image, info, mask, cloud)):
            self._diagnostic(
                DiagnosticStatus.WARN,
                "fusion inputs unavailable or outside sync threshold",
                {},
            )
            return
        try:
            camera_frame = str(self.get_parameter("camera_frame").value)
            points_lidar = cloud_xyz(cloud)
            if cloud.header.frame_id != "utlidar_lidar":
                cloud_transform = self._tf.lookup_transform(
                    "utlidar_lidar",
                    cloud.header.frame_id,
                    Time.from_msg(cloud.header.stamp),
                    timeout=Duration(seconds=0.05),
                )
                points_lidar = transform_points(
                    points_lidar, cloud_transform.transform
                )
            points_camera = transform_lidar_to_camera(
                points_lidar, self._lidar_to_camera
            )
            mask_array = self._bridge.imgmsg_to_cv2(mask, desired_encoding="mono8")
            intrinsic = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
            detection = detections.detections[0]
            center = detection.bbox.center.position
            bbox = (
                float(center.x - detection.bbox.size_x / 2.0),
                float(center.y - detection.bbox.size_y / 2.0),
                float(center.x + detection.bbox.size_x / 2.0),
                float(center.y + detection.bbox.size_y / 2.0),
            )
            delta_ms = abs(message_stamp_ns(cloud) - stamp) / 1e6
            result = localize_mask_points(
                points_camera,
                mask_array,
                intrinsic,
                bbox,
                delta_ms,
                self._fusion_parameters,
                distortion_coefficients=np.asarray(info.d, dtype=np.float64),
                distortion_model=info.distortion_model,
            )
            self._publish_debug(image, detection, result)
            if not result.localized_3d:
                self._diagnostic(
                    DiagnosticStatus.WARN,
                    "detected_2d=true, localized_3d=false",
                    {
                        "reason": result.reason,
                        "point_count": result.point_count,
                        "timestamp_delta_ms": result.timestamp_delta_ms,
                    },
                )
                return
            if not self._enabled:
                self._diagnostic(
                    DiagnosticStatus.WARN,
                    "diagnostic localization only; metric 3D output suppressed",
                    {
                        "point_count": result.point_count,
                        "timestamp_delta_ms": result.timestamp_delta_ms,
                        "authorizes_3d_output": "false",
                        "authorizes_motion": "false",
                    },
                )
                return
            self._publish_localization(detections, detection, result, camera_frame)
        except (TransformException, ValueError, RuntimeError) as exc:
            self._diagnostic(
                DiagnosticStatus.ERROR,
                "detected_2d=true, localized_3d=false",
                {"reason": str(exc)},
            )

    def _publish_localization(self, detections, detection, result, camera_frame) -> None:
        now_header = detections.header
        now_header.frame_id = camera_frame
        pose = PoseStamped()
        pose.header = now_header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
            result.position_camera_m
        )
        pose.pose.orientation.w = 1.0
        self._relative_pose_pub.publish(pose)

        detection3d = Detection3D()
        detection3d.header = now_header
        detection3d.id = detection.id
        detection3d.results = detection.results
        detection3d.bbox.center = pose.pose
        detection3d.bbox.size.x, detection3d.bbox.size.y, detection3d.bbox.size.z = (
            result.robust_size_m
        )
        array = Detection3DArray()
        array.header = now_header
        array.detections = [detection3d]
        self._detection3d_pub.publish(array)

        odom_frame = str(self.get_parameter("odom_frame").value)
        try:
            transform = self._tf.lookup_transform(
                odom_frame,
                "utlidar_lidar",
                Time.from_msg(pose.header.stamp),
                timeout=Duration(seconds=0.05),
            )
            point_lidar = transform_camera_to_lidar(
                np.asarray([result.position_camera_m]), self._lidar_to_camera
            )
            point = transform_points(
                point_lidar, transform.transform
            )[0]
            odom_pose = PoseStamped()
            odom_pose.header = pose.header
            odom_pose.header.frame_id = odom_frame
            odom_pose.pose.position.x = float(point[0])
            odom_pose.pose.position.y = float(point[1])
            odom_pose.pose.position.z = float(point[2])
            odom_pose.pose.orientation.w = 1.0
            self._odom_pose_pub.publish(odom_pose)
        except TransformException:
            pass
        self._diagnostic(
            DiagnosticStatus.OK,
            "detected_2d=true, localized_3d=true",
            {
                "point_count": result.point_count,
                "timestamp_delta_ms": result.timestamp_delta_ms,
                "confidence": result.confidence,
                "transform_source": "calibrated_extrinsics",
            },
        )

    def _publish_debug(self, image_message, detection, result) -> None:
        image = self._bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        center = detection.bbox.center.position
        x1 = int(round(center.x - detection.bbox.size_x / 2.0))
        y1 = int(round(center.y - detection.bbox.size_y / 2.0))
        x2 = int(round(center.x + detection.bbox.size_x / 2.0))
        y2 = int(round(center.y + detection.bbox.size_y / 2.0))
        color = (0, 255, 0) if result.localized_3d else (0, 165, 255)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            image,
            result.reason,
            (max(0, x1), max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )
        debug = self._bridge.cv2_to_imgmsg(image, encoding="bgr8")
        debug.header = image_message.header
        self._debug_pub.publish(debug)

    def _diagnostic(self, level: int, message: str, values: dict) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = level
        status.name = "go2w_rgb_lidar_fusion/gate"
        status.hardware_id = "go2w_builtin_rgb_lidar"
        status.message = message
        status.values = [KeyValue(key=str(key), value=str(value)) for key, value in values.items()]
        array.status = [status]
        self._status_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FusionNode()
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
