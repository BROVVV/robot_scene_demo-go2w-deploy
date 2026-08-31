from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def _rotation_matrix_xyzw(quaternion) -> np.ndarray:
    x, y, z, w = (float(value) for value in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def transform_lidar_points_to_odom(
    points: np.ndarray, odom_pose, lidar2base_quat_xyzw_xyz
) -> np.ndarray:
    """Apply odom<-base and base<-lidar so the output really is registered."""

    extrinsic = tuple(float(value) for value in lidar2base_quat_xyzw_xyz)
    if len(extrinsic) != 7 or not all(math.isfinite(value) for value in extrinsic):
        raise ValueError("LiDAR extrinsic must contain seven finite values")
    lidar_rotation = _rotation_matrix_xyzw(extrinsic[:4])
    lidar_translation = np.asarray(extrinsic[4:], dtype=np.float64)
    pose_quaternion = (
        odom_pose.orientation.x,
        odom_pose.orientation.y,
        odom_pose.orientation.z,
        odom_pose.orientation.w,
    )
    pose_rotation = _rotation_matrix_xyzw(pose_quaternion)
    pose_translation = np.asarray(
        (odom_pose.position.x, odom_pose.position.y, odom_pose.position.z),
        dtype=np.float64,
    )
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or not np.isfinite(xyz).all():
        raise ValueError("LiDAR points must be a finite N x 3 array")
    points_in_base = xyz @ lidar_rotation.T + lidar_translation
    return points_in_base @ pose_rotation.T + pose_translation


def _cloud_xyz(message: PointCloud2) -> np.ndarray:
    records = point_cloud2.read_points(
        message, field_names=("x", "y", "z"), skip_nans=True
    )
    return np.column_stack(
        tuple(np.asarray(records[name], dtype=np.float64) for name in ("x", "y", "z"))
    )


def _stamp_key(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


class OutputAdapter(Node):
    def __init__(self) -> None:
        super().__init__("go2w_lio_output_adapter")
        self.declare_parameter("input_timeout_seconds", 0.3)
        self.declare_parameter("maximum_path_poses", 10000)
        self.declare_parameter("expected_odom_frame", "odom")
        self.declare_parameter("expected_base_frame", "base_link")
        self.declare_parameter("expected_lidar_frame", "utlidar_lidar")
        self.declare_parameter(
            "extrinsic_lidar2base_quat_xyzw_xyz", Parameter.Type.DOUBLE_ARRAY
        )
        self._odom_frame = str(self.get_parameter("expected_odom_frame").value)
        self._base_frame = str(self.get_parameter("expected_base_frame").value)
        self._lidar_frame = str(self.get_parameter("expected_lidar_frame").value)
        self._lidar2base = tuple(
            float(value)
            for value in self.get_parameter(
                "extrinsic_lidar2base_quat_xyzw_xyz"
            ).value
        )
        if len(self._lidar2base) != 7:
            raise ValueError("resolved LiDAR-to-base extrinsic is required")
        self._last_odom_ns = None
        self._path = Path()
        self._pending_clouds: dict[int, PointCloud2] = {}
        self._pending_odometry: dict[int, Odometry] = {}
        self._odom_pub = self.create_publisher(Odometry, "/lio/odom", 10)
        self._path_pub = self.create_publisher(Path, "/lio/path", 10)
        self._cloud_pub = self.create_publisher(
            PointCloud2, "/lio/cloud_registered", qos_profile_sensor_data
        )
        self._status_pub = self.create_publisher(DiagnosticArray, "/lio/status", 10)
        self.create_subscription(Odometry, "/rko_lio/odom", self._odom, 10)
        self.create_subscription(
            PointCloud2, "/rko_lio/frame", self._cloud, QoSProfile(depth=10)
        )
        self.create_timer(0.1, self._watchdog)

    def _cloud(self, message: PointCloud2) -> None:
        if message.header.frame_id != self._lidar_frame:
            self._diagnostic(DiagnosticStatus.ERROR, "unexpected_cloud_frame")
            return
        self._pending_clouds[_stamp_key(message)] = message
        odometry = self._pending_odometry.pop(_stamp_key(message), None)
        if odometry is not None:
            self._publish_registered_cloud(message, odometry)
        while len(self._pending_clouds) > 4:
            del self._pending_clouds[min(self._pending_clouds)]

    def _odom(self, message: Odometry) -> None:
        if (
            message.header.frame_id != self._odom_frame
            or message.child_frame_id != self._base_frame
        ):
            self._diagnostic(DiagnosticStatus.ERROR, "unexpected_frame_contract")
            return
        values = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.linear.z,
            message.twist.twist.angular.x,
            message.twist.twist.angular.y,
            message.twist.twist.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            self._diagnostic(DiagnosticStatus.ERROR, "nonfinite_odometry")
            return
        try:
            _rotation_matrix_xyzw(values[3:7])
        except ValueError:
            self._diagnostic(DiagnosticStatus.ERROR, "invalid_odometry_quaternion")
            return

        self._last_odom_ns = self.get_clock().now().nanoseconds
        self._odom_pub.publish(message)
        self._path.header = message.header
        self._path.poses.append(self._pose_stamped(message))
        limit = int(self.get_parameter("maximum_path_poses").value)
        if len(self._path.poses) > limit:
            self._path.poses = self._path.poses[-limit:]
        self._path_pub.publish(self._path)

        cloud = self._pending_clouds.pop(_stamp_key(message), None)
        if cloud is None:
            self._pending_odometry[_stamp_key(message)] = message
            while len(self._pending_odometry) > 4:
                del self._pending_odometry[min(self._pending_odometry)]
            self._diagnostic(
                DiagnosticStatus.OK,
                "valid",
                registered_cloud_pending="true",
            )
            return
        self._publish_registered_cloud(cloud, message)

    def _publish_registered_cloud(
        self, cloud: PointCloud2, message: Odometry
    ) -> None:
        try:
            registered = transform_lidar_points_to_odom(
                _cloud_xyz(cloud), message.pose.pose, self._lidar2base
            )
        except ValueError as exc:
            self._diagnostic(DiagnosticStatus.ERROR, str(exc))
            return
        header = Header()
        header.stamp = message.header.stamp
        header.frame_id = self._odom_frame
        self._cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, registered))
        self._diagnostic(
            DiagnosticStatus.OK,
            "valid",
            registered_cloud_points=str(len(registered)),
        )

    @staticmethod
    def _pose_stamped(message):
        from geometry_msgs.msg import PoseStamped

        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose.pose
        return pose

    def _watchdog(self) -> None:
        if self._last_odom_ns is None:
            self._diagnostic(DiagnosticStatus.ERROR, "no_odometry")
            return
        age_ns = self.get_clock().now().nanoseconds - self._last_odom_ns
        limit_ns = float(self.get_parameter("input_timeout_seconds").value) * 1e9
        if age_ns > limit_ns:
            self._diagnostic(
                DiagnosticStatus.ERROR,
                "odometry_stale",
                age_seconds=f"{age_ns / 1e9:.6f}",
            )

    def _diagnostic(self, level: int, message: str, **values: str) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = level
        status.name = "go2w_lio/status"
        status.hardware_id = "go2w_utlidar"
        status.message = message
        details = {
            "pose_republished_when_stale": "false",
            "robot_motion_authorized": "false",
            "validation_scope": "stationary_read_only",
        }
        details.update(values)
        status.values = [KeyValue(key=key, value=value) for key, value in details.items()]
        array.status = [status]
        self._status_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OutputAdapter()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
