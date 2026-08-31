from __future__ import annotations

import copy
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32
from tf2_ros import Buffer, TransformException, TransformListener

from .config import load_safety_ready_config
from .preprocess_core import (
    collision_obstacles,
    directional_clearance,
    filter_points_base_link,
    laser_scan_ranges,
    transform_points,
)


class LidarPreprocessor(Node):
    def __init__(self) -> None:
        super().__init__("go2w_lidar_preprocessor")
        self.declare_parameter("config_file", "")
        self.declare_parameter("geometry_file", "")
        self._ready = False
        self._last_valid_ns = None
        try:
            self._config, self._parameters = load_safety_ready_config(
                str(self.get_parameter("config_file").value),
                str(self.get_parameter("geometry_file").value),
            )
            self._ready = True
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.get_logger().error(f"safety gate closed: {exc}")

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._filtered_pub = self.create_publisher(
            PointCloud2, "/go2w/lidar/cloud_filtered", qos_profile_sensor_data
        )
        self._obstacles_pub = self.create_publisher(
            PointCloud2, "/go2w/lidar/obstacles", qos_profile_sensor_data
        )
        self._collision_obstacles_pub = self.create_publisher(
            PointCloud2, "/go2w/lidar/collision_obstacles", qos_profile_sensor_data
        )
        self._scan_pub = self.create_publisher(
            LaserScan, "/go2w/lidar/scan", qos_profile_sensor_data
        )
        self._clearance_pub = self.create_publisher(
            Vector3Stamped, "/go2w/lidar/clearance", 10
        )
        self._diagnostic_clearance_pub = self.create_publisher(
            Vector3Stamped, "/go2w/diagnostics/lidar_clearance_raw", 10
        )
        self._front_pub = self.create_publisher(
            Float32, "/go2w/safety/front_clearance", 10
        )
        self._left_pub = self.create_publisher(
            Float32, "/go2w/safety/left_clearance", 10
        )
        self._right_pub = self.create_publisher(
            Float32, "/go2w/safety/right_clearance", 10
        )
        self._rotation_clearance_valid_pub = self.create_publisher(
            Bool, "/go2w/safety/rotation_clearance_valid", 10
        )
        self._fresh_pub = self.create_publisher(
            Bool, "/go2w/safety/lidar_fresh", 10
        )
        reliable_cloud_qos = QoSProfile(
            depth=5,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            PointCloud2,
            "/go2w/sensors/cloud",
            self._cloud,
            reliable_cloud_qos,
        )
        self.create_timer(0.1, self._publish_freshness)

    def _cloud(self, message: PointCloud2) -> None:
        if not self._ready:
            self._fresh_pub.publish(Bool(data=False))
            return
        if not message.header.frame_id:
            self.get_logger().error("cloud has no frame_id")
            self._fresh_pub.publish(Bool(data=False))
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                "base_link",
                message.header.frame_id,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.get_logger().warning(f"base_link TF unavailable: {exc}")
            self._fresh_pub.publish(Bool(data=False))
            return
        records = point_cloud2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=False
        )
        source_xyz = np.column_stack(
            tuple(
                np.asarray(records[name], dtype=np.float64)
                for name in ("x", "y", "z")
            )
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        xyz = transform_points(
            source_xyz,
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )
        filtered, obstacles = filter_points_base_link(xyz, self._parameters)
        collision_points = collision_obstacles(obstacles, self._parameters)
        header = copy.deepcopy(message.header)
        header.frame_id = "base_link"
        self._filtered_pub.publish(
            point_cloud2.create_cloud_xyz32(header, filtered.tolist())
        )
        self._obstacles_pub.publish(
            point_cloud2.create_cloud_xyz32(header, obstacles.tolist())
        )
        self._collision_obstacles_pub.publish(
            point_cloud2.create_cloud_xyz32(header, collision_points.tolist())
        )
        clearance = directional_clearance(collision_points, self._parameters)
        rotation_validation = self._config.get("rotation_clearance_validation") or {}
        rotation_clearance_valid = bool(rotation_validation.get("valid", False))
        clearance_message = Vector3Stamped()
        clearance_message.header = header
        clearance_message.vector.x = clearance.front
        clearance_message.vector.y = (
            clearance.left if rotation_clearance_valid else math.nan
        )
        clearance_message.vector.z = (
            clearance.right if rotation_clearance_valid else math.nan
        )
        self._clearance_pub.publish(clearance_message)
        diagnostic_clearance = Vector3Stamped()
        diagnostic_clearance.header = header
        diagnostic_clearance.vector.x = clearance.front
        diagnostic_clearance.vector.y = clearance.left
        diagnostic_clearance.vector.z = clearance.right
        self._diagnostic_clearance_pub.publish(diagnostic_clearance)
        self._front_pub.publish(Float32(data=clearance.front))
        self._left_pub.publish(
            Float32(data=clearance.left if rotation_clearance_valid else math.nan)
        )
        self._right_pub.publish(
            Float32(data=clearance.right if rotation_clearance_valid else math.nan)
        )
        self._rotation_clearance_valid_pub.publish(
            Bool(data=rotation_clearance_valid)
        )
        self._publish_scan(header, collision_points)
        self._last_valid_ns = self.get_clock().now().nanoseconds
        self._fresh_pub.publish(Bool(data=True))

    def _publish_scan(self, header, obstacles: np.ndarray) -> None:
        scan_config = self._config["scan"]
        angle_min = float(scan_config["angle_min_rad"])
        angle_max = float(scan_config["angle_max_rad"])
        angle_increment = float(scan_config["angle_increment_rad"])
        ranges = laser_scan_ranges(
            obstacles,
            angle_min=angle_min,
            angle_max=angle_max,
            angle_increment=angle_increment,
            range_min=self._parameters.minimum_range,
            range_max=self._parameters.maximum_range,
        )
        scan = LaserScan()
        scan.header = header
        scan.angle_min = angle_min
        scan.angle_max = angle_min + len(ranges) * angle_increment
        scan.angle_increment = angle_increment
        scan.range_min = self._parameters.minimum_range
        scan.range_max = self._parameters.maximum_range
        scan.ranges = ranges.tolist()
        self._scan_pub.publish(scan)

    def _publish_freshness(self) -> None:
        if not self._ready or self._last_valid_ns is None:
            self._fresh_pub.publish(Bool(data=False))
            return
        timeout_ns = int(
            float(self._config["safety"]["lidar_timeout_seconds"]) * 1e9
        )
        age = self.get_clock().now().nanoseconds - self._last_valid_ns
        self._fresh_pub.publish(Bool(data=0 <= age <= timeout_ns))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarPreprocessor()
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
