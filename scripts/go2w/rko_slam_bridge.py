#!/usr/bin/env python3
"""rko_lio → plain_slam `slam_3d_node` 桥接（离线 A/B，不改现有链路）。

`slam_3d_node` 只需要「一串带时间戳的位姿 + 一串扫描」就能做回环 / 位姿图 / 全局地图重建，
但它要求：

* 位姿是 ``geometry_msgs/PoseStamped``，扫描是 ``sensor_msgs/PointCloud2``；
* 两者 ``header.stamp`` **严格相等**（它用 message_filters 的 TimeSynchronizer 精确配对）；
* 点云字段为 ``x/y/z/intensity/timestamp``，且与位姿同坐标系。

`rko_lio` 的输出不满足后两点：``/rko_lio/frame`` 只有 ``x/y/z``（无 intensity/timestamp）
且 frame 是 ``pandarxt16_link_unvalidated``（LiDAR 系）。因此本节点：

1. 订阅 ``/rko_lio/odom`` 与 Pandar 原始点云（字段齐全）；
2. 用候选外参把点云从 LiDAR 系刚体变换到 ``base_link``（与 rko 位姿同系），字段原样保留；
3. 按「点云时间戳最近的里程计」配对，用同一 stamp 发布位姿与扫描。

用法（先 source scripts/go2w/setup_environment.sh）：

    /usr/bin/python3 scripts/go2w/rko_slam_bridge.py --ros-args \
      -p odom_topic:=/rko_lio/odom -p lidar_topic:=/hesai/pandarxt16/points_raw \
      -p pose_topic:=/go2w/rko_slam/imu_pose -p scan_topic:=/go2w/rko_slam/deskewed_scan
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import PointCloud2, PointField


def sensor_qos(depth: int = 5) -> QoSProfile:
    """与 slam_3d_node 的订阅（rmw_qos_profile_sensor_data）匹配。"""
    return QoSProfile(depth=depth,
                      history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def reliable_qos(depth: int = 5) -> QoSProfile:
    """给 WebUI 的 cloud relay 用：它以 RELIABLE 订阅，BEST_EFFORT 发布会收不到。"""
    return QoSProfile(depth=depth,
                      history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)


def quaternion_yaw(quaternion) -> float:
    return math.atan2(2.0 * (quaternion.z * quaternion.w),
                      1.0 - 2.0 * (quaternion.z * quaternion.z))


def rpy_degrees_to_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float):
    roll, pitch, yaw = (math.radians(v) for v in (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


class RkoSlamBridge(Node):
    def __init__(self) -> None:
        super().__init__("rko_slam_bridge")
        self.declare_parameter("odom_topic", "/rko_lio/odom")
        self.declare_parameter("lidar_topic", "/hesai/pandarxt16/points_raw")
        self.declare_parameter("pose_topic", "/go2w/rko_slam/imu_pose")
        self.declare_parameter("scan_topic", "/go2w/rko_slam/deskewed_scan")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("extrinsic_xyz", [0.130, 0.015, 0.014])
        self.declare_parameter("extrinsic_rpy_deg", [0.385, 0.905, 11.357])
        self.declare_parameter("max_odom_gap_s", 0.5)
        self.declare_parameter("odom_base_topic", "/go2w/rko_slam/odom_base")
        self.declare_parameter("aligned_scan_topic", "/go2w/rko_slam/aligned_scan")
        self.declare_parameter("odom_frame", "rko_odom")
        # scan_source: raw = 直接用 Pandar 原始点云（字段齐全但未去畸变）；
        #              deskewed = 用 rko_lio 已去畸变的扫描（需补齐 intensity/timestamp 字段）。
        self.declare_parameter("scan_source", "raw")
        self.declare_parameter("deskewed_scan_topic", "/rko_lio/frame")
        # 实测 rko 的 odom header.stamp 比同帧点云晚 0.099 s（见报告 §3.5.6）；
        # 查位姿时把点云时间戳映射到 odom 时间轴。
        self.declare_parameter("odom_time_offset_s", 0.099)

        self._translation = np.asarray(
            self.get_parameter("extrinsic_xyz").value, dtype=np.float64)
        rpy = self.get_parameter("extrinsic_rpy_deg").value
        self._rotation = rpy_degrees_to_matrix(*[float(v) for v in rpy])
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._max_gap = float(self.get_parameter("max_odom_gap_s").value)
        self._scan_source = str(self.get_parameter("scan_source").value)
        self._odom_offset = float(self.get_parameter("odom_time_offset_s").value)

        self._odom_buffer: deque = deque(maxlen=60)
        self._cloud_buffer: deque = deque(maxlen=80)
        self._gaps_ms: deque = deque(maxlen=1000)
        self._published = 0
        self._dropped = 0

        self._pose_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("pose_topic").value), sensor_qos())
        self._scan_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("scan_topic").value), sensor_qos())
        # 供评估器复用的别名话题（与 plain_slam 的输出契约同名，便于直接跑
        # map_quality_evaluator --topic-prefix /go2w/rko_slam）。
        self._odom_base_pub = self.create_publisher(
            Odometry, str(self.get_parameter("odom_base_topic").value), sensor_qos(20))
        self._aligned_scan_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("aligned_scan_topic").value),
            reliable_qos())

        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value),
            self._on_odom, sensor_qos(20))
        source_topic = (str(self.get_parameter("deskewed_scan_topic").value)
                        if self._scan_source == "deskewed"
                        else str(self.get_parameter("lidar_topic").value))
        self.create_subscription(
            PointCloud2, source_topic, self._on_cloud, sensor_qos())
        self.create_timer(10.0, self._report)
        self.get_logger().info(
            f"bridge up: scan_source={self._scan_source} ({source_topic}) "
            f"lidar->{self._target_frame} t={self._translation.tolist()} "
            f"rpy={list(rpy)} odom_offset={self._odom_offset}s")

    # ------------------------------------------------------------------ inputs
    def _on_odom(self, message: Odometry) -> None:
        """以里程计为驱动，为**点云时刻**求位姿后精确配对。

        rko_lio 的 odom 比点云晚到 100–400 ms（实时处理延迟）。此前直接取"最近的 odom"，
        导致位姿-点云错配 2–4 帧（实测 gap p50 100–400 ms）；这里改为在 odom 缓冲里
        **对点云时间戳做插值/外推**，两侧用同一 stamp 发布，配对误差回到毫秒级。
        """
        self._odom_base_pub.publish(message)
        self._odom_buffer.append((stamp_to_seconds(message.header.stamp), message))
        # 点云 10 Hz 而 rko odom 只有 ~6 Hz：一次 odom 回调要排空所有"已被覆盖"的点云，
        # 否则点云队列会持续积压（实测滞留 1.4 s）。
        while self._cloud_buffer:
            cloud_stamp, cloud_message = self._cloud_buffer[0]
            pose_message = self._pose_at(cloud_stamp + self._odom_offset)
            if pose_message is None:
                break
            self._cloud_buffer.popleft()
            self._gaps_ms.append(
                abs(self._odom_buffer[-1][0] - cloud_stamp) * 1000.0)

            transformed = self._prepare_scan(cloud_message, cloud_stamp)
            self._scan_pub.publish(transformed)
            self._aligned_scan_pub.publish(transformed)

            pose = PoseStamped()
            pose.header.stamp = cloud_message.header.stamp
            pose.header.frame_id = self._odom_frame
            pose.pose = pose_message.pose.pose
            self._pose_pub.publish(pose)
            self._published += 1

    def _on_cloud(self, message: PointCloud2) -> None:
        self._cloud_buffer.append((stamp_to_seconds(message.header.stamp), message))

    def _pose_at(self, stamp: float):
        """返回 stamp 时刻的位姿消息（缓冲内插值、缓冲外按最近两帧外推）。"""
        if not self._odom_buffer:
            return None
        before = [item for item in self._odom_buffer if item[0] <= stamp]
        after = [item for item in self._odom_buffer if item[0] > stamp]
        if before and after:
            older, newer = before[-1], after[0]
        elif before:
            if len(self._odom_buffer) < 2:
                return None
            newer = before[-1]
            older = before[-2] if len(before) >= 2 else self._odom_buffer[0]
        else:
            if len(self._odom_buffer) < 2:
                return None
            older, newer = self._odom_buffer[0], self._odom_buffer[1]
        span = newer[0] - older[0]
        if abs(span) < 1e-6:
            return older[1]
        ratio = (stamp - older[0]) / span
        return self._interpolate_odometry(older[1], newer[1], ratio)

    @staticmethod
    def _interpolate_odometry(older: Odometry, newer: Odometry, ratio: float) -> Odometry:
        first, second = older.pose.pose, newer.pose.pose
        out = Odometry()
        out.header = newer.header
        out.pose.pose.position.x = first.position.x + (second.position.x - first.position.x) * ratio
        out.pose.pose.position.y = first.position.y + (second.position.y - first.position.y) * ratio
        out.pose.pose.position.z = first.position.z + (second.position.z - first.position.z) * ratio
        yaw_first = quaternion_yaw(first.orientation)
        yaw_second = quaternion_yaw(second.orientation)
        delta = math.atan2(math.sin(yaw_second - yaw_first),
                           math.cos(yaw_second - yaw_first))
        yaw = yaw_first + delta * ratio
        out.pose.pose.orientation.z = math.sin(yaw / 2.0)
        out.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return out

    # ----------------------------------------------------------------- helpers
    def _prepare_scan(self, message: PointCloud2, stamp: float) -> PointCloud2:
        """按 scan_source 产出交给 slam_3d_node 的扫描（统一到 target_frame）。"""
        if self._scan_source == "deskewed":
            return self._repack_deskewed(message, stamp)
        out = self._transform_cloud(message)
        out.header.stamp = message.header.stamp
        out.header.frame_id = self._target_frame
        return out

    def _repack_deskewed(self, message: PointCloud2, stamp: float) -> PointCloud2:
        """把 rko 的去畸变扫描（仅 x/y/z）重打包为 slam_3d_node 需要的字段布局。

        ``ParsePSLAMCloud`` 会**无条件**读取 intensity 与 timestamp 字段（ros_utils.cpp:156/158），
        字段缺失时会越界读，所以必须补齐：intensity 置 0、timestamp 写该帧时间戳。
        点云同时按外参从 LiDAR 系变换到 ``target_frame``。
        """
        count = int(message.width) * int(message.height)
        stride = int(message.point_step)
        raw = np.frombuffer(message.data, dtype=np.uint8).reshape(count, stride)
        xyz = np.ascontiguousarray(raw[:, 0:12]).view(np.float32).reshape(-1, 3)
        transformed = (xyz.astype(np.float64) @ self._rotation.T) + self._translation
        payload = np.zeros((count, 24), dtype=np.uint8)
        payload[:, 0:12] = np.frombuffer(
            transformed.astype(np.float32).tobytes(), dtype=np.uint8).reshape(-1, 12)
        payload[:, 16:24] = np.frombuffer(
            np.full(count, stamp, dtype=np.float64).tobytes(),
            dtype=np.uint8).reshape(-1, 8)
        out = PointCloud2()
        out.header.stamp = message.header.stamp
        out.header.frame_id = self._target_frame
        out.height = 1
        out.width = count
        out.point_step = 24
        out.row_step = 24 * count
        out.is_bigendian = False
        out.is_dense = True
        out.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="timestamp", offset=16, datatype=PointField.FLOAT64, count=1),
        ]
        out.data = payload.tobytes()
        return out

    def _transform_cloud(self, message: PointCloud2) -> PointCloud2:
        points = int(message.width) * int(message.height)
        stride = int(message.point_step)
        buffer = np.frombuffer(message.data, dtype=np.uint8).reshape(points, stride).copy()
        xyz = np.ascontiguousarray(buffer[:, 0:12]).view(np.float32).reshape(-1, 3)
        transformed = (xyz.astype(np.float64) @ self._rotation.T) + self._translation
        buffer[:, 0:12] = np.frombuffer(
            transformed.astype(np.float32).tobytes(), dtype=np.uint8).reshape(-1, 12)
        out = PointCloud2()
        out.header = message.header
        out.height = message.height
        out.width = message.width
        out.fields = message.fields
        out.is_bigendian = message.is_bigendian
        out.point_step = message.point_step
        out.row_step = message.row_step
        out.is_dense = message.is_dense
        out.data = buffer.tobytes()
        return out

    def _report(self) -> None:
        if self._gaps_ms:
            ordered = sorted(self._gaps_ms)
            pick = lambda q: ordered[min(len(ordered) - 1, int(q * (len(ordered) - 1)))]
            gap_text = (f"gap_ms p50={pick(0.5):.1f} p95={pick(0.95):.1f} "
                        f"max={ordered[-1]:.1f}")
        else:
            gap_text = "gap_ms n/a"
        total = self._published + self._dropped
        rate = 100.0 * self._published / total if total else 0.0
        self.get_logger().info(
            f"published={self._published} dropped_no_odom={self._dropped} "
            f"paired={rate:.1f}% {gap_text} "
            f"odom_buffer={len(self._odom_buffer)} cloud_buffer={len(self._cloud_buffer)}")


def main() -> int:
    rclpy.init()
    node = RkoSlamBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
