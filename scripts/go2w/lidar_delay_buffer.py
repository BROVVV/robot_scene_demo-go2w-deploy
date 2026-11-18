#!/usr/bin/env python3
"""LiDAR 延迟缓冲：让去畸变（deskew）能用上完整的 IMU 覆盖。

问题（2026-09-10 实测，见 docs/SLAM_VALIDATION_REPORT.md §3.4）：
LiDAR 帧的时间窗跨越 0.1 s，而 IMU 数据比 LiDAR 帧晚到 ≈89 ms，
所以 LIO 收到一帧时，覆盖该帧**末端**的 IMU 还没到达 ——
实测每帧窗口内只有 3 个 IMU（覆盖 8.7%），而去畸变需要 ~25 个。
后果：91% 的点没有 IMU 支撑，dsk_max 达 2.7 cm，同一面墙被反复重建时层层偏移，
地图上表现为"墙很厚 / 多个角度稀疏叠加"。

本节点只做一件事：把点云**按到达时间缓冲 delay_s 秒后原样转发**（header 时间戳不改）。
插入位置：

    /go2w/slam/pandar_points ──► [本节点] ──► /go2w/slam/pandar_points_buffered ──► lio_3d_node

代价是整条 SLAM 链路延迟 delay_s（默认 0.12 s）——对建图没有影响。

用法（先 source scripts/go2w/setup_environment.sh）：

    /usr/bin/python3 scripts/go2w/lidar_delay_buffer.py --ros-args \
      -p input_topic:=/go2w/slam/pandar_points \
      -p output_topic:=/go2w/slam/pandar_points_buffered -p delay_s:=0.12
"""

from __future__ import annotations

import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import PointCloud2


def sensor_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST,
                      durability=DurabilityPolicy.VOLATILE)


def reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST,
                      durability=DurabilityPolicy.VOLATILE)


class LidarDelayBuffer(Node):
    def __init__(self) -> None:
        super().__init__("lidar_delay_buffer")
        self.declare_parameter("input_topic", "/go2w/slam/pandar_points")
        self.declare_parameter("output_topic", "/go2w/slam/pandar_points_buffered")
        self.declare_parameter("delay_s", 0.12)
        self.declare_parameter("check_period_s", 0.01)
        self._delay = float(self.get_parameter("delay_s").value)
        self._buffer: deque = deque()
        self._published = 0
        self._released_late = 0

        self._publisher = self.create_publisher(
            PointCloud2, str(self.get_parameter("output_topic").value), reliable_qos())
        self.create_subscription(
            PointCloud2, str(self.get_parameter("input_topic").value),
            self._on_cloud, sensor_qos())
        self.create_timer(float(self.get_parameter("check_period_s").value), self._release)
        self.create_timer(10.0, self._report)
        self.get_logger().info(
            f"lidar_delay_buffer: {self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value} delay={self._delay:.3f}s")

    def _on_cloud(self, message: PointCloud2) -> None:
        self._buffer.append((time.monotonic(), message))

    def _release(self) -> None:
        now = time.monotonic()
        while self._buffer and (now - self._buffer[0][0]) >= self._delay:
            enqueued, message = self._buffer.popleft()
            self._publisher.publish(message)
            self._published += 1

    def _report(self) -> None:
        self.get_logger().info(
            f"published={self._published} buffered={len(self._buffer)} "
            f"delay={self._delay:.3f}s")


def main() -> int:
    rclpy.init()
    node = LidarDelayBuffer()
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
