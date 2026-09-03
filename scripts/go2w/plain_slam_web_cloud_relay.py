#!/usr/bin/env python3
"""Republish bounded plain_slam clouds for the remote WebUI.

The LIO aligned scan is intentionally kept lossless for the local mapping
nodes, but a full Pandar frame is roughly 1.5 MiB at 10 Hz and the optimized
map cloud is around 1.7 MiB per keyframe.  On the Go2-W network those streams
can starve a Python display subscriber even though ROS 2 discovery and the
small odometry topics remain healthy.  This display-only relay runs beside
plain_slam and publishes two compact XYZ-only clouds for the WebUI host: the
sampled latest scan (preview) and the voxel-downsampled global map together
with its true SLAM point count.

It never publishes odometry, TF, commands or safety state.
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
import time
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

from app.spatial.pointcloud_web_codec import (
    extract_xyz_array,
    extract_xyz_points,
    fit_voxel_size,
)


def _sample(
    points: list[tuple[float, float, float]], limit: int
) -> list[tuple[float, float, float]]:
    """Select evenly spaced points without adding a geometry dependency."""
    if len(points) <= limit:
        return points
    stride = len(points) / float(limit)
    return [points[min(len(points) - 1, int(index * stride))] for index in range(limit)]


def build_compact_cloud(source: PointCloud2, points) -> PointCloud2:
    """Build a small XYZ-only message while preserving the source timestamp/frame."""
    if isinstance(points, np.ndarray):
        payload = bytearray(np.ascontiguousarray(points, dtype="<f4").tobytes())
        count = int(points.shape[0])
    else:
        payload = bytearray(len(points) * 12)
        count = len(points)
        for index, (x, y, z) in enumerate(points):
            struct.pack_into("<fff", payload, index * 12, float(x), float(y), float(z))
    output = PointCloud2()
    output.header.frame_id = str(source.header.frame_id or "pslam_odom")
    output.header.stamp = Time(
        sec=int(source.header.stamp.sec),
        nanosec=int(source.header.stamp.nanosec),
    )
    output.height = 1
    output.width = count
    output.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    output.is_bigendian = False
    output.point_step = 12
    output.row_step = len(payload)
    output.is_dense = True
    output.data = payload
    return output


class PlainSlamWebCloudRelay(Node):
    """Local-only input subscriber plus compact display publisher."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("plain_slam_web_cloud_relay")
        self._target_frame = str(args.target_frame)
        self._max_input_points = max(1, int(args.max_input_points))
        self._max_output_points = max(1, int(args.max_output_points))
        self._min_period = 1.0 / max(0.2, float(args.publish_rate))
        self._last_publish_monotonic = 0.0
        self._received = 0
        self._published = 0
        self._dropped = 0
        self._map_frame = str(args.map_frame)
        self._map_voxel_size = float(args.map_voxel_size)
        self._max_map_points = max(1_000, int(args.max_map_output_points))
        self._map_min_period = 1.0 / max(0.05, float(args.map_publish_rate))
        self._last_map_publish = 0.0
        self._pending_map: PointCloud2 | None = None
        self._map_revision = 0
        self._map_dropped = 0
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        # 地图是"最新一张就够"的全量历史：TRANSIENT_LOCAL 让晚启动的网页桥
        # 立刻拿到当前地图，不用等下一个关键帧。
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(PointCloud2, args.output_topic, qos)
        self.create_subscription(PointCloud2, args.input_topic, self._on_scan, qos)
        self._map_publisher = self.create_publisher(
            PointCloud2, args.map_output_topic, map_qos)
        self._map_info_publisher = self.create_publisher(
            String, args.map_info_topic, map_qos)
        self.create_subscription(
            PointCloud2, args.map_input_topic, self._on_map, map_qos)
        self.create_timer(1.0, self._flush_map)
        self.get_logger().info(
            f"display relay: {args.input_topic} -> {args.output_topic} "
            f"(max_output_points={self._max_output_points}, "
            f"target_frame={self._target_frame}); "
            f"{args.map_input_topic} -> {args.map_output_topic} "
            f"(max_map_points={self._max_map_points}, map_frame={self._map_frame})"
        )

    def _on_map(self, message: PointCloud2) -> None:
        """§9.1：整张权威地图整帧中继，只降分辨率，不裁区域、不改 frame_id。"""
        if str(message.header.frame_id or "") != self._map_frame:
            self._map_dropped += 1
            return
        self._pending_map = message
        self._flush_map()

    def _flush_map(self) -> None:
        message = self._pending_map
        if message is None:
            return
        now = time.monotonic()
        if now - self._last_map_publish < self._map_min_period:
            return
        self._pending_map = None
        self._last_map_publish = now
        points = extract_xyz_array(message)
        reduced, voxel_size = fit_voxel_size(
            points, self._map_voxel_size, self._max_map_points)
        compact = build_compact_cloud(message, reduced)
        self._map_publisher.publish(compact)
        self._map_revision += 1
        self._map_info_publisher.publish(String(data=json.dumps({
            "source_point_count": int(points.shape[0]),
            "relay_point_count": int(reduced.shape[0]),
            "relay_voxel_size_m": round(float(voxel_size), 4),
            "revision": self._map_revision,
            "frame_id": compact.header.frame_id,
        }, separators=(",", ":"))))
        self.get_logger().info(
            f"map_relay r{self._map_revision} source={points.shape[0]} "
            f"relayed={reduced.shape[0]} voxel={voxel_size:.3f}m "
            f"bytes={len(compact.data)} dropped={self._map_dropped}"
        )

    def _on_scan(self, message: PointCloud2) -> None:
        now = time.monotonic()
        self._received += 1
        if now - self._last_publish_monotonic < self._min_period:
            return
        frame_id = str(message.header.frame_id or "")
        if frame_id != self._target_frame:
            self._dropped += 1
            return
        points = extract_xyz_points(message, max_input_points=self._max_input_points)
        if not points:
            self._dropped += 1
            return
        compact = build_compact_cloud(
            message,
            _sample(points, self._max_output_points),
        )
        self._publisher.publish(compact)
        self._last_publish_monotonic = now
        self._published += 1
        if self._published == 1 or self._published % 20 == 0:
            self.get_logger().info(
                f"relay_diag received={self._received} published={self._published} "
                f"points={compact.width} bytes={len(compact.data)} dropped={self._dropped}"
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-topic", default="/go2w/slam/aligned_scan")
    parser.add_argument("--output-topic", default="/go2w/slam/web_scan")
    parser.add_argument("--target-frame", default="pslam_odom")
    parser.add_argument("--max-input-points", type=int, default=8_000)
    parser.add_argument("--max-output-points", type=int, default=4_000)
    parser.add_argument("--publish-rate", type=float, default=2.0)
    parser.add_argument("--map-input-topic", default="/go2w/slam/map_3d")
    parser.add_argument("--map-output-topic", default="/go2w/slam/web_map")
    parser.add_argument("--map-info-topic", default="/go2w/slam/web_map_info")
    parser.add_argument("--map-frame", default="pslam_map")
    parser.add_argument("--map-voxel-size", type=float, default=0.12)
    parser.add_argument("--max-map-output-points", type=int, default=60_000)
    parser.add_argument("--map-publish-rate", type=float, default=0.5,
                        help="max relayed map revisions per second")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = PlainSlamWebCloudRelay(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
