#!/usr/bin/env python3
"""Publish a bounded JSON snapshot of plain_slam PointCloud2 for the WebUI.

This process is display-only.  It subscribes to the isolated mapping topics,
never publishes ROS messages, and never touches the motion-authoritative
``/go2w/odom/fused`` chain.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from app.spatial.pointcloud_web_codec import BoundedVoxelCloud, extract_xyz_points


class PlainSlamWebBridge(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("plain_slam_web_bridge")
        self._output = Path(args.output).resolve()
        self._output.parent.mkdir(parents=True, exist_ok=True)
        self._cloud = BoundedVoxelCloud(
            voxel_size_m=args.voxel_size,
            max_points=args.max_accumulated_points,
        )
        self._max_web_points = args.max_web_points
        self._max_input_points = args.max_input_points
        self._last_scan_monotonic = 0.0
        self._last_source = ""
        self._last_frame = ""
        self._last_stamp = 0.0
        self._received_scans = 0

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, args.map_topic, self._on_map, map_qos)
        self.create_subscription(PointCloud2, args.scan_topic, self._on_scan, scan_qos)
        self.create_timer(1.0 / max(0.2, args.publish_rate), self._write_snapshot)
        self.get_logger().info(
            f"display-only bridge: {args.map_topic} + {args.scan_topic} -> {self._output}"
        )

    @staticmethod
    def _stamp_seconds(message: PointCloud2) -> float:
        stamp = message.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _decode(self, message: PointCloud2) -> list[tuple[float, float, float]]:
        return extract_xyz_points(message, max_input_points=self._max_input_points)

    def _on_map(self, message: PointCloud2) -> None:
        points = self._decode(message)
        if not points:
            return
        # The upstream map is authoritative global history when it updates.
        self._cloud.clear()
        self._cloud.update(points)
        self._remember(message, "map_3d")

    def _on_scan(self, message: PointCloud2) -> None:
        now = time.monotonic()
        if now - self._last_scan_monotonic < 0.18:
            return
        self._last_scan_monotonic = now
        points = self._decode(message)
        if not points:
            return
        # Aligned scans are already in the LIO world frame.  Accumulating their
        # voxels keeps the Web view growing even while the robot is between
        # sparse upstream map keyframes.
        self._cloud.update(points)
        self._received_scans += 1
        self._remember(message, "aligned_scan_accumulated")

    def _remember(self, message: PointCloud2, source: str) -> None:
        self._last_source = source
        self._last_frame = str(message.header.frame_id or "pslam_odom")
        self._last_stamp = self._stamp_seconds(message)

    def _write_snapshot(self) -> None:
        points = self._cloud.sampled(self._max_web_points)
        if points:
            xs = [point[0] for point in points]
            ys = [point[1] for point in points]
            zs = [point[2] for point in points]
            bounds: dict[str, Any] = {
                "min": [round(min(xs), 3), round(min(ys), 3), round(min(zs), 3)],
                "max": [round(max(xs), 3), round(max(ys), 3), round(max(zs), 3)],
            }
        else:
            bounds = {"min": [0.0, 0.0, 0.0], "max": [0.0, 0.0, 0.0]}
        payload = {
            "schema_version": "go2w_slam_web_cloud_v1",
            "available": bool(points),
            "source": self._last_source or "waiting_for_plain_slam",
            "frame_id": self._last_frame or "pslam_odom",
            "ros_stamp": self._last_stamp,
            "generated_at": time.time(),
            "point_count": len(points),
            "accumulated_voxels": len(self._cloud),
            "received_scans": self._received_scans,
            "voxel_size_m": self._cloud.voxel_size_m,
            "bounds": bounds,
            "points": [[round(x, 3), round(y, 3), round(z, 3)] for x, y, z in points],
            "mapping_mode": "mapping_assist",
            "motion_authorized": False,
            "safety_authorized": False,
        }
        temporary = self._output.with_suffix(self._output.suffix + f".{os.getpid()}.tmp")
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, separators=(",", ":")),
            encoding="utf-8",
        )
        os.replace(temporary, self._output)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--map-topic", default="/go2w/slam/map_3d")
    parser.add_argument("--scan-topic", default="/go2w/slam/aligned_scan")
    parser.add_argument("--voxel-size", type=float, default=0.12)
    parser.add_argument("--max-input-points", type=int, default=8_000)
    parser.add_argument("--max-accumulated-points", type=int, default=40_000)
    parser.add_argument("--max-web-points", type=int, default=20_000)
    parser.add_argument("--publish-rate", type=float, default=1.5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = PlainSlamWebBridge(args)
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
