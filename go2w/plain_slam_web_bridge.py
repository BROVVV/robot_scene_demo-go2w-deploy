#!/usr/bin/env python3
"""Publish a bounded JSON snapshot of plain_slam PointCloud2 for the WebUI.

This process is display-only.  It subscribes to the isolated mapping topics,
never publishes ROS messages, and never touches the motion-authoritative
``/go2w/odom/fused`` chain.
"""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
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
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

from app.spatial.pointcloud_web_codec import BoundedVoxelCloud, extract_xyz_points


def _wrap_pi(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


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
        self._accumulated_scans = 0
        self._dropped_scans = 0
        self._dropped_reason_counts: dict[str, int] = {}
        self._last_dropped_reason: str | None = None
        self._diagnostics_counter = 0
        self._diagnostics_every = max(1, int(args.diagnostics_every))
        # 计划书 §10.2：目标 world frame（plain_slam = pslam_odom）。
        self._target_frame = str(args.target_frame or "pslam_odom")
        # 计划书 §10.4：运动期保守门控（LIO 过渡期坏帧不永久污染累积图）。
        self._odom_pose: tuple[float, float, float] | None = None
        self._prev_odom: tuple[float, float, float, float] | None = None
        self._odom_ts: float | None = None
        self._yaw_rate_max = float(args.yaw_rate_max)
        self._speed_max = float(args.speed_max)
        self._settle_seconds = max(0.1, float(args.settle_seconds))
        self._stationary_since: float | None = None
        self._last_odom_dyaw = 0.0
        # LIO pose differences contain occasional one-sample jitter while the
        # robot is stopped.  Keep a short twist-based history so one noisy
        # sample cannot prevent the settle timer from ever completing, while
        # sustained real motion still resets the gate quickly.
        self._motion_history: deque[tuple[float, bool]] = deque(maxlen=256)
        self._motion_window_seconds = max(0.25, min(1.0, self._settle_seconds))
        # 计划书 §10.3：新 mapping session 清空旧 voxel accumulator。
        self._last_map_origin: tuple[float, float] | None = None
        self._reset_marker_path = (
            Path(args.reset_marker).resolve() if args.reset_marker else None
        )
        self._reset_marker_mtime: float | None = None
        # 启动即清空：避免旧 session 污染新实验。
        self._cloud.clear()

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
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, args.map_topic, self._on_map, map_qos)
        self.create_subscription(PointCloud2, args.scan_topic, self._on_scan, scan_qos)
        self.create_subscription(Odometry, args.odom_topic, self._on_odom, odom_qos)
        self.create_timer(1.0 / max(0.2, args.publish_rate), self._write_snapshot)
        self.get_logger().info(
            f"display-only bridge: {args.map_topic} + {args.scan_topic} -> "
            f"{self._output} (target_frame={self._target_frame}, "
            f"odom_topic={args.odom_topic})"
        )

    @staticmethod
    def _stamp_seconds(message: PointCloud2) -> float:
        stamp = message.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _decode(self, message: PointCloud2) -> list[tuple[float, float, float]]:
        return extract_xyz_points(message, max_input_points=self._max_input_points)

    def _on_odom(self, message: Odometry) -> None:
        """运动门控用的 odom 状态（默认 /go2w/slam/odom_base，pslam_odom）。"""
        now = time.monotonic()
        pose = message.pose.pose
        q = pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        x, y = float(pose.position.x), float(pose.position.y)
        if self._odom_pose is not None and self._odom_ts is not None:
            elapsed = max(1e-3, now - self._odom_ts)
            prev_x, prev_y, prev_yaw = self._odom_pose
            dyaw = _wrap_pi(yaw - prev_yaw)
            speed = math.hypot(x - prev_x, y - prev_y) / elapsed
            yaw_rate = abs(dyaw) / elapsed
            # Use the message-reported twist for the stationary decision.  A
            # finite-difference pose speed is intentionally not used here:
            # stationary LIO output can jitter enough to exceed 0.02 m/s for
            # isolated samples.  The finite-difference values remain useful
            # for the independent pose-jump guard below.
            twist_speed = math.hypot(
                float(message.twist.twist.linear.x),
                float(message.twist.twist.linear.y),
            )
            twist_yaw_rate = abs(float(message.twist.twist.angular.z))
            sample_moving = (
                twist_yaw_rate > self._yaw_rate_max
                or twist_speed > self._speed_max
            )
            self._motion_history.append((now, sample_moving))
            while (
                self._motion_history
                and now - self._motion_history[0][0] > self._motion_window_seconds
            ):
                self._motion_history.popleft()
            moving_fraction = (
                sum(1 for _, active in self._motion_history if active)
                / max(1, len(self._motion_history))
            )
            moving = moving_fraction >= 0.25
            self._last_odom_dyaw = dyaw
            if moving:
                self._stationary_since = None
            elif self._stationary_since is None:
                self._stationary_since = now
        else:
            self._stationary_since = now
        self._prev_odom = (
            (self._odom_pose[0], self._odom_pose[1], self._odom_pose[2], self._odom_ts)
            if self._odom_pose is not None and self._odom_ts is not None
            else None
        )
        self._odom_pose = (x, y, yaw)
        self._odom_ts = now

    def _stationary(self) -> bool:
        if self._stationary_since is None:
            return False
        return (time.monotonic() - self._stationary_since) >= self._settle_seconds

    def _pose_jump(self, message: PointCloud2) -> bool:
        """异常 pose jump 检测：只报警/丢弃坏帧，绝不自动调 LIO 参数。"""
        if self._prev_odom is None or self._odom_pose is None or self._odom_ts is None:
            return False
        px, py, _, prev_ts = self._prev_odom
        elapsed = max(1e-3, self._odom_ts - prev_ts)
        dx = self._odom_pose[0] - px
        dy = self._odom_pose[1] - py
        if math.hypot(dx, dy) / elapsed > 2.0:
            return True
        if abs(self._last_odom_dyaw) > 0.5:
            return True
        return False

    def _on_map(self, message: PointCloud2) -> None:
        points = self._decode(message)
        if not points:
            return
        frame_id = str(message.header.frame_id or self._target_frame)
        # plain_slam publishes its optimized history in ``pslam_map`` while
        # aligned_scan is already expressed in ``pslam_odom``.  This bridge
        # has no timestamped TF transform path, so a mismatched map cloud must
        # be rejected rather than silently relabelled as the target frame.
        # The aligned-scan path below remains the authoritative display-only
        # accumulator and supplies the WebUI cloud in pslam_odom.
        if frame_id != self._target_frame:
            self._dropped_scans += 1
            self._dropped_reason_counts["map_frame_mismatch"] = (
                self._dropped_reason_counts.get("map_frame_mismatch", 0) + 1
            )
            self._last_dropped_reason = "map_frame_mismatch"
            return
        origin = (message.info.origin.position.x, message.info.origin.position.y) \
            if hasattr(message, "info") and hasattr(message.info, "origin") else None
        # 计划书 §10.3：map frame 变化或地图原点大幅跳变 => 新 mapping session，
        # 清空旧 voxel accumulator，避免旧 session 污染新实验。
        new_session = frame_id != self._target_frame
        if self._last_map_origin is not None and origin is not None:
            jump = math.hypot(
                origin[0] - self._last_map_origin[0],
                origin[1] - self._last_map_origin[1],
            )
            if jump > 1.0:
                new_session = True
                self.get_logger().warn(
                    f"map origin jumped {jump:.2f}m -> clearing accumulated cloud"
                )
        if new_session:
            self._cloud.clear()
            self._accumulated_scans = 0
            self._dropped_scans = 0
            self._dropped_reason_counts = {}
        if origin is not None:
            self._last_map_origin = origin
        # The upstream map is authoritative global history when it updates.
        self._cloud.clear()
        self._cloud.update(points)
        self._remember(message, "map_3d")

    def _on_scan(self, message: PointCloud2) -> None:
        now = time.monotonic()
        if now - self._last_scan_monotonic < 0.18:
            return
        self._last_scan_monotonic = now
        self._received_scans += 1
        points = self._decode(message)
        frame_id = str(message.header.frame_id or "")
        stamp = self._stamp_seconds(message)
        dropped_reason = None
        # 计划书 §10.2：只有 pslam_odom 的 scan 才能直接累积；否则必须 TF 变换，
        # TF 不可用就丢弃该帧。绝不能把 sensor frame 点当 world point 累积。
        if frame_id != self._target_frame:
            dropped_reason = "frame_mismatch"
        elif not self._stationary():
            # 计划书 §10.4：运动期可显示 latest scan，但先不写入永久累积图。
            dropped_reason = "motion_active"
        elif self._pose_jump(message):
            dropped_reason = "pose_jump"
        if not points:
            dropped_reason = dropped_reason or "empty_scan"
        if dropped_reason is not None:
            self._dropped_scans += 1
            self._dropped_reason_counts[dropped_reason] = (
                self._dropped_reason_counts.get(dropped_reason, 0) + 1
            )
            self._last_dropped_reason = dropped_reason
            self._diagnostics(message, stamp, dropped_reason=dropped_reason)
            return
        # Aligned scans are already in the LIO world frame (pslam_odom).
        self._cloud.update(points)
        self._accumulated_scans += 1
        self._last_dropped_reason = None
        self._remember(message, "aligned_scan_accumulated")
        self._diagnostics(message, stamp, dropped_reason=None)

    def _diagnostics(self, message: PointCloud2, stamp: float,
                     *, dropped_reason: str | None) -> None:
        """计划书 §10.5：定期输出 scan 诊断（日志 + snapshot 字段）。"""
        self._diagnostics_counter += 1
        if self._diagnostics_counter % self._diagnostics_every != 0:
            return
        pose = self._odom_pose
        self.get_logger().info(
            "scan_diag "
            f"frame={str(message.header.frame_id or '')} "
            f"stamp={stamp:.3f} target={self._target_frame} "
            f"points={len(self._decode(message))} "
            f"pose=({pose[0] if pose else 0.0:.3f},"
            f"{pose[1] if pose else 0.0:.3f},"
            f"{pose[2] if pose else 0.0:.3f}) "
            f"dyaw={self._last_odom_dyaw:.4f} "
            f"stationary={self._stationary()} accumulated={len(self._cloud)} "
            f"dropped={dropped_reason or 'none'}"
        )

    def _remember(self, message: PointCloud2, source: str) -> None:
        self._last_source = source
        self._last_frame = str(message.header.frame_id or "pslam_odom")
        self._last_stamp = self._stamp_seconds(message)

    def _write_snapshot(self) -> None:
        self._check_reset_marker()
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
        pose = self._odom_pose
        payload = {
            "schema_version": "go2w_slam_web_cloud_v1",
            "available": bool(points),
            "source": self._last_source or "waiting_for_plain_slam",
            "frame_id": self._last_frame or self._target_frame,
            "target_map_frame": self._target_frame,
            "ros_stamp": self._last_stamp,
            "generated_at": time.time(),
            "point_count": len(points),
            "accumulated_voxels": len(self._cloud),
            "received_scans": self._received_scans,
            "accumulated_scans": self._accumulated_scans,
            "dropped_scans": self._dropped_scans,
            "dropped_reason_counts": dict(self._dropped_reason_counts),
            "last_dropped_reason": self._last_dropped_reason,
            "stationary": self._stationary(),
            "pose": (
                {"x": round(pose[0], 4), "y": round(pose[1], 4),
                 "yaw": round(pose[2], 4)}
                if pose is not None else None
            ),
            "delta_yaw_rad": round(self._last_odom_dyaw, 5),
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

    def _check_reset_marker(self) -> None:
        """计划书 §10.3：外部（例如新搜索 session）touch 该文件即清空累积图。"""
        if self._reset_marker_path is None:
            return
        try:
            mtime = self._reset_marker_path.stat().st_mtime
        except OSError:
            return
        if self._reset_marker_mtime is not None and mtime > self._reset_marker_mtime:
            self._cloud.clear()
            self._accumulated_scans = 0
            self._dropped_scans = 0
            self._dropped_reason_counts = {}
            self.get_logger().info("reset marker touched -> cleared accumulated cloud")
        self._reset_marker_mtime = mtime


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--map-topic", default="/go2w/slam/map_3d")
    parser.add_argument("--scan-topic", default="/go2w/slam/aligned_scan")
    parser.add_argument("--odom-topic", default="/go2w/slam/odom_base",
                        help="odometry used by the motion gate (pslam_odom)")
    parser.add_argument("--target-frame", default="pslam_odom",
                        help="world frame that aligned_scan must belong to")
    parser.add_argument("--voxel-size", type=float, default=0.12)
    parser.add_argument("--max-input-points", type=int, default=8_000)
    parser.add_argument("--max-accumulated-points", type=int, default=40_000)
    parser.add_argument("--max-web-points", type=int, default=20_000)
    parser.add_argument("--publish-rate", type=float, default=1.5)
    parser.add_argument("--yaw-rate-max", type=float, default=0.03,
                        help="max |yaw rate| rad/s to consider the robot stationary")
    parser.add_argument("--speed-max", type=float, default=0.02,
                        help="max planar speed m/s to consider the robot stationary")
    parser.add_argument("--settle-seconds", type=float, default=0.5,
                        help="continuous stationary window before accumulating scans")
    parser.add_argument("--diagnostics-every", type=int, default=30,
                        help="log scan diagnostics every N received scans")
    parser.add_argument("--reset-marker", default="",
                        help="optional file; touching it clears the accumulated voxels")
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
