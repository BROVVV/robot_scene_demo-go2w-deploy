#!/usr/bin/env python3
"""Publish a JSON snapshot of the plain_slam fixed-world map for the WebUI.

计划书 §9：永久地图只来自权威 SLAM 地图（``/go2w/slam/map_3d``，坐标系
``pslam_map``），每个 revision 整张替换；``aligned_scan`` 只做实时预览图层。
This process is display-only: it never publishes ROS messages and never
touches the motion-authoritative ``/go2w/odom/fused`` chain.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from app.spatial.pointcloud_web_codec import extract_xyz_array, extract_xyz_points
from app.spatial.slam_web_map_state import MotionEpisodeGate, SlamWebMapState


def _yaw_of(orientation) -> float:
    q = orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _stamp_seconds(message) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class PlainSlamWebBridge(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("plain_slam_web_bridge")
        self._output = Path(args.output).resolve()
        self._output.parent.mkdir(parents=True, exist_ok=True)
        self._state = SlamWebMapState(
            canonical_frame=args.map_frame,
            permanent_source=args.map_topic,
            voxel_size_m=args.voxel_size,
            max_global_voxels=args.max_global_voxels,
            max_web_points=args.max_web_points,
            gate=MotionEpisodeGate(speed_max=args.speed_max,
                                   yaw_rate_max=args.yaw_rate_max,
                                   settle_seconds=args.settle_seconds),
        )
        self._scan_topic = str(args.scan_topic)
        self._preview_frame = str(args.scan_frame)
        self._max_preview_points = max(100, int(args.max_preview_points))
        self._preview_interval = 1.0 / max(0.2, float(args.preview_rate))
        self._last_preview_monotonic = 0.0
        self._relayed_source_points = 0
        self._motion_odom_topic = str(args.motion_odom_topic)
        self._require_motion_odom = bool(args.require_motion_odom)
        self._motion_odom_seen = False
        self._diagnostics_every = max(1, int(args.diagnostics_every))
        self._diagnostics_counter = 0
        self._last_reject_log = 0.0
        self._reset_marker_path = (
            Path(args.reset_marker).resolve() if args.reset_marker else None
        )
        # 启动时先记下基线（文件还不存在就是 0.0），这样操作员第一次 touch 就能 reset。
        self._reset_marker_mtime = self._reset_marker_stamp()

        map_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        scan_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        odom_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                              reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(PointCloud2, args.map_topic, self._on_map, map_qos)
        if args.map_info_topic:
            self.create_subscription(String, args.map_info_topic,
                                     self._on_map_info, map_qos)
        self.create_subscription(PointCloud2, args.scan_topic, self._on_scan, scan_qos)
        self.create_subscription(Odometry, args.odom_topic, self._on_lio_odom, odom_qos)
        if self._motion_odom_topic != str(args.odom_topic):
            self.create_subscription(Odometry, self._motion_odom_topic,
                                     self._on_motion_odom, odom_qos)
        self.create_timer(1.0 / max(0.2, float(args.publish_rate)),
                          self._write_snapshot)
        self.get_logger().info(
            f"display-only bridge: permanent={args.map_topic}({args.map_frame}) "
            f"preview={args.scan_topic}({args.scan_frame}) -> {self._output} "
            f"[global<={args.max_global_voxels} voxels, web<={args.max_web_points} "
            f"points, lio_odom={args.odom_topic}, motion_odom={self._motion_odom_topic}]"
        )

    def _on_map_info(self, message: String) -> None:
        """§9.3：中继地图的真实 SLAM 点数，网页采样点数不能冒充 SLAM 点数。"""
        self._relayed_source_points = int(json.loads(message.data)
                                          .get("source_point_count", 0))

    def _on_map(self, message: PointCloud2) -> None:
        points = extract_xyz_array(message)
        frame_id = str(message.header.frame_id or "")
        state = self._state
        accepted = state.accept_map(
            points, frame_id=frame_id, stamp=_stamp_seconds(message),
            wall_time=time.time(),
            source_points=self._relayed_source_points or None,
        )
        if accepted:
            self.get_logger().info(
                f"map r{state.map_revision} session {state.session_id}: "
                f"source={state.source_map_points} cached={len(state.cloud)} "
                f"voxel={state.cloud.effective_voxel_size_m:.3f}m "
                f"extent={state.cloud.extent_m()}"
            )
            return
        now = time.monotonic()
        if now - self._last_reject_log >= 5.0:
            self._last_reject_log = now
            self.get_logger().warn(
                f"map rejected: {state.last_rejected_reason} (frame={frame_id}, "
                f"points={len(points)}, canonical={state.canonical_frame}, "
                f"health={state.health})"
            )

    def _on_scan(self, message: PointCloud2) -> None:
        """§9.1：最新 scan 只是 debug 预览层，永远不写进永久地图。"""
        now = time.monotonic()
        if now - self._last_preview_monotonic < self._preview_interval:
            return
        self._last_preview_monotonic = now
        frame_id = str(message.header.frame_id or "")
        if frame_id != self._preview_frame:
            self._state.reject("preview_frame_mismatch")
            return
        points = extract_xyz_points(message,
                                    max_input_points=self._max_preview_points)
        self._state.set_preview(points, frame_id=frame_id,
                                stamp=_stamp_seconds(message),
                                wall_time=time.time())
        self._diagnostics(frame_id, len(points))

    def _on_lio_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        self._state.note_lio(float(pose.position.x), float(pose.position.y),
                             _yaw_of(pose.orientation))

    def _on_motion_odom(self, message: Odometry) -> None:
        """Independent fused wheel odometry: the only motion-episode authority."""
        self._motion_odom_seen = True
        pose = message.pose.pose
        twist = message.twist.twist
        event = self._state.note_wheel(
            float(pose.position.x), float(pose.position.y),
            _yaw_of(pose.orientation),
            speed=math.hypot(float(twist.linear.x), float(twist.linear.y)),
            yaw_rate=float(twist.angular.z), now=time.monotonic(),
        )
        gate = self._state.gate
        detail = (f"wheel dxy={gate.wheel_delta[0]:.3f}m dyaw={gate.wheel_delta[1]:.1f}° "
                  f"pslam dxy={gate.lio_delta[0]:.3f}m dyaw={gate.lio_delta[1]:.1f}°")
        if event == "degraded":
            self.get_logger().error(
                f"episode {gate.episode_id}: {self._state.health_reason} [{detail}]")
        elif event == "finished":
            self.get_logger().info(f"episode {gate.episode_id} passed: {detail}")

    def _diagnostics(self, frame_id: str, preview_points: int) -> None:
        """§10.5：周期性打印建图诊断，和 snapshot 字段一致。"""
        self._diagnostics_counter += 1
        if self._diagnostics_counter % self._diagnostics_every:
            return
        state = self._state
        pose = state.lio_pose or (0.0, 0.0, 0.0)
        self.get_logger().info(
            f"map_diag health={state.health} session={state.session_id} "
            f"revision={state.map_revision} source={state.source_map_points} "
            f"cached={len(state.cloud)} preview={preview_points}({frame_id}) "
            f"episode={state.gate.episode_id} "
            f"wheel_dxy={state.gate.wheel_delta[0]:.3f} "
            f"pslam_dxy={state.gate.lio_delta[0]:.3f} "
            f"pose=({pose[0]:.3f},{pose[1]:.3f},{pose[2]:.3f}) "
            f"rejected={state.last_rejected_reason or 'none'}"
        )

    def _write_snapshot(self) -> None:
        self._check_reset_marker()
        payload = self._state.snapshot(now=time.monotonic(),
                                      generated_at=time.time())
        payload["motion_odom_topic"] = self._motion_odom_topic
        payload["motion_odom_seen"] = self._motion_odom_seen
        payload["motion_gate_active"] = bool(
            self._motion_odom_seen or not self._require_motion_odom)
        payload["scan_preview_topic"] = self._scan_topic
        temporary = self._output.with_suffix(self._output.suffix + f".{os.getpid()}.tmp")
        temporary.write_text(self._state.serialize(payload), encoding="utf-8")
        os.replace(temporary, self._output)

    def _reset_marker_stamp(self) -> float:
        if self._reset_marker_path is None:
            return 0.0
        try:
            return self._reset_marker_path.stat().st_mtime
        except OSError:
            return 0.0

    def _check_reset_marker(self) -> None:
        """§9.4：外部 touch 该文件即开启新 mapping session（旧消息全部丢弃）。"""
        if self._reset_marker_path is None:
            return
        mtime = self._reset_marker_stamp()
        if mtime > self._reset_marker_mtime:
            self._state.reset_session(
                stamp=self.get_clock().now().nanoseconds * 1e-9,
                reason="reset_marker")
            self.get_logger().info(
                f"reset marker touched -> mapping session {self._state.session_id}")
        self._reset_marker_mtime = mtime


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--map-topic", default="/go2w/slam/map_3d",
                        help="authoritative optimized map cloud (permanent history)")
    parser.add_argument("--map-info-topic", default="",
                        help="optional JSON String topic carrying the relayed map's "
                             "true SLAM point count")
    parser.add_argument("--map-frame", default="pslam_map",
                        help="canonical fixed world frame of the permanent map")
    parser.add_argument("--scan-topic", default="/go2w/slam/aligned_scan",
                        help="latest scan, preview layer only")
    parser.add_argument("--scan-frame", default="pslam_odom",
                        help="frame the preview scan is published in")
    parser.add_argument("--odom-topic", default="/go2w/slam/odom_base",
                        help="plain_slam/LIO odometry shown as the robot pose")
    parser.add_argument("--motion-odom-topic", default="/go2w/odom/fused",
                        help="independent fused odometry driving the drift gate")
    parser.add_argument("--require-motion-odom", action="store_true",
                        help="report the drift gate as inactive until it appears")
    parser.add_argument("--voxel-size", type=float, default=0.12)
    parser.add_argument("--max-global-voxels", type=int, default=300_000)
    parser.add_argument("--max-web-points", type=int, default=50_000)
    parser.add_argument("--max-preview-points", type=int, default=4_000)
    parser.add_argument("--preview-rate", type=float, default=2.0)
    parser.add_argument("--publish-rate", type=float, default=1.0)
    parser.add_argument("--yaw-rate-max", type=float, default=0.03,
                        help="max |yaw rate| rad/s to consider the robot stationary")
    parser.add_argument("--speed-max", type=float, default=0.02,
                        help="max planar speed m/s to consider the robot stationary")
    parser.add_argument("--settle-seconds", type=float, default=0.5,
                        help="continuous stationary window that closes an episode")
    parser.add_argument("--diagnostics-every", type=int, default=30,
                        help="log map diagnostics every N preview scans")
    parser.add_argument("--reset-marker", default="",
                        help="optional file; touching it starts a new mapping session")
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
