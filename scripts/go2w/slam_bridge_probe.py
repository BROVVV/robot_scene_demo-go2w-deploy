#!/usr/bin/env python3
"""SLAMBridge 真机探针（20260908 计划书 §5.1 / §10 / §31）。

订阅 ``/tf`` 的 ``map → odom`` 与 SLAM 的 ``odom → base``，喂给 :class:`SLAMBridge`，
然后按"图像拍摄时刻"查询 ``T_map_base`` / ``T_map_camera`` 并打印一条测试投影。
只读，不发任何运动命令。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage

from app.slam.slam_bridge import SLAMBridge, yaw_from_quaternion


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class SlamBridgeProbe(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("slam_bridge_probe")
        self.bridge = SLAMBridge(
            map_frame=args.map_frame,
            odom_frame=args.odom_frame,
            camera_frame=args.camera_frame,
            buffer_seconds=args.buffer_seconds,
        )
        self.test_point = tuple(float(v) for v in args.test_camera_point.split(","))
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )
        self.create_subscription(TFMessage, "/tf", self._on_tf, qos)
        self.create_subscription(Odometry, args.odom_topic, self._on_odom, qos)
        self.seen_frames: dict[str, int] = {}

    def _on_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            key = f"{parent}->{child}"
            self.seen_frames[key] = self.seen_frames.get(key, 0) + 1
            if parent == self.bridge.map_frame and child == self.bridge.odom_frame:
                translation = transform.transform.translation
                self.bridge.note_map_to_odom(
                    stamp_seconds(transform.header.stamp),
                    (translation.x, translation.y,
                     yaw_from_quaternion(transform.transform.rotation)),
                )

    def _on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.bridge.note_odom_to_base(
            stamp_seconds(message.header.stamp),
            (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)),
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-frame", default="pslam_map")
    parser.add_argument("--odom-frame", default="pslam_odom")
    parser.add_argument("--camera-frame", default="d435_color_optical_frame")
    parser.add_argument("--odom-topic", default="/go2w/slam/odom_base")
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--buffer-seconds", type=float, default=30.0)
    parser.add_argument("--test-camera-point", default="0.0,0.0,2.0")
    args = parser.parse_args()

    rclpy.init()
    node = SlamBridgeProbe(args)
    deadline = time.monotonic() + max(1.0, args.seconds)
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    status = node.bridge.status()
    latest = node.bridge.robot_pose_map()
    projected = node.bridge.camera_point_to_map(node.test_point)
    report = {
        "status": status,
        "tf_frames_seen": dict(sorted(node.seen_frames.items(),
                                      key=lambda item: -item[1])[:12]),
        "robot_pose_map": (None if latest is None
                           else [round(v, 4) for v in latest]),
        "test_camera_point": list(node.test_point),
        "test_point_in_map": (None if projected is None
                              else [round(v, 4) for v in projected]),
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    ok = status["map_to_odom_samples"] > 0 and status["odom_to_base_samples"] > 0
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
