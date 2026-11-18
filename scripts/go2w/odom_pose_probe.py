#!/usr/bin/env python3
"""读取一次 odom 位姿并打印 JSON（x, y, yaw, frame）。

用途：闭环/归位控制需要一个稳定的位姿读数源。SLAM 的 ``/go2w/slam/odom_base``
在回环优化时会跳变，因此**运动控制**应优先用轮式 ``/go2w/odom/fused``；
本脚本对两者都适用（订阅用 BEST_EFFORT，对 RELIABLE/BEST_EFFORT 发布者都兼容）。

用法（控制主机，先 source scripts/go2w/setup_environment.sh）：

    /usr/bin/python3 scripts/go2w/odom_pose_probe.py /go2w/odom/fused
"""

from __future__ import annotations

import json
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


def yaw_of(quaternion) -> float:
    return math.atan2(2.0 * (quaternion.z * quaternion.w),
                      1.0 - 2.0 * (quaternion.z * quaternion.z))


class PoseProbe(Node):
    def __init__(self, topic: str) -> None:
        # 节点名带 pid：多实例并存时同名节点会让订阅静默失效（实测）。
        super().__init__(f"odom_pose_probe_{os.getpid()}")
        self.message = None
        qos = QoSProfile(depth=10,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(Odometry, topic, self._on_odom, qos)

    def _on_odom(self, message) -> None:
        self.message = message


def probe(topic: str, timeout_s: float = 20.0, attempts: int = 3):
    for _ in range(attempts):
        node = PoseProbe(topic)
        deadline = time.time() + timeout_s
        while node.message is None and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
        if node.message is not None:
            pose = node.message.pose.pose
            result = {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "yaw": yaw_of(pose.orientation),
                "frame": node.message.header.frame_id,
            }
            node.destroy_node()
            return result
        node.destroy_node()
        time.sleep(1.0)
    return {"error": "no odom message"}


def main() -> int:
    topic = sys.argv[1] if len(sys.argv) > 1 else "/go2w/odom/fused"
    rclpy.init()
    try:
        result = probe(topic)
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
    print(json.dumps(result))
    return 0 if "error" not in result else 1


if __name__ == "__main__":
    raise SystemExit(main())
