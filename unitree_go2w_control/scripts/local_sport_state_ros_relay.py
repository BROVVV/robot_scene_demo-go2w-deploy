#!/usr/bin/env python3
"""Publish SDK-captured sport state into the localhost-only ROS graph.

This is the ROS half of ``sport_state_sdk_capture.py``. It publishes only
fresh state samples and has no motion client, publisher, service, or action
client other than the read-only state topic.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from unitree_go.msg import SportModeState


class LocalSportStateRelay(Node):
    def __init__(self, input_path: Path, output_topic: str, max_age: float) -> None:
        super().__init__("go2w_local_sport_state_relay")
        self._input_path = input_path
        self._max_age = max_age
        qos = QoSProfile(depth=20, reliability=QoSReliabilityPolicy.RELIABLE)
        self._publisher = self.create_publisher(SportModeState, output_topic, qos)
        self._timer = self.create_timer(0.02, self._publish_latest)
        self._last_capture_monotonic = 0.0
        self.get_logger().info(
            f"relaying read-only sport state {input_path} -> {output_topic}"
        )

    def _publish_latest(self) -> None:
        try:
            payload = json.loads(self._input_path.read_text(encoding="utf-8"))
        except (OSError, ValueError, json.JSONDecodeError):
            return
        try:
            capture_monotonic = float(payload["capture_monotonic"])
            if time.monotonic() - capture_monotonic > self._max_age:
                return
            message = SportModeState()
            message.stamp.sec = int(payload.get("stamp_sec", 0))
            message.stamp.nanosec = int(payload.get("stamp_nanosec", 0))
            message.error_code = int(payload["error_code"])
            message.mode = int(payload["mode"])
            message.position = [float(value) for value in payload["position"]]
            message.velocity = [float(value) for value in payload["velocity"]]
            message.yaw_speed = float(payload["yaw_speed"])
            message.imu_state.rpy = [float(value) for value in payload["imu_rpy"]]
        except (KeyError, TypeError, ValueError, IndexError):
            return
        if capture_monotonic <= self._last_capture_monotonic:
            return
        self._last_capture_monotonic = capture_monotonic
        self._publisher.publish(message)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output-topic", default="/go2w/motion/local_sportmodestate")
    parser.add_argument("--max-age", type=float, default=0.30)
    args = parser.parse_args()
    if args.max_age <= 0.0:
        raise SystemExit("--max-age must be positive")
    rclpy.init()
    node = LocalSportStateRelay(args.input, args.output_topic, args.max_age)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
