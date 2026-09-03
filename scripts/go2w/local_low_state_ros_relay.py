#!/usr/bin/env python3
"""Publish the captured wheel state into the localhost-only ROS graph."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from unitree_go.msg import LowState


class LocalLowStateRelay(Node):
    def __init__(self, input_path: Path, output_topic: str, max_age: float) -> None:
        super().__init__("go2w_local_low_state_relay")
        self._input_path = input_path
        self._max_age = max_age
        qos = QoSProfile(depth=20, reliability=QoSReliabilityPolicy.RELIABLE)
        self._publisher = self.create_publisher(LowState, output_topic, qos)
        self._timer = self.create_timer(0.02, self._publish_latest)
        self._last_capture_monotonic = 0.0
        self.get_logger().info(
            f"relaying read-only wheel state {input_path} -> {output_topic}"
        )

    def _publish_latest(self) -> None:
        try:
            payload = json.loads(self._input_path.read_text(encoding="utf-8"))
            capture_monotonic = float(payload["capture_monotonic"])
            wheel_q = payload["wheel_q"]
            wheel_dq = payload["wheel_dq"]
            if time.monotonic() - capture_monotonic > self._max_age:
                return
            if capture_monotonic <= self._last_capture_monotonic:
                return
            if len(wheel_q) != 4 or len(wheel_dq) != 4:
                return
            message = LowState()
            for index in range(4):
                message.motor_state[index + 12].q = float(wheel_q[index])
                message.motor_state[index + 12].dq = float(wheel_dq[index])
        except (OSError, ValueError, TypeError, KeyError, IndexError):
            return
        self._last_capture_monotonic = capture_monotonic
        self._publisher.publish(message)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output-topic", default="/go2w/motion/local_lowstate")
    parser.add_argument("--max-age", type=float, default=0.30)
    args = parser.parse_args()
    if args.max_age <= 0.0:
        raise SystemExit("--max-age must be positive")
    rclpy.init(args=[])
    node = LocalLowStateRelay(args.input, args.output_topic, args.max_age)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
