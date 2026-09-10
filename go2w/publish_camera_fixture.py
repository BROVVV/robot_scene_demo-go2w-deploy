#!/usr/bin/env python3
"""Publish a saved JPEG as Go2FrontVideoData for offline bridge testing."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from unitree_go.msg import Go2FrontVideoData


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True, type=Path)
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--hz", type=float, default=5.0)
    args = parser.parse_args()
    payload = args.image.read_bytes()
    if payload[:2] != b"\xff\xd8" or payload[-2:] != b"\xff\xd9":
        raise SystemExit("fixture is not a complete JPEG")
    rclpy.init()
    node = Node("go2w_camera_fixture_publisher")
    qos = QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )
    publisher = node.create_publisher(
        Go2FrontVideoData, "/frontvideostream", qos
    )
    try:
        # Allow DDS endpoint discovery before the first sample.
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        period = 1.0 / max(0.1, args.hz)
        for sequence in range(args.count):
            message = Go2FrontVideoData()
            message.time_frame = sequence + 1
            message.video720p = payload
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
