#!/usr/bin/env python3
"""Verify LiDAR freshness closes after the owned host input bridge stops."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class FreshnessWatcher(Node):
    def __init__(self) -> None:
        super().__init__("go2w_lidar_freshness_timeout_validator")
        self.started = time.monotonic()
        self.samples: list[dict] = []
        self.create_subscription(
            Bool, "/go2w/safety/lidar_fresh", self._freshness, 10
        )

    def _freshness(self, message: Bool) -> None:
        self.samples.append(
            {
                "elapsed_seconds": time.monotonic() - self.started,
                "fresh": bool(message.data),
            }
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--configured-timeout-seconds", type=float, default=0.3)
    parser.add_argument("--maximum-observed-seconds", type=float, default=0.6)
    parser.add_argument("--collection-seconds", type=float, default=1.2)
    args = parser.parse_args()

    rclpy.init()
    node = FreshnessWatcher()
    deadline = time.monotonic() + args.collection_seconds
    try:
        while (
            rclpy.ok()
            and time.monotonic() < deadline
            and (len(node.samples) < 8 or not any(not item["fresh"] for item in node.samples))
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
        false_samples = [item for item in node.samples if not item["fresh"]]
        first_false = false_samples[0]["elapsed_seconds"] if false_samples else None
        passed = bool(false_samples) and first_false <= args.maximum_observed_seconds
        passed = passed and all(not item["fresh"] for item in node.samples[-3:])
        payload = {
            "schema_version": "1.0",
            "validation_type": "lidar_freshness_after_owned_host_bridge_group_stop",
            "robot_motion_commanded": False,
            "configured_timeout_seconds": args.configured_timeout_seconds,
            "maximum_observed_seconds": args.maximum_observed_seconds,
            "samples": node.samples,
            "first_false_elapsed_seconds": first_false,
            "passed": passed,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 0 if passed else 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
