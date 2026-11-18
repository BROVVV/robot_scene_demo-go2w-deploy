#!/usr/bin/env python3
"""Verify the LIO adapter closes after its owned sensor bridge stops."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.node import Node


def diagnostic_level(value) -> int:
    return int.from_bytes(value, "little") if isinstance(value, bytes) else int(value)


class StaleWatcher(Node):
    def __init__(self) -> None:
        super().__init__("go2w_lio_stale_timeout_validator")
        self.started = time.monotonic()
        self.odom_events: list[float] = []
        self.status_events: list[dict] = []
        self.create_subscription(Odometry, "/lio/odom", self._odom, 10)
        self.create_subscription(DiagnosticArray, "/lio/status", self._status, 10)

    def _elapsed(self) -> float:
        return time.monotonic() - self.started

    def _odom(self, _message: Odometry) -> None:
        self.odom_events.append(self._elapsed())

    def _status(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name == "go2w_lio/status":
                self.status_events.append(
                    {
                        "elapsed_seconds": self._elapsed(),
                        "level": diagnostic_level(status.level),
                        "message": status.message,
                    }
                )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--configured-timeout-seconds", type=float, default=0.3)
    parser.add_argument("--maximum-observed-seconds", type=float, default=0.7)
    parser.add_argument("--collection-seconds", type=float, default=1.5)
    args = parser.parse_args()

    rclpy.init()
    node = StaleWatcher()
    deadline = time.monotonic() + args.collection_seconds
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        stale = [item for item in node.status_events if item["message"] == "odometry_stale"]
        first_stale = stale[0]["elapsed_seconds"] if stale else None
        odom_after_stale = (
            [value for value in node.odom_events if value > first_stale]
            if first_stale is not None
            else node.odom_events
        )
        passed = bool(stale) and first_stale <= args.maximum_observed_seconds
        passed = passed and not odom_after_stale
        passed = passed and len(node.status_events) >= 5
        passed = passed and all(
            item["message"] == "odometry_stale" for item in node.status_events[-3:]
        )
        payload = {
            "schema_version": "1.0",
            "validation_type": "lio_stale_after_owned_sensor_bridge_group_stop",
            "robot_motion_commanded": False,
            "configured_timeout_seconds": args.configured_timeout_seconds,
            "maximum_observed_seconds": args.maximum_observed_seconds,
            "first_stale_elapsed_seconds": first_stale,
            "odometry_events": node.odom_events,
            "odometry_events_after_first_stale": odom_after_stale,
            "status_events": node.status_events,
            "passed": passed,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
        return 0 if passed else 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
