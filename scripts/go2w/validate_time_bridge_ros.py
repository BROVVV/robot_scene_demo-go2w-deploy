#!/usr/bin/env python3
"""Read-only live acceptance check for the Go2-W sensor time bridge."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2


def seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


class Validator(Node):
    def __init__(self) -> None:
        super().__init__("go2w_time_bridge_validator")
        self.messages = {
            "source_cloud": [],
            "raw_cloud": [],
            "aligned_cloud": [],
            "source_imu": [],
            "raw_imu": [],
            "aligned_imu": [],
        }
        subscriptions = (
            (PointCloud2, "/utlidar/cloud", "source_cloud"),
            (PointCloud2, "/go2w/lio_input/cloud_raw", "raw_cloud"),
            (PointCloud2, "/go2w/sensors/cloud", "aligned_cloud"),
            (Imu, "/utlidar/imu", "source_imu"),
            (Imu, "/go2w/lio_input/imu_raw", "raw_imu"),
            (Imu, "/go2w/sensors/lidar_imu", "aligned_imu"),
        )
        self.subscriptions_keepalive = []
        for message_type, topic, key in subscriptions:
            self.subscriptions_keepalive.append(
                self.create_subscription(
                    message_type,
                    topic,
                    lambda message, key=key: self.messages[key].append(message),
                    qos_profile_sensor_data,
                )
            )


def stamp_key(message) -> tuple[int, int]:
    return message.header.stamp.sec, message.header.stamp.nanosec


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    rclpy.init()
    node = Validator()
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(len(items) >= 5 for items in node.messages.values()):
                break

        counts = {key: len(value) for key, value in node.messages.items()}
        errors = []
        if not all(value >= 5 for value in counts.values()):
            errors.append("fewer_than_5_messages_on_one_or_more_topics")

        source_cloud = {stamp_key(item): item for item in node.messages["source_cloud"]}
        raw_matches = 0
        for raw in node.messages["raw_cloud"]:
            source = source_cloud.get(stamp_key(raw))
            if source is not None and bytes(source.data) == bytes(raw.data):
                raw_matches += 1
        if raw_matches < 5:
            errors.append("raw_cloud_not_preserved")

        aligned_checks = 0
        fit = config["cloud"]
        for aligned, raw in zip(
            node.messages["aligned_cloud"], node.messages["raw_cloud"]
        ):
            expected = fit["scale"] * seconds(raw.header.stamp) + fit["offset_seconds"]
            if (
                abs(seconds(aligned.header.stamp) - expected) <= 2e-6
                and bytes(aligned.data) == bytes(raw.data)
                and aligned.fields == raw.fields
            ):
                aligned_checks += 1
        if aligned_checks < 5:
            errors.append("aligned_cloud_stamp_or_payload_mismatch")

        now = node.get_clock().now().nanoseconds / 1e9
        aligned_ages = [
            abs(now - seconds(item.header.stamp))
            for item in node.messages["aligned_cloud"][-5:]
        ]
        if aligned_ages and max(aligned_ages) > 2.0:
            errors.append("aligned_cloud_not_near_host_ros_time")

        result = {
            "passed": not errors,
            "errors": errors,
            "counts": counts,
            "raw_cloud_exact_payload_matches": raw_matches,
            "aligned_cloud_exact_checks": aligned_checks,
            "maximum_aligned_cloud_age_seconds": max(aligned_ages, default=None),
            "motion_topics_published": [],
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result, indent=2))
        return 0 if not errors else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
