#!/usr/bin/env python3
"""Require two live ROS messages with sensor QoS (Foxy and Humble)."""

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("topic")
    parser.add_argument("timeout_seconds", type=float)
    args = parser.parse_args()
    if args.timeout_seconds <= 0:
        parser.error("timeout_seconds must be positive")

    rclpy.init()
    node = rclpy.create_node("go2w_topic_flow_probe")
    deadline = time.monotonic() + args.timeout_seconds
    received = 0
    subscription = None

    def on_message(_message: object) -> None:
        nonlocal received
        received += 1

    try:
        while rclpy.ok() and time.monotonic() < deadline and received < 2:
            if subscription is None:
                types = [types for topic, types in node.get_topic_names_and_types()
                         if topic == args.topic]
                if types and len(types[0]) == 1:
                    message_class = get_message(types[0][0])
                    subscription = node.create_subscription(
                        message_class, args.topic, on_message, qos_profile_sensor_data
                    )
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if received >= 2:
        print("received 2 messages from " + args.topic)
        return 0
    print("no live flow from " + args.topic + " (received " + str(received) + ")")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
