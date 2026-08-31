#!/usr/bin/python3
# Copyright 2026 robot_scene_demo maintainers
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Host-clock IMU adapter with a mapping-only stationary fallback.

The Unitree LiDAR clock can retain its boot-time epoch after the DCU wall clock
is corrected. Pandar scans use the host clock, so feeding raw ``/utlidar/imu``
stamps to plain_slam can put the streams hundreds of seconds apart. Real IMU
samples are therefore republished on ``/go2w/slam/imu`` with their host receive
timestamp. Values and frame identity are preserved.

If the real stream is absent, a clearly marked static IMU is published only on
the mapping-assist output. It never enters motion, fused odometry, or safety.
"""

from __future__ import annotations

import argparse
import copy
import sys
import time

import rclpy
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu
from std_msgs.msg import Header, String


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--imu-topic", default="/utlidar/imu")
    parser.add_argument("--output-topic", default="/go2w/slam/imu")
    parser.add_argument("--status-topic", default="/go2w/slam/imu_status")
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--grace-s", type=float, default=2.0)
    # launch_ros appends ROS arguments to the executable command.
    raw = sys.argv[1:]
    if "--ros-args" in raw:
        raw = raw[: raw.index("--ros-args")]
    args = parser.parse_args(raw)

    rclpy.init(args=sys.argv)
    node = rclpy.create_node("imu_fallback_adapter")

    input_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )
    output_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=100,
    )
    imu_pub = node.create_publisher(Imu, args.output_topic, output_qos)
    status_pub = node.create_publisher(String, args.status_topic, 10)
    state = {"last_real": 0.0, "mode": "", "last_status": 0.0}

    def publish_mode(mode: str, *, force: bool = False) -> None:
        now = time.monotonic()
        if force or mode != state["mode"] or now - state["last_status"] >= 1.0:
            message = String()
            message.data = mode
            status_pub.publish(message)
            state["mode"] = mode
            state["last_status"] = now

    def on_real_imu(message: Imu) -> None:
        was_stale = time.monotonic() - state["last_real"] >= 1.0
        state["last_real"] = time.monotonic()
        aligned = copy.deepcopy(message)
        aligned.header.stamp = node.get_clock().now().to_msg()
        imu_pub.publish(aligned)
        publish_mode("IMU_SOURCE_OK_HOST_STAMP")
        if was_stale and state["mode"]:
            node.get_logger().info(
                f"real IMU active: {args.imu_topic} -> {args.output_topic} "
                "(host receive timestamp)"
            )

    node.create_subscription(Imu, args.imu_topic, on_real_imu, input_qos)

    period = 1.0 / max(1.0, args.rate)
    deadline = time.monotonic() + args.grace_s
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if state["last_real"] > 0.0:
        publish_mode("IMU_SOURCE_OK_HOST_STAMP", force=True)
        node.get_logger().info(
            f"real IMU detected: {args.imu_topic} -> {args.output_topic} "
            "with host receive timestamps"
        )
    else:
        publish_mode("SYNTHETIC_STATIC", force=True)
        node.get_logger().warn(
            f"{args.imu_topic} has no real data; publishing synthetic static "
            f"IMU on {args.output_topic} (mapping-assist only; robot must "
            "stay stationary)"
        )

    while rclpy.ok():
        # Real samples are republished directly by the callback.
        rclpy.spin_once(node, timeout_sec=period)
        if time.monotonic() - state["last_real"] < 1.0:
            publish_mode("IMU_SOURCE_OK_HOST_STAMP")
            continue

        if state["mode"] != "SYNTHETIC_STATIC":
            node.get_logger().warn(
                f"{args.imu_topic} became stale; enabling synthetic static "
                f"fallback on {args.output_topic}"
            )
        publish_mode("SYNTHETIC_STATIC")
        message = Imu()
        message.header = Header(
            frame_id="pslam_imu_synthetic_static",
            stamp=node.get_clock().now().to_msg(),
        )
        message.linear_acceleration.z = 9.81
        message.orientation.w = 1.0
        imu_pub.publish(message)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
