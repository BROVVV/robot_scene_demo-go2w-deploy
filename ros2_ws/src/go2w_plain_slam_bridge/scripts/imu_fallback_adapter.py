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

"""
Clock-safe IMU adapter with a mapping-only stationary fallback.

The Unitree LiDAR clock can retain its boot-time epoch after the DCU wall clock
is corrected. Pandar scans use the host clock, so feeding raw ``/utlidar/imu``
stamps to plain_slam can put the streams hundreds of seconds apart. Real IMU
samples are republished on ``/go2w/slam/imu`` with the sensor timestamp
preserved when possible, or a session-locked receive-sensor offset when the
clocks differ (计划书 §8.4: estimated in a startup window, then frozen, so
callback jitter during motion can no longer drag the IMU time axis).

Backward, duplicate and jumped stamps stop the stream and require a new mapping
session instead of silently re-aligning.

If the real stream is absent, a clearly marked static IMU is published only on
the mapping-assist output. It never enters motion, fused odometry, or safety.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
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

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
from imu_timestamp_aligner import ImuTimestampAligner  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--imu-topic", default="/utlidar/imu")
    parser.add_argument("--output-topic", default="/go2w/slam/imu")
    parser.add_argument("--status-topic", default="/go2w/slam/imu_status")
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--grace-s", type=float, default=2.0)
    parser.add_argument("--passthrough-tolerance-s", type=float, default=5.0)
    parser.add_argument("--offset-lock-samples", type=int, default=500,
                        help="startup samples used to estimate and then lock "
                             "the sensor->ROS clock offset (500 Hz IMU: ~1 s)")
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
    state = {"last_real": 0.0, "mode": "", "last_status": 0.0,
             "valid_real": 0, "invalid_real": 0, "diagnostics": {},
             "restart_logged": False}
    aligner = ImuTimestampAligner(
        passthrough_tolerance_s=args.passthrough_tolerance_s,
        lock_after_samples=args.offset_lock_samples,
    )

    def ros_now_seconds() -> float:
        value = node.get_clock().now()
        nanoseconds = getattr(value, "nanoseconds", None)
        if nanoseconds is not None:
            return float(nanoseconds) * 1e-9
        stamp = value.to_msg()
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def ros_stamp(seconds: float):
        stamp = Header().stamp
        whole = int(seconds)
        stamp.sec = whole
        stamp.nanosec = int(round((seconds - whole) * 1e9))
        if stamp.nanosec >= 1_000_000_000:
            stamp.sec += 1
            stamp.nanosec -= 1_000_000_000
        return stamp

    def publish_mode(mode: str, *, force: bool = False) -> None:
        # 计划书 §8.4：原始 stamp、修正 stamp、offset、锁定状态和模式进诊断。
        now = time.monotonic()
        if force or mode != state["mode"] or now - state["last_status"] >= 1.0:
            message = String()
            message.data = json.dumps(
                {"mode": mode, **state["diagnostics"]}, ensure_ascii=False)
            status_pub.publish(message)
            state["mode"] = mode
            state["last_status"] = now

    def on_real_imu(message: Imu) -> None:
        was_stale = time.monotonic() - state["last_real"] >= 1.0
        state["last_real"] = time.monotonic()
        raw_stamp = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9
        result = aligner.update(raw_stamp, ros_now_seconds())
        state["diagnostics"] = {
            "raw_sensor_sec": round(result.raw_sensor_sec, 6),
            "corrected_sec": (round(result.corrected_sec, 6)
                              if result.corrected_sec is not None else None),
            "offset_sec": round(result.estimated_offset_sec, 6),
            "offset_locked": result.offset_locked,
            "offset_samples": result.offset_samples,
            "offset_drift_sec": round(result.offset_drift_sec, 6),
            "restart_required": result.restart_required,
            "valid": result.valid,
            "reason": result.reason,
        }
        if not result.valid or result.corrected_sec is None:
            state["invalid_real"] += 1
            publish_mode(result.mode)
            if result.restart_required and not state["restart_logged"]:
                state["restart_logged"] = True
                node.get_logger().error(
                    f"IMU clock unusable: {result.reason}; no samples are sent "
                    f"to LIO until the mapping session is restarted")
            return
        # 只改 header stamp，其余字段原样转发（500 Hz 上不做整包深拷贝）。
        message.header.stamp = ros_stamp(result.corrected_sec)
        imu_pub.publish(message)
        state["valid_real"] += 1
        publish_mode(result.mode)
        if was_stale and state["mode"]:
            node.get_logger().info(
                f"real IMU active: {args.imu_topic} -> {args.output_topic} "
                f"({result.mode}, offset={result.estimated_offset_sec:.3f}s, "
                f"locked={result.offset_locked})"
            )

    node.create_subscription(Imu, args.imu_topic, on_real_imu, input_qos)

    period = 1.0 / max(1.0, args.rate)
    deadline = time.monotonic() + args.grace_s
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if state["last_real"] > 0.0:
        publish_mode(state["mode"] or "IMU_SOURCE_INVALID_STAMP", force=True)
        node.get_logger().info(
            f"real IMU detected: {args.imu_topic} -> {args.output_topic} "
            f"mode={state['mode']} valid={state['valid_real']} invalid={state['invalid_real']}"
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
            publish_mode(state["mode"])
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
