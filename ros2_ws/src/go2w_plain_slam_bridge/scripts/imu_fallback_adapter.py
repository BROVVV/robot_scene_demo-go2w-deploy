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

``/utlidar/imu`` reports the yaw rate with the opposite sign to the robot base,
so the yaw axis is negated before republishing. Measured 2026-09-02 on the live
robot, two turns in both directions (``/tmp/diag_signs.csv``):

* ``MODE_RELATIVE_YAW +30`` -> wheel odom ``+29.25``, action result
  ``actual_relative_yaw_deg +29.28``, raw gyro-z integral ``-28.51``;
* ``MODE_RELATIVE_YAW -30`` -> wheel odom ``-28.20``, action result ``-28.19``,
  raw gyro-z integral ``+27.38``.

Magnitudes agree within 3% while the sign is inverted every time. The
accelerometer is *not* inverted: stationary gravity reads ``(+2.76, -0.03,
+9.88)``, i.e. z-up and pitched ``atan2(2.76, 9.88) = 15.6 deg``, which matches
the ``imu_to_lidar`` extrinsic. With the pitch known, the theoretical gyro
projection of a ``+29.25`` base turn is ``(+7.92, 0, +28.15)``; the measurement
is ``(+7.9, -5.1, -28.5)``, so only the yaw axis needs flipping (the y term is
the body roll of a quadruped mid-turn). Feeding the uncorrected yaw rate makes
the LIO attitude walk the wrong way and scan matching absorbs the residual
rotation as translation -- the ``1.2-5.8 m`` phantom translation seen during
in-place turns.

If the real stream is absent, a clearly marked static IMU is published only on
the mapping-assist output. It never enters motion, fused odometry, or safety.
Its gravity vector reuses the last real sample rather than an idealised
``(0, 0, 9.81)``: the lidar IMU sits at 15.6 deg pitch, so the idealised vector
hands LIO a 2.7 m/s^2 horizontal residual the moment the fallback engages and
the position estimate diverges as ``0.5at^2``. Measured 2026-09-02 15:03 --
``/utlidar/imu`` went stale during a reboot, LIO drifted to 25 km and
``slam_3d_node`` aborted in ``SO3::exp`` on the resulting nan pose.
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
                             "the sensor->ROS clock offset (248 Hz IMU: ~2 s)")
    parser.add_argument("--max-offset-step-s", type=float, default=0.05,
                        help="post-lock clock-step tolerance; a step larger than "
                             "this re-locks the offset on the startup window. Must "
                             "stay below the consumer's tolerance: LIO only takes "
                             "IMU inside the scan's own span (measured p50 0.099 s, "
                             "minus the -25 ms stamp bias = ~0.075 s budget), so a "
                             "larger threshold lets a step silently freeze the IMU "
                             "axis. Measured arrival jitter is p50 4.482 ms / max "
                             "8.994 ms, so 0.05 s has 5.5x headroom")
    parser.add_argument("--gyro-sign", default="1,1,-1",
                        help="per-axis angular_velocity sign correction; the "
                             "Go2-W lidar IMU reports yaw rate inverted")
    parser.add_argument("--bias-lock-samples", type=int, default=500,
                        help="startup samples used to estimate and then lock "
                             "the gyro bias; the robot MUST be stationary for "
                             "these samples (0 disables bias removal)")
    parser.add_argument("--stamp-offset-s", type=float, default=0.0,
                        help="constant offset added to the published IMU stamp; "
                             "used to test the measured LiDAR-IMU skew "
                             "(0 = current behaviour)")
    # launch_ros appends ROS arguments to the executable command.
    raw = sys.argv[1:]
    if "--ros-args" in raw:
        raw = raw[: raw.index("--ros-args")]
    args = parser.parse_args(raw)
    gyro_sign = tuple(float(value) for value in args.gyro_sign.split(","))

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
             "restart_logged": False, "last_accel": None,
             "bias_sum": [0.0, 0.0, 0.0], "bias_samples": 0,
             "gyro_bias": None}
    aligner = ImuTimestampAligner(
        passthrough_tolerance_s=args.passthrough_tolerance_s,
        lock_after_samples=args.offset_lock_samples,
        max_offset_step_s=args.max_offset_step_s,
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
            "gyro_bias": ([round(value, 6) for value in state["gyro_bias"]]
                          if state["gyro_bias"] is not None else None),
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
        # 只改 header stamp、陀螺符号与零偏，其余字段原样转发（500 Hz 上不做整包深拷贝）。
        message.header.stamp = ros_stamp(
            result.corrected_sec + args.stamp_offset_s)
        gyro = [message.angular_velocity.x * gyro_sign[0],
                message.angular_velocity.y * gyro_sign[1],
                message.angular_velocity.z * gyro_sign[2]]
        # 计划书 §26.2：静止时实测 gyro-x 仍有 0.0112 rad/s 零偏，20 s 即积出
        # 0.22 rad roll 漂移，是旋转期墙面错位（Dataset A overlap 0.133 m）的首要嫌疑。
        # 与时间偏移同一套机制：启动窗口估计后锁定，运动期间绝不再改。
        if state["gyro_bias"] is None and args.bias_lock_samples > 0:
            for axis in range(3):
                state["bias_sum"][axis] += gyro[axis]
            state["bias_samples"] += 1
            if state["bias_samples"] >= args.bias_lock_samples:
                state["gyro_bias"] = [value / state["bias_samples"]
                                      for value in state["bias_sum"]]
                node.get_logger().info(
                    "gyro bias locked: [%.5f, %.5f, %.5f] rad/s (%d samples)"
                    % (state["gyro_bias"][0], state["gyro_bias"][1],
                       state["gyro_bias"][2], state["bias_samples"]))
        if state["gyro_bias"] is not None:
            for axis in range(3):
                gyro[axis] -= state["gyro_bias"][axis]
        message.angular_velocity.x = gyro[0]
        message.angular_velocity.y = gyro[1]
        message.angular_velocity.z = gyro[2]
        imu_pub.publish(message)
        state["valid_real"] += 1
        state["last_accel"] = (message.linear_acceleration.x,
                               message.linear_acceleration.y,
                               message.linear_acceleration.z)
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
        # 重力向量沿用最后一帧真实 IMU。雷达 IMU 静止时读 (+2.7, 0.0, +9.9)，
        # 即 15.6° pitch；发理想的 (0, 0, 9.81) 会让 LIO 在切换瞬间看到
        # 2.7 m/s² 的水平加速度残差，位置按 0.5at² 发散（实测 290 s 飞到 25 km）。
        accel = state["last_accel"] or (0.0, 0.0, 9.81)
        message.linear_acceleration.x = accel[0]
        message.linear_acceleration.y = accel[1]
        message.linear_acceleration.z = accel[2]
        message.orientation.w = 1.0
        imu_pub.publish(message)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
