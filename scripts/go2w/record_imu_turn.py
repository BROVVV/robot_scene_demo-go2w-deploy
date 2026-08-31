#!/usr/bin/env python3
"""Record raw lidar IMU + robot state during a small motion trial (read-only).

Subscribes to the raw DDS topics and writes JSONL lines with monotonic host
timestamps plus the original message stamps. It never publishes anything.
"""

from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from unitree_go.msg import LidarState, LowState, SportModeState


class ImuTurnRecorder(Node):
    def __init__(self, output: str) -> None:
        super().__init__("imu_turn_recorder")
        self._output = open(output, "a", encoding="utf-8")
        self._start = time.monotonic()
        self._imu = self.create_subscription(
            Imu, "/utlidar/imu", self._on_imu, qos_profile_sensor_data
        )
        self._sport = self.create_subscription(
            SportModeState,
            "/lf/sportmodestate",
            self._on_sport,
            QoSProfile(depth=20, reliability=2),  # BEST_EFFORT
        )
        self._low = self.create_subscription(
            LowState,
            "/lf/lowstate",
            self._on_low,
            QoSProfile(depth=20, reliability=2),  # BEST_EFFORT
        )
        self._lidar_state = self.create_subscription(
            LidarState,
            "/utlidar/lidar_state",
            self._on_lidar_state,
            QoSProfile(depth=10, reliability=2),  # BEST_EFFORT
        )
        self._odom = self.create_subscription(
            Odometry,
            "/lio/odom",
            self._on_odom,
            QoSProfile(depth=50, reliability=2),  # BEST_EFFORT
        )
        self._wheel_odom = self.create_subscription(
            Odometry,
            "/go2w/odom/wheel",
            self._on_wheel_odom,
            QoSProfile(depth=50, reliability=2),  # BEST_EFFORT
        )
        self.get_logger().info(f"recording to {output}")

    def _host_s(self) -> float:
        return round(time.monotonic(), 6)

    def _write(self, row: dict) -> None:
        self._output.write(json.dumps(row, ensure_ascii=False) + "\n")
        self._output.flush()

    def _stamp(self, stamp) -> dict:
        return {
            "sec": int(stamp.sec),
            "nanosec": int(stamp.nanosec),
        }

    def _on_imu(self, msg: Imu) -> None:
        self._write(
            {
                "type": "imu",
                "host_s": self._host_s(),
                "stamp": self._stamp(msg.header.stamp),
                "orientation": [
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w,
                ],
                "angular_velocity": [
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                ],
                "linear_acceleration": [
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ],
            }
        )

    def _on_sport(self, msg: SportModeState) -> None:
        self._write(
            {
                "type": "sport",
                "host_s": self._host_s(),
                "stamp": self._stamp(msg.stamp),
                "error_code": int(msg.error_code),
                "mode": int(msg.mode),
                "position": [float(v) for v in msg.position],
                "velocity": [float(v) for v in msg.velocity],
                "yaw_speed": float(msg.yaw_speed),
                "imu_quat": [float(v) for v in msg.imu_state.quaternion],
                "imu_gyro": [float(v) for v in msg.imu_state.gyroscope],
                "imu_acc": [float(v) for v in msg.imu_state.accelerometer],
                "imu_rpy": [float(v) for v in msg.imu_state.rpy],
            }
        )

    def _on_low(self, msg: LowState) -> None:
        wheels = [
            {
                "q": float(msg.motor_state[i].q),
                "dq": float(msg.motor_state[i].dq),
            }
            for i in range(12, 16)
        ]
        self._write(
            {
                "type": "low",
                "host_s": self._host_s(),
                "tick": int(msg.tick),
                "wheels": wheels,
                "imu_quat": [float(v) for v in msg.imu_state.quaternion],
                "imu_gyro": [float(v) for v in msg.imu_state.gyroscope],
                "imu_acc": [float(v) for v in msg.imu_state.accelerometer],
                "imu_rpy": [float(v) for v in msg.imu_state.rpy],
            }
        )

    def _on_lidar_state(self, msg: LidarState) -> None:
        self._write(
            {
                "type": "lidar_state",
                "host_s": self._host_s(),
                "stamp": float(msg.stamp),
                "software_version": str(msg.software_version),
                "imu_rpy": [float(v) for v in msg.imu_rpy],
                "error_state": int(msg.error_state),
            }
        )

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        twist = msg.twist.twist
        self._write(
            {
                "type": "odom",
                "host_s": self._host_s(),
                "stamp": self._stamp(msg.header.stamp),
                "frame_id": str(msg.header.frame_id),
                "child_frame_id": str(msg.child_frame_id),
                "position": [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                ],
                "orientation": [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ],
                "linear_velocity": [
                    twist.linear.x,
                    twist.linear.y,
                    twist.linear.z,
                ],
                "angular_velocity": [
                    twist.angular.x,
                    twist.angular.y,
                    twist.angular.z,
                ],
            }
        )

    def _on_wheel_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        twist = msg.twist.twist
        self._write(
            {
                "type": "wodom",
                "host_s": self._host_s(),
                "stamp": self._stamp(msg.header.stamp),
                "frame_id": str(msg.header.frame_id),
                "child_frame_id": str(msg.child_frame_id),
                "position": [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                ],
                "orientation": [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ],
                "angular_velocity": [
                    twist.angular.x,
                    twist.angular.y,
                    twist.angular.z,
                ],
            }
        )

    def stop(self) -> None:
        self._output.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--seconds", type=float, default=300.0)
    args = parser.parse_args()
    rclpy.init()
    node = ImuTurnRecorder(args.output)
    try:
        end = time.monotonic() + args.seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.stop()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
