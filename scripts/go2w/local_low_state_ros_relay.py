#!/usr/bin/env python3
"""Publish captured read-only posture into the localhost-only ROS graph."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

if __name__ == "__main__" and os.environ.get("GO2W_NATIVE_LOW_STATE_RELAY"):
    native = Path(os.environ["GO2W_NATIVE_LOW_STATE_RELAY"]).expanduser().resolve(strict=True)
    if not native.is_file() or not os.access(native, os.X_OK):
        raise SystemExit("Native LowState relay is not executable")
    os.execv(str(native), [str(native), *sys.argv[1:]])

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
        self._boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.get_logger().info(
            f"relaying read-only motor/IMU state {input_path} -> {output_topic}"
        )

    @staticmethod
    def _finite_vector(payload: dict, key: str, length: int) -> list[float]:
        values = payload[key]
        if not isinstance(values, list) or len(values) != length:
            raise ValueError(f"{key} must contain {length} values")
        result = [float(value) for value in values]
        if not all(math.isfinite(value) for value in result):
            raise ValueError(f"{key} contains a non-finite value")
        return result

    def _publish_latest(self) -> None:
        try:
            payload = json.loads(self._input_path.read_text(encoding="utf-8"))
            if (isinstance(payload["capture_monotonic"], bool)
                    or not isinstance(payload["capture_monotonic"], (int, float))
                    or isinstance(payload["capture_wall_time"], bool)
                    or not isinstance(payload["capture_wall_time"], (int, float))):
                return
            capture_monotonic = float(payload["capture_monotonic"])
            if not math.isfinite(capture_monotonic):
                return
            age = time.monotonic() - capture_monotonic
            wall_age = time.time() - float(payload["capture_wall_time"])
            if not (0 <= age <= self._max_age and 0 <= wall_age <= self._max_age):
                return
            if "capture_boot_id" in payload and payload["capture_boot_id"] != self._boot_id:
                return
            if capture_monotonic <= self._last_capture_monotonic:
                return
            message = LowState()
            if "motor_q" in payload or "motor_dq" in payload:
                motor_q = self._finite_vector(payload, "motor_q", 16)
                motor_dq = self._finite_vector(payload, "motor_dq", 16)
                for index, (q, dq) in enumerate(zip(motor_q, motor_dq)):
                    message.motor_state[index].q = q
                    message.motor_state[index].dq = dq
            else:
                # Backward compatibility while an old v1 capture file is being
                # atomically replaced during a rolling restart.
                wheel_q = self._finite_vector(payload, "wheel_q", 4)
                wheel_dq = self._finite_vector(payload, "wheel_dq", 4)
                for index, (q, dq) in enumerate(zip(wheel_q, wheel_dq)):
                    message.motor_state[index + 12].q = q
                    message.motor_state[index + 12].dq = dq
            if "imu_rpy" in payload:
                message.imu_state.rpy = self._finite_vector(payload, "imu_rpy", 3)
            if "imu_quaternion" in payload:
                message.imu_state.quaternion = self._finite_vector(
                    payload, "imu_quaternion", 4
                )
            if "imu_gyroscope" in payload:
                message.imu_state.gyroscope = self._finite_vector(
                    payload, "imu_gyroscope", 3
                )
            if "imu_accelerometer" in payload:
                message.imu_state.accelerometer = self._finite_vector(
                    payload, "imu_accelerometer", 3
                )
            if "imu_temperature" in payload:
                message.imu_state.temperature = int(payload["imu_temperature"])
            if "battery_soc" in payload:
                message.bms_state.soc = int(payload["battery_soc"])
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
