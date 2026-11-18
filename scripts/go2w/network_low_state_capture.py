#!/usr/bin/env python3
"""Capture network ROS LowState for the localhost-only motion graph.

The Jetson's safety action server intentionally uses a localhost-only Foxy
participant.  This process runs in the network ROS graph and copies the
read-only 16-motor posture plus IMU fields to an atomically replaced JSON
file.  The legacy four-wheel fields remain in the payload for existing motion
consumers.  It has no Unitree command client and never publishes a motion or
low-level control message.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import tempfile
import threading
import time
from pathlib import Path
from typing import Any

# An explicitly selected, natively built backend replaces only this process.
# Imports for reference/tests retain the Python implementation below.
if __name__ == "__main__" and os.environ.get("GO2W_NATIVE_LOW_STATE_CAPTURE"):
    native = Path(os.environ["GO2W_NATIVE_LOW_STATE_CAPTURE"]).expanduser().resolve(strict=True)
    if not native.is_file() or not os.access(native, os.X_OK):
        raise SystemExit("Native LowState capture is not executable")
    os.execv(str(native), [str(native), *sys.argv[1:]])

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unitree_go.msg import LowState


def _atomic_write(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, separators=(",", ":"))
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


class NetworkLowStateCapture(Node):
    def __init__(self, output: Path, topic: str) -> None:
        super().__init__("go2w_network_low_state_capture")
        self._output = output
        self._topic = topic
        self._lock = threading.Lock()
        self._latest: dict[str, Any] | None = None
        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(LowState, topic, self._on_state, qos)
        self.get_logger().info(f"capturing read-only {topic} -> {output}")

    def _on_state(self, message: LowState) -> None:
        motor_q = [float(message.motor_state[i].q) for i in range(16)]
        motor_dq = [float(message.motor_state[i].dq) for i in range(16)]
        payload = {
            "schema": "go2w.network_low_state.v2",
            "source_topic": self._topic,
            "capture_monotonic": time.monotonic(),
            "capture_wall_time": time.time(),
            "motor_q": motor_q,
            "motor_dq": motor_dq,
            "wheel_q": motor_q[12:16],
            "wheel_dq": motor_dq[12:16],
            "imu_quaternion": [float(value) for value in message.imu_state.quaternion],
            "imu_gyroscope": [float(value) for value in message.imu_state.gyroscope],
            "imu_accelerometer": [float(value) for value in message.imu_state.accelerometer],
            "imu_rpy": [float(value) for value in message.imu_state.rpy],
            "imu_temperature": int(message.imu_state.temperature),
            "battery_soc": int(message.bms_state.soc),
        }
        with self._lock:
            self._latest = payload
        _atomic_write(self._output, payload)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--topic", default="/lf/lowstate")
    args = parser.parse_args()

    stop = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stop.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    rclpy.init(args=[])
    node = NetworkLowStateCapture(args.output, args.topic)
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
