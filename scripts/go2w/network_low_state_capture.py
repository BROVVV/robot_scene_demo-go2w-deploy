#!/usr/bin/env python3
"""Capture network ROS LowState for the localhost-only motion graph.

The Jetson's safety action server intentionally uses a localhost-only Foxy
participant.  This process runs in the network ROS graph and copies only the
read-only wheel encoder fields to an atomically replaced JSON file.  It has no
Unitree command client and never publishes a motion or low-level control
message.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import tempfile
import threading
import time
from pathlib import Path
from typing import Any

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
        self._lock = threading.Lock()
        self._latest: dict[str, Any] | None = None
        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(LowState, topic, self._on_state, qos)
        self.get_logger().info(f"capturing read-only {topic} -> {output}")

    def _on_state(self, message: LowState) -> None:
        payload = {
            "capture_monotonic": time.monotonic(),
            "capture_wall_time": time.time(),
            "wheel_q": [float(message.motor_state[i].q) for i in range(12, 16)],
            "wheel_dq": [float(message.motor_state[i].dq) for i in range(12, 16)],
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
