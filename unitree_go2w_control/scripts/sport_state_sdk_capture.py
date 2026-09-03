#!/usr/bin/env python3
"""Read-only Unitree SDK sport-state capture for the Foxy motion stack.

The Jetson's Foxy ROS CycloneDDS participant is intentionally kept
localhost-only because its network SPDP path can crash. The Unitree SDK
participant can still read the DCU's bare-DDS state on eth0. This process
bridges only the small state fields needed by the motion safety monitor via an
atomic JSON file; it never creates a Sport client and never sends commands.
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

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_


def _atomic_write(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, separators=(",", ":"))
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _state_payload(message: SportModeState_) -> dict[str, Any]:
    return {
        "capture_monotonic": time.monotonic(),
        "capture_wall_time": time.time(),
        "stamp_sec": int(message.stamp.sec),
        "stamp_nanosec": int(message.stamp.nanosec),
        "error_code": int(message.error_code),
        "mode": int(message.mode),
        "position": [float(value) for value in message.position],
        "velocity": [float(value) for value in message.velocity],
        "yaw_speed": float(message.yaw_speed),
        "imu_rpy": [float(value) for value in message.imu_state.rpy],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", required=True)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--write-period", type=float, default=0.05)
    args = parser.parse_args()
    if args.write_period <= 0.0:
        raise SystemExit("--write-period must be positive")

    stop = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stop.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    lock = threading.Lock()
    latest: dict[str, Any] | None = None

    def on_state(message: SportModeState_) -> None:
        nonlocal latest
        with lock:
            latest = _state_payload(message)

    network = os.environ.get("GO2W_ROBOT_HOST_IP", args.interface)
    ChannelFactoryInitialize(0, network)
    subscriber = ChannelSubscriber("rt/lf/sportmodestate", SportModeState_)
    subscriber.Init(on_state, 100)
    try:
        while not stop.wait(args.write_period):
            with lock:
                payload = dict(latest) if latest is not None else None
            if payload is not None:
                _atomic_write(args.output, payload)
    finally:
        subscriber.Close()
        with lock:
            payload = dict(latest) if latest is not None else None
        if payload is not None:
            _atomic_write(args.output, payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
