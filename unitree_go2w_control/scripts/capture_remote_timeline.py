#!/usr/bin/env python3
"""Unified, receive-time-stamped, read-only timeline for posture capture."""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from unitree_api.msg import Request, Response
from unitree_go.msg import LowState, SportModeState, WirelessController


def wall_time() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="microseconds")


def request_dict(msg: Request) -> dict[str, Any]:
    return {
        "header": {
            "identity": {
                "id": int(msg.header.identity.id),
                "api_id": int(msg.header.identity.api_id),
            },
            "lease": {"id": int(msg.header.lease.id)},
            "policy": {
                "priority": int(msg.header.policy.priority),
                "noreply": bool(msg.header.policy.noreply),
            },
        },
        "parameter": msg.parameter,
        "binary": list(msg.binary),
    }


def response_dict(msg: Response) -> dict[str, Any]:
    return {
        "header": {
            "identity": {
                "id": int(msg.header.identity.id),
                "api_id": int(msg.header.identity.api_id),
            },
            "status": {"code": int(msg.header.status.code)},
        },
        "data": msg.data,
        "binary": list(msg.binary),
    }


def wireless_dict(msg: WirelessController) -> dict[str, Any]:
    keys = int(msg.keys) & 0xFFFF
    return {
        "lx": float(msg.lx),
        "ly": float(msg.ly),
        "rx": float(msg.rx),
        "ry": float(msg.ry),
        "keys": keys,
        "keys_hex": f"0x{keys:04x}",
        "set_bits": [bit for bit in range(16) if keys & (1 << bit)],
    }


def sport_state_dict(msg: SportModeState) -> dict[str, Any]:
    return {
        "stamp": {"sec": int(msg.stamp.sec), "nanosec": int(msg.stamp.nanosec)},
        "error_code": int(msg.error_code),
        "mode": int(msg.mode),
        "progress": float(msg.progress),
        "gait_type": int(msg.gait_type),
        "foot_raise_height": float(msg.foot_raise_height),
        "position": [float(v) for v in msg.position],
        "body_height": float(msg.body_height),
        "velocity": [float(v) for v in msg.velocity],
        "yaw_speed": float(msg.yaw_speed),
        "range_obstacle": [float(v) for v in msg.range_obstacle],
        "foot_force": [int(v) for v in msg.foot_force],
        "imu": {
            "quaternion": [float(v) for v in msg.imu_state.quaternion],
            "gyroscope": [float(v) for v in msg.imu_state.gyroscope],
            "accelerometer": [float(v) for v in msg.imu_state.accelerometer],
            "rpy": [float(v) for v in msg.imu_state.rpy],
        },
    }


def low_state_dict(msg: LowState) -> dict[str, Any]:
    return {
        "tick": int(msg.tick),
        "level_flag": int(msg.level_flag),
        "version": [int(v) for v in msg.version],
        "imu": {
            "quaternion": [float(v) for v in msg.imu_state.quaternion],
            "gyroscope": [float(v) for v in msg.imu_state.gyroscope],
            "accelerometer": [float(v) for v in msg.imu_state.accelerometer],
            "rpy": [float(v) for v in msg.imu_state.rpy],
        },
        "motors": [
            {
                "index": index,
                "mode": int(motor.mode),
                "q": float(motor.q),
                "dq": float(motor.dq),
                "tau_est": float(motor.tau_est),
                "temperature": int(motor.temperature),
                "lost": int(motor.lost),
            }
            for index, motor in enumerate(msg.motor_state)
        ],
        "foot_force": [int(v) for v in msg.foot_force],
        "foot_force_est": [int(v) for v in msg.foot_force_est],
        "power_v": float(msg.power_v),
        "power_a": float(msg.power_a),
    }


class TimelineNode(Node):
    def __init__(self, output: Path) -> None:
        super().__init__("go2w_passive_remote_timeline")
        output.parent.mkdir(parents=True, exist_ok=True)
        self._handle = output.open("a", buffering=1)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )
        specs = [
            (WirelessController, "/wirelesscontroller", wireless_dict),
            (
                WirelessController,
                "/wirelesscontroller_unprocessed",
                wireless_dict,
            ),
            (Request, "/api/sport/request", request_dict),
            (Response, "/api/sport/response", response_dict),
            (SportModeState, "/lf/sportmodestate", sport_state_dict),
            (LowState, "/lf/lowstate", low_state_dict),
        ]
        self._capture_subscriptions = []
        for msg_type, topic, converter in specs:
            callback = self._make_callback(topic, msg_type.__name__, converter)
            self._capture_subscriptions.append(
                self.create_subscription(msg_type, topic, callback, qos)
            )

    def _make_callback(self, topic: str, type_name: str, converter: Any) -> Any:
        def callback(msg: Any) -> None:
            record = {
                "receive_monotonic_ns": time.monotonic_ns(),
                "receive_wall_time": wall_time(),
                "topic": topic,
                "message_type": type_name,
                "message": converter(msg),
            }
            self._handle.write(json.dumps(record, ensure_ascii=False) + "\n")

        return callback

    def close_file(self) -> None:
        self._handle.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = TimelineNode(args.output)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close_file()
        try:
            node.destroy_node()
        except ValueError:
            # Humble can tear down subscription waitables during SIGINT first.
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
