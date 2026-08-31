#!/usr/bin/env python3
"""Read-only WirelessController logger with bit-level transition timing."""

from __future__ import annotations

import argparse
import csv
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from unitree_go.msg import WirelessController


def now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="microseconds")


class RemoteKeyMonitor(Node):
    def __init__(self, output_prefix: Path, topics: list[str]) -> None:
        super().__init__("go2w_passive_remote_key_monitor")
        output_prefix.parent.mkdir(parents=True, exist_ok=True)
        self._jsonl = output_prefix.with_suffix(".jsonl").open("a", buffering=1)
        self._csv_handle = output_prefix.with_suffix(".csv").open(
            "a", newline="", buffering=1
        )
        self._transitions = output_prefix.parent.joinpath(
            output_prefix.name + "_transitions.txt"
        ).open("a", buffering=1)
        self._writer = csv.DictWriter(
            self._csv_handle,
            fieldnames=[
                "receive_monotonic_ns",
                "receive_wall_time",
                "topic",
                "lx",
                "ly",
                "rx",
                "ry",
                "keys_decimal",
                "keys_hex",
                "keys_binary",
                "changed_bits",
                "pressed_bits",
                "released_bits",
                "previous_state_duration_s",
            ],
        )
        if self._csv_handle.tell() == 0:
            self._writer.writeheader()
        self._previous: dict[str, int] = {}
        self._previous_change_ns: dict[str, int] = {}
        self._bit_pressed_ns: dict[tuple[str, int], int] = {}
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self._capture_subscriptions = [
            self.create_subscription(
                WirelessController,
                topic,
                lambda msg, source=topic: self._callback(source, msg),
                qos,
            )
            for topic in topics
        ]

    def _callback(self, topic: str, msg: WirelessController) -> None:
        mono_ns = time.monotonic_ns()
        keys = int(msg.keys) & 0xFFFF
        previous = self._previous.get(topic, keys)
        changed_mask = previous ^ keys
        changed = [bit for bit in range(16) if changed_mask & (1 << bit)]
        pressed = [bit for bit in changed if keys & (1 << bit)]
        released = [bit for bit in changed if not keys & (1 << bit)]
        previous_duration = (
            None
            if topic not in self._previous_change_ns
            else (mono_ns - self._previous_change_ns[topic]) / 1_000_000_000
        )
        held_durations: dict[str, float] = {}
        for bit in pressed:
            self._bit_pressed_ns[(topic, bit)] = mono_ns
        for bit in released:
            start = self._bit_pressed_ns.pop((topic, bit), None)
            if start is not None:
                held_durations[str(bit)] = (mono_ns - start) / 1_000_000_000
        record = {
            "receive_monotonic_ns": mono_ns,
            "receive_wall_time": now_iso(),
            "topic": topic,
            "lx": float(msg.lx),
            "ly": float(msg.ly),
            "rx": float(msg.rx),
            "ry": float(msg.ry),
            "keys_decimal": keys,
            "keys_hex": f"0x{keys:04x}",
            "keys_binary": f"{keys:016b}",
            "set_bits": [bit for bit in range(16) if keys & (1 << bit)],
            "changed_bits": changed,
            "pressed_bits": pressed,
            "released_bits": released,
            "previous_state_duration_s": previous_duration,
            "released_bit_hold_s": held_durations,
        }
        self._jsonl.write(json.dumps(record, ensure_ascii=False) + "\n")
        self._writer.writerow(
            {
                key: record[key]
                for key in self._writer.fieldnames
            }
        )
        if changed or topic not in self._previous:
            self._transitions.write(
                f"{record['receive_wall_time']} mono_ns={mono_ns} "
                f"topic={topic} "
                f"keys={record['keys_hex']} set={record['set_bits']} "
                f"pressed={pressed} released={released} "
                f"previous_duration_s={previous_duration} held_s={held_durations}\n"
            )
            self._previous_change_ns[topic] = mono_ns
        self._previous[topic] = keys

    def close_files(self) -> None:
        self._jsonl.close()
        self._csv_handle.close()
        self._transitions.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-prefix", required=True, type=Path)
    parser.add_argument(
        "--topics",
        nargs="+",
        default=["/wirelesscontroller", "/wirelesscontroller_unprocessed"],
    )
    args = parser.parse_args()
    rclpy.init()
    node = RemoteKeyMonitor(args.output_prefix, args.topics)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close_files()
        try:
            node.destroy_node()
        except ValueError:
            # Humble may already remove subscription waitables while handling SIGINT.
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
