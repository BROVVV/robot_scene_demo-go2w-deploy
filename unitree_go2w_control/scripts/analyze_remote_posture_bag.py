#!/usr/bin/env python3
"""Offline-only rosbag2 analyzer. It never creates a ROS graph publisher."""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterator

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def iter_bag(path: Path) -> Iterator[tuple[str, str, Any, int]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_map = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    classes = {topic: get_message(type_name) for topic, type_name in type_map.items()}
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        yield topic, type_map[topic], deserialize_message(data, classes[topic]), timestamp


def parse_parameter(raw: str) -> Any:
    try:
        return json.loads(raw)
    except (json.JSONDecodeError, TypeError):
        return None


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture-root", required=True, type=Path)
    args = parser.parse_args()
    root = args.capture_root.resolve()
    bag_root = root / "rosbag"
    analysis = root / "analysis"
    analysis.mkdir(parents=True, exist_ok=True)
    bags = sorted(path for path in bag_root.iterdir() if (path / "metadata.yaml").exists())
    if not bags:
        raise SystemExit(f"no rosbag directories found under {bag_root}")

    stats: dict[tuple[str, str, str], dict[str, Any]] = {}
    requests: list[dict[str, Any]] = []
    responses: list[dict[str, Any]] = []
    key_rows: list[dict[str, Any]] = []
    posture_rows: list[dict[str, Any]] = []
    previous_keys: dict[tuple[str, str], int] = {}
    previous_request_ts: dict[str, int] = {}

    for bag in bags:
        round_name = bag.name
        manifest_path = root / "metadata" / f"{round_name}_round_manifest.json"
        if manifest_path.exists():
            with manifest_path.open(encoding="utf-8") as handle:
                action = json.load(handle).get("action", "unknown")
        else:
            action = "unknown"
        for topic, type_name, msg, timestamp in iter_bag(bag):
            key = (round_name, topic, type_name)
            entry = stats.setdefault(
                key,
                {
                    "round": round_name,
                    "topic": topic,
                    "type": type_name,
                    "count": 0,
                    "first_timestamp_ns": timestamp,
                    "last_timestamp_ns": timestamp,
                    "largest_gap_ns": 0,
                    "_previous": None,
                },
            )
            if entry["_previous"] is not None:
                entry["largest_gap_ns"] = max(
                    entry["largest_gap_ns"], timestamp - entry["_previous"]
                )
            entry["_previous"] = timestamp
            entry["count"] += 1
            entry["last_timestamp_ns"] = timestamp

            if topic == "/api/sport/request":
                raw = msg.parameter
                previous = previous_request_ts.get(round_name)
                requests.append(
                    {
                        "round": round_name,
                        "action": action,
                        "bag_timestamp_ns": timestamp,
                        "request_id": int(msg.header.identity.id),
                        "api_id": int(msg.header.identity.api_id),
                        "lease_id": int(msg.header.lease.id),
                        "priority": int(msg.header.policy.priority),
                        "noreply": bool(msg.header.policy.noreply),
                        "parameter_raw": raw,
                        "parameter_json": parse_parameter(raw),
                        "binary_length": len(msg.binary),
                        "delta_previous_request_s": None
                        if previous is None
                        else (timestamp - previous) / 1e9,
                    }
                )
                previous_request_ts[round_name] = timestamp
            elif topic == "/api/sport/response":
                responses.append(
                    {
                        "round": round_name,
                        "action": action,
                        "bag_timestamp_ns": timestamp,
                        "request_id": int(msg.header.identity.id),
                        "api_id": int(msg.header.identity.api_id),
                        "status_code": int(msg.header.status.code),
                        "data": msg.data,
                        "binary_length": len(msg.binary),
                    }
                )
            elif topic in ("/wirelesscontroller", "/wirelesscontroller_unprocessed"):
                keys = int(msg.keys) & 0xFFFF
                previous_key = (round_name, topic)
                previous = previous_keys.get(previous_key)
                if previous is None or previous != keys:
                    changed_mask = 0 if previous is None else previous ^ keys
                    key_rows.append(
                        {
                            "round": round_name,
                            "action": action,
                            "topic": topic,
                            "bag_timestamp_ns": timestamp,
                            "keys_decimal": keys,
                            "keys_hex": f"0x{keys:04x}",
                            "set_bits": [bit for bit in range(16) if keys & (1 << bit)],
                            "changed_bits": [
                                bit for bit in range(16) if changed_mask & (1 << bit)
                            ],
                            "pressed_bits": [
                                bit
                                for bit in range(16)
                                if changed_mask & (1 << bit) and keys & (1 << bit)
                            ],
                            "released_bits": [
                                bit
                                for bit in range(16)
                                if changed_mask & (1 << bit) and not keys & (1 << bit)
                            ],
                            "lx": float(msg.lx),
                            "ly": float(msg.ly),
                            "rx": float(msg.rx),
                            "ry": float(msg.ry),
                        }
                    )
                    previous_keys[previous_key] = keys
            elif topic == "/lf/sportmodestate":
                posture_rows.append(
                    {
                        "round": round_name,
                        "action": action,
                        "bag_timestamp_ns": timestamp,
                        "source": "sport",
                        "mode": int(msg.mode),
                        "progress": float(msg.progress),
                        "body_height": float(msg.body_height),
                        "error_code": int(msg.error_code),
                        "velocity": json.dumps([float(v) for v in msg.velocity]),
                        "yaw_speed": float(msg.yaw_speed),
                        "imu_rpy": json.dumps([float(v) for v in msg.imu_state.rpy]),
                        "leg_q": "",
                        "leg_dq": "",
                        "wheel_q": "",
                        "wheel_dq": "",
                    }
                )
            elif topic == "/lf/lowstate":
                motors = msg.motor_state
                posture_rows.append(
                    {
                        "round": round_name,
                        "action": action,
                        "bag_timestamp_ns": timestamp,
                        "source": "low",
                        "mode": "",
                        "progress": "",
                        "body_height": "",
                        "error_code": "",
                        "velocity": "",
                        "yaw_speed": "",
                        "imu_rpy": json.dumps([float(v) for v in msg.imu_state.rpy]),
                        "leg_q": json.dumps([float(motors[i].q) for i in range(12)]),
                        "leg_dq": json.dumps([float(motors[i].dq) for i in range(12)]),
                        "wheel_q": json.dumps([float(motors[i].q) for i in range(12, 16)]),
                        "wheel_dq": json.dumps([float(motors[i].dq) for i in range(12, 16)]),
                    }
                )

    topic_rows = []
    for entry in stats.values():
        duration_s = (entry["last_timestamp_ns"] - entry["first_timestamp_ns"]) / 1e9
        entry["average_rate_hz"] = (
            (entry["count"] - 1) / duration_s if duration_s > 0 else 0.0
        )
        entry.pop("_previous", None)
        topic_rows.append(entry)
    topic_rows.sort(key=lambda row: (row["round"], row["topic"]))
    with (analysis / "topic_counts.json").open("w", encoding="utf-8") as handle:
        json.dump(topic_rows, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    for filename, fields in (
        ("topic_rates.csv", ["round", "topic", "type", "count", "average_rate_hz"]),
        (
            "topic_time_ranges.csv",
            [
                "round",
                "topic",
                "first_timestamp_ns",
                "last_timestamp_ns",
                "largest_gap_ns",
            ],
        ),
    ):
        with (analysis / filename).open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(topic_rows)

    write_jsonl(analysis / "sport_requests.jsonl", requests)
    write_jsonl(analysis / "sport_responses.jsonl", responses)
    by_key_stream: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for row in key_rows:
        by_key_stream[(row["round"], row["topic"])].append(row)
    for rows in by_key_stream.values():
        rows.sort(key=lambda row: row["bag_timestamp_ns"])
        for index, row in enumerate(rows):
            row["state_duration_s"] = (
                None
                if index + 1 == len(rows)
                else (
                    rows[index + 1]["bag_timestamp_ns"] - row["bag_timestamp_ns"]
                )
                / 1e9
            )
    write_jsonl(analysis / "remote_key_transitions.jsonl", key_rows)
    with (analysis / "posture_state.csv").open("w", newline="", encoding="utf-8") as handle:
        fields = list(posture_rows[0]) if posture_rows else []
        if fields:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(posture_rows)

    with (analysis / "sport_request_sequence.md").open("w", encoding="utf-8") as handle:
        handle.write("# Sport 请求序列\n\n")
        if not requests:
            handle.write("未观察到公开 `/api/sport/request`。\n")
        for row in requests:
            handle.write(
                f"- {row['round']} ts={row['bag_timestamp_ns']} api={row['api_id']} "
                f"request={row['request_id']} lease={row['lease_id']} "
                f"parameter=`{row['parameter_raw']}`\n"
            )
    with (analysis / "remote_key_chords.md").open("w", encoding="utf-8") as handle:
        handle.write("# 遥控器 keys 变化\n\n")
        if not key_rows:
            handle.write("未捕获到遥控器 keys 消息。\n")
        for row in sorted(
            key_rows,
            key=lambda item: (item["round"], item["topic"], item["bag_timestamp_ns"]),
        ):
            handle.write(
                f"- {row['round']} `{row['topic']}` ts={row['bag_timestamp_ns']} "
                f"keys={row['keys_hex']} set={row['set_bits']} "
                f"pressed={row['pressed_bits']} released={row['released_bits']} "
                f"duration={row['state_duration_s']}s\n"
            )
    with (analysis / "posture_transitions.md").open("w", encoding="utf-8") as handle:
        handle.write("# 姿态状态范围\n\n")
        by_round: dict[str, list[dict[str, Any]]] = defaultdict(list)
        for row in posture_rows:
            if row["source"] == "sport":
                by_round[row["round"]].append(row)
        for round_name, rows in sorted(by_round.items()):
            handle.write(
                f"- {round_name}: modes={sorted({row['mode'] for row in rows})}, "
                f"body_height=[{min(row['body_height'] for row in rows):.4f}, "
                f"{max(row['body_height'] for row in rows):.4f}], "
                f"errors={sorted({row['error_code'] for row in rows})}\n"
            )
    print(f"OFFLINE_BAG_ANALYSIS=PASS bags={len(bags)} requests={len(requests)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
