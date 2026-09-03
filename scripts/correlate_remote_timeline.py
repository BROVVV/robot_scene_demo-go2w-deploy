#!/usr/bin/env python3
"""Correlate passive posture events with an observed remote-control chord."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any


WIRELESS_TOPICS = ("/wirelesscontroller", "/wirelesscontroller_unprocessed")


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    with path.open(encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def write_json(path: Path, value: Any) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(value, handle, ensure_ascii=False, indent=2)
        handle.write("\n")


def marker_time(markers: list[dict[str, Any]], name: str) -> int | None:
    for marker in markers:
        if marker.get("event") == name:
            return int(marker["receive_monotonic_ns"])
    return None


def choose_wireless_topic(records: list[dict[str, Any]]) -> str | None:
    counts = {
        topic: sum(record.get("topic") == topic for record in records)
        for topic in WIRELESS_TOPICS
    }
    if counts[WIRELESS_TOPICS[0]]:
        return WIRELESS_TOPICS[0]
    if counts[WIRELESS_TOPICS[1]]:
        return WIRELESS_TOPICS[1]
    return None


def remote_transitions(
    records: list[dict[str, Any]], topic: str, marker_ns: int
) -> tuple[list[dict[str, Any]], int]:
    samples = [record for record in records if record.get("topic") == topic]
    samples.sort(key=lambda record: int(record["receive_monotonic_ns"]))
    # The remote topic can be event-driven. If no sample precedes the marker,
    # idle is unknown on the wire and the neutral key mask is the safe baseline.
    baseline = 0
    for sample in samples:
        if int(sample["receive_monotonic_ns"]) <= marker_ns:
            baseline = int(sample["message"]["keys"])
        else:
            break
    transitions: list[dict[str, Any]] = []
    previous = baseline
    for sample in samples:
        timestamp = int(sample["receive_monotonic_ns"])
        if timestamp < marker_ns - 2_000_000_000:
            continue
        keys = int(sample["message"]["keys"])
        if keys == previous:
            continue
        changed = previous ^ keys
        transitions.append(
            {
                "receive_monotonic_ns": timestamp,
                "keys": keys,
                "keys_hex": f"0x{keys:04x}",
                "set_bits": [bit for bit in range(16) if keys & (1 << bit)],
                "pressed_bits": [
                    bit for bit in range(16) if changed & (1 << bit) and keys & (1 << bit)
                ],
                "released_bits": [
                    bit
                    for bit in range(16)
                    if changed & (1 << bit) and not keys & (1 << bit)
                ],
            }
        )
        previous = keys
    return transitions, baseline


def chord_from_transitions(
    transitions: list[dict[str, Any]], marker_ns: int, baseline: int
) -> dict[str, Any] | None:
    candidates = [
        item
        for item in transitions
        if marker_ns <= item["receive_monotonic_ns"] <= marker_ns + 30_000_000_000
        and item["keys"] != baseline
        and item["keys"] != 0
    ]
    if not candidates:
        return None
    max_bits = max(len(item["set_bits"]) for item in candidates)
    chord = next(item for item in candidates if len(item["set_bits"]) == max_bits)
    later = [
        item
        for item in transitions
        if item["receive_monotonic_ns"] > chord["receive_monotonic_ns"]
        and item["keys"] != chord["keys"]
    ]
    end_ns = later[0]["receive_monotonic_ns"] if later else None
    return {
        **chord,
        "duration_s": None
        if end_ns is None
        else (end_ns - chord["receive_monotonic_ns"]) / 1e9,
    }


def summarize_round(
    round_name: str,
    direction: str,
    records: list[dict[str, Any]],
    markers: list[dict[str, Any]],
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    imminent_ns = marker_time(markers, "operator_action_imminent")
    stable_ns = marker_time(markers, "operator_reports_posture_stable")
    wireless_topic = choose_wireless_topic(records)
    transitions: list[dict[str, Any]] = []
    baseline_keys: int | None = None
    chord: dict[str, Any] | None = None
    if imminent_ns is not None and wireless_topic is not None:
        transitions, baseline_keys = remote_transitions(records, wireless_topic, imminent_ns)
        chord = chord_from_transitions(transitions, imminent_ns, baseline_keys)
    t0 = (
        int(chord["receive_monotonic_ns"])
        if chord is not None
        else imminent_ns
    )
    if t0 is None:
        raise ValueError(f"{round_name}: missing operator marker")

    events: list[dict[str, Any]] = []
    if imminent_ns is not None:
        events.append(
            {
                "round": round_name,
                "relative_time_s": (imminent_ns - t0) / 1e9,
                "event": "operator_action_imminent",
                "details": "terminal marker",
            }
        )
    if stable_ns is not None:
        events.append(
            {
                "round": round_name,
                "relative_time_s": (stable_ns - t0) / 1e9,
                "event": "operator_reports_posture_stable",
                "details": "terminal marker",
            }
        )
    for item in transitions:
        events.append(
            {
                "round": round_name,
                "relative_time_s": (item["receive_monotonic_ns"] - t0) / 1e9,
                "event": "remote_keys",
                "details": (
                    f"topic={wireless_topic} keys={item['keys_hex']} "
                    f"set={item['set_bits']} pressed={item['pressed_bits']} "
                    f"released={item['released_bits']}"
                ),
            }
        )

    sport_records = [
        record for record in records if record.get("topic") == "/lf/sportmodestate"
    ]
    sport_records.sort(key=lambda record: int(record["receive_monotonic_ns"]))
    before = [
        record for record in sport_records if int(record["receive_monotonic_ns"]) <= t0
    ]
    baseline_state = before[-1]["message"] if before else (
        sport_records[0]["message"] if sport_records else None
    )
    previous_mode = None if baseline_state is None else int(baseline_state["mode"])
    baseline_height = None if baseline_state is None else float(baseline_state["body_height"])
    mode_after = previous_mode
    heights: list[float] = []
    first_height_change = False
    for record in sport_records:
        timestamp = int(record["receive_monotonic_ns"])
        if timestamp < t0 - 2_000_000_000:
            continue
        msg = record["message"]
        mode = int(msg["mode"])
        height = float(msg["body_height"])
        heights.append(height)
        if previous_mode is None or mode != previous_mode:
            events.append(
                {
                    "round": round_name,
                    "relative_time_s": (timestamp - t0) / 1e9,
                    "event": "mode_change",
                    "details": f"mode={mode} progress={msg['progress']} body_height={height:.4f}",
                }
            )
            previous_mode = mode
            mode_after = mode
        if (
            not first_height_change
            and baseline_height is not None
            and abs(height - baseline_height) >= 0.02
        ):
            events.append(
                {
                    "round": round_name,
                    "relative_time_s": (timestamp - t0) / 1e9,
                    "event": "body_height_change",
                    "details": f"from={baseline_height:.4f} to={height:.4f}",
                }
            )
            first_height_change = True

    low_records = [record for record in records if record.get("topic") == "/lf/lowstate"]
    low_records.sort(key=lambda record: int(record["receive_monotonic_ns"]))
    low_before = [
        record for record in low_records if int(record["receive_monotonic_ns"]) <= t0
    ]
    baseline_leg_q = None
    if low_before:
        baseline_leg_q = [
            float(motor["q"]) for motor in low_before[-1]["message"]["motors"][:12]
        ]
    first_leg_change = False
    for record in low_records:
        timestamp = int(record["receive_monotonic_ns"])
        if timestamp < t0 or baseline_leg_q is None or first_leg_change:
            continue
        leg_q = [float(motor["q"]) for motor in record["message"]["motors"][:12]]
        delta = max(abs(value - base) for value, base in zip(leg_q, baseline_leg_q))
        if delta >= 0.05:
            events.append(
                {
                    "round": round_name,
                    "relative_time_s": (timestamp - t0) / 1e9,
                    "event": "leg_joint_change",
                    "details": f"max_abs_q_delta={delta:.4f}",
                }
            )
            first_leg_change = True

    requests: list[dict[str, Any]] = []
    responses: list[dict[str, Any]] = []
    for record in records:
        timestamp = int(record["receive_monotonic_ns"])
        topic = record.get("topic")
        msg = record.get("message", {})
        if topic == "/api/sport/request":
            identity = msg["header"]["identity"]
            request = {
                "relative_time_s": (timestamp - t0) / 1e9,
                "api_id": int(identity["api_id"]),
                "request_id": int(identity["id"]),
                "lease_id": int(msg["header"]["lease"]["id"]),
                "parameter": msg["parameter"],
            }
            requests.append(request)
            events.append(
                {
                    "round": round_name,
                    "relative_time_s": request["relative_time_s"],
                    "event": "sport_request",
                    "details": (
                        f"api={request['api_id']} request={request['request_id']} "
                        f"lease={request['lease_id']} parameter={request['parameter']}"
                    ),
                }
            )
        elif topic == "/api/sport/response":
            identity = msg["header"]["identity"]
            response = {
                "relative_time_s": (timestamp - t0) / 1e9,
                "api_id": int(identity["api_id"]),
                "request_id": int(identity["id"]),
                "status_code": int(msg["header"]["status"]["code"]),
            }
            responses.append(response)
            events.append(
                {
                    "round": round_name,
                    "relative_time_s": response["relative_time_s"],
                    "event": "sport_response",
                    "details": (
                        f"api={response['api_id']} request={response['request_id']} "
                        f"status={response['status_code']}"
                    ),
                }
            )

    events.sort(key=lambda row: row["relative_time_s"])
    summary = {
        "round": round_name,
        "direction": direction,
        "wireless_topic": wireless_topic,
        "baseline_keys": baseline_keys,
        "chord": chord,
        "mode_before": None if baseline_state is None else int(baseline_state["mode"]),
        "mode_after": mode_after,
        "body_height_before": baseline_height,
        "body_height_min": min(heights) if heights else None,
        "body_height_max": max(heights) if heights else None,
        "leg_joint_change_observed": first_leg_change,
        "requests": requests,
        "responses": responses,
        "operator_stable_delay_s": None
        if stable_ns is None
        else (stable_ns - t0) / 1e9,
    }
    return summary, events


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture-root", required=True, type=Path)
    args = parser.parse_args()
    root = args.capture_root.resolve()
    analysis = root / "analysis"
    analysis.mkdir(parents=True, exist_ok=True)
    grouped: dict[str, list[dict[str, Any]]] = {"down": [], "up": []}
    summaries: list[dict[str, Any]] = []

    for timeline_path in sorted((root / "text").glob("*_combined_timeline.jsonl")):
        round_name = timeline_path.name.removesuffix("_combined_timeline.jsonl")
        if not round_name.startswith(("D", "U")):
            continue
        manifest_path = root / "metadata" / f"{round_name}_round_manifest.json"
        if not manifest_path.exists():
            continue
        with manifest_path.open(encoding="utf-8") as handle:
            manifest = json.load(handle)
        if not manifest.get("end_time"):
            # Interrupted or deliberately discarded round; retain raw evidence but
            # exclude it from repetition and path classification.
            continue
        direction = "down" if round_name.startswith("D") else "up"
        markers = read_jsonl(root / "text" / f"{round_name}_operator_markers.jsonl")
        records = read_jsonl(timeline_path)
        if not markers or not records:
            continue
        summary, events = summarize_round(round_name, direction, records, markers)
        summaries.append(summary)
        grouped[direction].extend(events)

    write_json(analysis / "action_round_summaries.json", summaries)
    for direction in ("down", "up"):
        rows = sorted(
            grouped[direction], key=lambda row: (row["round"], row["relative_time_s"])
        )
        with (analysis / f"{direction}_timeline.csv").open(
            "w", newline="", encoding="utf-8"
        ) as handle:
            writer = csv.DictWriter(
                handle, fieldnames=["round", "relative_time_s", "event", "details"]
            )
            writer.writeheader()
            writer.writerows(rows)
        with (analysis / f"{direction}_timeline.md").open("w", encoding="utf-8") as handle:
            handle.write(f"# {direction} 遥控器动作时间轴\n\n")
            if not rows:
                handle.write("没有可关联的动作轮次。\n")
            for row in rows:
                handle.write(
                    f"- {row['round']} t={row['relative_time_s']:+.6f}s "
                    f"{row['event']}: {row['details']}\n"
                )
            if rows and not any(row["event"] == "sport_request" for row in rows):
                handle.write("\n未观察到公开 Sport Request。\n")
    print(
        f"TIMELINE_CORRELATION=PASS rounds={len(summaries)} "
        f"down_events={len(grouped['down'])} up_events={len(grouped['up'])}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
