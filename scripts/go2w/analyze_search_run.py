#!/usr/bin/env python3
"""Summarise one autonomous-search JSONL run for offline regression.

The input is an event log, not a command source.  This tool never talks to
ROS and never authorises motion; it turns the fields already emitted by the
worker into compact evidence that can be compared across runs.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
from typing import Any, Iterable


def _number(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _pose(value: Any) -> tuple[float, float, float] | None:
    if not isinstance(value, dict):
        return None
    x, y, yaw = (_number(value.get(key)) for key in ("x", "y", "yaw"))
    if x is None or y is None:
        return None
    return x, y, yaw or 0.0


def _target_state(event: dict[str, Any]) -> str:
    value = event.get("target_state")
    if value is None and isinstance(event.get("target_match"), dict):
        value = event["target_match"].get("target_state")
    if value is None:
        value = "PRESENT" if event.get("target_present") else "ABSENT"
    text = str(value).strip().upper()
    return text if text in {"ABSENT", "POSSIBLE", "PRESENT"} else "UNKNOWN"


def load_events(path: str | Path) -> tuple[list[dict[str, Any]], int]:
    """Load valid JSON objects and return ``(events, malformed_line_count)``."""
    events: list[dict[str, Any]] = []
    malformed = 0
    with Path(path).open("r", encoding="utf-8") as stream:
        for line in stream:
            if not line.strip():
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                malformed += 1
                continue
            if isinstance(value, dict):
                events.append(value)
            else:
                malformed += 1
    return events, malformed


def analyse_events(
    events: Iterable[dict[str, Any]],
    *,
    malformed_lines: int = 0,
    source: str | None = None,
) -> dict[str, Any]:
    """Return stable, JSON-safe regression metrics for a worker event stream."""
    events = list(events)
    event_counts = Counter(str(item.get("event") or "<missing>") for item in events)
    pslam_poses: list[tuple[float, float, float]] = []
    places: set[str] = set()
    local_scans: list[dict[str, Any]] = []
    turns: list[float] = []
    target_states: list[str] = []
    mapping_health: list[str] = []
    wheel_deltas: list[float] = []
    pslam_deltas: list[float] = []
    current_place_mismatches: list[dict[str, Any]] = []

    for event in events:
        name = str(event.get("event") or "")
        if name == "spatial_pose_validation":
            pose = _pose(event.get("accepted_pose")) if event.get("accepted") else None
            if pose is None:
                pose = _pose(event.get("raw_pose"))
            if pose is not None:
                pslam_poses.append(pose)
            health = event.get("health")
            if health:
                mapping_health.append(str(health))
            delta = _number(event.get("pslam_delta_xy_m"))
            if delta is not None:
                pslam_deltas.append(delta)
            wheel = _number(event.get("wheel_delta_xy_m"))
            if wheel is not None:
                wheel_deltas.append(wheel)
        elif name == "spatial_pose_updated":
            pose = _pose(event.get("pose"))
            if pose is not None:
                pslam_poses.append(pose)
        elif name == "place_updated":
            place = event.get("place")
            if isinstance(place, dict) and place.get("place_id"):
                places.add(str(place["place_id"]))
        elif name == "local_scan_selected":
            item = {
                "goal_id": event.get("goal_id"),
                "relative_dyaw": event.get("relative_dyaw"),
                "heading_sector": event.get("heading_sector"),
                "steps": event.get("steps"),
                "place_id": event.get("place_id"),
            }
            local_scans.append(item)
            delta = _number(event.get("relative_dyaw"))
            if delta is not None:
                turns.append(delta)
        elif name == "selected_goal":
            goal = event.get("goal")
            provenance = goal.get("provenance") if isinstance(goal, dict) else None
            decision_place = event.get("current_place_id") or event.get("place_id")
            scan_place = provenance.get("place_id") if isinstance(provenance, dict) else None
            if decision_place and scan_place and str(decision_place) != str(scan_place):
                current_place_mismatches.append({
                    "decision_place_id": decision_place,
                    "local_scan_place_id": scan_place,
                    "goal_id": goal.get("goal_id") if isinstance(goal, dict) else None,
                })
        elif name in {"observation", "match", "target_state_changed", "target_possible_verify"}:
            state = _target_state(event)
            if state != "UNKNOWN":
                target_states.append(state)
        elif name in {"mapping_health_changed", "spatial_pose_validation"}:
            health = event.get("mapping_health") or event.get("health")
            if health:
                mapping_health.append(str(health))
        elif name in {"motion_observation_summary", "motion_summary"}:
            wheel = _number((event.get("wheel") or {}).get("delta_xy_m"))
            pslam = _number((event.get("pslam") or {}).get("delta_xy_m"))
            if wheel is not None:
                wheel_deltas.append(wheel)
            if pslam is not None:
                pslam_deltas.append(pslam)

    # Old runs only have ``spatial_pose_updated`` and new runs may emit both
    # validation and update events.  Keep the metric deterministic by using
    # the max radius and max consecutive displacement, not duplicate counts.
    unique_poses: list[tuple[float, float, float]] = []
    for pose in pslam_poses:
        if not unique_poses or pose != unique_poses[-1]:
            unique_poses.append(pose)
    radii = [math.hypot(x, y) for x, y, _ in unique_poses]
    consecutive_deltas = [
        math.hypot(curr[0] - prev[0], curr[1] - prev[1])
        for prev, curr in zip(unique_poses, unique_poses[1:])
    ]
    return {
        "schema_version": "go2w.search_replay_analysis_v1",
        "source": source,
        "event_count": len(events),
        "malformed_line_count": int(malformed_lines),
        "event_counts": dict(sorted(event_counts.items())),
        "target_state_sequence": target_states,
        "full_semantic_error_count": sum(
            1 for item in events
            if item.get("event") in {"semantic_error", "semantic_timeout"}
            or "FULL_SEMANTIC" in str(item.get("error_code") or "")
        ),
        "mapping_health_sequence": mapping_health,
        "place_ids": sorted(places),
        "place_count": len(places),
        "local_scan": {
            "count": len(local_scans),
            "step_sequence": [item.get("steps") for item in local_scans],
            "turn_sequence_deg": turns,
            "goal_ids": [item.get("goal_id") for item in local_scans],
            "place_ids": [item.get("place_id") for item in local_scans],
        },
        "pslam": {
            "pose_count": len(unique_poses),
            "max_radius_m": max(radii, default=None),
            "max_consecutive_delta_m": max(consecutive_deltas, default=None),
            "max_reported_delta_m": max(pslam_deltas, default=None),
        },
        "wheel": {
            "sample_count": len(wheel_deltas),
            "max_delta_xy_m": max(wheel_deltas, default=None),
        },
        "current_place_mismatch_count": len(current_place_mismatches),
        "current_place_mismatches": current_place_mismatches,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("events", type=Path, help="worker events.jsonl")
    parser.add_argument("--output", type=Path, help="write the JSON analysis here")
    args = parser.parse_args()
    events, malformed = load_events(args.events)
    result = analyse_events(events, malformed_lines=malformed, source=str(args.events))
    encoded = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
