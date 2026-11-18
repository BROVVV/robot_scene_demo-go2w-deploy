#!/usr/bin/env python3
"""Read-only diagnostics for a recorded Go2-W search session.

The source session is opened only for reading.  Optional output is written to a
different path and includes hashes so later reports cannot silently substitute
different evidence.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any, Iterable

import numpy as np


def _json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _jsonl(path: Path) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except FileNotFoundError:
        return result
    for line_number, line in enumerate(lines, start=1):
        if not line.strip():
            continue
        try:
            item = json.loads(line)
        except json.JSONDecodeError as exc:
            result.append({"_parse_error": str(exc), "_line": line_number})
            continue
        if isinstance(item, dict):
            result.append(item)
    return result


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _map_metrics(session: Path, timeline: list[dict[str, Any]]) -> dict[str, Any]:
    replay = session / "replay"
    map_events = [item for item in timeline if item.get("kind") == "map"]
    stored: list[dict[str, Any]] = []
    for item in map_events:
        map_hash = str(item.get("hash") or "")
        path = replay / "maps" / f"{map_hash}.npz"
        if not map_hash or not path.is_file():
            continue
        with np.load(path) as archive:
            grid = np.asarray(archive["grid"])
        stored.append({
            "hash": map_hash,
            "t": item.get("t"),
            "wall": item.get("wall"),
            "frame_id": item.get("frame_id"),
            "width": int(grid.shape[1]),
            "height": int(grid.shape[0]),
            "resolution": float(item.get("resolution") or 0.0),
            "origin": list(item.get("origin") or [])[:2],
            "unknown": int(np.count_nonzero(grid < 0)),
            "free": int(np.count_nonzero((grid >= 0) & (grid < 50))),
            "occupied": int(np.count_nonzero(grid >= 50)),
        })
    final = stored[-1] if stored else None
    stored_hashes = {item["hash"] for item in stored}
    content_stop_t = None
    if map_events:
        changes = [item for item in map_events if item.get("new_content") is True]
        if changes:
            content_stop_t = changes[-1].get("t")
    return {
        "published_samples": len(map_events),
        "stored_samples": len(stored_hashes),
        "resolved_map_events": len(stored),
        "distinct_hashes": len({item.get("hash") for item in map_events if item.get("hash")}),
        "final": final,
        "content_stop_t": content_stop_t,
        "origin_shift_m": _origin_shift(stored),
    }


def _origin_shift(samples: list[dict[str, Any]]) -> dict[str, float] | None:
    origins = [item["origin"] for item in samples if len(item.get("origin") or []) >= 2]
    if len(origins) < 2:
        return None
    dx = float(origins[-1][0]) - float(origins[0][0])
    dy = float(origins[-1][1]) - float(origins[0][1])
    return {"dx": dx, "dy": dy, "norm": float((dx * dx + dy * dy) ** 0.5)}


def _localization_metrics(timeline: list[dict[str, Any]]) -> dict[str, Any]:
    items = [item for item in timeline if item.get("kind") == "localization"]
    invalid = [item for item in items if item.get("valid") is not True]
    transitions: list[dict[str, Any]] = []
    previous: bool | None = None
    for item in items:
        current = item.get("valid") is True
        if previous is None or current != previous:
            transitions.append({"t": item.get("t"), "wall": item.get("wall"), "valid": current})
        previous = current
    return {
        "samples": len(items),
        "invalid_samples": len(invalid),
        "transitions": transitions,
        "last": items[-1] if items else None,
    }


def _event_metrics(events: list[dict[str, Any]]) -> dict[str, Any]:
    counts: dict[str, int] = {}
    for item in events:
        name = str(item.get("event") or "unknown")
        counts[name] = counts.get(name, 0) + 1
    failure_events = [
        item for item in events
        if any(token in str(item.get("event") or "").casefold()
               for token in ("fail", "invalid", "lost", "cancel", "timeout"))
    ]
    decisions = [item for item in events if item.get("event") in {
        "search_decision", "decision_recorded", "goal_selected", "spatial_intent_selected"
    }]
    return {
        "counts": counts,
        "failure_events": failure_events[-50:],
        "decision_events": decisions[-50:],
    }


def _semantic_metrics(graph: dict[str, Any]) -> dict[str, Any]:
    objects = list(graph.get("objects") or [])
    statuses: dict[str, int] = {}
    with_map = 0
    methods: dict[str, int] = {}
    for item in objects:
        statuses[str(item.get("status") or "unknown")] = statuses.get(
            str(item.get("status") or "unknown"), 0
        ) + 1
        if item.get("map_xyz") is not None:
            with_map += 1
        provenance = item.get("provenance") or {}
        method = str(provenance.get("depth_method") or provenance.get("depth_source") or "unknown")
        methods[method] = methods.get(method, 0) + 1
    return {
        "objects": len(objects),
        "objects_with_map_xyz": with_map,
        "statuses": statuses,
        "depth_methods": methods,
        "places": len(graph.get("places") or []),
        "frontiers": len(graph.get("frontiers") or []),
        "graph_revision": graph.get("revision"),
    }


def _inventory(session: Path) -> dict[str, Any]:
    patterns = {
        "occupancy_npz": "replay/maps/*.npz",
        "rgb_frames": "replay/frames/*",
        "timeline": "replay/timeline.jsonl",
        "poses": "replay/poses.jsonl",
        "events": "events.jsonl",
    }
    inventory: dict[str, Any] = {}
    for key, pattern in patterns.items():
        paths = sorted(session.glob(pattern))
        inventory[key] = {
            "count": len(paths),
            "bytes": sum(path.stat().st_size for path in paths if path.is_file()),
        }
    raw_patterns = ("*.db3", "*.mcap", "*.pcd", "*.ply", "*pointcloud*", "*joint*", "*imu*")
    raw = [path for pattern in raw_patterns for path in session.rglob(pattern) if path.is_file()]
    inventory["raw_motion_sensor_candidates"] = [str(path.relative_to(session)) for path in sorted(set(raw))]
    inventory["evidence_class"] = (
        "raw_motion_sensor_data_present" if raw else "derived_2d_map_and_event_record_only"
    )
    return inventory


def diagnose(session: Path) -> dict[str, Any]:
    if not session.is_dir():
        raise FileNotFoundError(session)
    timeline = _jsonl(session / "replay" / "timeline.jsonl")
    events = _jsonl(session / "events.jsonl")
    graph = _json(session / "webui_graph.json")
    manifest = _json(session / "replay" / "manifest.json")
    state = _json(session / "webui_state.json")
    source_files = [
        path for path in (
            session / "events.jsonl",
            session / "webui_graph.json",
            session / "webui_state.json",
            session / "replay" / "timeline.jsonl",
            session / "replay" / "poses.jsonl",
            session / "replay" / "manifest.json",
        ) if path.is_file()
    ]
    timestamps = [float(item["t"]) for item in timeline if isinstance(item.get("t"), (int, float))]
    return {
        "schema_version": "go2w_map_semantic_session_diagnostic_v1",
        "read_only": True,
        "session": str(session.resolve()),
        "session_id": manifest.get("session_id") or state.get("session_id") or session.name,
        "backend": manifest.get("backend") or state.get("backend"),
        "target": manifest.get("target") or state.get("target"),
        "recording_duration_s": (max(timestamps) - min(timestamps)) if len(timestamps) >= 2 else None,
        "map": _map_metrics(session, timeline),
        "localization": _localization_metrics(timeline),
        "semantics": _semantic_metrics(graph),
        "events": _event_metrics(events),
        "inventory": _inventory(session),
        "missing": [str(path.relative_to(session)) for path in (
            session / "events.jsonl",
            session / "webui_graph.json",
            session / "webui_state.json",
            session / "replay" / "timeline.jsonl",
            session / "replay" / "poses.jsonl",
            session / "replay" / "manifest.json",
        ) if not path.is_file()],
        "evidence_hashes": {
            str(path.relative_to(session)): _sha256(path) for path in source_files
        },
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("session", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--split-baseline", action="store_true")
    args = parser.parse_args(list(argv) if argv is not None else None)
    result = diagnose(args.session.resolve())
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    if args.output is not None:
        output = args.output.resolve()
        if args.session.resolve() in output.parents:
            raise SystemExit("refusing to write diagnostics inside source session")
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered + "\n", encoding="utf-8")
        if args.split_baseline:
            (output.parent / "data_inventory.json").write_text(
                json.dumps(result["inventory"], ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            (output.parent / "evidence_hashes.json").write_text(
                json.dumps(result["evidence_hashes"], ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
    sys.stdout.write(rendered + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
