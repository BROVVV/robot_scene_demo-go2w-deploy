"""Read the latest mapping-assist point-cloud snapshot for FastAPI."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any


def unavailable_snapshot(reason: str) -> dict[str, Any]:
    return {
        "schema_version": "go2w_slam_web_cloud_v1",
        "available": False,
        "reason": reason,
        "points": [],
        "point_count": 0,
        "mapping_mode": "mapping_assist",
        "motion_authorized": False,
        "safety_authorized": False,
    }


def load_slam_map_snapshot(path: str | Path) -> dict[str, Any]:
    snapshot_path = Path(path)
    if not snapshot_path.is_file():
        return unavailable_snapshot("snapshot_not_ready")
    try:
        value = json.loads(snapshot_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return unavailable_snapshot("snapshot_unreadable")
    if not isinstance(value, dict) or not isinstance(value.get("points"), list):
        return unavailable_snapshot("snapshot_invalid")
    generated_at = value.get("generated_at")
    try:
        age = max(0.0, time.time() - float(generated_at))
    except (TypeError, ValueError):
        age = None
    result = dict(value)
    result["age_seconds"] = age
    result["fresh"] = age is not None and age <= 3.0
    result["available"] = bool(value.get("available", True)) and bool(value["points"])
    result["point_count"] = len(value["points"])
    # These are facts about this isolated display path, not inferred flags.
    result["mapping_mode"] = "mapping_assist"
    result["motion_authorized"] = False
    result["safety_authorized"] = False
    return result
