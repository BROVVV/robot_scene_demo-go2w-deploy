"""Read the latest mapping-assist point-cloud snapshot for FastAPI."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any


def unavailable_snapshot(reason: str) -> dict[str, Any]:
    return {
        "schema_version": "go2w_slam_web_cloud_v2",
        "available": False,
        "reason": reason,
        "points": [],
        "point_count": 0,
        "mapping_mode": "mapping_assist",
        "motion_authorized": False,
        "safety_authorized": False,
        "mapping_health": "UNAVAILABLE",
        "health_reason": "建图快照还没生成",
        "lio_pose_valid": False,
        "motion_odom_seen": False,
        "canonical_frame": "pslam_map",
        "permanent_source": "/go2w/slam/map_3d",
        "mapping_session_id": 0,
        "map_revision": 0,
        "source_map_points": 0,
        "global_cached_voxels": 0,
        "web_display_points": 0,
        "map_extent_m": [0.0, 0.0, 0.0],
        "capacity_limited": False,
        "preview": {"frame_id": "", "ros_stamp": 0.0, "point_count": 0, "points": []},
        "rejected_counts": {},
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
    now = time.time()
    generated_at = value.get("generated_at")
    try:
        age = max(0.0, now - float(generated_at))
    except (TypeError, ValueError):
        age = None
    result = dict(value)
    result["age_seconds"] = age
    result["fresh"] = age is not None and age <= 3.0
    result["available"] = bool(value.get("available", True)) and bool(value["points"])
    result["point_count"] = len(value["points"])
    result["web_display_points"] = len(value["points"])
    # §9.3：全局地图和当前 scan 是两个时间轴，网页必须分别显示，不能混成一个。
    result["map_age_seconds"] = _age(now, value.get("map_updated_at"))
    result["preview_age_seconds"] = _age(
        now, (value.get("preview") or {}).get("wall_time"))
    # These are facts about this isolated display path, not inferred flags.
    result["mapping_mode"] = "mapping_assist"
    result["motion_authorized"] = False
    result["safety_authorized"] = False
    return result


def _age(now: float, stamp: Any) -> float | None:
    try:
        value = float(stamp)
    except (TypeError, ValueError):
        return None
    return max(0.0, now - value) if value > 0.0 else None
