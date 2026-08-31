"""Topology-native frontier model.

A frontier belongs to a Place.  It is derived from Place heading coverage plus
local depth/LiDAR free-space hints, never from a global occupancy raster.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Any

FRONTIER_STATE_OPEN = "OPEN"
FRONTIER_STATE_SELECTED = "SELECTED"
FRONTIER_STATE_TRAVERSED = "TRAVERSED"
FRONTIER_STATE_BLOCKED = "BLOCKED"
FRONTIER_STATE_STALE = "STALE"


@dataclass
class TopologicalFrontier:
    frontier_id: str
    parent_place_id: str
    bearing_deg: float
    local_distance_hint_m: float | None = None
    semantic_score: float = 0.0
    information_gain: float = 0.0
    visit_count: int = 0
    failure_count: int = 0
    status: str = FRONTIER_STATE_OPEN
    source: str = "topology_heading_gap"
    created_at: float | None = None
    last_seen_at: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "TopologicalFrontier":
        return cls(
            frontier_id=str(value.get("frontier_id") or ""),
            parent_place_id=str(value.get("parent_place_id") or ""),
            bearing_deg=float(value.get("bearing_deg", 0.0)),
            local_distance_hint_m=value.get("local_distance_hint_m"),
            semantic_score=float(value.get("semantic_score", 0.0)),
            information_gain=float(value.get("information_gain", 0.0)),
            visit_count=int(value.get("visit_count", 0)),
            failure_count=int(value.get("failure_count", 0)),
            status=str(value.get("status", FRONTIER_STATE_OPEN)),
            source=str(value.get("source", "topology_heading_gap")),
            created_at=value.get("created_at"),
            last_seen_at=value.get("last_seen_at"),
        )


def merge_frontier(
    existing: TopologicalFrontier | None,
    *,
    parent_place_id: str,
    bearing_deg: float,
    merge_bearing_deg: float = 15.0,
    information_gain: float = 0.0,
    semantic_score: float = 0.0,
    now: float | None = None,
) -> TopologicalFrontier:
    """Upsert a heading-based frontier into a Place, merging nearby bearings."""
    if existing is None:
        return TopologicalFrontier(
            frontier_id=f"F{abs(hash((parent_place_id, round(bearing_deg / 30.0)))) % 100000:05d}",
            parent_place_id=parent_place_id,
            bearing_deg=bearing_deg,
            information_gain=information_gain,
            semantic_score=semantic_score,
            created_at=now,
            last_seen_at=now,
        )
    delta = abs((existing.bearing_deg - bearing_deg + 180.0) % 360.0 - 180.0)
    if delta <= merge_bearing_deg:
        existing.bearing_deg = (existing.bearing_deg + bearing_deg) / 2.0
        existing.information_gain = max(existing.information_gain, information_gain)
        existing.semantic_score = max(existing.semantic_score, semantic_score)
        existing.last_seen_at = now
        return existing
    # Different bearing bin: caller should create a new frontier instead.
    return TopologicalFrontier(
        frontier_id=f"F{abs(hash((parent_place_id, round(bearing_deg / 30.0)))) % 100000:05d}",
        parent_place_id=parent_place_id,
        bearing_deg=bearing_deg,
        information_gain=information_gain,
        semantic_score=semantic_score,
        created_at=now,
        last_seen_at=now,
    )