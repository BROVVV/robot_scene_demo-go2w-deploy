"""Spatial negative memory and frontier visit bookkeeping.

This memory tracks what has been tried spatially so PSG hypotheses can be
downgraded and blacklisted after repeated negative observations.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any


@dataclass
class FrontierMemoryEntry:
    frontier_id: str
    visit_count: int = 0
    selected_count: int = 0
    fail_count: int = 0
    semantic_gain_after_visit: float = 0.0
    blacklisted: bool = False

    def to_dict(self) -> dict[str, Any]:
        return {
            "frontier_id": self.frontier_id,
            "visit_count": self.visit_count,
            "selected_count": self.selected_count,
            "fail_count": self.fail_count,
            "semantic_gain_after_visit": self.semantic_gain_after_visit,
            "blacklisted": self.blacklisted,
        }


@dataclass
class RegionHypothesisMemory:
    region_id: str
    searched_places: list[str] = field(default_factory=list)
    searched_viewpoints: int = 0
    negative_count: int = 0
    confidence: float = 0.5
    state: str = "PREDICTED"

    def to_dict(self) -> dict[str, Any]:
        return {
            "region_id": self.region_id,
            "searched_places": self.searched_places,
            "searched_viewpoints": self.searched_viewpoints,
            "negative_count": self.negative_count,
            "confidence": self.confidence,
            "state": self.state,
        }


class SpatialMemory:
    def __init__(self) -> None:
        self.frontiers: dict[str, FrontierMemoryEntry] = {}
        self.regions: dict[str, RegionHypothesisMemory] = {}
        self.blacklist: set[str] = set()

    def mark_frontier_selected(self, frontier_id: str) -> None:
        entry = self.frontiers.setdefault(frontier_id, FrontierMemoryEntry(frontier_id))
        entry.selected_count += 1

    def mark_frontier_visited(self, frontier_id: str, *, gain: float = 0.0, failed: bool = False) -> None:
        entry = self.frontiers.setdefault(frontier_id, FrontierMemoryEntry(frontier_id))
        entry.visit_count += 1
        entry.semantic_gain_after_visit = max(entry.semantic_gain_after_visit, float(gain))
        if failed:
            entry.fail_count += 1
            if entry.fail_count >= 2:
                entry.blacklisted = True
                self.blacklist.add(frontier_id)

    def region_negative(self, region_id: str, *, place_id: str | None = None) -> None:
        entry = self.regions.setdefault(
            region_id, RegionHypothesisMemory(region_id=region_id)
        )
        entry.searched_viewpoints += 1
        entry.negative_count += 1
        if place_id and place_id not in entry.searched_places:
            entry.searched_places.append(place_id)
        entry.confidence = max(0.05, entry.confidence - 0.2)
        if entry.negative_count >= 3 or entry.confidence < 0.2:
            entry.state = "REJECTED"
            self.blacklist.add(region_id)

    def to_dict(self) -> dict[str, Any]:
        return {
            "frontiers": {k: v.to_dict() for k, v in self.frontiers.items()},
            "regions": {k: v.to_dict() for k, v in self.regions.items()},
            "blacklist": sorted(self.blacklist),
        }
