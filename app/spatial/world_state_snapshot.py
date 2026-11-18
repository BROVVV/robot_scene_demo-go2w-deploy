"""Immutable, versioned world-state envelopes for search decisions."""

from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import json
import time
from typing import Any


SCHEMA_VERSION = "go2w_world_state_v1"


@dataclass(frozen=True)
class WorldRevisions:
    map_id: str
    map_epoch: str
    graph_revision: int
    grid_revision: int
    semantic_revision: int
    frame_id: str = "map"
    sensor_profile_id: str = "unknown"

    def to_dict(self) -> dict[str, Any]:
        return {
            "map_id": self.map_id,
            "map_epoch": self.map_epoch,
            "graph_revision": self.graph_revision,
            "grid_revision": self.grid_revision,
            "semantic_revision": self.semantic_revision,
            "frame_id": self.frame_id,
            "sensor_profile_id": self.sensor_profile_id,
        }


@dataclass(frozen=True)
class WorldStateSnapshot:
    snapshot_id: str
    created_at: float
    revisions: WorldRevisions
    task: dict[str, Any]
    robot: dict[str, Any]
    map_summary: dict[str, Any]
    objects: tuple[dict[str, Any], ...] = ()
    topology: dict[str, Any] = field(default_factory=dict)
    search_memory: dict[str, Any] = field(default_factory=dict)
    candidates: tuple[dict[str, Any], ...] = ()
    feedback: dict[str, Any] = field(default_factory=dict)
    health: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "snapshot_id": self.snapshot_id,
            "created_at": self.created_at,
            **self.revisions.to_dict(),
            "task": _copy(self.task),
            "robot": _copy(self.robot),
            "map": _copy(self.map_summary),
            "objects": [_copy(item) for item in self.objects],
            "topology": _copy(self.topology),
            "search_memory": _copy(self.search_memory),
            "candidates": [_copy(item) for item in self.candidates],
            "feedback": _copy(self.feedback),
            "health": _copy(self.health),
        }


def build_world_state_snapshot(
    *,
    revisions: WorldRevisions,
    task: dict[str, Any] | None,
    robot: dict[str, Any] | None,
    map_summary: dict[str, Any] | None,
    objects: list[dict[str, Any]] | tuple[dict[str, Any], ...] | None,
    topology: dict[str, Any] | None,
    search_memory: dict[str, Any] | None = None,
    candidates: list[dict[str, Any]] | tuple[dict[str, Any], ...] | None = None,
    feedback: dict[str, Any] | None = None,
    health: dict[str, Any] | None = None,
    created_at: float | None = None,
) -> WorldStateSnapshot:
    """Create one coherent snapshot and derive a deterministic content ID."""
    timestamp = float(time.time() if created_at is None else created_at)
    body = {
        **revisions.to_dict(),
        "task": task or {},
        "robot": robot or {},
        "map": map_summary or {},
        "objects": list(objects or ()),
        "topology": topology or {},
        "search_memory": search_memory or {},
        "candidates": list(candidates or ()),
        "feedback": feedback or {},
        "health": health or {},
    }
    digest = hashlib.sha256(
        json.dumps(body, sort_keys=True, ensure_ascii=False, separators=(",", ":"),
                   default=str).encode("utf-8")
    ).hexdigest()[:16]
    return WorldStateSnapshot(
        snapshot_id=f"snapshot_{digest}",
        created_at=timestamp,
        revisions=revisions,
        task=_copy(task or {}),
        robot=_copy(robot or {}),
        map_summary=_copy(map_summary or {}),
        objects=tuple(_copy(item) for item in (objects or ())),
        topology=_copy(topology or {}),
        search_memory=_copy(search_memory or {}),
        candidates=tuple(_copy(item) for item in (candidates or ())),
        feedback=_copy(feedback or {}),
        health=_copy(health or {}),
    )


def compatible_revisions(left: WorldRevisions, right: WorldRevisions) -> bool:
    """Coordinates can only be mixed within the same map identity and epoch."""
    return (
        left.map_id == right.map_id
        and left.map_epoch == right.map_epoch
        and left.frame_id == right.frame_id
        and left.sensor_profile_id == right.sensor_profile_id
    )


def _copy(value: Any) -> Any:
    return json.loads(json.dumps(value, ensure_ascii=False, default=str))
