"""Bounded, anchor-aware ray evidence for a shadow navigation grid.

Localization keyframes and mapping evidence have different update rates.  This
module stores observations relative to a stable keyframe, then rebuilds the
grid with the latest optimized keyframe poses.  It intentionally has no ROS
publisher: production can compare the shadow result before selecting a single
``/map`` authority.
"""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass
import math
from typing import Any, Iterable

import numpy as np

from app.slam.keyframe_graph import KeyframeGraphStore


@dataclass(frozen=True)
class GridSpec:
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int

    def __post_init__(self) -> None:
        if self.resolution <= 0 or not math.isfinite(self.resolution):
            raise ValueError("resolution must be finite and positive")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("grid dimensions must be positive")


@dataclass(frozen=True)
class AnchoredRayObservation:
    observation_id: str
    anchor_id: str
    map_id: str
    map_epoch: str
    source_stamp: float
    origin_in_anchor: tuple[float, float]
    endpoints_in_anchor: tuple[tuple[float, float], ...]
    endpoint_is_hit: tuple[bool, ...]
    correlation_group: str
    source: str = "lidar"

    def __post_init__(self) -> None:
        if len(self.endpoints_in_anchor) != len(self.endpoint_is_hit):
            raise ValueError("every endpoint needs an explicit hit/miss flag")
        if not self.observation_id or not self.anchor_id or not self.correlation_group:
            raise ValueError("observation, anchor, and correlation ids are required")


@dataclass(frozen=True)
class EvidenceGridResult:
    grid: np.ndarray
    hit_counts: np.ndarray
    miss_counts: np.ndarray
    used_observations: int
    skipped_observations: int
    graph_revision: int
    map_id: str
    map_epoch: str


class AnchoredEvidenceGrid:
    def __init__(
        self, spec: GridSpec, *, max_observations_per_anchor: int = 120,
        max_evidence_per_cell: int = 20, occupied_min_hits: int = 2,
        clear_miss_ratio: float = 3.0,
    ) -> None:
        self.spec = spec
        self.max_observations_per_anchor = max(1, int(max_observations_per_anchor))
        self.max_evidence_per_cell = max(1, int(max_evidence_per_cell))
        self.occupied_min_hits = max(1, int(occupied_min_hits))
        self.clear_miss_ratio = max(1.0, float(clear_miss_ratio))
        self._by_anchor: dict[str, deque[AnchoredRayObservation]] = defaultdict(
            lambda: deque(maxlen=self.max_observations_per_anchor)
        )
        self._ids: set[str] = set()

    def add(self, observation: AnchoredRayObservation) -> bool:
        """Add exactly once; duplicate publications cannot increase evidence."""
        if observation.observation_id in self._ids:
            return False
        bucket = self._by_anchor[observation.anchor_id]
        evicted = bucket[0].observation_id if len(bucket) == bucket.maxlen else None
        bucket.append(observation)
        self._ids.add(observation.observation_id)
        if evicted is not None:
            self._ids.discard(evicted)
        return True

    def render(self, graph: KeyframeGraphStore) -> EvidenceGridResult:
        snapshot = graph.snapshot()
        if snapshot is None:
            return self._empty("", "", 0, skipped=self.observation_count)
        hits = np.zeros((self.spec.height, self.spec.width), dtype=np.uint16)
        misses = np.zeros_like(hits)
        hit_groups: dict[tuple[int, int], set[str]] = defaultdict(set)
        miss_groups: dict[tuple[int, int], set[str]] = defaultdict(set)
        used = skipped = 0
        for observation in self.observations():
            if (observation.map_id != snapshot.map_id
                    or observation.map_epoch != snapshot.map_epoch):
                skipped += 1
                continue
            origin = graph.reproject(
                observation.anchor_id, (*observation.origin_in_anchor, 0.0),
                map_id=observation.map_id, map_epoch=observation.map_epoch,
            )
            if origin is None:
                skipped += 1
                continue
            used += 1
            origin_cell = self._cell(origin[0], origin[1])
            if origin_cell is None:
                skipped += 1
                used -= 1
                continue
            for endpoint_local, is_hit in zip(
                observation.endpoints_in_anchor, observation.endpoint_is_hit
            ):
                endpoint = graph.reproject(
                    observation.anchor_id, (*endpoint_local, 0.0),
                    map_id=observation.map_id, map_epoch=observation.map_epoch,
                )
                if endpoint is None:
                    continue
                end_cell = self._cell(endpoint[0], endpoint[1])
                if end_cell is None:
                    continue
                ray = list(_bresenham(origin_cell, end_cell))
                free_cells = ray[:-1] if is_hit else ray
                for cell in free_cells:
                    miss_groups[cell].add(observation.correlation_group)
                if is_hit:
                    hit_groups[end_cell].add(observation.correlation_group)
        for (row, col), groups in hit_groups.items():
            hits[row, col] = min(self.max_evidence_per_cell, len(groups))
        for (row, col), groups in miss_groups.items():
            misses[row, col] = min(self.max_evidence_per_cell, len(groups))
        grid = np.full(hits.shape, -1, dtype=np.int8)
        occupied = (
            (hits >= self.occupied_min_hits)
            & (hits.astype(float) * self.clear_miss_ratio >= misses.astype(float))
        )
        free = (
            (misses >= 1)
            & (misses.astype(float) >= hits.astype(float) * self.clear_miss_ratio + 1.0)
        )
        grid[free] = 0
        grid[occupied] = 100
        return EvidenceGridResult(
            grid=grid, hit_counts=hits, miss_counts=misses,
            used_observations=used, skipped_observations=skipped,
            graph_revision=snapshot.graph_revision, map_id=snapshot.map_id,
            map_epoch=snapshot.map_epoch,
        )

    @property
    def observation_count(self) -> int:
        return sum(len(bucket) for bucket in self._by_anchor.values())

    def observations(self) -> Iterable[AnchoredRayObservation]:
        for anchor_id in sorted(self._by_anchor):
            yield from tuple(self._by_anchor[anchor_id])

    def _cell(self, x: float, y: float) -> tuple[int, int] | None:
        col = int(math.floor((x - self.spec.origin_x) / self.spec.resolution))
        row = int(math.floor((y - self.spec.origin_y) / self.spec.resolution))
        if row < 0 or col < 0 or row >= self.spec.height or col >= self.spec.width:
            return None
        return row, col

    def _empty(self, map_id: str, map_epoch: str, revision: int,
               *, skipped: int) -> EvidenceGridResult:
        shape = (self.spec.height, self.spec.width)
        return EvidenceGridResult(
            grid=np.full(shape, -1, dtype=np.int8),
            hit_counts=np.zeros(shape, dtype=np.uint16),
            miss_counts=np.zeros(shape, dtype=np.uint16),
            used_observations=0, skipped_observations=skipped,
            graph_revision=revision, map_id=map_id, map_epoch=map_epoch,
        )


def _bresenham(start: tuple[int, int], end: tuple[int, int]):
    """Yield (row, col), including both endpoints."""
    row0, col0 = start
    row1, col1 = end
    dx = abs(col1 - col0)
    sx = 1 if col0 < col1 else -1
    dy = -abs(row1 - row0)
    sy = 1 if row0 < row1 else -1
    error = dx + dy
    while True:
        yield row0, col0
        if row0 == row1 and col0 == col1:
            break
        twice = 2 * error
        if twice >= dy:
            error += dy
            col0 += sx
        if twice <= dx:
            error += dx
            row0 += sy
