"""Metric frontier extraction from a free/unknown occupancy map.

The frontier definition is strictly ``Free <-> Unknown`` boundary, never a
heading sector.  This is a lightweight 4-connected component extractor; it is
not a full Nav2 planner.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Any

from app.spatial.models import FrontierCandidate, SpatialMapSnapshot, SpatialPose


class FrontierExtractor:
    def __init__(
        self,
        *,
        min_component_size: int = 10,
        max_candidates: int = 8,
        min_distance_m: float = 0.0,
        max_distance_m: float = 20.0,
    ) -> None:
        self.min_component_size = max(1, int(min_component_size))
        self.max_candidates = max(1, int(max_candidates))
        self.min_distance_m = float(min_distance_m)
        self.max_distance_m = float(max_distance_m)

    def extract(
        self,
        map_snapshot: SpatialMapSnapshot | None,
        robot_pose: SpatialPose | None = None,
    ) -> list[FrontierCandidate]:
        if map_snapshot is None or map_snapshot.width <= 0 or map_snapshot.height <= 0:
            return []
        grid = self._build_grid(map_snapshot)
        frontier_cells = self._boundary_cells(grid, map_snapshot.width, map_snapshot.height)
        components = self._connected_components(frontier_cells)
        candidates: list[FrontierCandidate] = []
        for i, cells in enumerate(components):
            if len(cells) < self.min_component_size:
                continue
            centroid = self._centroid(cells, map_snapshot)
            if robot_pose is not None:
                dx = centroid[0] - robot_pose.x
                dy = centroid[1] - robot_pose.y
                dist = math.hypot(dx, dy)
                if dist < self.min_distance_m or dist > self.max_distance_m:
                    continue
                bearing = math.degrees(math.atan2(dy, dx)) - math.degrees(robot_pose.yaw)
                bearing = (bearing + 180.0) % 360.0 - 180.0
            else:
                dist = None
                bearing = None
            size_score = min(1.0, len(cells) / max(20.0, self.min_component_size * 4))
            candidates.append(
                FrontierCandidate(
                    frontier_id=f"frontier_{i:02d}_{int(centroid[0] * 100)}_{int(centroid[1] * 100)}",
                    position=(round(centroid[0], 3), round(centroid[1], 3)),
                    frame=map_snapshot.source,
                    bearing_deg=round(bearing, 2) if bearing is not None else None,
                    distance_m=round(dist, 3) if dist is not None else None,
                    size_score=round(size_score, 4),
                    spatial_information_gain=round(size_score, 4),
                    reachable=True,
                    nearby_semantics=[],
                    provenance={"source": "metric_frontier_extractor", "cells": len(cells)},
                )
            )
        candidates.sort(key=lambda item: item.spatial_information_gain, reverse=True)
        return candidates[: self.max_candidates]

    @staticmethod
    def _build_grid(map_snapshot: SpatialMapSnapshot) -> list[list[int]]:
        grid = [[2] * map_snapshot.width for _ in range(map_snapshot.height)]
        for x, y in map_snapshot.free:
            if 0 <= x < map_snapshot.width and 0 <= y < map_snapshot.height:
                grid[y][x] = 0
        for x, y in map_snapshot.occupied:
            if 0 <= x < map_snapshot.width and 0 <= y < map_snapshot.height:
                grid[y][x] = 1
        return grid

    def _boundary_cells(
        self, grid: list[list[int]], width: int, height: int
    ) -> set[tuple[int, int]]:
        cells: set[tuple[int, int]] = set()
        for y in range(height):
            for x in range(width):
                if grid[y][x] != 0:
                    continue
                for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                    if 0 <= nx < width and 0 <= ny < height and grid[ny][nx] == 2:
                        cells.add((x, y))
                        break
        return cells

    def _connected_components(
        self, cells: set[tuple[int, int]]
    ) -> list[list[tuple[int, int]]]:
        visited: set[tuple[int, int]] = set()
        components: list[list[tuple[int, int]]] = []
        for start in cells:
            if start in visited:
                continue
            queue = deque([start])
            visited.add(start)
            comp: list[tuple[int, int]] = []
            while queue:
                x, y = queue.popleft()
                comp.append((x, y))
                for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                    neighbor = (nx, ny)
                    if neighbor in cells and neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)
            components.append(comp)
        return components

    def _centroid(
        self, cells: list[tuple[int, int]], map_snapshot: SpatialMapSnapshot
    ) -> tuple[float, float]:
        if not cells:
            return map_snapshot.origin
        ox, oy = map_snapshot.origin
        res = map_snapshot.resolution_m or 1.0
        sx = sum(x for x, _ in cells) / len(cells)
        sy = sum(y for _, y in cells) / len(cells)
        return (ox + sx * res, oy + sy * res)
