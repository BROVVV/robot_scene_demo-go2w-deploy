"""SemanticRoutePlanner: grid/topological route planner for long-term goals.

The planner decides *how to get there* once the semantic layer has decided
*where to go*.  It supports:

* metric grid A* on an RTAB-Map occupancy snapshot (with footprint inflation),
* topological fallback via PlaceGraph movement edges,
* route to a known persistent object through the Place that observed it.

The output is a JSON-safe :class:`RoutePlan` which the local executor can
consume either as a full metric route (future backends) or as the first
segment direction (current short-step Go2-W backend).
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Any

from app.spatial.models import SpatialMapSnapshot, SpatialPose
from app.spatial.place_graph import PlaceGraph
from app.spatial.semantic_object_map import SemanticObjectMap

GRID_PATH_UNKNOWN_COST = 3.0


@dataclass
class RoutePlan:
    route_id: str
    frame_id: str
    target_type: str
    target_id: str | None
    target_position: tuple[float, float] | None
    waypoints: list[tuple[float, float]] = field(default_factory=list)
    place_sequence: list[str] = field(default_factory=list)
    path_length_m: float | None = None
    reachable: bool = False
    planner_source: str = "none"
    cost_components: dict[str, float] = field(default_factory=dict)
    provenance: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "route_id": self.route_id,
            "frame_id": self.frame_id,
            "target_type": self.target_type,
            "target_id": self.target_id,
            "target_position": list(self.target_position) if self.target_position else None,
            "waypoints": [list(p) for p in self.waypoints],
            "place_sequence": self.place_sequence,
            "path_length_m": self.path_length_m,
            "reachable": self.reachable,
            "planner_source": self.planner_source,
            "cost_components": self.cost_components,
            "provenance": self.provenance,
        }

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "RoutePlan":
        position = value.get("target_position")
        waypoints = value.get("waypoints") or []
        return cls(
            route_id=str(value.get("route_id") or "route"),
            frame_id=str(value.get("frame_id") or "map"),
            target_type=str(value.get("target_type") or ""),
            target_id=value.get("target_id"),
            target_position=tuple(position) if position else None,
            waypoints=[tuple(p) for p in waypoints],
            place_sequence=list(value.get("place_sequence") or []),
            path_length_m=value.get("path_length_m"),
            reachable=bool(value.get("reachable", False)),
            planner_source=str(value.get("planner_source") or "none"),
            cost_components=dict(value.get("cost_components") or {}),
            provenance=dict(value.get("provenance") or {}),
        )


class SemanticRoutePlanner:
    def __init__(
        self,
        *,
        inflation_radius_m: float = 0.25,
        allow_unknown: bool = False,
        max_waypoints: int = 32,
        resolution_m: float = 0.05,
    ) -> None:
        self.inflation_radius_m = float(inflation_radius_m)
        self.allow_unknown = bool(allow_unknown)
        self.max_waypoints = int(max_waypoints)
        self.resolution_m = float(resolution_m)
        self._sequence = 0

    def plan(
        self,
        *,
        start_pose: SpatialPose | None,
        target_type: str,
        target_id: str | None = None,
        target_position: tuple[float, float] | None = None,
        map_snapshot: SpatialMapSnapshot | None = None,
        place_graph: PlaceGraph | None = None,
        object_map: SemanticObjectMap | None = None,
        frame_id: str = "map",
    ) -> RoutePlan | None:
        """Plan a route from the current robot pose to a target.

        Priority:
        1. metric grid A* when a map and both start/goal positions exist,
        2. topological PlaceGraph route towards the Place that observed a
           target object or towards a frontier associated with a Place,
        3. direct bearing-only fallback (target unreachable / no map).
        """
        self._sequence += 1
        route_id = f"route_{self._sequence:03d}"
        start = (start_pose.x, start_pose.y) if start_pose is not None else None
        target_pos = target_position
        target_pos = self._resolve_target_position(
            target_type, target_id, object_map, place_graph, target_position
        )
        if start is None:
            return self._fallback_route(
                route_id=route_id, frame_id=frame_id, target_type=target_type,
                target_id=target_id, target_position=target_pos,
                place_graph=place_graph, reason="no robot pose",
            )
        if map_snapshot is not None and target_pos is not None:
            route = self._plan_grid(
                route_id=route_id, frame_id=frame_id, start=start,
                target=target_pos, map_snapshot=map_snapshot,
                target_type=target_type, target_id=target_id,
            )
            if route is not None:
                return route
        if place_graph is not None:
            route = self._plan_topological(
                route_id=route_id, frame_id=frame_id, start=start,
                target_type=target_type, target_id=target_id,
                target_pos=target_pos, place_graph=place_graph,
                object_map=object_map,
            )
            if route is not None:
                return route
        return self._fallback_route(
            route_id=route_id, frame_id=frame_id, target_type=target_type,
            target_id=target_id, target_position=target_pos,
            place_graph=place_graph, reason="no metric route available",
        )

    def _resolve_target_position(
        self,
        target_type: str,
        target_id: str | None,
        object_map: SemanticObjectMap | None,
        place_graph: PlaceGraph | None,
        fallback: tuple[float, float] | None,
    ) -> tuple[float, float] | None:
        if fallback is not None:
            return tuple(float(v) for v in fallback)
        if target_type in {"OBJECT", "TARGET_OBJECT"} and object_map is not None and target_id:
            entry = object_map.objects.get(target_id)
            if entry and entry.map_xyz is not None:
                return (entry.map_xyz[0], entry.map_xyz[1])
        if target_type in {"FRONTIER", "FRONTIER_CANDIDATE"} and place_graph is not None and target_id:
            # Frontier positions are stored outside PlaceGraph; caller should
            # normally pass target_position explicitly for frontiers.
            for place in place_graph.places.values():
                for edge in place_graph.edges:
                    if edge.from_place == place.place_id and edge.to_place == target_id:
                        if place.pose:
                            return (place.pose.x, place.pose.y)
        return None

    def _plan_grid(
        self,
        *,
        route_id: str,
        frame_id: str,
        start: tuple[float, float],
        target: tuple[float, float],
        map_snapshot: SpatialMapSnapshot,
        target_type: str,
        target_id: str | None,
    ) -> RoutePlan | None:
        grid, occupied, unknown = self._build_grid(map_snapshot)
        if grid is None:
            return None
        res = map_snapshot.resolution_m or self.resolution_m
        origin = map_snapshot.origin
        start_cell = self._to_cell(start, origin, res)
        target_cell = self._to_cell(target, origin, res)
        if start_cell is None or target_cell is None:
            return None
        start_cell = self._clamp_cell(start_cell, grid)
        target_cell = self._clamp_cell(target_cell, grid)
        if self._collides(target_cell, grid, occupied):
            # Goal cell occupied: search nearest free neighbor.
            target_cell = self._nearest_free_cell(target_cell, grid, occupied)
            if target_cell is None:
                return None
        path = self._astar(start_cell, target_cell, grid, occupied, unknown, allow_unknown=self.allow_unknown)
        if path is None:
            return None
        waypoints = [self._to_world(cell, origin, res) for cell in path]
        waypoints = waypoints[:: max(1, len(waypoints) // max(1, self.max_waypoints))]
        path_length = sum(
            math.dist(waypoints[i], waypoints[i + 1])
            for i in range(len(waypoints) - 1)
        )
        return RoutePlan(
            route_id=route_id,
            frame_id=frame_id,
            target_type=target_type,
            target_id=target_id,
            target_position=target,
            waypoints=waypoints,
            place_sequence=[],
            path_length_m=round(path_length, 3),
            reachable=True,
            planner_source="grid_astar",
            cost_components={"path_length_m": round(path_length, 3), "cells": len(path)},
            provenance={"map_revision": map_snapshot.revision},
        )

    def _plan_topological(
        self,
        *,
        route_id: str,
        frame_id: str,
        start: tuple[float, float],
        target_type: str,
        target_id: str | None,
        target_pos: tuple[float, float] | None,
        place_graph: PlaceGraph,
        object_map: SemanticObjectMap | None,
    ) -> RoutePlan | None:
        # Build adjacency from PlaceGraph movement edges.
        adj: dict[str, list[tuple[str, float]]] = {}
        for place in place_graph.places.values():
            adj.setdefault(place.place_id, [])
        for edge in place_graph.edges:
            adj.setdefault(edge.from_place, []).append(
                (edge.to_place, edge.observed_displacement_m or 1.0)
            )
            adj.setdefault(edge.to_place, []).append(
                (edge.from_place, edge.observed_displacement_m or 1.0)
            )
        current_place_id = place_graph.current_place().place_id if place_graph.current_place() else None
        if current_place_id is None:
            return None
        target_place_id = None
        if target_type in {"FRONTIER", "FRONTIER_CANDIDATE"} and target_id:
            for place in place_graph.places.values():
                if place.place_id == target_id:
                    target_place_id = place.place_id
                    break
            if target_place_id is None and target_pos is not None:
                target_place_id, _ = place_graph.nearest_place_id(
                    SpatialPose(x=target_pos[0], y=target_pos[1]), max_distance=2.0
                )
        elif target_type in {"OBJECT", "TARGET_OBJECT"} and object_map is not None and target_id:
            entry = object_map.objects.get(target_id)
            if entry and entry.seen_from_places:
                target_place_id = entry.seen_from_places[-1]
        if target_place_id is None:
            return None
        path = self._dijkstra(current_place_id, target_place_id, adj)
        if not path:
            return None
        waypoints: list[tuple[float, float]] = []
        place_sequence = path
        for place_id in path:
            place = place_graph.places.get(place_id)
            if place and place.pose:
                waypoints.append((place.pose.x, place.pose.y))
        path_len = sum(
            math.dist(waypoints[i], waypoints[i + 1]) for i in range(len(waypoints) - 1)
        )
        return RoutePlan(
            route_id=route_id,
            frame_id=frame_id,
            target_type=target_type,
            target_id=target_id,
            target_position=target_pos,
            waypoints=waypoints,
            place_sequence=place_sequence,
            path_length_m=round(path_len, 3) if waypoints else None,
            reachable=bool(waypoints),
            planner_source="place_graph_topological",
            cost_components={"path_length_m": round(path_len, 3) if waypoints else 0.0},
            provenance={"target_place": target_place_id},
        )

    def _fallback_route(
        self,
        *,
        route_id: str,
        frame_id: str,
        target_type: str,
        target_id: str | None,
        target_position: tuple[float, float] | None,
        place_graph: PlaceGraph | None,
        reason: str,
    ) -> RoutePlan:
        return RoutePlan(
            route_id=route_id,
            frame_id=frame_id,
            target_type=target_type,
            target_id=target_id,
            target_position=target_position,
            waypoints=[],
            place_sequence=[],
            path_length_m=None,
            reachable=False,
            planner_source="unavailable",
            cost_components={},
            provenance={"reason": reason, "place_graph": place_graph is not None},
        )

    # ------------------------------------------------------------------ #
    # Grid helpers                                                        #
    # ------------------------------------------------------------------ #
    def _build_grid(
        self, snapshot: SpatialMapSnapshot
    ) -> tuple[list[list[int]], set[tuple[int, int]], set[tuple[int, int]]] | None:
        if snapshot.width <= 0 or snapshot.height <= 0:
            return None
        grid = [[0 for _ in range(snapshot.width)] for _ in range(snapshot.height)]
        for x, y in snapshot.occupied:
            if 0 <= y < snapshot.height and 0 <= x < snapshot.width:
                grid[y][x] = 1
        for x, y in snapshot.unknown:
            if 0 <= y < snapshot.height and 0 <= x < snapshot.width and grid[y][x] == 0:
                grid[y][x] = 2
        # Inflate obstacles by the robot footprint radius.
        inflation_cells = max(1, int(round(self.inflation_radius_m / (snapshot.resolution_m or self.resolution_m))))
        occupied = {(x, y) for x, y in snapshot.occupied}
        grown: set[tuple[int, int]] = set()
        for x, y in list(occupied):
            for dx in range(-inflation_cells, inflation_cells + 1):
                for dy in range(-inflation_cells, inflation_cells + 1):
                    if dx * dx + dy * dy <= inflation_cells * inflation_cells:
                        grown.add((x + dx, y + dy))
        for (x, y) in list(grown):
            if 0 <= y < snapshot.height and 0 <= x < snapshot.width:
                grid[y][x] = 1
        unknown = {(x, y) for x, y in snapshot.unknown}
        return grid, grown, unknown

    def _to_cell(
        self, point: tuple[float, float], origin: tuple[float, float], res: float
    ) -> tuple[int, int] | None:
        return (int(math.floor((point[0] - origin[0]) / res)), int(math.floor((point[1] - origin[1]) / res)))

    def _clamp_cell(
        self, cell: tuple[int, int], grid: list[list[int]]
    ) -> tuple[int, int]:
        x = max(0, min(len(grid[0]) - 1, cell[0]))
        y = max(0, min(len(grid) - 1, cell[1]))
        return x, y

    def _collides(
        self, cell: tuple[int, int], grid: list[list[int]], occupied: set[tuple[int, int]]
    ) -> bool:
        x, y = cell
        if y < 0 or y >= len(grid) or x < 0 or x >= len(grid[0]):
            return True
        return grid[y][x] == 1

    def _nearest_free_cell(
        self, cell: tuple[int, int], grid: list[list[int]], occupied: set[tuple[int, int]]
    ) -> tuple[int, int] | None:
        best: tuple[int, int] | None = None
        best_d = float("inf")
        for y in range(len(grid)):
            for x in range(len(grid[0])):
                if grid[y][x] == 1:
                    continue
                d = (x - cell[0]) ** 2 + (y - cell[1]) ** 2
                if d < best_d:
                    best_d = d
                    best = (x, y)
        return best

    def _astar(
        self,
        start: tuple[int, int],
        goal: tuple[int, int],
        grid: list[list[int]],
        occupied: set[tuple[int, int]],
        unknown: set[tuple[int, int]],
        *,
        allow_unknown: bool,
    ) -> list[tuple[float, float]] | None:
        height = len(grid)
        width = len(grid[0])
        open_heap: list[tuple[float, int, int, tuple[int, int]]] = []
        start_h = self._heuristic(start, goal)
        heapq.heappush(open_heap, (start_h, 0, 0, start))
        came_from: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
        g_score: dict[tuple[int, int], float] = {start: 0.0}
        directions = [
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1),
        ]
        while open_heap:
            _, _, _, current = heapq.heappop(open_heap)
            if current == goal:
                path: list[tuple[float, float]] = []
                node: tuple[int, int] | None = current
                while node is not None:
                    path.append((float(node[0]), float(node[1])))
                    node = came_from.get(node)
                path.reverse()
                return path
            for dx, dy in directions:
                nx, ny = current[0] + dx, current[1] + dy
                if ny < 0 or ny >= height or nx < 0 or nx >= width:
                    continue
                if grid[ny][nx] == 1:
                    continue
                if grid[ny][nx] == 2 and not allow_unknown:
                    continue
                neighbor = (nx, ny)
                step_cost = math.hypot(dx, dy)
                if grid[ny][nx] == 2:
                    step_cost *= GRID_PATH_UNKNOWN_COST
                tentative = g_score[current] + step_cost
                if tentative < g_score.get(neighbor, float("inf")):
                    g_score[neighbor] = tentative
                    came_from[neighbor] = current
                    priority = tentative + self._heuristic(neighbor, goal)
                    heapq.heappush(open_heap, (priority, tentative, abs(nx) + abs(ny), neighbor))
        return None

    @staticmethod
    def _heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _to_world(
        self, cell: tuple[int, int], origin: tuple[float, float], res: float
    ) -> tuple[float, float]:
        return (
            round(origin[0] + (cell[0] + 0.5) * res, 3),
            round(origin[1] + (cell[1] + 0.5) * res, 3),
        )

    def _dijkstra(
        self, start: str, goal: str, adj: dict[str, list[tuple[str, float]]]
    ) -> list[str]:
        heap: list[tuple[float, str]] = [(0.0, start)]
        dist: dict[str, float] = {start: 0.0}
        prev: dict[str, str | None] = {start: None}
        while heap:
            d, node = heapq.heappop(heap)
            if node == goal:
                path: list[str] = []
                cur: str | None = node
                while cur is not None:
                    path.append(cur)
                    cur = prev.get(cur)
                path.reverse()
                return path
            for neighbor, cost in adj.get(node, []):
                nd = d + cost
                if nd < dist.get(neighbor, float("inf")):
                    dist[neighbor] = nd
                    prev[neighbor] = node
                    heapq.heappush(heap, (nd, neighbor))
        return []