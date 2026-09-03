"""Topological route planner and one-edge executor.

The planner computes a Place/Edge path through :class:`PlaceGraph`.  The
executor consumes only the first edge of that route and returns a local
primitive; the outer loop always re-observes and replans after each step.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import asdict, dataclass, field
from typing import Any
from uuid import uuid4

from app.navigation.models import (
    GOAL_RELATIVE_MOVE,
    GOAL_ROTATE_VIEW,
    ExplorationGoal,
)
from app.spatial.models import MovementEdge
from app.spatial.place_graph import PlaceGraph
from app.spatial.topological_frontier import TopologicalFrontier

EDGE_STATUS_OPEN = "OPEN"
EDGE_STATUS_DEGRADED = "DEGRADED"
EDGE_STATUS_BLOCKED = "BLOCKED"
EDGE_STATUS_STALE = "STALE"


@dataclass
class TopologicalRoute:
    route_id: str
    place_path: list[str]
    edge_path: list[str]
    target_frontier_id: str | None = None
    total_cost: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class TopologyRoutePlanner:
    max_edge_failures: int = 2
    blocked_edge_retry_sec: float = 120.0

    def plan(
        self,
        *,
        place_graph: PlaceGraph,
        current_place_id: str,
        frontier: TopologicalFrontier | None = None,
        target_place_id: str | None = None,
        now: float | None = None,
    ) -> TopologicalRoute | None:
        places = place_graph.places
        if current_place_id not in places:
            return None
        target_id = target_place_id
        if target_id is None and frontier is not None:
            target_id = frontier.parent_place_id
        if target_id is None or target_id not in places:
            return None
        if target_id == current_place_id:
            # Same-place frontier: no NAV_EDGE is required.
            return TopologicalRoute(
                route_id=f"route_{uuid4().hex[:10]}",
                place_path=[current_place_id],
                edge_path=[],
                target_frontier_id=frontier.frontier_id if frontier else None,
                total_cost=0.0,
            )
        graph: dict[str, list[tuple[float, str, str]]] = {
            pid: [] for pid in places
        }
        now = now if now is not None else __import__("time").time()
        for edge in place_graph.edges:
            if self._edge_blocked(edge, now):
                continue
            cost = self._edge_cost(edge)
            graph[edge.from_place].append((cost, edge.to_place, edge.edge_id))
            if not edge.provenance.get("directed"):
                graph[edge.to_place].append((cost, edge.from_place, edge.edge_id))
        dist = {current_place_id: 0.0}
        prev: dict[str, tuple[str, str] | None] = {
            current_place_id: None
        }
        heap = [(0.0, current_place_id)]
        while heap:
            d, node = heapq.heappop(heap)
            if node == target_id:
                break
            if d > dist.get(node, math.inf):
                continue
            for cost, nxt, eid in graph.get(node, []):
                nd = d + cost
                if nd < dist.get(nxt, math.inf):
                    dist[nxt] = nd
                    prev[nxt] = (node, eid)
                    heapq.heappush(heap, (nd, nxt))
        if target_id not in prev or prev[target_id] is None and target_id != current_place_id:
            return None
        # Reconstruct place/edge path.
        place_path = []
        edge_path = []
        cur: str | None = target_id
        while cur is not None:
            place_path.append(cur)
            entry = prev[cur]
            if entry is None:
                break
            _, eid = entry
            edge_path.append(eid)
            cur = entry[0]
        place_path.reverse()
        edge_path.reverse()
        return TopologicalRoute(
            route_id=f"route_{uuid4().hex[:10]}",
            place_path=place_path,
            edge_path=edge_path,
            target_frontier_id=frontier.frontier_id if frontier else None,
            total_cost=dist.get(target_id, 0.0),
        )

    @staticmethod
    def _edge_cost(edge: MovementEdge) -> float:
        base = max(0.1, float(edge.cost or edge.observed_displacement_m or 1.0))
        penalty = float(edge.failure_count) * 2.0
        if edge.status == EDGE_STATUS_DEGRADED:
            penalty += 3.0
        return base + penalty

    @staticmethod
    def _edge_blocked(edge: MovementEdge, now: float) -> bool:
        if edge.status == EDGE_STATUS_BLOCKED:
            if edge.last_failure_at is None:
                return True
            age = now - float(edge.last_failure_at)
            return age < 120.0
        return False


@dataclass
class TopologyRouteExecutor:
    """Return only the local primitive for the first route edge."""

    max_turn_deg: float = 30.0
    forward_step_m: float = 0.3
    # Frontier bearings are recomputed from every BEV frame, so the target angle
    # moves with the robot's own heading and the residual flips sign every cycle
    # while decaying by ~0.8.  Measured search_20260902_172911: cycle 4 starts at
    # -30 deg and the residual only drops under 5 deg on cycle 15, so 11 of a
    # 20-step budget go to pure head-swinging before the first advance.  A 15 deg
    # window closes on cycle 8 (-14.6 deg) instead, and one 0.3 m step at 15 deg
    # is off by under 0.08 m laterally.
    frontier_align_deg: float = 15.0

    def next_goal(
        self,
        *,
        route: TopologicalRoute,
        current_place_id: str,
        place_graph: PlaceGraph,
        current_yaw_deg: float = 0.0,
        capabilities: Any = None,
        frontier: TopologicalFrontier | None = None,
    ) -> ExplorationGoal | None:
        if not route or not route.place_path:
            return None
        if len(route.place_path) < 2:
            # Same-place frontier: rotate toward the frontier bearing.
            if route.target_frontier_id is not None:
                frontier = frontier or self._find_frontier(place_graph, route.target_frontier_id)
                bearing = frontier.bearing_deg if frontier is not None else current_yaw_deg
                delta = (bearing - current_yaw_deg + 180.0) % 360.0 - 180.0
                if abs(delta) > self.frontier_align_deg:
                    return ExplorationGoal(
                        goal_id=f"route_turn_{route.route_id}",
                        goal_type=GOAL_ROTATE_VIEW,
                        relative_dyaw=max(-self.max_turn_deg, min(self.max_turn_deg, delta)),
                        semantic_reason="same-place frontier rotate",
                        provenance={"route": route.to_dict()},
                    )
                return ExplorationGoal(
                    goal_id=f"route_move_{route.route_id}",
                    goal_type=GOAL_RELATIVE_MOVE,
                    relative_dx=self.forward_step_m,
                    semantic_reason="same-place frontier advance",
                    provenance={"route": route.to_dict()},
                )
            return None
        from_id = route.place_path[0]
        to_id = route.place_path[1]
        edge = self._find_edge(place_graph, from_id, to_id)
        if edge is None:
            return None
        target_place = place_graph.places.get(to_id)
        if target_place is None or target_place.pose is None:
            # Without a pose we still advance a short forward step in the
            # current heading toward the adjacent Place.
            return ExplorationGoal(
                goal_id=f"route_move_{route.route_id}",
                goal_type=GOAL_RELATIVE_MOVE,
                relative_dx=min(self.forward_step_m, 0.30),
                semantic_reason=f"advance along {edge.edge_id} to {to_id}",
                provenance={"route": route.to_dict(), "edge": edge.to_dict()},
            )
        target_yaw = math.degrees(target_place.pose.yaw)
        delta = (target_yaw - current_yaw_deg + 180.0) % 360.0 - 180.0
        if abs(delta) > 5.0:
            return ExplorationGoal(
                goal_id=f"route_turn_{route.route_id}",
                goal_type=GOAL_ROTATE_VIEW,
                relative_dyaw=max(-self.max_turn_deg, min(self.max_turn_deg, delta)),
                semantic_reason=f"rotate toward {to_id}",
                provenance={"route": route.to_dict(), "edge": edge.to_dict()},
            )
        return ExplorationGoal(
            goal_id=f"route_move_{route.route_id}",
            goal_type=GOAL_RELATIVE_MOVE,
            relative_dx=min(self.forward_step_m, edge.observed_displacement_m or self.forward_step_m),
            semantic_reason=f"execute first edge {edge.edge_id} -> {to_id}",
            provenance={"route": route.to_dict(), "edge": edge.to_dict()},
        )

    @staticmethod
    def _find_edge(
        place_graph: PlaceGraph, from_id: str, to_id: str
    ) -> MovementEdge | None:
        for edge in place_graph.edges:
            if (
                (edge.from_place == from_id and edge.to_place == to_id)
                or (
                    not edge.provenance.get("directed")
                    and edge.from_place == to_id
                    and edge.to_place == from_id
                )
            ):
                return edge
        return None

    @staticmethod
    def _find_frontier(
        place_graph: PlaceGraph, frontier_id: str
    ) -> TopologicalFrontier | None:
        # PlaceGraph doesn't store frontiers yet; the SemanticNavigationGraph
        # keeps them.  This helper is only for same-place rotation fallback.
        return None