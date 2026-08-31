from __future__ import annotations

import unittest

from app.navigation.models import GOAL_RELATIVE_MOVE, GOAL_ROTATE_VIEW
from app.navigation.topology_route_executor import (
    TopologicalRoute,
    TopologyRouteExecutor,
    TopologyRoutePlanner,
)
from app.spatial.models import SpatialPose
from app.spatial.place_graph import PlaceGraph
from app.spatial.topological_frontier import TopologicalFrontier


def _graph() -> PlaceGraph:
    graph = PlaceGraph()
    graph.register_observation(
        observation_id="o1", heading_sector=0, objects=[],
        pose=SpatialPose(x=0.0, y=0.0, yaw=0.0),
        timestamp=1.0,
    )
    graph.register_observation(
        observation_id="o2", heading_sector=0, objects=[],
        pose=SpatialPose(x=1.0, y=0.0, yaw=0.0),
        observed_displacement_m=1.0, timestamp=2.0,
    )
    graph.register_observation(
        observation_id="o3", heading_sector=0, objects=[],
        pose=SpatialPose(x=2.0, y=0.0, yaw=0.0),
        observed_displacement_m=1.0, timestamp=3.0,
    )
    return graph


class TestTopologyRoutePlanner(unittest.TestCase):
    def test_plans_shortest_path(self) -> None:
        graph = _graph()
        pid = graph.current_place().place_id
        first = graph.places[pid]
        # Force current to first place by moving back.
        graph.register_observation(
            observation_id="o4", heading_sector=0, objects=[],
            pose=SpatialPose(x=0.0, y=0.0, yaw=0.0),
            timestamp=4.0,
        )
        current = graph.current_place().place_id
        target = first.place_id if current != first.place_id else graph.places[pid].place_id
        # Use the last place as target.
        target = pid
        route = TopologyRoutePlanner().plan(
            place_graph=graph,
            current_place_id=current,
            target_place_id=target,
        )
        self.assertIsNotNone(route)
        self.assertEqual(route.place_path[0], current)
        self.assertEqual(route.place_path[-1], target)

    def test_blocked_edge_is_skipped(self) -> None:
        graph = _graph()
        pids = list(graph.places.keys())
        graph.record_edge_failure(pids[0], pids[1], reason="blocked", max_failures=1)
        # Current at p0, target p2. Only p0->p1 blocked should force no path
        # unless there is an alternate (there is none), so plan returns None.
        route = TopologyRoutePlanner().plan(
            place_graph=graph,
            current_place_id=pids[0],
            target_place_id=pids[2],
        )
        self.assertIsNone(route)


class TestTopologyRouteExecutor(unittest.TestCase):
    def test_executor_only_first_edge(self) -> None:
        graph = _graph()
        pids = list(graph.places.keys())
        route = TopologicalRoute(
            route_id="r1",
            place_path=pids,
            edge_path=[],
            target_frontier_id=None,
            total_cost=2.0,
        )
        goal = TopologyRouteExecutor().next_goal(
            route=route,
            current_place_id=pids[0],
            place_graph=graph,
            current_yaw_deg=0.0,
        )
        self.assertIsNotNone(goal)
        self.assertIn(goal.goal_type, {GOAL_RELATIVE_MOVE, GOAL_ROTATE_VIEW})
        if goal.goal_type == GOAL_RELATIVE_MOVE:
            self.assertGreater(goal.relative_dx, 0.0)
        self.assertIn("first edge", goal.semantic_reason or "")

    def test_same_place_frontier_returns_local_goal(self) -> None:
        graph = _graph()
        current = graph.current_place().place_id
        frontier = TopologicalFrontier(
            frontier_id="F01", parent_place_id=current, bearing_deg=90.0
        )
        route = TopologicalRoute(
            route_id="r2",
            place_path=[current],
            edge_path=[],
            target_frontier_id=frontier.frontier_id,
            total_cost=0.0,
        )
        goal = TopologyRouteExecutor().next_goal(
            route=route,
            current_place_id=current,
            place_graph=graph,
            current_yaw_deg=0.0,
        )
        self.assertIsNotNone(goal)
        if goal.goal_type == GOAL_ROTATE_VIEW:
            self.assertGreater(goal.relative_dyaw, 0.0)


if __name__ == "__main__":
    unittest.main()