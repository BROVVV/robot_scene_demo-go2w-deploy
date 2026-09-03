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

    def _same_place_goal(self, bearing_deg: float, yaw_deg: float):
        graph = _graph()
        current = graph.current_place().place_id
        frontier = TopologicalFrontier(
            frontier_id="F01", parent_place_id=current, bearing_deg=bearing_deg
        )
        route = TopologicalRoute(
            route_id="r-align",
            place_path=[current],
            edge_path=[],
            target_frontier_id=frontier.frontier_id,
            total_cost=0.0,
        )
        return TopologyRouteExecutor().next_goal(
            route=route,
            current_place_id=current,
            place_graph=graph,
            current_yaw_deg=yaw_deg,
            frontier=frontier,
        )

    def test_residual_bearing_error_advances_instead_of_oscillating(self) -> None:
        """搜索 search_20260902_172911 的回归：|delta| 落到 15° 内就必须前进。

        BEV frontier 每帧重算，目标角跟着机器狗朝向跑，残差每个 cycle 反号并
        按 ~0.8 衰减。旧的 5° 窗要等到残差降到 5° 才闭合：实测 cycle 4 起 -30°、
        16.1°、-14.6°、12.3°、-12.2°、7.9°、-7.0°、8.3°、-8.2°，直到 cycle 15
        才第一次 advance，20 步预算里 11 步是纯摆头。15° 窗在 -14.6° 就闭合。
        """
        for delta in (7.85, -7.0, 8.27, -8.15, 12.3, -14.55):
            goal = self._same_place_goal(bearing_deg=delta, yaw_deg=0.0)
            self.assertEqual(goal.goal_type, GOAL_RELATIVE_MOVE, msg=f"delta={delta}")
            self.assertGreater(goal.relative_dx, 0.0)
            self.assertIn("advance", goal.semantic_reason or "")

    def test_large_bearing_error_still_rotates_first(self) -> None:
        """对准窗放宽不等于取消对准：20° 以上仍先转向（cycle 6 实测 -20.32°）。"""
        goal = self._same_place_goal(bearing_deg=-20.32, yaw_deg=0.0)
        self.assertEqual(goal.goal_type, GOAL_ROTATE_VIEW)
        self.assertLess(goal.relative_dyaw, 0.0)
        self.assertGreaterEqual(goal.relative_dyaw, -30.0)


if __name__ == "__main__":
    unittest.main()