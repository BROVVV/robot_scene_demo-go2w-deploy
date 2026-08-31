"""Unit tests for the session-aware live exploration planner (plan section 10)."""

from __future__ import annotations

import unittest

from app.navigation.exploration_config import ScoringWeights
from app.navigation.exploration_graph import ExplorationGraph, ObservationNode
from app.navigation.exploration_planner import (
    score_exploration_goal,
    select_exploration_goal,
)
from app.navigation.models import (
    GOAL_INSPECT_ANCHOR,
    GOAL_RELATIVE_MOVE,
    GOAL_REVISIT_NODE,
    GOAL_ROTATE_VIEW,
    ExplorationGoal,
)
from app.navigation.robot_backend import NavigationStatus, navigation_result


def _goal(goal_type: str, *, sector: int | None = None, node_id: str | None = None,
          dyaw: float | None = None, relevance: float = 0.0,
          info: float = 0.0, source: str = "test") -> ExplorationGoal:
    return ExplorationGoal(
        goal_id=f"g_{goal_type}_{sector}_{node_id}",
        goal_type=goal_type,
        heading_sector=sector,
        target_node_id=node_id,
        relative_dyaw=dyaw,
        semantic_relevance=relevance,
        expected_information_gain=info,
        provenance={"source": source},
    )


def _graph_with_node() -> ExplorationGraph:
    graph = ExplorationGraph(session_id="t")
    graph.add_observation(
        ObservationNode(node_id="n1", timestamp=1.0, heading_sector=3)
    )
    return graph


class TestLiveExplorationPlanner(unittest.TestCase):
    def test_semantic_anchor_beats_unrelated(self) -> None:
        graph = ExplorationGraph(session_id="t")
        anchor_goal = _goal(GOAL_INSPECT_ANCHOR, sector=3, dyaw=25.0,
                            relevance=0.8, source="semantic_anchor")
        plain_goal = _goal(GOAL_ROTATE_VIEW, sector=4, dyaw=30.0,
                           relevance=0.0, source="unvisited_sector")
        scored = select_exploration_goal([plain_goal, anchor_goal], graph=graph)
        self.assertEqual(scored.goal.goal_id, anchor_goal.goal_id)

    def test_visited_penalty_reduces_score(self) -> None:
        graph = _graph_with_node()
        graph.mark_visited("n1")
        first = score_exploration_goal(
            _goal(GOAL_REVISIT_NODE, sector=3, node_id="n1", dyaw=0.0),
            graph=graph,
        )
        graph.mark_visited("n1")
        second = score_exploration_goal(
            _goal(GOAL_REVISIT_NODE, sector=3, node_id="n1", dyaw=0.0),
            graph=graph,
        )
        self.assertLess(second.score, first.score)

    def test_negative_evidence_penalty(self) -> None:
        graph = _graph_with_node()
        graph.mark_negative("n1", reason="not found")
        goal = _goal(GOAL_REVISIT_NODE, sector=3, node_id="n1", dyaw=0.0)
        goal.provenance["negative_memory_penalty"] = 0.9
        scored = score_exploration_goal(goal, graph=graph)
        self.assertGreaterEqual(scored.components["negative_evidence_penalty"], 0.5)
        self.assertIn("负证据", " ".join(scored.reasons))

    def test_navigation_failure_penalty(self) -> None:
        graph = _graph_with_node()
        for _ in range(2):
            graph.record_navigation(
                navigation_result("g", NavigationStatus.FAILED),
                goal_type="ROTATE_VIEW", heading_sector=3,
            )
        scored = score_exploration_goal(
            _goal(GOAL_ROTATE_VIEW, sector=3, dyaw=10.0), graph=graph,
        )
        self.assertGreaterEqual(scored.components["navigation_failure_penalty"], 0.5)

    def test_oscillation_penalty(self) -> None:
        graph = ExplorationGraph(session_id="t")
        graph.recent_goals.append({"heading_sector": 6, "target_node_id": None})
        graph.recent_goals.append({"heading_sector": 7, "target_node_id": None})
        graph.recent_goals.append({"heading_sector": 6, "target_node_id": None})
        scored = score_exploration_goal(
            _goal(GOAL_ROTATE_VIEW, sector=6, dyaw=30.0), graph=graph,
        )
        self.assertGreaterEqual(scored.components["oscillation_penalty"], 0.5)
        self.assertIn("振荡", " ".join(scored.reasons))

    def test_weights_are_configurable(self) -> None:
        graph = ExplorationGraph(session_id="t")
        goal = _goal(GOAL_ROTATE_VIEW, sector=4, dyaw=30.0, relevance=0.5)
        heavy_semantic = ScoringWeights(semantic_relevance=5.0)
        base = score_exploration_goal(goal, graph=graph)
        heavy = score_exploration_goal(goal, graph=graph, weights=heavy_semantic)
        self.assertGreater(heavy.score, base.score)

    def test_exclusion_filters(self) -> None:
        graph = ExplorationGraph(session_id="t")
        goals = [
            _goal(GOAL_ROTATE_VIEW, sector=1, dyaw=30.0),
            _goal(GOAL_ROTATE_VIEW, sector=2, dyaw=60.0),
            _goal(GOAL_ROTATE_VIEW, sector=3, dyaw=90.0),
        ]
        scored = select_exploration_goal(goals, graph=graph,
                                         exclude_sectors={1, 2})
        self.assertEqual(scored.goal.heading_sector, 3)

    def test_forward_cost_higher_than_turn(self) -> None:
        graph = ExplorationGraph(session_id="t")
        turn = score_exploration_goal(
            _goal(GOAL_ROTATE_VIEW, sector=1, dyaw=30.0), graph=graph,
        )
        forward = score_exploration_goal(
            _goal(GOAL_RELATIVE_MOVE, dyaw=None), graph=graph,
        )
        self.assertGreater(forward.components["estimated_motion_cost"],
                           turn.components["estimated_motion_cost"])


if __name__ == "__main__":
    unittest.main()
