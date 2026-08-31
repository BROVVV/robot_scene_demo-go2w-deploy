"""Unit tests for live exploration candidate generation (plan section 9)."""

from __future__ import annotations

import unittest

from app.navigation.candidate_goal_generator import (
    generate_live_exploration_candidates,
)
from app.navigation.exploration_config import CandidateConfig
from app.navigation.exploration_graph import ExplorationGraph, ObservationNode
from app.navigation.models import (
    GOAL_INSPECT_ANCHOR,
    GOAL_RELATIVE_MOVE,
    GOAL_REVISIT_NODE,
    GOAL_ROTATE_VIEW,
    LiveObservation,
)
from app.navigation.robot_backend import RobotCapabilities
from app.reasoning.semantic_navigation.models import (
    SearchDirective,
    SearchDirectiveKind,
)


def _observation(objects=None, target_present: bool = False,
                 bundle_id: str = "obs_1") -> LiveObservation:
    objects = objects or []
    return LiveObservation(
        bundle_id=bundle_id,
        timestamp=1.0,
        scene_objects=[
            {
                "label": label, "label_zh": label, "name": label,
                "position_2d": "center",
                "bbox_2d": [0.4, 0.3, 0.6, 0.7],
            }
            for label in objects
        ],
        scene_graph={"nodes": [], "edges": []},
        target_match={"target_present": target_present},
        pose={"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
        heading_sector=0,
        provenance={"source": "test"},
    )


def _graph() -> ExplorationGraph:
    graph = ExplorationGraph(session_id="t")
    graph.add_observation(
        ObservationNode(node_id="node_a", timestamp=1.0, heading_sector=3,
                        heading=90.0, objects=["water dispenser"],
                        reachable_state="OBSERVED")
    )
    return graph


class TestLiveCandidateGoalGenerator(unittest.TestCase):
    def test_unvisited_sectors_generated(self) -> None:
        obs = _observation()
        graph = ExplorationGraph(session_id="t")
        goals = generate_live_exploration_candidates(
            observation=obs, graph=graph, current_yaw_deg=0.0,
            config=CandidateConfig(heading_sectors=12),
        )
        rotate = [g for g in goals if g.goal_type == GOAL_ROTATE_VIEW]
        self.assertGreaterEqual(len(rotate), 3)
        # Current sector (0) must not appear as a zero-turn candidate.
        self.assertTrue(all(abs(g.relative_dyaw or 0.0) >= 1.0 for g in rotate))

    def test_directive_inspect_anchor(self) -> None:
        obs = _observation(objects=["water dispenser"])
        directive = SearchDirective(
            directive_id="d1", kind=SearchDirectiveKind.INSPECT_ANCHOR,
            source_backend="semantic_navigation", match_state="partial_match",
            confidence=0.8, preferred_heading_delta_deg=25.0,
            anchor_label="water dispenser",
        )
        goals = generate_live_exploration_candidates(
            observation=obs, graph=_graph(), directive=directive,
            current_yaw_deg=0.0,
        )
        inspect = [g for g in goals if g.goal_type == GOAL_INSPECT_ANCHOR]
        self.assertTrue(inspect)
        self.assertEqual(inspect[0].semantic_anchor, "water dispenser")
        self.assertGreaterEqual(inspect[0].semantic_relevance, 0.4)

    def test_anchor_in_view_boosts_inspect_goal(self) -> None:
        obs = _observation(objects=["water dispenser"])
        goals = generate_live_exploration_candidates(
            observation=obs, graph=_graph(), anchor_labels=["water dispenser"],
            current_yaw_deg=0.0,
        )
        inspect = [g for g in goals if g.goal_type == GOAL_INSPECT_ANCHOR]
        self.assertTrue(inspect)
        self.assertEqual(inspect[0].provenance.get("source"), "semantic_anchor")
        self.assertGreaterEqual(inspect[0].semantic_relevance, 0.7)

    def test_graph_interest_node_revisit(self) -> None:
        obs = _observation()
        graph = _graph()
        graph.mark_semantic_interest("node_a", anchor="water dispenser")
        goals = generate_live_exploration_candidates(
            observation=obs, graph=graph, current_yaw_deg=0.0,
        )
        revisits = [g for g in goals if g.goal_type == GOAL_REVISIT_NODE]
        self.assertTrue(revisits)
        self.assertEqual(revisits[0].target_node_id, "node_a")

    def test_last_known_candidate_revisit(self) -> None:
        obs = _observation(target_present=False)
        graph = _graph()
        graph.mark_target_candidate("node_a")
        goals = generate_live_exploration_candidates(
            observation=obs, graph=graph, current_yaw_deg=0.0,
        )
        revisits = [g for g in goals if g.goal_type == GOAL_REVISIT_NODE
                    and g.provenance.get("source") == "last_known"]
        self.assertTrue(revisits)

    def test_fallback_forward_requires_capability(self) -> None:
        obs = _observation()
        graph = ExplorationGraph(session_id="t")
        caps_relative = RobotCapabilities(supports_relative_translation=True)
        goals = generate_live_exploration_candidates(
            observation=obs, graph=graph, capabilities=caps_relative,
            current_yaw_deg=0.0, max_candidates=50,
        )
        self.assertTrue(any(g.goal_type == GOAL_RELATIVE_MOVE for g in goals))
        caps_none = RobotCapabilities()
        goals2 = generate_live_exploration_candidates(
            observation=obs, graph=graph, capabilities=caps_none,
            current_yaw_deg=0.0, max_candidates=50,
        )
        self.assertFalse(any(g.goal_type == GOAL_RELATIVE_MOVE for g in goals2))

    def test_negative_memory_penalty_recorded(self) -> None:
        class FakeNegativeMemory:
            def sector_penalty(self, key, sector):
                return 0.7, ["ev_1"]

        obs = _observation()
        goals = generate_live_exploration_candidates(
            observation=obs, graph=_graph(), negative_memory=FakeNegativeMemory(),
            negative_target_key="target", current_yaw_deg=30.0,
        )
        penalized = [g for g in goals
                     if g.provenance.get("negative_memory_penalty")]
        self.assertTrue(penalized)
        self.assertIn("ev_1", penalized[0].provenance.get("negative_memory_refs", []))

    def test_turn_only_removes_forward(self) -> None:
        obs = _observation()
        goals = generate_live_exploration_candidates(
            observation=obs, graph=ExplorationGraph(session_id="t"),
            capabilities=RobotCapabilities(supports_relative_translation=True),
            current_yaw_deg=0.0, turn_only=True, max_candidates=50,
        )
        self.assertFalse(any(g.goal_type == GOAL_RELATIVE_MOVE for g in goals))

    def test_candidate_dedupe(self) -> None:
        obs = _observation()
        graph = ExplorationGraph(session_id="t")
        goals = generate_live_exploration_candidates(
            observation=obs, graph=graph, current_yaw_deg=0.0,
        )
        keys = [(g.goal_type, g.heading_sector, g.target_node_id) for g in goals]
        self.assertEqual(len(keys), len(set(keys)))


if __name__ == "__main__":
    unittest.main()
