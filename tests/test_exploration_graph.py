"""Unit tests for the session spatial-semantic ExplorationGraph."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from app.navigation.exploration_graph import (
    ExplorationEdge,
    ExplorationGraph,
    NodeState,
    ObservationNode,
)
from app.navigation.robot_backend import NavigationStatus, navigation_result


def _node(node_id: str, sector: int | None = None, **kwargs) -> ObservationNode:
    return ObservationNode(node_id=node_id, timestamp=1.0,
                           heading_sector=sector, **kwargs)


class TestExplorationGraph(unittest.TestCase):
    def test_add_and_merge_observation(self) -> None:
        graph = ExplorationGraph(session_id="s1")
        graph.add_observation(_node("n1", sector=2, objects=["desk"]))
        graph.add_observation(_node("n1", sector=2, objects=["chair"]))
        node = graph.get_node("n1")
        self.assertEqual(node.objects, ["desk", "chair"])
        self.assertEqual(len(graph.nodes), 1)

    def test_sector_coverage_survives_merge(self) -> None:
        graph = ExplorationGraph(session_id="s1")
        graph.add_observation(_node("n1", sector=2))
        graph.add_observation(_node("n1", sector=2))
        self.assertEqual(graph.sector_visited_count(2), 1)
        self.assertEqual(graph.sector_visited_count(3), 0)

    def test_node_state_transitions(self) -> None:
        graph = ExplorationGraph(session_id="s1")
        graph.add_observation(_node("n1", sector=0))
        graph.mark_visited("n1")
        self.assertEqual(graph.get_node("n1").visited_count, 1)
        graph.mark_negative("n1", reason="not found")
        self.assertEqual(graph.get_node("n1").negative_evidence_count, 1)
        graph.mark_semantic_interest("n1", anchor="water dispenser")
        self.assertEqual(graph.get_node("n1").reachable_state,
                         NodeState.SEMANTIC_INTEREST.value)
        graph.mark_target_candidate("n1")
        self.assertEqual(graph.get_node("n1").reachable_state,
                         NodeState.TARGET_CANDIDATE.value)
        graph.mark_target_confirmed("n1")
        self.assertEqual(graph.get_node("n1").reachable_state,
                         NodeState.TARGET_CONFIRMED.value)

    def test_unreachable_after_failures(self) -> None:
        graph = ExplorationGraph(session_id="s1")
        graph.add_observation(_node("n1", sector=0))
        for _ in range(2):
            graph.record_navigation(
                navigation_result("g", NavigationStatus.FAILED),
                goal_type="ROTATE_VIEW", target_node_id="n1", heading_sector=0,
            )
        self.assertEqual(graph.get_node("n1").reachable_state,
                         NodeState.UNREACHABLE.value)
        self.assertNotIn(graph.get_node("n1"), graph.unvisited_nodes())

    def test_sector_failure_count(self) -> None:
        graph = ExplorationGraph(session_id="s1")
        graph.record_navigation(
            navigation_result("g1", NavigationStatus.FAILED),
            goal_type="ROTATE_VIEW", heading_sector=3,
        )
        graph.record_navigation(
            navigation_result("g2", NavigationStatus.FAILED),
            goal_type="ROTATE_VIEW", heading_sector=3,
        )
        graph.record_navigation(
            navigation_result("g3", NavigationStatus.SUCCEEDED),
            goal_type="ROTATE_VIEW", heading_sector=4,
        )
        self.assertEqual(graph.sector_failure_count(3), 2)
        self.assertEqual(graph.sector_failure_count(4), 0)

    def test_edges_recorded(self) -> None:
        graph = ExplorationGraph(session_id="s1")
        graph.add_observation(_node("n1", sector=0))
        graph.add_observation(_node("n2", sector=1))
        graph.record_navigation(
            navigation_result("g1", NavigationStatus.SUCCEEDED,
                              observed_motion={"yaw_delta_deg": 30.0}),
            goal_type="ROTATE_VIEW", target_node_id="n2", source_node_id="n1",
            heading_sector=1,
        )
        self.assertEqual(len(graph.edges), 1)
        self.assertEqual(graph.edges[0].source_node_id, "n1")
        self.assertEqual(graph.edges[0].action_type, "ROTATE_VIEW")

    def test_oscillation_penalty(self) -> None:
        graph = ExplorationGraph(session_id="s1")
        graph.recent_goals.append({"heading_sector": 6, "target_node_id": None})
        graph.recent_goals.append({"heading_sector": 7, "target_node_id": None})
        graph.recent_goals.append({"heading_sector": 6, "target_node_id": None})
        # A-B-A repeating would continue the 2-cycle.
        self.assertGreater(graph.oscillation_penalty(6, None), 0.5)
        self.assertEqual(graph.oscillation_penalty(8, None), 0.0)

    def test_serialization_roundtrip(self) -> None:
        graph = ExplorationGraph(session_id="s1")
        graph.add_observation(_node("n1", sector=2, objects=["desk"]))
        graph.mark_visited("n1")
        graph.record_navigation(
            navigation_result("g1", NavigationStatus.SUCCEEDED),
            goal_type="ROTATE_VIEW", heading_sector=2,
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "graph.json"
            graph.save(path)
            loaded = ExplorationGraph.load(path)
        self.assertEqual(loaded.session_id, "s1")
        self.assertEqual(loaded.sector_visited_count(2), 1)
        self.assertEqual(len(loaded.nodes), 1)
        self.assertEqual(loaded.get_node("n1").visited_count, 1)
        self.assertEqual(len(loaded.recent_goals), 1)

    def test_semantic_neighbors(self) -> None:
        graph = ExplorationGraph(session_id="s1")
        graph.add_observation(_node("n1", objects=["water dispenser", "desk"]))
        graph.add_observation(_node("n2", objects=["chair"]))
        matches = graph.semantic_neighbors("water dispenser")
        self.assertEqual([node.node_id for node in matches], ["n1"])


if __name__ == "__main__":
    unittest.main()
