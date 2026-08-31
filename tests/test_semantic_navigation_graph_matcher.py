from __future__ import annotations

import unittest

from app.reasoning.semantic_navigation.goal_graph_builder import GoalGraphBuilder
from app.reasoning.semantic_navigation.graph_matcher import SemanticNavigationGraphMatcher
from app.reasoning.semantic_navigation.models import GraphMatchState
from app.video.schemas import SceneGraph, SceneGraphEdge, SceneGraphNode
from app.video.target_profile import TargetProfile


def _profile():
    return TargetProfile(
        raw_query="找饮水机旁边的蓝色垃圾桶", canonical_name_zh="垃圾桶",
        primary_labels_en=["trash can"], aliases_en=["waste bin"],
        attributes=["blue"], relation_constraints=["trash can near water dispenser"],
        context_labels_zh=["饮水机"], context_labels_en=["water dispenser"],
    )


def _node(node_id, label, label_zh="", attributes=None):
    return SceneGraphNode(
        node_id=node_id, node_type="object", label=label, label_zh=label_zh,
        category="object", source="observed", confidence=0.9,
        evidence_level="observed_confirmed", based_on=["frame_1"],
        can_confirm_target=True, attributes=attributes or {},
    )


class GraphMatcherTests(unittest.TestCase):
    def setUp(self):
        self.profile = _profile()
        self.goal = GoalGraphBuilder().build(self.profile)
        self.matcher = SemanticNavigationGraphMatcher()

    def test_zero_and_partial_anchor(self):
        zero = self.matcher.match(self.goal, SceneGraph(), target_profile=self.profile)
        self.assertEqual(zero.state, GraphMatchState.ZERO)
        partial = self.matcher.match(
            self.goal, SceneGraph(nodes=[_node("water", "water dispenser", "饮水机")]),
            target_profile=self.profile,
        )
        self.assertEqual(partial.state, GraphMatchState.PARTIAL)
        self.assertFalse(partial.target_node_visually_present)

    def test_alias_attributes_relation_produce_strong_but_not_confirmed(self):
        graph = SceneGraph(
            nodes=[
                _node("bin", "waste bin", "蓝色垃圾桶", {"attributes": ["blue"]}),
                _node("water", "water dispenser", "饮水机"),
            ],
            edges=[SceneGraphEdge(
                edge_id="e1", source_node_id="bin", target_node_id="water",
                relation="near", source="observed", confidence=0.9,
                evidence_level="observed_confirmed",
            )],
        )
        result = self.matcher.match(self.goal, graph, target_profile=self.profile)
        self.assertEqual(result.state, GraphMatchState.STRONG)
        self.assertTrue(result.target_node_visually_present)
        self.assertNotIn("confirmed", result.state.value)
        self.assertTrue(result.matched_relations)
        self.assertTrue(result.attribute_support)

    def test_attribute_mismatch_downgrades_match(self):
        graph = SceneGraph(
            nodes=[
                _node("bin", "waste bin", "红色垃圾桶", {"attributes": ["red"]}),
                _node("water", "water dispenser", "饮水机"),
            ],
            edges=[SceneGraphEdge(
                edge_id="e1", source_node_id="bin", target_node_id="water",
                relation="near", source="observed", confidence=0.9,
                evidence_level="observed_confirmed",
            )],
        )
        result = self.matcher.match(self.goal, graph, target_profile=self.profile)
        self.assertEqual(result.state, GraphMatchState.PARTIAL)
        self.assertIn("attribute_mismatch:goal_target", result.warnings)

    def test_next_to_scene_relation_satisfies_near_goal_relation(self):
        graph = SceneGraph(
            nodes=[
                _node("bin", "waste bin", "蓝色垃圾桶", {"attributes": ["blue"]}),
                _node("water", "water dispenser", "饮水机"),
            ],
            edges=[SceneGraphEdge(
                edge_id="e1", source_node_id="bin", target_node_id="water",
                relation="next_to", source="observed", confidence=0.9,
                evidence_level="observed_confirmed",
            )],
        )
        result = self.matcher.match(self.goal, graph, target_profile=self.profile)
        self.assertEqual(result.state, GraphMatchState.STRONG)
        self.assertTrue(result.matched_relations)

    def test_relation_mismatch_cannot_be_strong(self):
        graph = SceneGraph(nodes=[
            _node("bin", "waste bin", "蓝色垃圾桶", {"attributes": ["blue"]}),
            _node("water", "water dispenser", "饮水机"),
        ])
        result = self.matcher.match(self.goal, graph, target_profile=self.profile)
        self.assertEqual(result.state, GraphMatchState.PARTIAL)
        self.assertTrue(result.unmatched_relations)


if __name__ == "__main__":
    unittest.main()
