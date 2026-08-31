from __future__ import annotations

import unittest

from app.reasoning.semantic_navigation.goal_graph_builder import GoalGraphBuilder
from app.video.target_profile import TargetProfile


class GoalGraphBuilderTests(unittest.TestCase):
    def test_plain_phone_does_not_invent_room_or_table(self):
        profile = TargetProfile(
            raw_query="找手机", canonical_name_zh="手机",
            primary_labels_en=["phone"], aliases_zh=["手机"],
        )
        graph = GoalGraphBuilder().build(profile)
        self.assertEqual(len(graph.nodes), 1)
        self.assertEqual(graph.nodes[0].role, "target")
        self.assertEqual(graph.edges, [])

    def test_explicit_near_relation_builds_anchor(self):
        profile = TargetProfile(
            raw_query="寻找饮水机旁边的蓝色垃圾桶",
            canonical_name_zh="垃圾桶", primary_labels_en=["trash can"],
            aliases_zh=["垃圾桶"], colors=["blue"], attributes=["蓝色"],
            relation_constraints=["trash can next to water dispenser"],
            context_labels_zh=["饮水机"], context_labels_en=["water dispenser"],
        )
        graph = GoalGraphBuilder().build(profile)
        anchors = [node for node in graph.nodes if node.role == "anchor"]
        self.assertTrue(any("饮水机" in node.label or "water dispenser" in node.label for node in anchors))
        self.assertTrue(any(edge.relation == "near" for edge in graph.edges))
        self.assertIn("blue", graph.nodes[0].attributes)

    def test_context_is_inferred_not_explicit_fact(self):
        profile = TargetProfile(
            raw_query="找消防器材", canonical_name_zh="消防器材",
            primary_labels_en=["fire equipment"],
            context_labels_zh=["墙"], context_labels_en=["wall"],
        )
        graph = GoalGraphBuilder().build(profile)
        context = next(node for node in graph.nodes if node.role == "context")
        self.assertEqual(context.evidence_level, "inferred")
        self.assertEqual(graph.edges[0].relation, "context_hint")


if __name__ == "__main__":
    unittest.main()
