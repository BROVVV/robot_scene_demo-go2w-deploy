from __future__ import annotations

import unittest

from app.reasoning.semantic_navigation.models import (
    GraphMatchResult, GraphMatchState, SearchDirectiveKind, SearchReasoningContext,
)
from app.reasoning.semantic_navigation.search_reasoner import HybridReasoner, SemanticNavigationReasoner
from app.reasoning.semantic_navigation.semantic_memory import SemanticSearchMemory


class SearchReasonerTests(unittest.TestCase):
    def _context(self, state, score=0.0, anchors=None):
        match = GraphMatchResult(
            state=state, score=score,
            supporting_anchor_scene_node_ids=anchors or [],
        )
        return SearchReasoningContext(
            graph_match=match, scene_graph={"nodes": [], "edges": []},
            negative_memory=SemanticSearchMemory(),
        )

    def test_zero_explores_unseen_without_forward(self):
        directive = SemanticNavigationReasoner().propose(self._context(GraphMatchState.ZERO))
        self.assertEqual(directive.kind, SearchDirectiveKind.EXPLORE_UNSEEN)
        self.assertFalse(directive.allow_forward)
        self.assertIsNone(directive.preferred_distance_m)

    def test_partial_inspects_anchor(self):
        context = self._context(GraphMatchState.PARTIAL, 0.5, ["anchor"])
        context.scene_graph = {
            "nodes": [{"node_id": "anchor", "label": "water dispenser", "attributes": {"position_2d": "left"}}],
            "edges": [],
        }
        directive = SemanticNavigationReasoner().propose(context)
        self.assertEqual(directive.kind, SearchDirectiveKind.INSPECT_ANCHOR)
        self.assertGreater(directive.preferred_heading_delta_deg, 0)

    def test_strong_only_reobserves_and_never_confirms(self):
        directive = SemanticNavigationReasoner().propose(
            self._context(GraphMatchState.STRONG, 0.9)
        )
        self.assertEqual(directive.kind, SearchDirectiveKind.REOBSERVE_SECTOR)
        self.assertNotIn("confirm", directive.kind.value)

    def test_hybrid_uses_auxiliary_hint_only_as_zero_match_tie_break(self):
        context = self._context(GraphMatchState.ZERO)
        context.auxiliary_hints = [{
            "hint_id": "psg:view_left",
            "source": "psg",
            "heading_delta_deg": 30.0,
            "confidence": 0.8,
            "can_confirm_target": False,
        }]
        directive = HybridReasoner().propose(context)
        self.assertEqual(directive.preferred_heading_delta_deg, 30.0)
        self.assertIn("psg:view_left", directive.evidence_refs)
        self.assertFalse(directive.allow_forward)

    def test_negative_memory_outranks_auxiliary_hint(self):
        context = self._context(GraphMatchState.ZERO)
        context.negative_memory.add_negative(
            target_key="target", heading_sector=1,
            reason="not seen", source_event_id="negative_left",
            confidence=0.9,
        )
        context.auxiliary_hints = [{
            "hint_id": "psg:view_left",
            "source": "psg",
            "heading_delta_deg": 30.0,
            "confidence": 1.0,
            "can_confirm_target": False,
        }]
        directive = HybridReasoner().propose(context)
        self.assertEqual(directive.preferred_heading_delta_deg, -30.0)
        self.assertNotIn("psg:view_left", directive.evidence_refs)

    def test_semantic_navigation_backend_does_not_consume_auxiliary_hints(self):
        context = self._context(GraphMatchState.ZERO)
        context.auxiliary_hints = [{
            "hint_id": "psg:view_left",
            "heading_delta_deg": 30.0,
            "confidence": 1.0,
            "can_confirm_target": False,
        }]
        directive = SemanticNavigationReasoner().propose(context)
        self.assertEqual(directive.preferred_heading_delta_deg, -30.0)
        self.assertNotIn("psg:view_left", directive.evidence_refs)


if __name__ == "__main__":
    unittest.main()
