from __future__ import annotations

import unittest

from app.live_robot.search_directive_adapter import directive_to_step_plan
from app.live_robot.step_search_runner import StepSearchConfig
from app.reasoning.semantic_navigation.models import SearchDirective, SearchDirectiveKind


def _directive(heading=None, *, confidence=0.9, forward=False):
    return SearchDirective(
        directive_id="d1", kind=SearchDirectiveKind.INSPECT_ANCHOR,
        source_backend="semantic_navigation", match_state="partial_match",
        confidence=confidence, preferred_heading_delta_deg=heading,
        preferred_distance_m=0.15 if forward else None,
        allow_forward=forward,
    )


class DirectiveAdapterTests(unittest.TestCase):
    def setUp(self):
        self.config = StepSearchConfig(target="phone")

    def test_turn_sign_and_clamp(self):
        self.assertEqual(directive_to_step_plan(
            _directive(20), scan_index=0, distance_m=0, step_config=self.config
        ).step, "l20")
        self.assertEqual(directive_to_step_plan(
            _directive(-20), scan_index=0, distance_m=0, step_config=self.config
        ).step, "r20")
        self.assertEqual(directive_to_step_plan(
            _directive(100), scan_index=0, distance_m=0, step_config=self.config
        ).step, "l30")

    def test_forward_default_disabled_and_radius_guarded(self):
        disabled = directive_to_step_plan(
            _directive(None, forward=True), scan_index=0, distance_m=0,
            step_config=self.config, allow_forward=False,
        )
        self.assertEqual(disabled.step, "r30")
        near_limit = directive_to_step_plan(
            _directive(None, forward=True), scan_index=0, distance_m=0.95,
            step_config=self.config, allow_forward=True,
        )
        self.assertEqual(near_limit.step, "r30")

    def test_low_confidence_falls_back(self):
        plan = directive_to_step_plan(
            _directive(20, confidence=0.2), scan_index=0, distance_m=0,
            step_config=self.config,
        )
        self.assertEqual(plan.step, "r30")


if __name__ == "__main__":
    unittest.main()
