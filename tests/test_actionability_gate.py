from __future__ import annotations

import unittest

from app.planning.actionability_gate import validate_hypotheses
from app.schemas import (
    Actionability,
    LLMSearchHypothesis,
    NodeObservationStatus,
    QuadrupedActionPrimitive,
    default_quadruped_capability,
)


def _hypothesis(text: str) -> LLMSearchHypothesis:
    return LLMSearchHypothesis(
        hypothesis_id="hyp_001",
        target_name="手机",
        candidate_region_zh="桌子附近",
        candidate_region_type="anchor_area",
        supporting_visible_anchor_ids=["obj_001"],
        supporting_visible_anchor_names=["桌子"],
        human_like_rationale_zh=text,
        expected_visual_cues_zh=["手机轮廓"],
        suggested_detector_prompts_en=["phone"],
        suggested_verification_question_zh=text,
        confidence=0.7,
        uncertainty_zh="尚未视觉确认",
        actionability=Actionability.ROBOT_EXECUTABLE,
        quadruped_view_strategy=[QuadrupedActionPrimitive.MOVE_FORWARD_SHORT],
    )


class ActionabilityGateTest(unittest.TestCase):
    def test_unanchored_hypothesis_cannot_move_into_unknown_area(self) -> None:
        hypothesis = _hypothesis("前方可能有目标")
        hypothesis = hypothesis.model_copy(
            update={
                "supporting_visible_anchor_ids": [],
                "supporting_visible_anchor_names": [],
            }
        )
        result = validate_hypotheses(
            [hypothesis],
            default_quadruped_capability(),
        )
        item = result.hypotheses[0]
        self.assertNotIn(
            QuadrupedActionPrimitive.MOVE_FORWARD_SHORT,
            item.quadruped_view_strategy,
        )
        self.assertIn(
            QuadrupedActionPrimitive.SCAN_LEFT_TO_RIGHT,
            item.quadruped_view_strategy,
        )

    def test_container_actions_require_human(self) -> None:
        result = validate_hypotheses(
            [_hypothesis("打开抽屉并翻找背包")],
            default_quadruped_capability(),
        )
        item = result.hypotheses[0]
        self.assertEqual(item.actionability, Actionability.NEEDS_HUMAN)
        self.assertEqual(item.status, NodeObservationStatus.UNREACHABLE)
        self.assertNotIn(
            QuadrupedActionPrimitive.MOVE_FORWARD_SHORT,
            item.quadruped_view_strategy,
        )

    def test_look_down_is_rewritten_to_viewpoint(self) -> None:
        result = validate_hypotheses(
            [_hypothesis("靠近后低头观察桌面")],
            default_quadruped_capability(),
        )
        item = result.hypotheses[0]
        self.assertEqual(item.actionability, Actionability.ROBOT_VIEWPOINT_ONLY)
        self.assertEqual(
            item.quadruped_view_strategy,
            [
                QuadrupedActionPrimitive.CENTER_VIEW_ON_REGION,
                QuadrupedActionPrimitive.STOP_AND_REOBSERVE,
            ],
        )
        self.assertNotIn("低头", item.suggested_verification_question_zh)


if __name__ == "__main__":
    unittest.main()
