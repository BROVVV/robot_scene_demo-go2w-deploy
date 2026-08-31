from __future__ import annotations

import json
import unittest
from pathlib import Path

from app.reasoning.visual_grounding_gate import apply_visual_grounding_gate
from app.schemas import (
    Actionability,
    LLMSearchHypothesis,
    NodeObservationStatus,
    QuadrupedActionPrimitive,
    SceneAnalysisResult,
)


ROOT = Path(__file__).resolve().parents[1]


def _hypothesis() -> LLMSearchHypothesis:
    return LLMSearchHypothesis(
        hypothesis_id="hyp_001",
        target_name="手机",
        candidate_region_zh="桌子附近",
        candidate_region_type="anchor_area",
        supporting_visible_anchor_ids=["obj_002"],
        supporting_visible_anchor_names=["手机候选"],
        human_like_rationale_zh="仅为推测",
        expected_visual_cues_zh=["手机"],
        suggested_detector_prompts_en=["phone"],
        suggested_verification_question_zh="重新观察",
        confidence=0.5,
        uncertainty_zh="未确认",
        actionability=Actionability.NEEDS_REOBSERVATION,
        quadruped_view_strategy=[
            QuadrupedActionPrimitive.STOP_AND_REOBSERVE
        ],
    )


class VisualGroundingGateTest(unittest.TestCase):
    def test_inference_cannot_mark_target_found(self) -> None:
        scene_data = json.loads(
            (ROOT / "examples/mock_knowledge_aware_result.json").read_text(
                encoding="utf-8"
            )
        )["base_scene"]
        scene = SceneAnalysisResult.model_validate(scene_data)
        result = apply_visual_grounding_gate([_hypothesis()], scene)[0]
        self.assertFalse(scene.target_decision.is_present)
        self.assertEqual(result.status, NodeObservationStatus.INFERRED)
        self.assertTrue(result.should_not_mark_found)

    def test_visual_confirmation_upgrades_hypothesis(self) -> None:
        scene = SceneAnalysisResult.model_validate(
            json.loads(
                (ROOT / "examples/mock_scene_result.json").read_text(
                    encoding="utf-8"
                )
            )
        )
        result = apply_visual_grounding_gate([_hypothesis()], scene)[0]
        self.assertEqual(result.status, NodeObservationStatus.OBSERVED)
        self.assertFalse(result.should_not_mark_found)


if __name__ == "__main__":
    unittest.main()
