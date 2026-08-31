from __future__ import annotations

import json
import unittest
from pathlib import Path
from unittest.mock import patch

from app.schemas import (
    Actionability,
    LLMReasoningResult,
    LLMSearchHypothesis,
    NodeObservationStatus,
    QuadrupedActionPrimitive,
    SceneAnalysisResult,
)
from app.services.knowledge_aware_analyzer import KnowledgeAwareAnalyzer


ROOT = Path(__file__).resolve().parents[1]


class DynamicVisualRetryTest(unittest.TestCase):
    def test_dynamic_prompts_trigger_visual_upgrade(self) -> None:
        invisible = SceneAnalysisResult.model_validate(
            json.loads(
                (ROOT / "examples/mock_knowledge_aware_result.json").read_text(
                    encoding="utf-8"
                )
            )["base_scene"]
        )
        visible = SceneAnalysisResult.model_validate(
            json.loads(
                (ROOT / "examples/mock_scene_result.json").read_text(
                    encoding="utf-8"
                )
            )
        )
        prompts_seen: list[str] = []

        def retry(prompts: list[str]) -> SceneAnalysisResult:
            prompts_seen.extend(prompts)
            return visible

        with patch(
            "app.services.knowledge_aware_analyzer."
            "LLMSituatedSearchReasoner.reason",
            return_value=_reasoning(),
        ):
            result = KnowledgeAwareAnalyzer(
                update_kb=False,
                enable_llm_reasoning=True,
                allow_remote_reasoning=False,
                visual_retry_callback=retry,
            ).enrich_base_scene(invisible, "找到手机")

        self.assertIn("phone", prompts_seen)
        self.assertTrue(result.visual_grounding_report["attempted"])
        self.assertTrue(result.visual_grounding_report["upgraded_target"])
        self.assertTrue(result.base_scene.target_decision.is_present)
        self.assertTrue(
            any(
                item.status == NodeObservationStatus.OBSERVED
                and not item.should_not_mark_found
                for item in result.llm_reasoning.hypotheses
            )
        )


def _reasoning() -> LLMReasoningResult:
    return LLMReasoningResult(
        scene_interpretation_zh="办公室",
        target_search_logic_zh="根据可见桌子选择新视角。",
        hypotheses=[
            LLMSearchHypothesis(
                hypothesis_id="hyp_phone",
                target_name="手机",
                candidate_region_zh="桌子右侧可见区域",
                candidate_region_type="visible_anchor_area",
                image_region_hint="right/middle/midground",
                supporting_visible_anchor_ids=["obj_001"],
                supporting_visible_anchor_names=["办公桌"],
                human_like_rationale_zh="桌子是当前可见锚点。",
                expected_visual_cues_zh=["手机屏幕或机身"],
                suggested_detector_prompts_en=["phone", "smartphone"],
                suggested_verification_question_zh="转向右侧并重新观察。",
                confidence=0.8,
                uncertainty_zh="尚未视觉确认。",
                actionability=Actionability.NEEDS_REOBSERVATION,
                quadruped_view_strategy=[
                    QuadrupedActionPrimitive.TURN_RIGHT,
                    QuadrupedActionPrimitive.STOP_AND_REOBSERVE,
                ],
            )
        ],
        global_uncertainty_zh="单帧存在遮挡。",
        recommended_next_observation_zh="向右重观测。",
        no_target_found_policy_zh="未确认前保持 inferred。",
    )


if __name__ == "__main__":
    unittest.main()
