from __future__ import annotations

import json
import unittest
from pathlib import Path

from app.reasoning.llm_situated_search_reasoner import LLMSituatedSearchReasoner
from app.reasoning.observed_facts_builder import build_observed_scene_facts
from app.schemas import (
    LLMReasoningRequest,
    NodeObservationStatus,
    SceneAnalysisResult,
    default_quadruped_capability,
)


ROOT = Path(__file__).resolve().parents[1]


class LLMReasoningSchemaTest(unittest.TestCase):
    def test_fallback_is_structured_and_never_marks_inference_found(self) -> None:
        scene = SceneAnalysisResult.model_validate(
            json.loads(
                (ROOT / "examples/mock_knowledge_aware_result.json").read_text(
                    encoding="utf-8"
                )
            )["base_scene"]
        )
        request = LLMReasoningRequest(
            target_text="找到手机",
            observed_facts=build_observed_scene_facts(scene, "找到手机"),
            capability_contract=default_quadruped_capability(),
        )
        result = LLMSituatedSearchReasoner(
            auto_create_client=False
        ).reason(request)
        self.assertFalse(result.reasoning_available)
        self.assertTrue(result.hypotheses)
        self.assertTrue(
            all(
                item.status == NodeObservationStatus.INFERRED
                and item.should_not_mark_found
                for item in result.hypotheses
            )
        )


if __name__ == "__main__":
    unittest.main()
