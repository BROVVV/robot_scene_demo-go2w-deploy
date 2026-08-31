from __future__ import annotations

import unittest
from types import SimpleNamespace

from app.config import Settings
from app.reasoning.llm_prior_generator import LLMPriorGenerator, LLMPriorInput


class _FakeCompletions:
    def create(self, **kwargs):
        return SimpleNamespace(
            choices=[
                SimpleNamespace(
                    message=SimpleNamespace(
                        content="""{
                          "target": "手机",
                          "prior_source": "llm_runtime_commonsense",
                          "can_confirm_target": true,
                          "commonsense_hypotheses": [{
                            "hypothesis_id": "hyp_001",
                            "anchor_label": "桌子",
                            "priority": 0.9,
                            "reason_zh": "手机可能临时放在桌面。",
                            "status": "confirmed",
                            "evidence_type": "memory"
                          }],
                          "suggested_detector_prompts": ["phone", "smartphone"]
                        }"""
                    )
                )
            ]
        )


class _FakeClient:
    def __init__(self):
        self.chat = SimpleNamespace(completions=_FakeCompletions())


class LLMPriorGeneratorTest(unittest.TestCase):
    def test_sanitizes_runtime_prior_as_non_confirming_hypothesis(self) -> None:
        result = LLMPriorGenerator(
            settings=Settings(siliconflow_api_key="test"),
            client=_FakeClient(),
        ).generate(LLMPriorInput(target="手机"))

        self.assertEqual(result["prior_source"], "llm_runtime_commonsense")
        self.assertFalse(result["can_confirm_target"])
        self.assertEqual(
            result["commonsense_hypotheses"][0]["status"],
            "hypothesis",
        )
        self.assertEqual(
            result["commonsense_hypotheses"][0]["evidence_type"],
            "llm_commonsense",
        )


if __name__ == "__main__":
    unittest.main()
