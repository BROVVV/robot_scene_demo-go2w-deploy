from __future__ import annotations

import unittest

from app.config import Settings
from app.reasoning.llm_prior_generator import LLMPriorGenerator, LLMPriorInput


class NoHandwrittenPriorFallbackTest(unittest.TestCase):
    def test_llm_failure_returns_visual_only_without_static_locations(self) -> None:
        result = LLMPriorGenerator(
            settings=Settings(siliconflow_api_key=""),
            auto_create_client=False,
        ).generate(LLMPriorInput(target="手机"))

        self.assertEqual(result["fallback_mode"], "visual_only_no_handcrafted_prior")
        self.assertEqual(result["commonsense_hypotheses"], [])
        self.assertEqual(result["suggested_detector_prompts"], [])


if __name__ == "__main__":
    unittest.main()
