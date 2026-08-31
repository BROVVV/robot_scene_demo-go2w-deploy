from __future__ import annotations

import unittest

from app.reasoning.dynamic_detector_prompts import build_dynamic_detector_prompts


class DynamicDetectorPromptsTest(unittest.TestCase):
    def test_uses_target_and_runtime_prior_without_handwritten_prompt(self) -> None:
        result = build_dynamic_detector_prompts(
            "找到手机",
            {
                "available": True,
                "suggested_detector_prompts": ["phone", "smartphone"],
            },
        )

        self.assertFalse(result["handwritten_prompt_used"])
        self.assertEqual(result["prompts"][:3], ["手机", "phone", "smartphone"])


if __name__ == "__main__":
    unittest.main()
