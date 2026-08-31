from __future__ import annotations

import json
import unittest
from pathlib import Path

from app.schemas import SceneAnalysisResult


class OutputSchemaTest(unittest.TestCase):
    def test_old_scene_json_remains_valid_with_new_optional_fields(self) -> None:
        payload = json.loads(Path("examples/mock_scene_result.json").read_text(encoding="utf-8"))
        result = SceneAnalysisResult.model_validate(payload)
        self.assertTrue(hasattr(result, "candidate_objects"))
        self.assertEqual(result.candidate_objects, [])

    def test_enhanced_candidate_fields_are_accepted(self) -> None:
        payload = json.loads(Path("examples/mock_scene_result.json").read_text(encoding="utf-8"))
        payload["objects"][0]["final_score"] = 0.8
        payload["objects"][0]["decision"] = "confirmed"
        payload["candidate_summary"] = {
            "num_raw_candidates": 1,
            "num_verified": 1,
            "num_confirmed": 1,
            "num_rejected": 0
        }
        result = SceneAnalysisResult.model_validate(payload)
        self.assertEqual(result.objects[0].decision, "confirmed")


if __name__ == "__main__":
    unittest.main()
