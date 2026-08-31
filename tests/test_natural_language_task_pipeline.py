from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from app.task_understanding.task_pipeline import (
    navigation_target_text,
    prepare_navigation_task_from_text,
    write_task_understanding_outputs,
)


class FakeLLMClient:
    def __init__(self, payload: dict):
        self.payload = payload

    def chat(self, **_: object) -> dict:
        return self.payload


def _locate_phone_payload() -> dict:
    return {
        "raw_task": "帮我找到手机",
        "primary_intent": "locate_object",
        "target": {"name_zh": "手机", "name_en": "phone", "category": "object"},
        "requested_subtasks": [
            {
                "id": "subtask_1",
                "type": "locate_object",
                "object": "手机",
                "is_navigation_relevant": True,
            }
        ],
        "confidence": {"intent": 0.95, "safety": 0.95, "target": 0.95},
    }


class NaturalLanguageTaskPipelineTest(unittest.TestCase):
    def test_pipeline_writes_structured_outputs(self) -> None:
        parsed, gate, nav_task = prepare_navigation_task_from_text(
            "帮我找到手机",
            llm_client=FakeLLMClient(_locate_phone_payload()),
            enable_verifier=False,
        )

        with tempfile.TemporaryDirectory() as tmp_dir:
            paths = write_task_understanding_outputs(tmp_dir, parsed, gate, nav_task)

            self.assertIn("parsed_task", paths)
            self.assertIn("natural_language_task_parse", paths)
            self.assertIn("capability_gate_report", paths)
            self.assertIn("navigation_task_plan", paths)
            parsed_payload = json.loads(paths["parsed_task"].read_text(encoding="utf-8"))
            self.assertEqual(parsed_payload["initial_visibility_state"], "unknown")
            self.assertTrue(Path(paths["actionability_report"]).is_file())

    def test_navigation_target_text_extracts_target_only(self) -> None:
        _, _, nav_task = prepare_navigation_task_from_text(
            "帮我找到手机",
            llm_client=FakeLLMClient(_locate_phone_payload()),
            enable_verifier=False,
        )

        self.assertEqual(navigation_target_text(nav_task), "手机")


if __name__ == "__main__":
    unittest.main()
