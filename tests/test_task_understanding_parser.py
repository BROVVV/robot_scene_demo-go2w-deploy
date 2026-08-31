from __future__ import annotations

import unittest

from app.task_understanding.intent_parser import parse_natural_language_task
from app.task_understanding.schemas import SubtaskType, TargetCategory, TaskIntent


class FakeLLMClient:
    def __init__(self, payload: dict):
        self.payload = payload

    def chat(self, **_: object) -> dict:
        return self.payload


class TaskUnderstandingParserTest(unittest.TestCase):
    def test_parse_locate_object_keeps_visibility_unknown(self) -> None:
        task = parse_natural_language_task(
            "帮我找到手机",
            llm_client=FakeLLMClient(
                {
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
            ),
            enable_verifier=False,
        )

        self.assertEqual(task.primary_intent, TaskIntent.LOCATE_OBJECT)
        self.assertEqual(task.target.category, TargetCategory.OBJECT)
        self.assertEqual(task.target.raw_text, "手机")
        self.assertEqual(task.initial_visibility_state, "unknown")
        self.assertTrue(task.requires_visual_grounding)

    def test_parse_mixed_person_and_harm(self) -> None:
        task = parse_natural_language_task(
            "找到张三然后执行危险动作",
            llm_client=FakeLLMClient(
                {
                    "raw_task": "找到张三然后执行危险动作",
                    "primary_intent": "mixed",
                    "target": {"name_zh": "张三", "category": "person"},
                    "requested_subtasks": [
                        {
                            "id": "subtask_1",
                            "type": "locate_person",
                            "object": "张三",
                            "is_navigation_relevant": True,
                        },
                        {
                            "id": "subtask_2",
                            "type": "physical_harm",
                            "object": "张三",
                            "requires_harmful_action": True,
                            "is_navigation_relevant": False,
                        },
                    ],
                    "confidence": {"intent": 0.95, "safety": 0.98, "target": 0.95},
                }
            ),
            enable_verifier=False,
        )

        self.assertEqual(task.primary_intent, TaskIntent.MIXED)
        self.assertEqual(task.target.category, TargetCategory.PERSON)
        self.assertIn(SubtaskType.LOCATE_PERSON, {item.type for item in task.subtasks})
        self.assertIn(SubtaskType.PHYSICAL_HARM, {item.type for item in task.subtasks})
        self.assertEqual(task.initial_visibility_state, "unknown")

    def test_llm_unavailable_does_not_use_rule_fallback(self) -> None:
        task = parse_natural_language_task("帮我找到手机", use_llm=False)

        self.assertEqual(task.primary_intent, TaskIntent.UNKNOWN)
        self.assertEqual(task.parser_source, "llm_unavailable")
        self.assertFalse(task.requested_subtasks)


if __name__ == "__main__":
    unittest.main()
