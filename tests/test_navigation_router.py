from __future__ import annotations

import unittest

from app.task_understanding.task_pipeline import prepare_navigation_task_from_text


class FakeLLMClient:
    def __init__(self, payload: dict):
        self.payload = payload

    def chat(self, **_: object) -> dict:
        return self.payload


class NavigationRouterTest(unittest.TestCase):
    def test_person_target_uses_conservative_safe_distance_constraints(self) -> None:
        _, _, nav_task = prepare_navigation_task_from_text(
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
                        },
                    ],
                }
            ),
            enable_verifier=False,
        )

        self.assertTrue(nav_task.executable)
        self.assertEqual(nav_task.initial_visibility_state, "unknown")
        self.assertEqual(nav_task.execution_constraints["stop_distance_policy"], "safe_distance")
        self.assertEqual(nav_task.execution_constraints["max_speed_policy"], "conservative")
        self.assertFalse(nav_task.execution_constraints["chase_or_ram_allowed"])

    def test_unsupported_task_does_not_enter_navigation_pipeline(self) -> None:
        _, _, nav_task = prepare_navigation_task_from_text(
            "执行非导航任务",
            llm_client=FakeLLMClient(
                {
                    "raw_task": "执行非导航任务",
                    "primary_intent": "non_navigation",
                    "target": {"name_zh": "", "category": "unknown"},
                    "requested_subtasks": [
                        {
                            "id": "subtask_1",
                            "type": "non_navigation",
                            "object": "",
                            "is_navigation_relevant": False,
                        }
                    ],
                }
            ),
            enable_verifier=False,
        )

        self.assertFalse(nav_task.executable)
        self.assertFalse(nav_task.requires_visual_grounding)


if __name__ == "__main__":
    unittest.main()
