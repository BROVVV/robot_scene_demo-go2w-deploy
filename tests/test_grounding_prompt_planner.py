from __future__ import annotations

import unittest

from app.config import Settings
from app.perception.grounding_prompt_planner import (
    GroundingPromptError,
    GroundingPromptPlanner,
)
from app.task_understanding.schemas import (
    NavigationTask,
    ParsedTask,
    TargetCategory,
    TaskIntent,
    TaskTarget,
)


class MockPromptClient:
    def __init__(self, *responses: dict) -> None:
        self.responses = list(responses)

    def chat(self, **kwargs):
        if not self.responses:
            raise AssertionError("No mock response left.")
        return self.responses.pop(0)


class GroundingPromptPlannerTest(unittest.TestCase):
    def test_bedroom_prompt_uses_llm_proxy_objects(self) -> None:
        parsed_task = ParsedTask(
            raw_task="找到卧室",
            primary_intent=TaskIntent.FIND_ROOM,
            target=TaskTarget(name_zh="卧室", category=TargetCategory.ROOM),
        )
        nav_task = NavigationTask(
            executable=True,
            navigation_goal="find_room",
            target=parsed_task.target,
            source_raw_task=parsed_task.raw_task,
        )
        client = MockPromptClient(
            {
                "target_name_zh": "卧室",
                "target_name_en": "bedroom",
                "target_category": "room",
                "grounding_strategy": "scene_proxy_objects",
                "direct_terms_en": ["bedroom"],
                "proxy_object_terms_en": [
                    "bed",
                    "wardrobe",
                    "nightstand",
                    "pillow",
                    "blanket",
                    "curtain",
                    "window",
                ],
                "context_anchor_terms_en": ["door", "doorway", "room entrance"],
                "state_terms_en": [],
                "grounding_prompt": (
                    "bed . wardrobe . nightstand . pillow . blanket . curtain . "
                    "window . door . doorway . room entrance ."
                ),
                "requires_proxy_objects": True,
                "requires_scene_confirmation": True,
                "requires_state_verification": False,
                "reason_zh": "卧室是场景类别，需要用可见代理物体确认。",
            }
        )
        planner = GroundingPromptPlanner(
            llm_client=client,
            settings=Settings(siliconflow_api_key=""),
        )

        plan = planner.build(parsed_task=parsed_task, navigation_task=nav_task)

        self.assertTrue(plan.is_valid_for_grounding_dino)
        self.assertEqual(plan.grounding_strategy, "scene_proxy_objects")
        self.assertTrue(plan.requires_scene_confirmation)
        self.assertIn("bed", plan.grounding_prompt)
        self.assertIn("doorway", plan.grounding_prompt)
        self.assertNotEqual(plan.grounding_prompt.strip(), "bedroom .")

    def test_empty_llm_prompt_fails_fast(self) -> None:
        parsed_task = ParsedTask(raw_task="找到卧室")
        planner = GroundingPromptPlanner(
            llm_client=MockPromptClient({"grounding_prompt": ""}),
            settings=Settings(siliconflow_api_key=""),
        )

        with self.assertRaisesRegex(GroundingPromptError, "prompt is empty"):
            planner.build(parsed_task=parsed_task)


if __name__ == "__main__":
    unittest.main()
