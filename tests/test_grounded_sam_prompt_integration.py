from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

from app.config import Settings
from app.detectors.grounded_sam_subprocess import GroundedSAMSubprocessDetector
from app.task_understanding.schemas import (
    NavigationTask,
    ParsedTask,
    TargetCategory,
    TaskIntent,
    TaskTarget,
)
from app.video.target_profile import TargetProfile


class MockPromptClient:
    def __init__(self, *responses: dict) -> None:
        self.responses = list(responses)

    def chat(self, **kwargs):
        if not self.responses:
            raise AssertionError("No mock response left.")
        return self.responses.pop(0)


def _room_task() -> tuple[ParsedTask, NavigationTask]:
    target = TaskTarget(name_zh="卧室", category=TargetCategory.ROOM)
    parsed_task = ParsedTask(
        raw_task="找到卧室",
        primary_intent=TaskIntent.FIND_ROOM,
        target=target,
    )
    nav_task = NavigationTask(
        executable=True,
        navigation_goal="find_room",
        target=target,
        source_raw_task=parsed_task.raw_task,
    )
    return parsed_task, nav_task


class GroundedSAMPromptIntegrationTest(unittest.TestCase):
    def test_detector_passes_grounding_prompt_plan_to_worker(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            prompt_plan_path = Path(tmpdir) / "grounding_prompt_plan.json"
            retry_plan_path = Path(tmpdir) / "grounding_prompt_retry_plan.json"
            parsed_task, nav_task = _room_task()
            detector = GroundedSAMSubprocessDetector(
                Settings(
                    siliconflow_api_key="",
                    grounded_sam_root="/tmp",
                    grounded_sam_python="python",
                    enable_sam2=False,
                    grounding_prompt_debug_output=str(prompt_plan_path),
                    grounding_prompt_retry_debug_output=str(retry_plan_path),
                ),
                target_profile=TargetProfile(
                    raw_query="找到卧室",
                    canonical_name_zh="卧室",
                    target_type="room",
                ),
                parsed_task=parsed_task,
                navigation_task=nav_task,
                llm_client=MockPromptClient(
                    {
                        "target_name_zh": "卧室",
                        "target_category": "room",
                        "grounding_strategy": "scene_proxy_objects",
                        "proxy_object_terms_en": ["bed", "wardrobe", "window"],
                        "context_anchor_terms_en": ["door", "doorway"],
                        "grounding_prompt": (
                            "bed . wardrobe . window . door . doorway ."
                        ),
                        "requires_proxy_objects": True,
                        "requires_scene_confirmation": True,
                    }
                ),
            )
            commands: list[list[str]] = []

            def fake_run(command, **kwargs):
                commands.append(command)
                output = Path(command[command.index("--output") + 1])
                output.write_text(
                    json.dumps(
                        {
                            "objects": [
                                {
                                    "label": "bed",
                                    "score": 0.9,
                                    "bbox_2d": [0.1, 0.2, 0.5, 0.7],
                                }
                            ]
                        }
                    ),
                    encoding="utf-8",
                )
                return SimpleNamespace(returncode=0, stdout="", stderr="")

            image = Path(tmpdir) / "image.jpg"
            image.write_bytes(b"mock")
            with patch(
                "app.detectors.grounded_sam_subprocess.subprocess.run",
                side_effect=fake_run,
            ):
                objects = detector.detect(str(image), "卧室")

            prompt = commands[0][commands[0].index("--text-prompt") + 1]
            self.assertIn("bed", prompt)
            self.assertIn("doorway", prompt)
            self.assertNotIn("office chair", prompt)
            self.assertEqual(objects[0].label, "bed")
            self.assertTrue(prompt_plan_path.is_file())

    def test_detector_retries_llm_prompt_when_no_candidates(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            parsed_task, nav_task = _room_task()
            detector = GroundedSAMSubprocessDetector(
                Settings(
                    siliconflow_api_key="",
                    grounded_sam_root="/tmp",
                    grounded_sam_python="python",
                    enable_sam2=False,
                    grounding_prompt_debug_output=str(
                        Path(tmpdir) / "grounding_prompt_plan.json"
                    ),
                    grounding_prompt_retry_debug_output=str(
                        Path(tmpdir) / "grounding_prompt_retry_plan.json"
                    ),
                ),
                parsed_task=parsed_task,
                navigation_task=nav_task,
                llm_client=MockPromptClient(
                    {
                        "target_name_zh": "卧室",
                        "target_category": "room",
                        "grounding_strategy": "scene_proxy_objects",
                        "proxy_object_terms_en": ["bed", "wardrobe", "window"],
                        "context_anchor_terms_en": ["door"],
                        "grounding_prompt": "bed . wardrobe . window . door .",
                    },
                    {
                        "retry_prompt": (
                            "bed . wardrobe . window . curtain . door . doorway ."
                        ),
                        "added_terms_en": ["curtain", "doorway"],
                        "reason_zh": "增加入口和大件锚点提高召回。",
                    },
                ),
            )
            commands: list[list[str]] = []

            def fake_run(command, **kwargs):
                commands.append(command)
                output = Path(command[command.index("--output") + 1])
                objects = [] if len(commands) == 1 else [
                    {
                        "label": "doorway",
                        "score": 0.8,
                        "bbox_2d": [0.2, 0.1, 0.8, 0.9],
                    }
                ]
                output.write_text(json.dumps({"objects": objects}), encoding="utf-8")
                return SimpleNamespace(returncode=0, stdout="", stderr="")

            image = Path(tmpdir) / "image.jpg"
            image.write_bytes(b"mock")
            with patch(
                "app.detectors.grounded_sam_subprocess.subprocess.run",
                side_effect=fake_run,
            ):
                objects = detector.detect(str(image), "卧室")

            retry_prompt = commands[1][commands[1].index("--text-prompt") + 1]
            self.assertIn("curtain", retry_prompt)
            self.assertIn("doorway", retry_prompt)
            self.assertEqual(objects[0].label, "doorway")


if __name__ == "__main__":
    unittest.main()
