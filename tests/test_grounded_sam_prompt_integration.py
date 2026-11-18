from __future__ import annotations

import json
from pathlib import Path
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


class _FakeStdin:
    def __init__(self) -> None:
        self.requests: list[dict] = []
        self.closed = False

    def write(self, text: str) -> None:
        self.requests.append(json.loads(text))

    def flush(self) -> None:
        pass

    def close(self) -> None:
        self.closed = True


class _FakeWorker:
    """Persistent JSONL worker：stdin 记录请求，readline 按序返回预置响应。"""

    def __init__(self, response_lines: list[str]) -> None:
        self._lines = list(response_lines)
        self.stdin = _FakeStdin()

    @property
    def stdout(self) -> "_FakeWorker":
        return self

    def readline(self) -> str:
        if not self._lines:
            return ""
        return self._lines.pop(0)

    def poll(self) -> None:
        return None

    def terminate(self) -> None:
        pass

    def wait(self, timeout=None) -> int:
        return 0

    def kill(self) -> None:
        pass


def _response(objects: list[dict]) -> str:
    return json.dumps({"objects": objects}) + "\n"


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
            worker = _FakeWorker(
                [
                    _response(
                        [
                            {
                                "label": "bed",
                                "score": 0.9,
                                "bbox_2d": [0.1, 0.2, 0.5, 0.7],
                            }
                        ]
                    )
                ]
            )
            image = Path(tmpdir) / "image.jpg"
            image.write_bytes(b"mock")
            with patch(
                "app.detectors.grounded_sam_subprocess.subprocess.Popen",
                return_value=worker,
            ):
                objects = detector.detect(str(image), "卧室")

            prompt = worker.stdin.requests[0]["text_prompt"]
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
            # 第一次请求返回空（触发 LLM prompt 扩展重试），第二次返回 doorway。
            worker = _FakeWorker(
                [
                    _response([]),
                    _response(
                        [
                            {
                                "label": "doorway",
                                "score": 0.8,
                                "bbox_2d": [0.2, 0.1, 0.8, 0.9],
                            }
                        ]
                    ),
                ]
            )
            image = Path(tmpdir) / "image.jpg"
            image.write_bytes(b"mock")
            with patch(
                "app.detectors.grounded_sam_subprocess.subprocess.Popen",
                return_value=worker,
            ):
                objects = detector.detect(str(image), "卧室")

            retry_prompt = worker.stdin.requests[1]["text_prompt"]
            self.assertIn("curtain", retry_prompt)
            self.assertIn("doorway", retry_prompt)
            self.assertEqual(objects[0].label, "doorway")


if __name__ == "__main__":
    unittest.main()
