from __future__ import annotations

import unittest
from unittest.mock import patch

from app.reasoning.task_parser import (
    convert_new_task_to_legacy_format,
    parse_robot_task,
    parse_robot_task_with_optional_llm,
)
from app.task_understanding.capability_gate import evaluate_actionability
from app.task_understanding.navigation_router import route_navigation_task
from app.task_understanding.schemas import ParsedTask, RequestedSubtask, TaskArea, TaskTarget
from app.task_understanding.task_pipeline import TaskUnderstandingResult


class TaskParserTest(unittest.TestCase):
    def test_convert_find_object(self) -> None:
        result = _task_result(
            ParsedTask(
                raw_task="找到桌子上的手机",
                primary_intent="locate_object",
                target=TaskTarget(
                    name_zh="手机",
                    name_en="phone",
                    category="object",
                    relations=["桌子上"],
                ),
                requested_subtasks=[
                    RequestedSubtask(
                        id="1",
                        type="locate_object",
                        object="手机",
                        is_navigation_relevant=True,
                    )
                ],
            )
        )

        task = convert_new_task_to_legacy_format(result)

        self.assertEqual(task.task_type, "find_object")
        self.assertEqual(task.target_object, "phone")
        self.assertIn("桌子上", task.constraints)

    def test_convert_floor_door_inspection(self) -> None:
        result = _task_result(
            ParsedTask(
                raw_task="检查这层楼哪些门处于指定状态",
                primary_intent="check_door_state",
                target=TaskTarget(name_zh="门", name_en="door", category="door"),
                area=TaskArea(name_zh="这层楼", category="floor"),
                requested_subtasks=[
                    RequestedSubtask(
                        id="1",
                        type="inspect_door_state",
                        object="门",
                        state_query="open",
                        is_navigation_relevant=True,
                    )
                ],
            )
        )

        task = convert_new_task_to_legacy_format(result)

        self.assertEqual(task.task_type, "check_door_state")
        self.assertEqual(task.target_object, "door")
        self.assertEqual(task.scope, "current_floor")
        self.assertEqual(task.parsed_slots["subtask"], "check_door_state")
        self.assertEqual(task.parsed_slots["state"], "open")

    def test_convert_find_room(self) -> None:
        result = _task_result(
            ParsedTask(
                raw_task="找到 503 房间",
                primary_intent="find_room",
                target=TaskTarget(name_zh="503", category="room"),
                requested_subtasks=[
                    RequestedSubtask(
                        id="1",
                        type="find_room",
                        object="503",
                        is_navigation_relevant=True,
                    )
                ],
            )
        )

        task = convert_new_task_to_legacy_format(result)

        self.assertEqual(task.task_type, "find_room")
        self.assertEqual(task.target_room, "503")

    def test_parse_robot_task_delegates_to_new_pipeline(self) -> None:
        result = _task_result(
            ParsedTask(
                raw_task="找到手机",
                primary_intent="locate_object",
                target=TaskTarget(name_zh="手机", name_en="phone", category="object"),
                requested_subtasks=[
                    RequestedSubtask(
                        id="1",
                        type="locate_object",
                        object="手机",
                        is_navigation_relevant=True,
                    )
                ],
            )
        )

        with patch(
            "app.reasoning.task_parser.run_task_understanding_pipeline",
            return_value=result,
        ):
            task = parse_robot_task("找到手机")

        self.assertEqual(task.task_type, "find_object")
        self.assertEqual(task.target_object, "phone")

    def test_optional_llm_parser_accepts_valid_legacy_json(self) -> None:
        def good_parser(text: str) -> dict:
            return {
                "task_id": "task_llm",
                "raw_text": text,
                "task_type": "summarize_scene",
                "target_object": None,
                "target_location": None,
                "target_room": None,
                "scope": "current_scene",
                "constraints": [],
                "parsed_slots": {},
                "confidence": 0.8,
            }

        task = parse_robot_task_with_optional_llm("总结当前场景", good_parser)

        self.assertEqual(task.task_type, "summarize_scene")
        self.assertEqual(task.task_id, "task_llm")


def _task_result(parsed: ParsedTask) -> TaskUnderstandingResult:
    actionability = evaluate_actionability(parsed)
    navigation_task = route_navigation_task(parsed, actionability)
    return TaskUnderstandingResult(
        parsed_task=parsed,
        actionability=actionability,
        navigation_task=navigation_task,
        user_feedback=actionability.user_feedback,
    )


if __name__ == "__main__":
    unittest.main()
