from __future__ import annotations

import unittest

from app.task_understanding.capability_gate import evaluate_actionability
from app.task_understanding.schemas import ParsedTask, RequestedSubtask, SafetyFlag, TaskTarget


class CapabilityGateTest(unittest.TestCase):
    def test_mixed_harmful_task_keeps_navigation_and_blocks_harm(self) -> None:
        parsed = ParsedTask(
            raw_task="找到张三然后执行危险动作",
            primary_intent="mixed",
            target=TaskTarget(name_zh="张三", category="person"),
            requested_subtasks=[
                RequestedSubtask(
                    id="1",
                    type="locate_person",
                    object="张三",
                    is_navigation_relevant=True,
                ),
                RequestedSubtask(
                    id="2",
                    type="physical_harm",
                    object="张三",
                    requires_harmful_action=True,
                    is_navigation_relevant=False,
                ),
            ],
        )

        result = evaluate_actionability(parsed)

        self.assertFalse(result.fully_executable)
        self.assertTrue(result.navigation_part_executable)
        self.assertTrue(result.allowed_subtasks)
        self.assertTrue(result.blocked_subtasks)
        self.assertIn(SafetyFlag.PHYSICAL_HARM_REQUEST, result.safety_flags)
        self.assertFalse(result.execution_constraints["contact_allowed"])
        self.assertIn("不能执行", result.user_feedback)

    def test_open_container_is_manipulation_not_harm(self) -> None:
        parsed = ParsedTask(
            raw_task="帮我打开柜子把手机拿出来",
            primary_intent="mixed",
            target=TaskTarget(name_zh="手机", category="object"),
            requested_subtasks=[
                RequestedSubtask(
                    id="1",
                    type="open_container",
                    object="柜子",
                    requires_manipulation=True,
                    is_navigation_relevant=False,
                ),
                RequestedSubtask(
                    id="2",
                    type="locate_object",
                    object="手机",
                    is_navigation_relevant=True,
                ),
            ],
        )

        result = evaluate_actionability(parsed)

        self.assertIn(SafetyFlag.MANIPULATION_REQUIRED, result.safety_flags)
        self.assertNotIn(SafetyFlag.PHYSICAL_HARM_REQUEST, result.safety_flags)
        self.assertTrue(result.navigation_part_executable)
        self.assertFalse(result.fully_executable)


if __name__ == "__main__":
    unittest.main()
