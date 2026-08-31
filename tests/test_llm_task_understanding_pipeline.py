from __future__ import annotations

from app.task_understanding.task_pipeline import run_task_understanding_pipeline


class FakeLLMClient:
    def __init__(self, payload: dict):
        self.payload = payload

    def chat(self, **_: object) -> dict:
        return self.payload


def test_open_container_take_phone_not_harm() -> None:
    result = run_task_understanding_pipeline(
        "帮我打开柜子把手机拿出来",
        llm_client=FakeLLMClient(
            {
                "raw_task": "帮我打开柜子把手机拿出来",
                "primary_intent": "mixed",
                "target": {"name_zh": "手机", "name_en": "phone", "category": "object"},
                "requested_subtasks": [
                    {
                        "id": "subtask_1",
                        "type": "open_container",
                        "object": "柜子",
                        "requires_manipulation": True,
                        "is_navigation_relevant": False,
                    },
                    {
                        "id": "subtask_2",
                        "type": "pick_up_object",
                        "object": "手机",
                        "requires_manipulation": True,
                        "is_navigation_relevant": False,
                    },
                    {
                        "id": "subtask_3",
                        "type": "locate_object",
                        "object": "手机",
                        "is_navigation_relevant": True,
                    },
                ],
                "confidence": {"intent": 0.95, "safety": 0.98, "target": 0.95},
            }
        ),
        enable_verifier=False,
    )

    assert result.parsed_task.primary_intent == "mixed"
    assert "manipulation_required" in result.actionability.safety_flags
    assert "physical_harm_request" not in result.actionability.safety_flags
    assert result.actionability.navigation_part_executable is True
    assert result.actionability.fully_executable is False


def test_harm_request_keeps_only_safe_navigation_part() -> None:
    result = run_task_understanding_pipeline(
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
            }
        ),
        enable_verifier=False,
    )

    assert "physical_harm_request" in result.actionability.safety_flags
    assert result.navigation_task.executable is True
    assert result.navigation_task.execution_constraints["contact_allowed"] is False
    assert result.navigation_task.execution_constraints["requires_safe_standoff_distance"] is True
