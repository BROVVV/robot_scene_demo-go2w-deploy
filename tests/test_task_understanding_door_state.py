from __future__ import annotations

from app.task_understanding.task_pipeline import run_task_understanding_pipeline


class FakeLLMClient:
    def chat(self, **_: object) -> dict:
        return {
            "raw_task": "检查这层楼哪些门打开了",
            "primary_intent": "check_door_state",
            "target": {"name_zh": "门", "name_en": "door", "category": "door"},
            "area": {"name_zh": "这层楼", "category": "floor"},
            "requested_subtasks": [
                {
                    "id": "subtask_1",
                    "type": "inspect_door_state",
                    "object": "门",
                    "state_query": "open",
                    "is_navigation_relevant": True,
                }
            ],
        }


def test_check_open_doors_not_harm() -> None:
    result = run_task_understanding_pipeline(
        "检查这层楼哪些门打开了",
        llm_client=FakeLLMClient(),
        enable_verifier=False,
    )

    assert result.parsed_task.primary_intent == "check_door_state"
    assert result.parsed_task.target.category == "door"
    assert result.actionability.navigation_part_executable is True
    assert "physical_harm_request" not in result.actionability.safety_flags
    assert "manipulation_required" not in result.actionability.safety_flags
