from __future__ import annotations

from app.task_understanding.task_pipeline import run_task_understanding_pipeline


class FakeLLMClient:
    def chat(self, **_: object) -> dict:
        return {
            "raw_task": "找到 503 房间",
            "primary_intent": "find_room",
            "target": {"name_zh": "503 房间", "category": "room"},
            "requested_subtasks": [
                {
                    "id": "subtask_1",
                    "type": "find_room",
                    "object": "503 房间",
                    "is_navigation_relevant": True,
                }
            ],
        }


def test_find_room() -> None:
    result = run_task_understanding_pipeline(
        "找到 503 房间",
        llm_client=FakeLLMClient(),
        enable_verifier=False,
    )

    assert result.parsed_task.primary_intent == "find_room"
    assert result.parsed_task.target.category == "room"
    assert result.actionability.navigation_part_executable is True
    assert "physical_harm_request" not in result.actionability.safety_flags
