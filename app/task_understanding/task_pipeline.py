"""End-to-end LLM-first natural-language task preparation helpers."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, is_dataclass
from enum import Enum
from pathlib import Path
from typing import Any

from app.task_understanding.capability_gate import evaluate_actionability
from app.task_understanding.llm_task_interpreter import LLMTaskInterpreter, unavailable_task
from app.task_understanding.navigation_router import route_navigation_task
from app.task_understanding.schemas import (
    ActionabilityResult,
    NavigationTask,
    ParsedTask,
    SafetyFlag,
)


@dataclass
class TaskUnderstandingResult:
    parsed_task: ParsedTask
    actionability: ActionabilityResult
    navigation_task: NavigationTask
    user_feedback: str


def run_task_understanding_pipeline(
    task_text: str,
    llm_client: Any | None = None,
    enable_verifier: bool = True,
) -> TaskUnderstandingResult:
    interpreter = LLMTaskInterpreter(
        llm_client=llm_client,
        enable_verifier=enable_verifier,
    )
    parsed_task = interpreter.parse(task_text)
    actionability = evaluate_actionability(parsed_task)
    navigation_task = route_navigation_task(parsed_task, actionability)
    user_feedback = build_pipeline_feedback(parsed_task, actionability, navigation_task)
    return TaskUnderstandingResult(
        parsed_task=parsed_task,
        actionability=actionability,
        navigation_task=navigation_task,
        user_feedback=user_feedback,
    )


def prepare_navigation_task_from_text(
    task_text: str,
    use_llm: bool = True,
    llm_client: Any | None = None,
    enable_verifier: bool = True,
) -> tuple[ParsedTask, ActionabilityResult, NavigationTask]:
    if not use_llm and llm_client is None:
        parsed_task = unavailable_task(
            task_text,
            reason="llm_task_understanding_unavailable",
        )
        actionability = evaluate_actionability(parsed_task)
        navigation_task = route_navigation_task(parsed_task, actionability)
        return parsed_task, actionability, navigation_task
    result = run_task_understanding_pipeline(
        task_text,
        llm_client=llm_client,
        enable_verifier=enable_verifier if use_llm else False,
    )
    return result.parsed_task, result.actionability, result.navigation_task


def build_pipeline_feedback(
    parsed_task: ParsedTask,
    actionability: ActionabilityResult,
    navigation_task: NavigationTask,
) -> str:
    if actionability.user_feedback_zh:
        return actionability.user_feedback_zh
    if navigation_task.executable:
        return "任务已通过能力门控，可进入导航观察流程。"
    if parsed_task.execution_recommendation.user_feedback_zh:
        return parsed_task.execution_recommendation.user_feedback_zh
    return "任务未通过自然语言理解或能力门控，不会进入导航执行流程。"


def parsed_task_to_dict(parsed_task: ParsedTask) -> dict[str, Any]:
    return _jsonable(parsed_task)


def gate_result_to_dict(gate_result: ActionabilityResult) -> dict[str, Any]:
    return _jsonable(gate_result)


def navigation_task_to_dict(nav_task: NavigationTask) -> dict[str, Any]:
    return _jsonable(nav_task)


def navigation_target_text(nav_task: NavigationTask, fallback: str | None = None) -> str:
    target = nav_task.target
    return (
        target.name_zh
        or target.name_en
        or fallback
        or nav_task.source_raw_task
    )


def write_task_understanding_outputs(
    output_dir: str | Path,
    parsed_task: ParsedTask,
    gate_result: ActionabilityResult,
    nav_task: NavigationTask,
) -> dict[str, Path]:
    path = Path(output_dir)
    path.mkdir(parents=True, exist_ok=True)

    parsed_payload = parsed_task_to_dict(parsed_task)
    gate_payload = gate_result_to_dict(gate_result)
    nav_payload = navigation_task_to_dict(nav_task)

    outputs = {
        "parsed_task": _write_json(path / "parsed_task.json", parsed_payload),
        "natural_language_task_parse": _write_json(
            path / "natural_language_task_parse.json",
            parsed_payload,
        ),
        "capability_gate": _write_json(
            path / "capability_gate_result.json",
            gate_payload,
        ),
        "capability_gate_report": _write_json(
            path / "capability_gate_report.json",
            gate_payload,
        ),
        "navigation_task": _write_json(path / "navigation_task.json", nav_payload),
        "navigation_task_plan": _write_json(
            path / "navigation_task_plan.json",
            nav_payload,
        ),
        "actionability_report": _write_actionability_report(
            path / "actionability_report.md",
            parsed_task,
            gate_result,
            nav_task,
        ),
        "task_understanding_feedback": _write_feedback(
            path / "task_understanding_feedback.md",
            gate_result.user_feedback_zh,
        ),
    }
    if SafetyFlag.LLM_TASK_VERIFICATION_FAILED in gate_result.safety_flags:
        outputs["task_understanding_verification_failed"] = _write_json(
            path / "task_understanding_verification_failed.json",
            parsed_payload,
        )
    return outputs


def _write_json(path: Path, payload: dict[str, Any]) -> Path:
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return path


def _write_feedback(path: Path, feedback: str) -> Path:
    path.write_text((feedback or "无反馈。") + "\n", encoding="utf-8")
    return path


def _write_actionability_report(
    path: Path,
    parsed_task: ParsedTask,
    gate_result: ActionabilityResult,
    nav_task: NavigationTask,
) -> Path:
    allowed = "\n".join(
        f"- {_value(item.type)}: {item.description}"
        for item in gate_result.allowed_subtasks
    ) or "- 无"
    blocked = "\n".join(
        f"- {_value(item.type)}: {item.description}；原因：{item.blocked_reason or '已拦截'}"
        for item in gate_result.blocked_subtasks
    ) or "- 无"
    flags = "、".join(_value(flag) for flag in gate_result.safety_flags) or "none"
    path.write_text(
        "\n".join(
            [
                "# Natural Language Task Actionability Report",
                "",
                f"原始任务：{parsed_task.raw_task}",
                f"主意图：{_value(parsed_task.primary_intent)}",
                f"导航目标：{nav_task.navigation_goal}",
                f"初始可见性：{parsed_task.initial_visibility_state}",
                f"可进入导航管线：{nav_task.executable}",
                f"安全标记：{flags}",
                "",
                "## 允许执行的导航子任务",
                allowed,
                "",
                "## 已拦截的子任务",
                blocked,
                "",
                "## 用户反馈",
                gate_result.user_feedback_zh,
                "",
            ]
        ),
        encoding="utf-8",
    )
    return path


def _jsonable(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if is_dataclass(value):
        return {key: _jsonable(item) for key, item in asdict(value).items()}
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_jsonable(item) for item in value]
    return value


def _value(value: Any) -> str:
    return value.value if isinstance(value, Enum) else str(value)
