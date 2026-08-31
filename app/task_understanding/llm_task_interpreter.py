"""LLM-first interpreter for full-sentence robot task understanding."""

from __future__ import annotations

import json
from dataclasses import asdict
from enum import Enum
from typing import Any

from openai import OpenAI

from app.config import get_settings
from app.task_understanding.prompt_templates import (
    LLM_TASK_INTERPRETER_SYSTEM_PROMPT,
    LLM_TASK_INTERPRETER_USER_PROMPT_TEMPLATE,
    LLM_TASK_VERIFIER_SYSTEM_PROMPT,
    LLM_TASK_VERIFIER_USER_PROMPT_TEMPLATE,
)
from app.task_understanding.schemas import (
    ExecutionRecommendation,
    ParsedTask,
    RequestedSubtask,
    SafetyAssessment,
    SafetyFlag,
    SubtaskType,
    TargetCategory,
    TaskArea,
    TaskIntent,
    TaskParseConfidence,
    TaskTarget,
)
from app.utils.json_utils import extract_json_from_text


VERIFIER_CONFIDENCE_THRESHOLD = 0.75


class LLMTaskInterpreter:
    """Parse a user task with an LLM and validate the structured result."""

    def __init__(self, llm_client: Any | None = None, enable_verifier: bool = True):
        self.llm_client = llm_client
        self.enable_verifier = enable_verifier

    def parse(self, task_text: str) -> ParsedTask:
        text = task_text.strip()
        if not text:
            raise ValueError("自然语言任务不能为空。")

        try:
            payload = self._call_interpreter(text)
            parsed = _parsed_task_from_payload(text, payload)
        except Exception as exc:
            return unavailable_task(text, reason=str(exc))

        if self.enable_verifier:
            try:
                verifier_payload = self._call_verifier(text, parsed)
                if not _verifier_accepts(verifier_payload):
                    return verification_failed_task(
                        text,
                        parsed,
                        verifier_payload,
                    )
            except Exception as exc:
                return verification_failed_task(text, parsed, {"error": str(exc)})

        return parsed

    def _call_interpreter(self, task_text: str) -> dict[str, Any]:
        return _coerce_json_payload(
            _call_text_llm(
                self.llm_client,
                LLM_TASK_INTERPRETER_SYSTEM_PROMPT,
                LLM_TASK_INTERPRETER_USER_PROMPT_TEMPLATE.format(task_text=task_text),
            )
        )

    def _call_verifier(self, task_text: str, parsed: ParsedTask) -> dict[str, Any]:
        parsed_json = json.dumps(asdict(parsed), ensure_ascii=False, indent=2)
        return _coerce_json_payload(
            _call_text_llm(
                self.llm_client,
                LLM_TASK_VERIFIER_SYSTEM_PROMPT,
                LLM_TASK_VERIFIER_USER_PROMPT_TEMPLATE.format(
                    task_text=task_text,
                    parsed_task_json=parsed_json,
                ),
            )
        )


def unavailable_task(task_text: str, reason: str = "") -> ParsedTask:
    return ParsedTask(
        raw_task=task_text,
        task_summary="自然语言任务理解失败，未生成可执行任务。",
        primary_intent=TaskIntent.UNKNOWN,
        requested_subtasks=[],
        safety_assessment=SafetyAssessment(
            contains_non_navigation_request=True,
            risk_level="medium",
            reason="llm_task_understanding_unavailable_or_invalid",
        ),
        execution_recommendation=ExecutionRecommendation(
            can_execute_fully=False,
            can_execute_navigation_part=False,
            unsupported_subtasks=[SafetyFlag.LLM_TASK_UNDERSTANDING_UNAVAILABLE.value],
            user_feedback_zh=(
                "当前自然语言任务理解模型不可用或解析结果不可靠，因此不会执行该任务。"
                "请恢复 LLM 服务后重试，或重新描述任务。"
            ),
        ),
        confidence=TaskParseConfidence(intent=0.0, safety=0.0, target=0.0),
        raw_llm_response={"error": reason} if reason else {},
        parser_source="llm_unavailable",
        notes=[SafetyFlag.LLM_TASK_UNDERSTANDING_UNAVAILABLE.value],
    )


def verification_failed_task(
    task_text: str,
    parsed: ParsedTask | None = None,
    verifier_payload: dict[str, Any] | None = None,
) -> ParsedTask:
    return ParsedTask(
        raw_task=task_text,
        task_summary="自然语言任务解析结果未通过一致性审查。",
        primary_intent=TaskIntent.UNKNOWN,
        target=parsed.target if parsed else TaskTarget(),
        area=parsed.area if parsed else TaskArea(),
        requested_subtasks=[],
        safety_assessment=SafetyAssessment(
            contains_non_navigation_request=True,
            risk_level="medium",
            reason="llm_task_verification_failed",
        ),
        execution_recommendation=ExecutionRecommendation(
            can_execute_fully=False,
            can_execute_navigation_part=False,
            unsupported_subtasks=[SafetyFlag.LLM_TASK_VERIFICATION_FAILED.value],
            user_feedback_zh=(
                "当前自然语言任务解析结果不可靠，因此不会执行该任务。"
                "请重新描述任务，或人工确认解析结果。"
            ),
        ),
        confidence=TaskParseConfidence(intent=0.0, safety=0.0, target=0.0),
        raw_llm_response={
            "candidate_parse": asdict(parsed) if parsed else None,
            "verifier": verifier_payload or {},
        },
        parser_source="llm_verification_failed",
        notes=[SafetyFlag.LLM_TASK_VERIFICATION_FAILED.value],
    )


def _call_text_llm(
    llm_client: Any | None,
    system_prompt: str,
    user_prompt: str,
) -> Any:
    client = llm_client or _default_openai_client()

    if hasattr(client, "chat") and callable(client.chat):
        try:
            return client.chat(system_prompt=system_prompt, user_prompt=user_prompt)
        except TypeError:
            return client.chat(
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt},
                ]
            )

    response = client.chat.completions.create(
        model=get_settings().reasoning_model,
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt},
        ],
        temperature=0.0,
        max_tokens=min(1600, get_settings().siliconflow_reasoning_max_tokens),
    )
    return response.choices[0].message.content


def _default_openai_client() -> OpenAI:
    settings = get_settings()
    if not settings.siliconflow_api_key:
        raise RuntimeError(SafetyFlag.LLM_TASK_UNDERSTANDING_UNAVAILABLE.value)
    return OpenAI(
        api_key=settings.siliconflow_api_key,
        base_url=settings.siliconflow_base_url,
        timeout=settings.siliconflow_reasoning_timeout_seconds,
    )


def _coerce_json_payload(raw: Any) -> dict[str, Any]:
    if isinstance(raw, dict):
        return raw
    if isinstance(raw, str):
        return extract_json_from_text(raw)
    try:
        content = raw.choices[0].message.content
    except Exception as exc:
        raise ValueError("LLM response does not contain message content.") from exc
    if not isinstance(content, str):
        raise ValueError("LLM response content is not text.")
    return extract_json_from_text(content)


def _parsed_task_from_payload(raw_text: str, payload: dict[str, Any]) -> ParsedTask:
    target = _target_from_payload(payload.get("target"))
    area = _area_from_payload(payload.get("area"))
    subtasks = _subtasks_from_payload(payload, target)
    confidence = _confidence_from_payload(payload.get("confidence"))
    parsed = ParsedTask(
        raw_task=str(payload.get("raw_task") or raw_text),
        language=str(payload.get("language") or "zh"),
        task_summary=str(payload.get("task_summary") or ""),
        primary_intent=_enum_or_value(TaskIntent, payload.get("primary_intent")),
        target=target,
        area=area,
        requested_subtasks=subtasks,
        safety_assessment=_safety_from_payload(payload.get("safety_assessment")),
        execution_recommendation=_recommendation_from_payload(
            payload.get("execution_recommendation")
        ),
        confidence=confidence,
        initial_visibility_state="unknown",
        raw_llm_response=payload,
        parser_source="llm",
        notes=[str(item) for item in payload.get("notes", []) if str(item).strip()]
        if isinstance(payload.get("notes"), list)
        else [],
    )
    if parsed.primary_intent == TaskIntent.UNKNOWN and not parsed.requested_subtasks:
        raise ValueError("LLM task parse did not contain an actionable schema.")
    return parsed


def _target_from_payload(payload: Any) -> TaskTarget:
    if not isinstance(payload, dict):
        return TaskTarget()
    category = _enum_or_value(TargetCategory, payload.get("category"))
    name_zh = str(
        payload.get("name_zh")
        or payload.get("raw_text")
        or payload.get("name")
        or ""
    ).strip()
    return TaskTarget(
        name_zh=name_zh,
        name_en=str(payload.get("name_en") or "").strip(),
        category=category,
        attributes=_string_list(payload.get("attributes")),
        relations=_string_list(payload.get("relations")),
    )


def _area_from_payload(payload: Any) -> TaskArea:
    if not isinstance(payload, dict):
        return TaskArea()
    return TaskArea(
        name_zh=str(payload.get("name_zh") or payload.get("raw_text") or "").strip(),
        category=_enum_or_value(TargetCategory, payload.get("category")),
    )


def _subtasks_from_payload(
    payload: dict[str, Any],
    target: TaskTarget,
) -> list[RequestedSubtask]:
    raw_subtasks = payload.get("requested_subtasks")
    if raw_subtasks is None:
        raw_subtasks = payload.get("subtasks")
    if not isinstance(raw_subtasks, list):
        return []
    subtasks = []
    for index, item in enumerate(raw_subtasks, start=1):
        if not isinstance(item, dict):
            continue
        subtask_type = _enum_or_value(
            SubtaskType,
            item.get("type") or item.get("subtask_type"),
        )
        subtasks.append(
            RequestedSubtask(
                id=str(item.get("id") or f"subtask_{index}"),
                type=subtask_type,
                object=str(
                    item.get("object")
                    or item.get("target_object")
                    or target.name_zh
                    or ""
                ),
                recipient_or_target=str(item.get("recipient_or_target") or ""),
                state_query=str(item.get("state_query") or ""),
                is_navigation_relevant=bool(
                    item.get("is_navigation_relevant", item.get("navigation_relevant", False))
                ),
                requires_visual_grounding=bool(
                    item.get("requires_visual_grounding", True)
                ),
                requires_manipulation=bool(item.get("requires_manipulation", False)),
                requires_physical_contact=bool(
                    item.get("requires_physical_contact", False)
                ),
                requires_harmful_action=bool(item.get("requires_harmful_action", False)),
                semantic_role=str(item.get("semantic_role") or "main_goal"),
                llm_reason=str(
                    item.get("llm_reason")
                    or item.get("description")
                    or subtask_type.value
                ),
                requested_by_user=bool(item.get("requested_by_user", True)),
            )
        )
    return subtasks


def _safety_from_payload(payload: Any) -> SafetyAssessment:
    if not isinstance(payload, dict):
        return SafetyAssessment()
    return SafetyAssessment(
        contains_physical_harm_request=bool(
            payload.get("contains_physical_harm_request", False)
        ),
        contains_manipulation_request=bool(
            payload.get("contains_manipulation_request", False)
        ),
        contains_privacy_sensitive_request=bool(
            payload.get("contains_privacy_sensitive_request", False)
        ),
        contains_non_navigation_request=bool(
            payload.get("contains_non_navigation_request", False)
        ),
        risk_level=str(payload.get("risk_level") or "none"),
        reason=str(payload.get("reason") or ""),
    )


def _recommendation_from_payload(payload: Any) -> ExecutionRecommendation:
    if not isinstance(payload, dict):
        return ExecutionRecommendation()
    return ExecutionRecommendation(
        can_execute_fully=bool(payload.get("can_execute_fully", False)),
        can_execute_navigation_part=bool(
            payload.get("can_execute_navigation_part", False)
        ),
        allowed_navigation_subtasks=_string_list(
            payload.get("allowed_navigation_subtasks")
        ),
        unsupported_subtasks=_string_list(payload.get("unsupported_subtasks")),
        user_feedback_zh=str(payload.get("user_feedback_zh") or ""),
    )


def _confidence_from_payload(payload: Any) -> TaskParseConfidence:
    if not isinstance(payload, dict):
        return TaskParseConfidence()
    return TaskParseConfidence(
        intent=_float_between_zero_and_one(payload.get("intent")),
        safety=_float_between_zero_and_one(payload.get("safety")),
        target=_float_between_zero_and_one(payload.get("target")),
    )


def _verifier_accepts(payload: dict[str, Any]) -> bool:
    return bool(payload.get("is_consistent")) and _float_between_zero_and_one(
        payload.get("confidence")
    ) >= VERIFIER_CONFIDENCE_THRESHOLD


def _enum_or_value(enum_cls: type[Enum], value: Any) -> Any:
    try:
        raw = value.value if isinstance(value, Enum) else str(value)
        return enum_cls(raw)
    except Exception:
        return enum_cls.UNKNOWN if hasattr(enum_cls, "UNKNOWN") else str(value or "")


def _string_list(value: Any) -> list[str]:
    if not isinstance(value, list):
        return []
    return [str(item).strip() for item in value if str(item).strip()]


def _float_between_zero_and_one(value: Any) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    return max(0.0, min(1.0, number))
