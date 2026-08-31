"""Capability and safety gate for structured robot subtasks."""

from __future__ import annotations

from dataclasses import replace

from app.task_understanding.schemas import (
    ActionabilityResult,
    ParsedTask,
    RequestedSubtask,
    SafetyFlag,
    SubtaskType,
    TargetCategory,
)


NAVIGATION_EXECUTABLE_SUBTASKS = {
    SubtaskType.LOCATE_OBJECT,
    SubtaskType.LOCATE_PERSON,
    SubtaskType.FIND_ROOM,
    SubtaskType.LOCATE_AREA,
    SubtaskType.INSPECT_DOOR_STATE,
    SubtaskType.PATROL_AREA,
    SubtaskType.INSPECT_AREA,
    SubtaskType.OBSERVE,
    SubtaskType.OBSERVE_AREA,
    SubtaskType.CHECK_PASSABLE_AREA,
    SubtaskType.SEARCH_TARGET,
    SubtaskType.APPROACH_TARGET,
    SubtaskType.NAVIGATE_TO_VIEWPOINT,
    SubtaskType.STOP_AND_REPORT,
}

MANIPULATION_SUBTASKS = {
    SubtaskType.OPEN_CONTAINER,
    SubtaskType.OPEN_DOOR,
    SubtaskType.PICK_UP_OBJECT,
    SubtaskType.MOVE_OBJECT,
    SubtaskType.DELIVER_OBJECT,
    SubtaskType.MANIPULATE_OBJECT,
}

HARMFUL_SUBTASKS = {
    SubtaskType.PHYSICAL_HARM,
    SubtaskType.PHYSICAL_ASSAULT,
    SubtaskType.DAMAGE_OBJECT,
    SubtaskType.DAMAGE_PROPERTY,
    SubtaskType.CHASE_OR_RAM,
}

DEFAULT_EXECUTION_CONSTRAINTS = {
    "contact_allowed": False,
    "manipulation_allowed": False,
    "harmful_action_allowed": False,
    "chase_or_ram_allowed": False,
    "requires_safe_standoff_distance": False,
    "requires_stop_after_motion": True,
    "requires_stop_after_navigation": True,
    "stop_distance_policy": "safe_distance",
    "max_speed_policy": "normal_or_conservative",
}


def evaluate_actionability(parsed_task: ParsedTask) -> ActionabilityResult:
    safety_flags: list[SafetyFlag] = []
    allowed_subtasks: list[RequestedSubtask] = []
    blocked_subtasks: list[RequestedSubtask] = []
    blocked_reasons: list[str] = []

    if parsed_task.parser_source == "llm_unavailable":
        safety_flags.append(SafetyFlag.LLM_TASK_UNDERSTANDING_UNAVAILABLE)
        blocked_reasons.append("llm_task_understanding_unavailable")
    if parsed_task.parser_source == "llm_verification_failed":
        safety_flags.append(SafetyFlag.LLM_TASK_VERIFICATION_FAILED)
        blocked_reasons.append("llm_task_verification_failed")

    for subtask in parsed_task.requested_subtasks:
        subtask_type = _subtask_type(subtask)

        if subtask.requires_harmful_action or subtask_type in HARMFUL_SUBTASKS:
            _append_unique(safety_flags, SafetyFlag.PHYSICAL_HARM_REQUEST)
            blocked_subtasks.append(
                replace(
                    subtask,
                    allowed_by_capability=False,
                    allowed_by_safety=False,
                    blocked_reason=f"{subtask_type.value}: harmful action is not allowed",
                )
            )
            blocked_reasons.append(f"{subtask_type.value}: harmful action is not allowed")
            continue

        if subtask.requires_manipulation or subtask_type in MANIPULATION_SUBTASKS:
            _append_unique(safety_flags, SafetyFlag.MANIPULATION_REQUIRED)
            _append_unique(safety_flags, SafetyFlag.UNSUPPORTED_MANIPULATION)
            blocked_subtasks.append(
                replace(
                    subtask,
                    allowed_by_capability=False,
                    allowed_by_safety=False,
                    blocked_reason=(
                        f"{subtask_type.value}: robot has no manipulation capability"
                    ),
                )
            )
            blocked_reasons.append(
                f"{subtask_type.value}: robot has no manipulation capability"
            )
            continue

        if subtask.is_navigation_relevant or subtask_type in NAVIGATION_EXECUTABLE_SUBTASKS:
            allowed_subtasks.append(
                replace(
                    subtask,
                    allowed_by_capability=True,
                    allowed_by_safety=True,
                    blocked_reason=None,
                )
            )
            continue

        _append_unique(safety_flags, SafetyFlag.NON_NAVIGATION_REQUEST)
        blocked_subtasks.append(
            replace(
                subtask,
                allowed_by_capability=False,
                allowed_by_safety=False,
                blocked_reason=f"{subtask_type.value}: non-navigation or unsupported task",
            )
        )
        blocked_reasons.append(f"{subtask_type.value}: non-navigation or unsupported task")

    fully_executable = len(blocked_subtasks) == 0 and len(allowed_subtasks) > 0
    navigation_part_executable = len(allowed_subtasks) > 0
    constraints = dict(DEFAULT_EXECUTION_CONSTRAINTS)
    if _category_value(parsed_task.target.category) == TargetCategory.PERSON.value:
        constraints.update(
            {
                "requires_safe_standoff_distance": True,
                "person_target_requires_safe_distance": True,
                "max_speed_policy": "conservative",
            }
        )

    return ActionabilityResult(
        fully_executable=fully_executable,
        navigation_part_executable=navigation_part_executable,
        allowed_subtasks=allowed_subtasks,
        blocked_subtasks=blocked_subtasks,
        safety_flags=safety_flags,
        blocked_reasons=blocked_reasons,
        user_feedback_zh=build_actionability_feedback(
            parsed_task,
            allowed_subtasks,
            blocked_subtasks,
            safety_flags,
        ),
        execution_constraints=constraints,
    )


def evaluate_capability_and_safety(parsed_task: ParsedTask) -> ActionabilityResult:
    return evaluate_actionability(parsed_task)


def build_actionability_feedback(
    parsed_task: ParsedTask,
    allowed_subtasks: list[RequestedSubtask],
    blocked_subtasks: list[RequestedSubtask],
    safety_flags: list[SafetyFlag],
) -> str:
    if parsed_task.execution_recommendation.user_feedback_zh:
        return parsed_task.execution_recommendation.user_feedback_zh

    target_text = _target_text(parsed_task)
    flag_values = {flag.value for flag in safety_flags}
    if allowed_subtasks and not blocked_subtasks:
        if parsed_task.primary_intent == "check_door_state":
            return "我会巡查指定区域，识别门的位置，并判断门的状态。"
        if parsed_task.primary_intent == "find_room":
            return f"我会尝试寻找{target_text}，并移动到更合适的观察位置。"
        return f"我会执行导航相关部分：搜索或观察{target_text}，并反馈视觉结果。"

    if allowed_subtasks and blocked_subtasks:
        allowed_text = "、".join(_subtask_label(item) for item in allowed_subtasks)
        blocked_text = "、".join(_subtask_label(item) for item in blocked_subtasks)
        if SafetyFlag.PHYSICAL_HARM_REQUEST.value in flag_values:
            return (
                f"我可以尝试执行安全导航部分：{allowed_text}，并在安全距离处停止和反馈结果。"
                f"我不能执行{blocked_text}，因此不会生成任何接触、追撞或伤害动作。"
            )
        if SafetyFlag.MANIPULATION_REQUIRED.value in flag_values:
            return (
                f"我不能操作物体，因为当前机器狗没有开合、抓取或搬运能力。"
                f"我可以执行导航观察部分：{allowed_text}。不可执行部分：{blocked_text}。"
            )
        return f"我可以执行导航部分：{allowed_text}。不可执行部分：{blocked_text}。"

    if blocked_subtasks:
        blocked_text = "、".join(_subtask_label(item) for item in blocked_subtasks)
        return f"该任务没有可执行的导航子任务，已拦截：{blocked_text}。"

    if parsed_task.execution_recommendation.user_feedback_zh:
        return parsed_task.execution_recommendation.user_feedback_zh
    return "当前自然语言任务理解不可用或解析结果不可靠，因此不会执行该任务。"


def _subtask_type(subtask: RequestedSubtask) -> SubtaskType:
    try:
        raw = subtask.type.value if isinstance(subtask.type, SubtaskType) else str(subtask.type)
        return SubtaskType(raw)
    except Exception:
        return SubtaskType.UNKNOWN


def _category_value(category: object) -> str:
    return category.value if isinstance(category, TargetCategory) else str(category)


def _append_unique(flags: list[SafetyFlag], flag: SafetyFlag) -> None:
    if flag not in flags:
        flags.append(flag)


def _target_text(parsed_task: ParsedTask) -> str:
    return parsed_task.target.name_zh or parsed_task.target.name_en or parsed_task.raw_task


def _subtask_label(subtask: RequestedSubtask) -> str:
    return subtask.object or subtask.recipient_or_target or str(subtask.type)
