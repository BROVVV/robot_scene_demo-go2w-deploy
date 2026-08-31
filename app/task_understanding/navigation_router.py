"""Route capability-gated subtasks into executable navigation plans."""

from __future__ import annotations

from app.task_understanding.schemas import (
    ActionabilityResult,
    NavigationTask,
    ParsedTask,
    TaskIntent,
)


def route_navigation_task(
    parsed_task: ParsedTask,
    actionability: ActionabilityResult,
) -> NavigationTask:
    constraints = dict(actionability.execution_constraints)
    constraints.update(
        {
            "initial_visibility_state": "unknown",
            "requires_visual_grounding": True,
            "contact_allowed": False,
            "harmful_action_allowed": False,
            "manipulation_allowed": False,
            "stop_after_motion": True,
        }
    )

    if not actionability.navigation_part_executable:
        return NavigationTask(
            executable=False,
            navigation_goal="none",
            target=parsed_task.target,
            area=parsed_task.area,
            subtasks=[],
            constraints={**constraints, "reason": "no executable navigation subtask"},
            user_feedback_zh=actionability.user_feedback_zh,
            source_raw_task=parsed_task.raw_task,
            intent=parsed_task.primary_intent,
            blocked_subtasks=actionability.blocked_subtasks,
            requires_visual_grounding=False,
        )

    intent = _intent_value(parsed_task.primary_intent)
    if intent == TaskIntent.FIND_ROOM.value:
        goal = "find_room"
    elif intent == TaskIntent.CHECK_DOOR_STATE.value:
        goal = "inspect_door_state"
    elif intent == TaskIntent.LOCATE_PERSON.value:
        goal = "locate_person_safe_distance"
        constraints["requires_safe_standoff_distance"] = True
    elif intent == TaskIntent.LOCATE_OBJECT.value:
        goal = "locate_object"
    elif intent in {TaskIntent.INSPECT_AREA.value, TaskIntent.PATROL_AREA.value}:
        goal = intent
    elif intent == TaskIntent.CHECK_PASSABLE_AREA.value:
        goal = "check_passable_area"
    else:
        goal = "mixed_navigation"

    return NavigationTask(
        executable=True,
        navigation_goal=goal,
        target=parsed_task.target,
        area=parsed_task.area,
        subtasks=actionability.allowed_subtasks,
        constraints=constraints,
        user_feedback_zh=actionability.user_feedback_zh,
        source_raw_task=parsed_task.raw_task,
        intent=parsed_task.primary_intent,
        blocked_subtasks=actionability.blocked_subtasks,
        requires_visual_grounding=True,
    )


def build_navigation_task(
    parsed_task: ParsedTask,
    gate_result: ActionabilityResult,
) -> NavigationTask:
    return route_navigation_task(parsed_task, gate_result)


def _intent_value(intent: str | TaskIntent) -> str:
    return intent.value if isinstance(intent, TaskIntent) else str(intent)
