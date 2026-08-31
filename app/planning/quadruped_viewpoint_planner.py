"""Translate validated hypotheses into conservative quadruped viewpoint steps."""

from __future__ import annotations

from app.schemas import (
    Actionability,
    LLMSearchHypothesis,
    MotionHorizonDecision,
    QuadrupedActionPrimitive,
    QuadrupedSearchPlan,
    RobotCapabilityContract,
    RobotTask,
    TaskPlan,
    TaskPlanStep,
    ViewpointStep,
)


MOTION_PRIMITIVES = {
    QuadrupedActionPrimitive.TURN_LEFT,
    QuadrupedActionPrimitive.TURN_RIGHT,
    QuadrupedActionPrimitive.MOVE_FORWARD_SHORT,
    QuadrupedActionPrimitive.MOVE_BACKWARD_SHORT,
    QuadrupedActionPrimitive.RETURN_TO_LAST_SAFE_POSE,
}


def plan_quadruped_viewpoints(
    *,
    target_text: str,
    target_found: bool,
    hypotheses: list[LLMSearchHypothesis],
    contract: RobotCapabilityContract,
    include_non_executable_steps: bool = True,
    motion_horizon_decision: MotionHorizonDecision | None = None,
) -> QuadrupedSearchPlan:
    steps: list[ViewpointStep] = []
    notes: list[str] = []
    if target_found:
        steps.append(
            _step(
                1,
                QuadrupedActionPrimitive.STOP_AND_REOBSERVE,
                None,
                "目标已由视觉确认；保持安全距离并重新观察。",
                0.9,
            )
        )
    else:
        for hypothesis in sorted(
            hypotheses, key=lambda item: item.confidence, reverse=True
        ):
            if hypothesis.actionability in {
                Actionability.NEEDS_HUMAN,
                Actionability.UNSAFE_OR_IMPOSSIBLE,
            }:
                notes.append(
                    f"{hypothesis.candidate_region_zh}：{hypothesis.suggested_verification_question_zh}"
                )
                if not include_non_executable_steps:
                    continue
            strategy = _effective_strategy(hypothesis)
            for primitive_index, primitive in enumerate(strategy):
                if primitive not in contract.allowed_primitives:
                    continue
                if primitive == QuadrupedActionPrimitive.MOVE_FORWARD_SHORT:
                    horizon_m = (
                        motion_horizon_decision.recommended_distance_m
                        if motion_horizon_decision is not None
                        else contract.max_forward_step_m
                    )
                    reason = (
                        f"向候选方向移动到下一观察点（建议 {horizon_m:.2f} m，"
                        "真实避障由平台底层负责）。"
                    )
                else:
                    reason = hypothesis.suggested_verification_question_zh
                steps.append(
                    _step(
                        len(steps) + 1,
                        primitive,
                        hypothesis.image_region_hint,
                        reason,
                        hypothesis.confidence,
                        motion_horizon_decision,
                    )
                )
                if (
                    primitive in MOTION_PRIMITIVES
                    and contract.require_stop_after_each_motion
                    and (
                        primitive_index + 1 >= len(strategy)
                        or strategy[primitive_index + 1]
                        != QuadrupedActionPrimitive.STOP_AND_REOBSERVE
                    )
                ):
                    steps.append(
                        _step(
                            len(steps) + 1,
                            QuadrupedActionPrimitive.STOP_AND_REOBSERVE,
                            hypothesis.image_region_hint,
                            "运动后停止并重新观察，等待新的视觉证据。",
                            hypothesis.confidence,
                            motion_horizon_decision,
                        )
                    )
            if len(steps) >= 8:
                break
    if not steps:
        steps.append(
            _step(
                1,
                QuadrupedActionPrimitive.STOP_AND_REOBSERVE,
                None,
                "没有安全可执行候选，保持停止并重新观察。",
                0.3,
            )
        )
    return QuadrupedSearchPlan(
        plan_id="quadruped_search_plan_001",
        target_text=target_text,
        target_found=target_found,
        plan_type="observe_confirmed_target" if target_found else "situated_viewpoint_search",
        steps=steps,
        non_executable_notes_zh=notes,
        should_export_ros2_dry_run=True,
        motion_horizon_decision=motion_horizon_decision,
    )


def _effective_strategy(
    hypothesis: LLMSearchHypothesis,
) -> list[QuadrupedActionPrimitive]:
    strategy = list(hypothesis.quadruped_view_strategy)
    meaningful = [
        item
        for item in strategy
        if item != QuadrupedActionPrimitive.STOP_AND_REOBSERVE
    ]
    if meaningful:
        return strategy
    hint = (hypothesis.image_region_hint or "").lower()
    context = " ".join(
        [
            hypothesis.candidate_region_type,
            hypothesis.candidate_region_zh,
            hypothesis.human_like_rationale_zh,
        ]
    ).lower()
    if "left" in hint:
        return [
            QuadrupedActionPrimitive.TURN_LEFT,
            QuadrupedActionPrimitive.STOP_AND_REOBSERVE,
        ]
    if "right" in hint:
        return [
            QuadrupedActionPrimitive.TURN_RIGHT,
            QuadrupedActionPrimitive.STOP_AND_REOBSERVE,
        ]
    if any(term in context for term in ["走廊", "门口", "door", "corridor"]):
        return [
            QuadrupedActionPrimitive.SCAN_LEFT_TO_RIGHT,
            QuadrupedActionPrimitive.STOP_AND_REOBSERVE,
        ]
    if any(term in context for term in ["较远", "太远", "far", "distant"]):
        return [
            QuadrupedActionPrimitive.MOVE_FORWARD_SHORT,
            QuadrupedActionPrimitive.STOP_AND_REOBSERVE,
        ]
    return [
        QuadrupedActionPrimitive.CENTER_VIEW_ON_REGION,
        QuadrupedActionPrimitive.STOP_AND_REOBSERVE,
    ]


def _step(
    index: int,
    primitive: QuadrupedActionPrimitive,
    region: str | None,
    reason: str,
    information_gain: float,
    motion_horizon_decision: MotionHorizonDecision | None = None,
) -> ViewpointStep:
    is_forward = primitive == QuadrupedActionPrimitive.MOVE_FORWARD_SHORT
    return ViewpointStep(
        step_id=f"view_{index:03d}",
        primitive=primitive,
        target_image_region=region,
        reason_zh=reason,
        expected_information_gain=max(0.0, min(1.0, information_gain)),
        safety_level=(
            "human_required"
            if primitive
            in {
                QuadrupedActionPrimitive.ASK_HUMAN_FOR_OCCLUDED_AREA,
                QuadrupedActionPrimitive.MARK_UNREACHABLE,
            }
            else "conservative"
        ),
        requires_reobserve_after=True,
        motion_horizon_m=(
            motion_horizon_decision.recommended_distance_m
            if motion_horizon_decision is not None and is_forward
            else None
        ),
        motion_policy=(
            motion_horizon_decision.motion_policy
            if motion_horizon_decision is not None and is_forward
            else None
        ),
        requires_stop_observation=(
            motion_horizon_decision.requires_stop_after_motion
            if motion_horizon_decision is not None
            else True
        ),
        platform_obstacle_avoidance_assumed=(
            motion_horizon_decision.platform_obstacle_avoidance_assumed
            if motion_horizon_decision is not None
            else False
        ),
    )


def quadruped_plan_to_task_plan(
    plan: QuadrupedSearchPlan,
    task: RobotTask,
) -> TaskPlan:
    steps: list[TaskPlanStep] = []
    for index, viewpoint in enumerate(plan.steps, start=1):
        primitive = viewpoint.primitive
        if primitive in {
            QuadrupedActionPrimitive.TURN_LEFT,
            QuadrupedActionPrimitive.TURN_RIGHT,
        }:
            action_type = "turn"
        elif primitive in {
            QuadrupedActionPrimitive.MOVE_FORWARD_SHORT,
            QuadrupedActionPrimitive.MOVE_BACKWARD_SHORT,
            QuadrupedActionPrimitive.RETURN_TO_LAST_SAFE_POSE,
        }:
            action_type = "move"
        elif primitive == QuadrupedActionPrimitive.STOP_AND_REOBSERVE:
            action_type = "stop"
        elif primitive in {
            QuadrupedActionPrimitive.ASK_HUMAN_FOR_OCCLUDED_AREA,
            QuadrupedActionPrimitive.MARK_UNREACHABLE,
        }:
            action_type = "summarize"
        else:
            action_type = "observe"
        steps.append(
            TaskPlanStep(
                step_id=index,
                action_type=action_type,  # type: ignore[arg-type]
                target=viewpoint.target_image_region,
                description_zh=viewpoint.reason_zh,
                expected_result=(
                    "获得新的视觉证据，或保持目标未确认状态。"
                ),
                depends_on=[index - 1] if index > 1 else [],
                confidence=viewpoint.expected_information_gain,
            )
        )
    plan_type = (
        task.task_type
        if task.task_type
        in {
            "find_object",
            "count_objects",
            "inspect_area",
            "check_door_state",
            "navigate_to_location",
        }
        else "general"
    )
    return TaskPlan(
        plan_type=plan_type,  # type: ignore[arg-type]
        summary_zh=(
            "目标已由视觉确认，保持安全距离重观测。"
            if plan.target_found
            else "目标尚未视觉确认，按机械狗视角动作逐步获取新证据。"
        ),
        steps=steps,
        success_conditions=[
            "只有获得 bbox、mask、crop 复核或明确视觉证据后才确认目标。",
            "不可验证区域记录为需人工或不可达。",
        ],
        uncertainty_notes=[
            "当前计划仅描述下一视角，不等价于真实导航闭环。",
            *plan.non_executable_notes_zh,
        ],
    )
