"""Export route plans as ROS2-friendly Twist command data."""

from __future__ import annotations

import json
import math
from datetime import UTC, datetime
from pathlib import Path

from app.config import Settings, get_settings
from app.planning.motion_horizon import (
    estimate_motion_horizon,
    has_target_candidate,
    infer_scene_type_from_result,
    infer_task_phase_from_result,
)
from app.schemas import (
    MotionHorizonDecision,
    QuadrupedActionPrimitive,
    QuadrupedSearchPlan,
    Ros2MotionCommand,
    Ros2MotionPlan,
    Ros2Twist,
    Ros2Vector3,
    RouteStep,
    SceneAnalysisResult,
)


DEFAULT_CMD_VEL_TOPIC = "/cmd_vel"
DEFAULT_FRAME_ID = "base_link"
DEFAULT_COMMAND_RATE_HZ = 10.0
DEFAULT_LINEAR_SPEED_MPS = 0.25
DEFAULT_ANGULAR_SPEED_RADPS = 0.5
DEFAULT_STOP_DURATION_SEC = 1.0
DEFAULT_MAX_FORWARD_STEP_M = 0.5
MIN_COMMAND_DURATION_SEC = 0.2


def build_ros2_motion_plan(
    result: SceneAnalysisResult,
    *,
    generated_at: str | None = None,
    topic: str = DEFAULT_CMD_VEL_TOPIC,
    frame_id: str = DEFAULT_FRAME_ID,
    command_rate_hz: float = DEFAULT_COMMAND_RATE_HZ,
    linear_speed_mps: float = DEFAULT_LINEAR_SPEED_MPS,
    angular_speed_radps: float = DEFAULT_ANGULAR_SPEED_RADPS,
    settings: Settings | None = None,
) -> Ros2MotionPlan:
    """Convert a scene route plan to dry-run ROS2 /cmd_vel command data."""

    if linear_speed_mps <= 0:
        raise ValueError("linear_speed_mps must be positive")
    if angular_speed_radps <= 0:
        raise ValueError("angular_speed_radps must be positive")
    if command_rate_hz <= 0:
        raise ValueError("command_rate_hz must be positive")

    route_plan = result.route_plan
    steps = sorted(route_plan.steps, key=lambda item: item.step_id)
    commands: list[Ros2MotionCommand] = []
    effective_settings = settings or get_settings()
    scene_type = infer_scene_type_from_result(result)
    task_phase = infer_task_phase_from_result(result)
    target_candidate_visible = has_target_candidate(result)
    plan_decision: MotionHorizonDecision | None = None
    for step_index, step in enumerate(steps):
        step_decision = _decision_for_route_step(
            step,
            scene_type=scene_type,
            task_phase=task_phase,
            target_candidate_visible=target_candidate_visible,
            settings=effective_settings,
        )
        if step_decision is not None:
            plan_decision = plan_decision or step_decision
        commands.append(
            _command_from_step(
                index=len(commands) + 1,
                step=step,
                topic=topic,
                linear_speed_mps=linear_speed_mps,
                angular_speed_radps=angular_speed_radps,
                motion_horizon_decision=step_decision,
            )
        )
        next_is_stop = (
            step_index + 1 < len(steps)
            and steps[step_index + 1].action == "stop"
        )
        if step.action in {
            "move_forward",
            "move_backward",
            "turn_left",
            "turn_right",
        } and not next_is_stop:
            commands.append(
                _safe_stop_command(
                    topic=topic,
                    index=len(commands) + 1,
                    description_zh="运动后强制停止并重新观察。",
                )
            )
    if not commands:
        commands.append(_safe_stop_command(topic=topic, index=1))
    if plan_decision is None:
        plan_decision = estimate_motion_horizon(
            requested_distance_m=None,
            scene_type=scene_type,
            task_phase=task_phase,
            target_candidate_visible=target_candidate_visible,
            target_confirming=task_phase == "confirm_target",
            llm_recommended_horizon_m=None,
            settings=effective_settings,
        )

    timestamp = generated_at or datetime.now(UTC).isoformat()
    return Ros2MotionPlan(
        plan_id=f"ros2_motion_{timestamp.replace(':', '').replace('+', 'Z')}",
        generated_at=timestamp,
        dry_run=True,
        topic=topic,
        frame_id=frame_id,
        route_type=route_plan.route_type,
        route_summary_zh=route_plan.summary_zh,
        command_rate_hz=command_rate_hz,
        commands=commands,
        safety_notes_zh=[
            *route_plan.safety_notes_zh,
            "当前输出为 dry_run 数据，不会直接控制机器狗。",
            "单次前进/后退距离由动态运动视界策略裁剪，不再固定为 0.5 米。",
            "每个高层移动或转向动作后仍插入停止重观测。",
            "本项目只输出高层 motion plan；真实避障、急停和可行走区域由平台安全层负责。",
        ],
        integration_notes_zh=[
            "ROS2 节点可按 commands 顺序向 /cmd_vel 发布 geometry_msgs/msg/Twist。",
            "每条 command 持续发布 duration_sec 秒，发布频率使用 command_rate_hz。",
            "可先运行 python scripts/publish_ros2_motion_plan.py outputs/ros2_motion_plan.json 预览指令。",
            "接入 ROS2 后可运行 python scripts/publish_ros2_motion_plan.py outputs/ros2_motion_plan.json --execute --allow-dry-run-plan 发布指令。",
            "所有 command 执行完毕后应再次发布零速度 Twist，确保机器狗停止。",
        ],
        platform_obstacle_avoidance_assumed=(
            plan_decision.platform_obstacle_avoidance_assumed
        ),
        dynamic_motion_horizon_enabled=plan_decision.enabled,
        motion_horizon_profile=plan_decision.profile,
        motion_horizon_decision=plan_decision.model_dump(mode="json"),
    )


def export_ros2_motion_plan(
    result: SceneAnalysisResult,
    output_path: str | Path,
    *,
    settings: Settings | None = None,
) -> Path:
    path = Path(output_path)
    plan = build_ros2_motion_plan(result, settings=settings)
    path.write_text(
        json.dumps(plan.model_dump(mode="json"), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return path


def build_quadruped_ros2_motion_plan(
    plan: QuadrupedSearchPlan,
    *,
    generated_at: str | None = None,
    topic: str = DEFAULT_CMD_VEL_TOPIC,
    frame_id: str = DEFAULT_FRAME_ID,
    command_rate_hz: float = DEFAULT_COMMAND_RATE_HZ,
    linear_speed_mps: float = DEFAULT_LINEAR_SPEED_MPS,
    angular_speed_radps: float = DEFAULT_ANGULAR_SPEED_RADPS,
    settings: Settings | None = None,
) -> Ros2MotionPlan:
    effective_settings = settings or get_settings()
    plan_decision = plan.motion_horizon_decision or estimate_motion_horizon(
        requested_distance_m=None,
        scene_type="unknown",
        task_phase="confirm_target" if plan.target_found else "search",
        target_candidate_visible=plan.target_found,
        target_confirming=plan.target_found,
        llm_recommended_horizon_m=None,
        settings=effective_settings,
    )
    route_steps: list[RouteStep] = []
    non_motion_notes: list[str] = []
    for viewpoint in plan.steps:
        primitive = viewpoint.primitive
        if primitive == QuadrupedActionPrimitive.TURN_LEFT:
            _append_route_step(route_steps, "turn_left", viewpoint.reason_zh, angle=20.0)
        elif primitive == QuadrupedActionPrimitive.TURN_RIGHT:
            _append_route_step(route_steps, "turn_right", viewpoint.reason_zh, angle=20.0)
        elif primitive == QuadrupedActionPrimitive.MOVE_FORWARD_SHORT:
            requested = viewpoint.motion_horizon_m or plan_decision.recommended_distance_m
            decision = estimate_motion_horizon(
                requested_distance_m=requested,
                scene_type=plan_decision.scene_type,
                task_phase=plan_decision.task_phase,
                target_candidate_visible=plan_decision.task_phase
                in {"confirm_target", "approach_candidate"},
                target_confirming=plan.target_found,
                llm_recommended_horizon_m=plan_decision.llm_recommended_horizon_m,
                settings=effective_settings,
            )
            _append_route_step(
                route_steps,
                "move_forward",
                viewpoint.reason_zh,
                distance=decision.recommended_distance_m,
            )
        elif primitive == QuadrupedActionPrimitive.MOVE_BACKWARD_SHORT:
            _append_route_step(
                route_steps,
                "move_backward",
                viewpoint.reason_zh,
                distance=min(0.3, plan_decision.recommended_distance_m),
            )
        elif primitive == QuadrupedActionPrimitive.RETURN_TO_LAST_SAFE_POSE:
            _append_route_step(
                route_steps,
                "move_backward",
                viewpoint.reason_zh,
                distance=min(0.3, plan_decision.recommended_distance_m),
            )
        elif primitive == QuadrupedActionPrimitive.SCAN_LEFT_TO_RIGHT:
            _append_route_step(route_steps, "turn_left", "扫描左侧视野。", angle=20.0)
            _append_route_step(route_steps, "stop", "停止并观察左侧视野。")
            _append_route_step(route_steps, "turn_right", "扫描至右侧视野。", angle=40.0)
        elif primitive in {
            QuadrupedActionPrimitive.STOP_AND_REOBSERVE,
            QuadrupedActionPrimitive.CENTER_VIEW_ON_REGION,
        }:
            _append_route_step(route_steps, "stop", viewpoint.reason_zh)
        else:
            non_motion_notes.append(
                f"{primitive.value}: {viewpoint.reason_zh}"
            )

    commands: list[Ros2MotionCommand] = []
    for index, route_step in enumerate(route_steps):
        commands.append(
            _command_from_step(
                index=len(commands) + 1,
                step=route_step,
                topic=topic,
                linear_speed_mps=linear_speed_mps,
                angular_speed_radps=angular_speed_radps,
                motion_horizon_decision=(
                    plan_decision
                    if route_step.action in {"move_forward", "move_backward"}
                    else None
                ),
            )
        )
        next_is_stop = (
            index + 1 < len(route_steps)
            and route_steps[index + 1].action == "stop"
        )
        if route_step.action != "stop" and not next_is_stop:
            commands.append(
                _safe_stop_command(
                    topic=topic,
                    index=len(commands) + 1,
                    description_zh="视角动作后强制停止并重新观察。",
                )
            )
    if not commands:
        commands.append(_safe_stop_command(topic=topic, index=1))

    timestamp = generated_at or datetime.now(UTC).isoformat()
    return Ros2MotionPlan(
        plan_id=f"quadruped_ros2_{timestamp.replace(':', '').replace('+', 'Z')}",
        generated_at=timestamp,
        dry_run=True,
        topic=topic,
        frame_id=frame_id,
        route_type=plan.plan_type,
        route_summary_zh=f"机械狗下一视角计划：{plan.target_text}",
        command_rate_hz=command_rate_hz,
        commands=commands,
        safety_notes_zh=[
            "仅导出 turn / short move / stop 对应的 dry-run Twist。",
            "center_view_on_region 只导出停止重观测，不假设相机云台存在。",
            "请求人工和标记不可达不会转换成运动指令。",
            "本项目不实现避障；真实执行前必须由平台、SLAM、急停或厂商安全策略兜底。",
            *plan.non_executable_notes_zh,
            *non_motion_notes,
        ],
        integration_notes_zh=[
            "该文件来自 quadruped_search_plan，而不是单图距离路线。",
            "所有运动指令后均包含零速度停止指令。",
        ],
        platform_obstacle_avoidance_assumed=(
            plan_decision.platform_obstacle_avoidance_assumed
        ),
        dynamic_motion_horizon_enabled=plan_decision.enabled,
        motion_horizon_profile=plan_decision.profile,
        motion_horizon_decision=plan_decision.model_dump(mode="json"),
    )


def export_quadruped_ros2_motion_plan(
    plan: QuadrupedSearchPlan,
    output_path: str | Path,
    *,
    settings: Settings | None = None,
) -> Path:
    path = Path(output_path)
    payload = build_quadruped_ros2_motion_plan(plan, settings=settings)
    path.write_text(
        json.dumps(payload.model_dump(mode="json"), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return path


def _append_route_step(
    steps: list[RouteStep],
    action: str,
    description: str,
    *,
    distance: float | None = None,
    angle: float | None = None,
) -> None:
    steps.append(
        RouteStep(
            step_id=len(steps) + 1,
            action=action,  # type: ignore[arg-type]
            distance_m=distance,
            turn_angle_deg=angle,
            description_zh=description,
        )
    )


def _command_from_step(
    *,
    index: int,
    step: RouteStep,
    topic: str,
    linear_speed_mps: float,
    angular_speed_radps: float,
    motion_horizon_decision: MotionHorizonDecision | None = None,
) -> Ros2MotionCommand:
    linear_x = 0.0
    angular_z = 0.0
    duration_sec = DEFAULT_STOP_DURATION_SEC

    if step.action == "move_forward":
        distance = _motion_distance(step, motion_horizon_decision)
        linear_x = linear_speed_mps
        duration_sec = _duration(distance / linear_speed_mps)
    elif step.action == "move_backward":
        distance = _motion_distance(step, motion_horizon_decision)
        linear_x = -linear_speed_mps
        duration_sec = _duration(distance / linear_speed_mps)
    elif step.action == "turn_left":
        angle_rad = math.radians(abs(step.turn_angle_deg or 0.0))
        angular_z = angular_speed_radps
        duration_sec = _duration(angle_rad / angular_speed_radps)
    elif step.action == "turn_right":
        angle_rad = math.radians(abs(step.turn_angle_deg or 0.0))
        angular_z = -angular_speed_radps
        duration_sec = _duration(angle_rad / angular_speed_radps)
    elif step.action == "stop":
        duration_sec = DEFAULT_STOP_DURATION_SEC

    return Ros2MotionCommand(
        command_id=f"cmd_{index:03d}",
        route_step_id=step.step_id,
        source_action=step.action,
        topic=topic,
        twist=Ros2Twist(
            linear=Ros2Vector3(x=linear_x),
            angular=Ros2Vector3(z=angular_z),
        ),
        duration_sec=duration_sec,
        distance_m=(
            distance
            if step.action in {"move_forward", "move_backward"}
            else step.distance_m
        ),
        turn_angle_deg=step.turn_angle_deg,
        description_zh=step.description_zh,
        interruptible_by_platform=(
            bool(motion_horizon_decision)
            and motion_horizon_decision.platform_obstacle_avoidance_assumed
            and step.action in {"move_forward", "move_backward"}
        ),
        platform_obstacle_avoidance_assumed=(
            motion_horizon_decision.platform_obstacle_avoidance_assumed
            if motion_horizon_decision is not None
            else False
        ),
        requires_stop_after_motion=(
            motion_horizon_decision.requires_stop_after_motion
            if motion_horizon_decision is not None
            else True
        ),
        observe_while_moving=(
            motion_horizon_decision.observe_while_moving
            if motion_horizon_decision is not None
            else False
        ),
    )


def _safe_stop_command(
    topic: str,
    index: int,
    description_zh: str = "无可执行路线，保持停止。",
) -> Ros2MotionCommand:
    return Ros2MotionCommand(
        command_id=f"cmd_{index:03d}",
        route_step_id=None,
        source_action="stop",
        topic=topic,
        twist=Ros2Twist(),
        duration_sec=DEFAULT_STOP_DURATION_SEC,
        description_zh=description_zh,
    )


def _duration(value: float) -> float:
    return round(max(MIN_COMMAND_DURATION_SEC, value), 3)


def _decision_for_route_step(
    step: RouteStep,
    *,
    scene_type: str,
    task_phase: str,
    target_candidate_visible: bool,
    settings: Settings,
) -> MotionHorizonDecision | None:
    if step.action not in {"move_forward", "move_backward"}:
        return None
    return estimate_motion_horizon(
        requested_distance_m=step.distance_m,
        scene_type=scene_type,
        task_phase=task_phase,
        target_candidate_visible=target_candidate_visible,
        target_confirming=task_phase == "confirm_target",
        llm_recommended_horizon_m=None,
        settings=settings,
    )


def _motion_distance(
    step: RouteStep,
    decision: MotionHorizonDecision | None,
) -> float:
    if decision is not None:
        return decision.recommended_distance_m
    return max(0.0, step.distance_m or 0.0)
