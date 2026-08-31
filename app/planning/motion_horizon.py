"""High-level adaptive motion horizon planning.

This module decides only how long a high-level motion segment may be.
Obstacle avoidance and emergency interruption remain the responsibility of
the robot platform, SDK, ROS safety layer, or operator.
"""

from __future__ import annotations

from app.config import Settings
from app.schemas import MotionHorizonDecision, SceneAnalysisResult


OPEN_SCENES = {
    "outdoor",
    "open_area",
    "platform_assisted_open_area",
    "hall",
    "large_room",
    "yard",
    "road",
    "corridor_long",
}
INDOOR_SCENES = {
    "indoor",
    "platform_assisted_indoor",
    "office",
    "home",
    "corridor",
    "lab",
    "room",
}


def estimate_motion_horizon(
    *,
    requested_distance_m: float | None,
    scene_type: str,
    task_phase: str,
    target_candidate_visible: bool,
    target_confirming: bool,
    llm_recommended_horizon_m: float | None,
    settings: Settings,
) -> MotionHorizonDecision:
    """Estimate the final high-level movement distance for one segment."""

    requested = _positive_or_none(requested_distance_m)
    scene = _normalize_token(scene_type) or "unknown"
    phase = _normalize_token(task_phase) or "search"
    profile = _normalize_token(settings.motion_horizon_profile) or "platform_assisted_auto"
    absolute_max = max(0.0, settings.motion_absolute_max_step_m)
    strict_max = _bounded_max(settings.motion_strict_safe_max_step_m, absolute_max)

    if not settings.enable_dynamic_motion_horizon:
        final = min(requested if requested is not None else strict_max, strict_max)
        return MotionHorizonDecision(
            enabled=False,
            profile=profile,
            platform_obstacle_avoidance_assumed=settings.platform_obstacle_avoidance_assumed,
            scene_type=scene,
            task_phase=phase,
            motion_policy="strict_safe",
            recommended_distance_m=round(final, 3),
            max_allowed_distance_m=round(strict_max, 3),
            original_requested_distance_m=requested,
            clipped_distance_m=round(final, 3),
            llm_recommended_horizon_m=_positive_or_none(llm_recommended_horizon_m),
            requires_stop_after_motion=True,
            observe_while_moving=False,
            source="disabled",
            confidence=0.6,
            decision_reason_zh="动态移动视界未启用，使用严格安全单步距离。",
        )

    if profile == "strict_safe" or not settings.platform_obstacle_avoidance_assumed:
        default_step = strict_max
        max_step = strict_max
        policy = "strict_safe"
        reason = "未启用平台避障假设或处于严格安全档位，使用保守单步距离。"
        source = "rule"
        confidence = 0.7
        shorten_reason = (
            "platform_obstacle_avoidance_not_assumed"
            if not settings.platform_obstacle_avoidance_assumed
            else None
        )
    elif (
        settings.motion_shorten_on_target_candidate
        and (
            target_candidate_visible
            or target_confirming
            or phase in {"confirm_target", "approach_candidate"}
        )
    ):
        default_step = min(
            settings.motion_target_confirm_max_step_m,
            settings.motion_platform_indoor_default_step_m,
        )
        max_step = settings.motion_target_confirm_max_step_m
        policy = "target_candidate_confirm"
        reason = "目标候选已出现或处于确认阶段，缩短移动距离并准备停止重观测。"
        source = "rule"
        confidence = 0.78
        shorten_reason = "target_candidate_or_confirming"
    elif profile == "platform_assisted_open_area" or (
        profile == "platform_assisted_auto" and scene in OPEN_SCENES
    ):
        default_step = settings.motion_platform_open_default_step_m
        max_step = settings.motion_platform_open_max_step_m
        policy = "platform_assisted_open_area"
        reason = "当前为开放区域搜索阶段，平台具备基础避障能力，允许较长移动段以提高搜索效率。"
        source = "rule"
        confidence = 0.75
        shorten_reason = None
    elif profile == "platform_assisted_indoor" or (
        profile == "platform_assisted_auto" and scene in INDOOR_SCENES
    ):
        default_step = settings.motion_platform_indoor_default_step_m
        max_step = settings.motion_platform_indoor_max_step_m
        policy = "platform_assisted_indoor"
        reason = "室内搜索阶段，未发现目标候选，平台具备基础避障，可移动到下一个观察点后重观测。"
        source = "rule"
        confidence = 0.72
        shorten_reason = None
    else:
        default_step = settings.motion_platform_fallback_step_m
        max_step = default_step
        policy = "platform_assisted_fallback"
        reason = "场景类型不确定，使用平台避障辅助的保守默认移动距离。"
        source = "fallback"
        confidence = 0.62
        shorten_reason = "unknown_scene_fallback"

    llm_value = _positive_or_none(llm_recommended_horizon_m)
    if (
        settings.motion_allow_llm_recommended_horizon
        and llm_value is not None
        and profile != "strict_safe"
        and settings.platform_obstacle_avoidance_assumed
    ):
        weight = max(0.0, min(1.0, settings.motion_llm_horizon_weight))
        blended = weight * llm_value + (1.0 - weight) * default_step
        source = "mixed"
    else:
        blended = default_step

    max_allowed = _bounded_max(max_step, absolute_max)
    recommended_limit = min(max(0.0, blended), max_allowed)
    if requested is None:
        final = recommended_limit
    else:
        final = min(requested, recommended_limit, max_allowed)
    final = max(0.0, final)

    return MotionHorizonDecision(
        enabled=True,
        profile=profile,
        platform_obstacle_avoidance_assumed=settings.platform_obstacle_avoidance_assumed,
        scene_type=scene,
        task_phase=phase,
        motion_policy=policy,
        recommended_distance_m=round(final, 3),
        max_allowed_distance_m=round(max_allowed, 3),
        original_requested_distance_m=requested,
        clipped_distance_m=round(final, 3),
        llm_recommended_horizon_m=llm_value,
        requires_stop_after_motion=settings.motion_default_stop_and_reobserve,
        observe_while_moving=settings.motion_enable_observe_while_moving,
        soft_observe_interval_sec=(
            settings.motion_soft_observe_interval_sec
            if settings.motion_enable_observe_while_moving
            else None
        ),
        shorten_reason=shorten_reason,
        decision_reason_zh=reason,
        source=source,
        confidence=confidence,
    )


def infer_scene_type_from_result(result: SceneAnalysisResult) -> str:
    """Infer a coarse scene label from the available single-image output."""

    text = " ".join(
        [
            result.scene_summary_zh,
            result.route_plan.summary_zh,
            *[obj.name for obj in result.objects],
            *[obj.name_zh for obj in result.objects],
            *[obj.category for obj in result.objects],
        ]
    ).lower()
    if any(term in text for term in ["outdoor", "road", "yard", "操场", "道路", "室外"]):
        return "open_area"
    if any(term in text for term in ["大厅", "hall", "large room", "开阔"]):
        return "hall"
    if any(term in text for term in ["长走廊", "corridor long", "long corridor"]):
        return "corridor_long"
    if any(term in text for term in ["corridor", "走廊"]):
        return "corridor"
    if any(
        term in text
        for term in ["office", "desk", "monitor", "keyboard", "办公室", "桌", "显示器"]
    ):
        return "office"
    if any(term in text for term in ["room", "indoor", "房间", "室内"]):
        return "indoor"
    return "unknown"


def infer_task_phase_from_result(result: SceneAnalysisResult) -> str:
    if result.target_decision.is_present:
        return "confirm_target"
    if _has_target_candidate(result):
        return "approach_candidate"
    return "search"


def has_target_candidate(result: SceneAnalysisResult) -> bool:
    return _has_target_candidate(result)


def _has_target_candidate(result: SceneAnalysisResult) -> bool:
    if result.target_decision.is_present:
        return True
    for item in result.candidate_objects:
        if item.get("decision") in {"candidate", "confirmed"}:
            return True
        score = item.get("final_score", item.get("confidence", 0.0))
        try:
            if float(score) >= 0.45:
                return True
        except (TypeError, ValueError):
            continue
    return bool((result.candidate_summary or {}).get("num_candidate", 0))


def _positive_or_none(value: float | None) -> float | None:
    if value is None:
        return None
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None
    if numeric <= 0:
        return None
    return numeric


def _bounded_max(value: float, absolute_max: float) -> float:
    return max(0.0, min(max(0.0, value), absolute_max))


def _normalize_token(value: str | None) -> str:
    return " ".join(str(value or "").strip().lower().replace("-", "_").split())
