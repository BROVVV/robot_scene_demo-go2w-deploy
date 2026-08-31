"""Convert high-level semantic directives into the existing safe StepPlan."""

from __future__ import annotations

from app.live_robot.step_planner import PlanKind, StepPlan, describe_step, plan_scan_step
from app.reasoning.semantic_navigation.models import SearchDirective


def directive_to_step_plan(
    directive: SearchDirective,
    *,
    scan_index: int,
    distance_m: float,
    step_config,
    min_confidence: float = 0.55,
    allow_forward: bool = False,
    max_turn_deg: float = 30.0,
) -> StepPlan:
    fallback = _legacy(scan_index, distance_m, step_config)
    if not isinstance(directive, SearchDirective):
        return fallback
    if directive.fallback_to_legacy or directive.confidence < min_confidence:
        return fallback
    heading = directive.preferred_heading_delta_deg
    if isinstance(heading, (int, float)) and abs(float(heading)) >= 1.0:
        clamped = max(-abs(max_turn_deg), min(abs(max_turn_deg), float(heading)))
        degrees = max(1, int(round(abs(clamped))))
        step = f"l{degrees}" if clamped > 0.0 else f"r{degrees}"
        return StepPlan(
            kind=PlanKind.SCAN, step=step,
            description_zh=f"语义选向：{describe_step(step)}；{directive.reason_zh}",
            phase="SEMANTIC_SEARCH",
        )
    if allow_forward and directive.allow_forward:
        estimate = float(getattr(step_config, "forward_estimate_m", 0.15))
        radius = float(getattr(step_config, "max_radius_m", 0.0))
        if radius <= 0.0 or distance_m + estimate < radius:
            return StepPlan(
                kind=PlanKind.SCAN, step="f",
                description_zh=f"语义短步：{describe_step('f')}；{directive.reason_zh}",
                phase="SEMANTIC_SEARCH",
            )
    return fallback


def _legacy(scan_index: int, distance_m: float, step_config) -> StepPlan:
    return plan_scan_step(
        scan_index,
        scan_turn_deg=float(getattr(step_config, "scan_turn_deg", 30.0)),
        scan_span=int(getattr(step_config, "scan_span", 3)),
        distance_m=distance_m,
        max_radius_m=float(getattr(step_config, "max_radius_m", 0.0)),
        forward_estimate_m=float(getattr(step_config, "forward_estimate_m", 0.15)),
    )
