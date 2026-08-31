"""Format route plans for display."""

from __future__ import annotations

from app.schemas import RouteStep, SceneAnalysisResult


ROUTE_TYPE_LABELS = {
    "approach_visible_target": "接近可见目标",
    "explore_likely_location": "探索可能位置",
}


def format_route_plan(result: SceneAnalysisResult) -> str:
    route_plan = result.route_plan
    route_type = ROUTE_TYPE_LABELS.get(route_plan.route_type, route_plan.route_type)

    lines = [
        "路线规划：",
        f"路线类型：{route_type}",
        f"路线摘要：{route_plan.summary_zh}",
    ]

    if route_plan.steps:
        for step in sorted(route_plan.steps, key=lambda item: item.step_id):
            lines.append(f"{step.step_id}. {_format_route_step(step)}")
    else:
        lines.append("暂无路线步骤")

    if route_plan.safety_notes_zh:
        lines.append("安全说明：")
        lines.extend(f"- {note}" for note in route_plan.safety_notes_zh)

    return "\n".join(lines)


def print_route_plan(result: SceneAnalysisResult) -> None:
    print(format_route_plan(result))


def _format_route_step(step: RouteStep) -> str:
    if step.action == "move_forward" and step.distance_m is not None:
        return f"向前走 {_format_number(step.distance_m)} 米"
    if step.action == "move_backward" and step.distance_m is not None:
        return f"向后退 {_format_number(step.distance_m)} 米"
    if step.action == "turn_left" and step.turn_angle_deg is not None:
        return f"左转 {_format_number(step.turn_angle_deg)} 度"
    if step.action == "turn_right" and step.turn_angle_deg is not None:
        return f"右转 {_format_number(step.turn_angle_deg)} 度"
    if step.action == "stop":
        return step.description_zh or "停止"

    return step.description_zh


def _format_number(value: float) -> str:
    return f"{value:g}"
