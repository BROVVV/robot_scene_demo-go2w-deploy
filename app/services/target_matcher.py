"""Format target matching decisions for display."""

from __future__ import annotations

from app.schemas import SceneAnalysisResult


def format_target_decision(result: SceneAnalysisResult) -> str:
    decision = result.target_decision
    lines = [
        f"目标：{decision.target_text}",
        f"判断：{'在当前场景中' if decision.is_present else '未在当前场景中发现'}",
    ]

    if decision.is_present:
        matched_ids = "、".join(decision.matched_object_ids) or "无"
        lines.append(f"匹配物体：{matched_ids}")
        lines.append(f"原因：{decision.match_reason_zh}")
    else:
        lines.append("匹配物体：无")
        lines.append(f"原因：{decision.match_reason_zh}")
        if result.route_plan.summary_zh:
            lines.append(f"可能位置/探索依据：{result.route_plan.summary_zh}")

    lines.append(f"置信度：{decision.confidence:.2f}")
    return "\n".join(lines)


def print_target_decision(result: SceneAnalysisResult) -> None:
    print(format_target_decision(result))
