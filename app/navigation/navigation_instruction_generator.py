"""Generate precise human-readable instructions from any NavigationPlan."""

from __future__ import annotations

import math
from typing import Any

from .models import NavigationPlan, Pose2D


def generate_navigation_instructions(plan: NavigationPlan) -> list[dict[str, Any]]:
    unit = "m" if plan.scale_status == "metric" else "相对单位"
    distance_word = "前进" if plan.scale_status == "metric" else "沿当前路径前进"
    steps = [
        {
            "step": 1,
            "instruction": "从当前机器人位置出发。",
            "state": "planned",
            "waypoint_type": "start",
        }
    ]
    counter = 2
    for previous, current in zip(plan.path, plan.path[1:]):
        turn, turn_deg = _turn_instruction(previous, current)
        distance = previous.distance_to(current)
        if turn_deg >= 0.5 and distance >= 0.005:
            instruction = f"{turn} {distance_word} {distance:.2f} {unit}，停止并重新观察。"
        elif turn_deg >= 0.5:
            instruction = f"{turn}，停止并重新观察。"
        else:
            instruction = f"{distance_word} {distance:.2f} {unit}，停止并重新观察。"
        steps.append(
            {
                "step": counter,
                "instruction": instruction,
                "state": "planned",
                "distance": round(distance, 4),
                "scale_status": plan.scale_status,
            }
        )
        counter += 1
    if plan.navigation_strategy == "exploration":
        final = "到达探索 frontier 后重新观察，若仍未发现目标则选择下一个探索点。"
    elif plan.navigation_strategy == "last_known_reobserve":
        final = "到达目标最后已知观察区域后重新观察；若目标仍丢失则重新规划。"
    elif plan.navigation_strategy == "candidate_navigation":
        final = "到达疑似目标观察位姿后重新检测目标。"
    else:
        final = "到达目标前方观察位姿后停止并重新确认目标。"
    steps.append(
        {
            "step": counter,
            "instruction": final,
            "state": "reobserve",
            "waypoint_type": plan.waypoints[-1].waypoint_type if plan.waypoints else "goal",
        }
    )
    return steps


def _turn_instruction(previous: Pose2D, current: Pose2D) -> tuple[str, float]:
    heading = math.atan2(current.y - previous.y, current.x - previous.x)
    delta = _normalize_angle(heading - previous.yaw)
    degrees = abs(math.degrees(delta))
    if degrees >= 0.5:
        return (("左转" if delta > 0 else "右转") + f" {degrees:.0f}°", degrees)
    return "沿当前方向", 0.0


def _normalize_angle(value: float) -> float:
    while value > math.pi:
        value -= math.tau
    while value < -math.pi:
        value += math.tau
    return value
