"""Generate verification actions for scene hypotheses."""

from __future__ import annotations

from app.schemas import RobotTask


def plan_verification_action(candidate_location: str, task: RobotTask) -> str:
    if task.task_type == "check_door_state":
        return f"靠近{candidate_location}，观察门缝、门把手和门板角度以确认开关状态。"
    if task.task_type == "inspect_area":
        return f"将{candidate_location}加入巡查点，从当前位置按走廊顺序靠近并重新观察。"
    if task.task_type == "count_objects":
        return f"移动到能覆盖{candidate_location}的新视角，重新计数并和已有目标去重。"
    if task.task_type in {"find_room", "navigate_to_location"}:
        return f"沿当前可通行方向靠近{candidate_location}，确认门牌或空间标识。"
    return f"靠近{candidate_location}并重新观察，优先确认目标是否被遮挡或位于相关物体附近。"
