"""Dispatch RobotTask instances to task-specific planning strategies."""

from __future__ import annotations

from app.planning import task_strategies
from app.schemas import (
    PredictiveSceneGraph,
    RobotTask,
    SceneAnalysisResult,
    SceneHypothesis,
    TaskPlan,
)


def plan_task(
    scene: SceneAnalysisResult,
    task: RobotTask,
    hypotheses: list[SceneHypothesis] | None = None,
    psg: PredictiveSceneGraph | None = None,
) -> TaskPlan:
    hypotheses = hypotheses or []

    if task.task_type == "find_object":
        return task_strategies.plan_find_object(scene, task, hypotheses, psg)
    if task.task_type == "count_objects":
        return task_strategies.plan_count_objects(scene, task)
    if task.task_type == "inspect_area":
        return task_strategies.plan_inspect_area(task, psg)
    if task.task_type == "check_door_state":
        return task_strategies.plan_check_door_state(task)
    if task.task_type in {"find_room", "navigate_to_location"}:
        return task_strategies.plan_navigate_to_location(task)
    return task_strategies.plan_general(task)
