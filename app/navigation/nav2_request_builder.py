"""Build validated Nav2 requests from CLI/UI values."""

from __future__ import annotations

from datetime import UTC, datetime
from uuid import uuid4

from .nav2_config import Nav2Settings
from .nav2_models import Nav2Mode, Nav2Pose, Nav2Request, SafetyConfirmation


def make_request(*, mode: Nav2Mode | str, goal: Nav2Pose | None, settings: Nav2Settings,
                 start: Nav2Pose | None = None, use_current_start: bool = True,
                 allow_execute: bool = False, operator_confirmed: bool = False,
                 footprint_confirmed: bool = False, estop_confirmed: bool = False,
                 capability_gate_result: dict | None = None,
                 source: str = "manual_cli") -> Nav2Request:
    selected = Nav2Mode(mode)
    now = datetime.now(UTC)
    request = Nav2Request(
        request_id=f"nav2_{now:%Y%m%d_%H%M%S}_{uuid4().hex[:6]}",
        created_at=now.isoformat(), mode=selected, goal_pose=goal, start_pose=start,
        use_current_robot_pose_as_start=use_current_start, namespace=settings.namespace,
        map_frame=settings.map_frame, robot_base_frame=settings.base_frame,
        odom_frame=settings.odom_frame, goal_source=source,
        planner_id=settings.planner_id, controller_id=settings.controller_id,
        goal_checker_id=settings.goal_checker_id, behavior_tree=settings.behavior_tree,
        planning_timeout_sec=settings.planning_timeout_seconds,
        execution_timeout_sec=settings.execution_timeout_seconds,
        feedback_interval_sec=settings.feedback_interval_seconds,
        cmd_vel_topic=settings.cmd_vel_topic, global_plan_topic=settings.global_plan_topic,
        local_plan_topic=settings.local_plan_topic,
        allow_execute=allow_execute and settings.allow_execute,
        safety_confirmation=SafetyConfirmation(
            webui_confirmed=operator_confirmed,
            environment_allowed=settings.allow_execute,
            footprint_confirmed=footprint_confirmed and settings.footprint_confirmed,
            emergency_stop_confirmed=estop_confirmed and settings.emergency_stop_confirmed,
        ),
        capability_gate_result=dict(capability_gate_result or {}),
    )
    request.validate()
    return request
