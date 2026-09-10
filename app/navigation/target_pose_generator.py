"""Generate safe observation poses / approach viewpoints for confirmed targets.

计划书 §20：单点 observation pose 保留（离线视频链路仍在用），但真机接近
必须使用围绕目标的 standoff ring，并且每个位姿都要过 clearance / LOS /
route / path clearance / view quality 五道门。
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

from .clearance_map import ClearanceMap
from .models import NavigationWaypoint, Pose2D
from .search_policy_config import TargetApproachConfig
from .viewpoint_models import (
    VIEWPOINT_APPROACH_TARGET,
    TargetApproachContext,
    ViewpointCandidate,
    ViewpointRequest,
)
from .viewpoint_planner import ViewpointPlanner, ViewpointPlannerConfig


def generate_observation_goal(
    localization: dict[str, Any],
    observation_distance: float = 1.5,
) -> NavigationWaypoint | None:
    target_pose_payload = localization.get("target_pose")
    if not target_pose_payload:
        source = localization.get("source_frame_pose")
        if not source:
            return None
        pose = Pose2D.from_dict(source)
        waypoint_type = "candidate"
        label = "回到目标线索帧重新观察"
    else:
        target_pose = Pose2D.from_dict(target_pose_payload)
        source_pose = Pose2D.from_dict(localization.get("source_frame_pose") or target_pose_payload)
        dx = target_pose.x - source_pose.x
        dy = target_pose.y - source_pose.y
        yaw_to_target = math.atan2(dy, dx) if abs(dx) + abs(dy) > 1e-6 else target_pose.yaw
        distance = observation_distance if target_pose.scale_status == "metric" else min(observation_distance, 1.0)
        pose = Pose2D(
            x=target_pose.x - math.cos(yaw_to_target) * distance,
            y=target_pose.y - math.sin(yaw_to_target) * distance,
            yaw=yaw_to_target,
            frame_id=target_pose.frame_id,
            source="target_observation_pose",
            scale_status=target_pose.scale_status,
            provenance={
                **target_pose.provenance,
                "goal_type": "observation_pose",
                "observation_distance": distance,
            },
        )
        waypoint_type = "observation"
        label = "目标观察位姿"
    if localization.get("goal_type") == "candidate":
        waypoint_type = "candidate"
        label = "疑似目标观察位姿"
    return NavigationWaypoint(
        waypoint_id="goal_observation",
        pose=pose,
        source_frame_id=localization.get("source_frame_id"),
        semantic_label=label,
        waypoint_type=waypoint_type,
        confidence=float(localization.get("confidence", 0.45)),
        provenance=dict(localization.get("provenance") or {}),
    )


def generate_target_approach_viewpoints(
    context: TargetApproachContext,
    *,
    robot_pose: Sequence[float],
    clearance_map: ClearanceMap,
    policy: TargetApproachConfig | None = None,
    planner: ViewpointPlanner | None = None,
    traversal_failures: Mapping[str, int] | None = None,
    now: float | None = None,
) -> list[ViewpointCandidate]:
    """§20.1：metric 目标 → 围绕 map_xyz 采样 standoff ring 并排序。

    只有 ``METRIC_RGBD`` 且带 ``map_xyz`` 的目标才有 global 位姿可言；
    其余等级返回空列表，由 ``ApproachController`` 走 local visual servo（§20.2），
    绝不伪造 global pose。
    """
    policy = policy or TargetApproachConfig()
    if context.spatial_quality != "METRIC_RGBD" or context.map_xyz is None:
        return []
    planner = planner or ViewpointPlanner(
        ViewpointPlannerConfig(target_ring_angles_deg=tuple(policy.approach_ring_angles_deg))
    )
    request = ViewpointRequest(
        mode=VIEWPOINT_APPROACH_TARGET,
        subject_id=context.object_id or context.label,
        subject_position=(float(context.map_xyz[0]), float(context.map_xyz[1])),
        desired_distance_min_m=policy.minimum_standoff_m,
        desired_distance_max_m=policy.maximum_standoff_m,
        preferred_distance_m=policy.preferred_standoff_m,
        require_line_of_sight=policy.require_line_of_sight,
    )
    return planner.generate_and_rank(
        request,
        robot_pose=robot_pose,
        clearance_map=clearance_map,
        traversal_failures=traversal_failures,
        now=now,
    )
