"""Generate safe observation poses for confirmed visual targets."""

from __future__ import annotations

import math
from typing import Any

from .models import NavigationWaypoint, Pose2D


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
