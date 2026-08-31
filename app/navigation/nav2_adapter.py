"""Safe adapter from visual navigation plans to Nav2 map goals."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .models import NavigationPlan, Pose2D
from .nav2_models import Nav2Pose


@dataclass
class Nav2AdaptationResult:
    allowed: bool
    reason: str
    goal_pose: Nav2Pose | None = None

    def to_dict(self) -> dict:
        return {
            "allowed": self.allowed,
            "reason": self.reason,
            "goal_pose": self.goal_pose.to_dict() if self.goal_pose else None,
        }


def adapt_visual_plan_to_nav2_goal(
    plan: NavigationPlan,
    *,
    map_frame: str = "map",
    require_metric: bool = True,
    transform: dict | list | tuple | None = None,
) -> Nav2AdaptationResult:
    if require_metric and plan.scale_status != "metric":
        return Nav2AdaptationResult(False, "No metric scale: current video plan is visual/relative only")
    if plan.goal_pose is None:
        return Nav2AdaptationResult(False, "No goal pose in visual navigation plan")
    goal_pose = plan.goal_pose
    if goal_pose.frame_id != map_frame and transform is not None:
        goal_pose = transform_video_map_pose_to_map(goal_pose, transform, map_frame=map_frame)
    if goal_pose.frame_id != map_frame:
        return Nav2AdaptationResult(False, "No T_map_video_map transform or map-frame goal pose")
    if not goal_pose.provenance:
        return Nav2AdaptationResult(False, "Goal provenance is required before Nav2 handoff")
    pose = Nav2Pose(
        frame_id=map_frame,
        x=goal_pose.x,
        y=goal_pose.y,
        yaw_rad=goal_pose.yaw,
        source="video_navigation_metric_goal",
        provenance={
            **goal_pose.provenance,
            "source_plan_id": plan.plan_id,
            "scale_verified": True,
        },
    )
    return Nav2AdaptationResult(True, "Metric map-frame goal can be sent to Nav2", pose)


def transform_video_map_pose_to_map(
    pose: Pose2D,
    transform: dict | list | tuple,
    *,
    map_frame: str = "map",
) -> Pose2D:
    if isinstance(transform, dict):
        tx = float(transform.get("x", transform.get("tx", 0.0)))
        ty = float(transform.get("y", transform.get("ty", 0.0)))
        yaw = float(transform.get("yaw", transform.get("yaw_rad", 0.0)))
        source = transform.get("source", "T_map_video_map")
    else:
        tx, ty, yaw = (float(value) for value in transform[:3])
        source = "T_map_video_map"
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return Pose2D(
        x=tx + cos_yaw * pose.x - sin_yaw * pose.y,
        y=ty + sin_yaw * pose.x + cos_yaw * pose.y,
        yaw=_normalize_angle(pose.yaw + yaw),
        frame_id=map_frame,
        source="video_map_to_map_transform",
        scale_status=pose.scale_status,
        provenance={
            **pose.provenance,
            "coordinate_frame": map_frame,
            "transform_source": source,
            "T_map_video_map": {"x": tx, "y": ty, "yaw": yaw},
            "scale_verified": pose.scale_status == "metric",
        },
    )


def _normalize_angle(value: float) -> float:
    while value > math.pi:
        value -= math.tau
    while value < -math.pi:
        value += math.tau
    return value
