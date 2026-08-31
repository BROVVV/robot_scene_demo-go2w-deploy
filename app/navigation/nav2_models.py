"""Pure-Python Nav2 IPC models. This module intentionally never imports ROS."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any

SCHEMA_VERSION = "1.0"


class Nav2ValidationError(ValueError):
    pass


class Nav2Mode(str, Enum):
    DISABLED = "disabled"
    OFFLINE_PREVIEW = "offline_preview"
    VISUAL_PREVIEW = "visual_preview"
    PLAN_ONLY = "plan_only"
    EXECUTE = "execute"


class Nav2JobState(str, Enum):
    CREATED = "created"
    VALIDATING = "validating"
    UNAVAILABLE = "unavailable"
    READY = "ready"
    PLANNING = "planning"
    PLANNED = "planned"
    EXECUTING = "executing"
    CANCELING = "canceling"
    CANCELED = "canceled"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    TIMED_OUT = "timed_out"

    @property
    def terminal(self) -> bool:
        return self in {
            self.UNAVAILABLE, self.CANCELED, self.SUCCEEDED, self.FAILED, self.TIMED_OUT
        }


def yaw_to_quaternion(yaw: float) -> dict[str, float]:
    if not math.isfinite(yaw):
        raise Nav2ValidationError("yaw_rad 必须是有限数值")
    return {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)}


def quaternion_to_yaw(q: dict[str, float]) -> float:
    x, y, z, w = (float(q.get(k, 0.0)) for k in ("x", "y", "z", "w"))
    if not all(math.isfinite(v) for v in (x, y, z, w)):
        raise Nav2ValidationError("四元数必须是有限数值")
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-12:
        raise Nav2ValidationError("四元数不能为零")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


@dataclass
class Nav2Pose:
    frame_id: str = "map"
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw_rad: float = 0.0
    source: str = "manual_cli"
    provenance: dict[str, Any] = field(default_factory=dict)
    stamp_sec: int = 0

    def validate(self, *, map_frame: str | None = None) -> None:
        if not self.frame_id.strip():
            raise Nav2ValidationError("frame_id 不能为空")
        if not all(math.isfinite(float(v)) for v in (self.x, self.y, self.z, self.yaw_rad)):
            raise Nav2ValidationError("位姿坐标必须是有限数值")
        if map_frame and self.frame_id != map_frame and not self.provenance:
            raise Nav2ValidationError("非 map frame 位姿必须提供可转换来源证明")
        if self.source in {"image_pixel", "video_pixel", ""}:
            raise Nav2ValidationError("仅有图像像素坐标不能作为 Nav2 目标")

    @property
    def quaternion(self) -> dict[str, float]:
        return yaw_to_quaternion(self.yaw_rad)

    def to_dict(self) -> dict[str, Any]:
        self.validate()
        value = asdict(self)
        value["quaternion"] = self.quaternion
        return value

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "Nav2Pose":
        yaw = value.get("yaw_rad")
        if yaw is None and value.get("quaternion"):
            yaw = quaternion_to_yaw(value["quaternion"])
        pose = cls(
            frame_id=str(value.get("frame_id", "map")),
            x=float(value.get("x", 0)),
            y=float(value.get("y", 0)),
            z=float(value.get("z", 0)),
            yaw_rad=float(yaw or 0),
            source=str(value.get("source", "manual_cli")),
            provenance=dict(value.get("provenance") or {}),
            stamp_sec=int(value.get("stamp_sec", 0)),
        )
        pose.validate()
        return pose


@dataclass
class SafetyConfirmation:
    webui_confirmed: bool = False
    environment_allowed: bool = False
    footprint_confirmed: bool = False
    emergency_stop_confirmed: bool = False

    @property
    def complete(self) -> bool:
        return all(asdict(self).values())


@dataclass
class Nav2Request:
    request_id: str
    mode: Nav2Mode
    goal_pose: Nav2Pose | None
    created_at: str
    namespace: str = ""
    map_frame: str = "map"
    robot_base_frame: str = "base_link"
    odom_frame: str = "odom"
    goal_source: str = "manual_cli"
    start_pose: Nav2Pose | None = None
    waypoints: list[Nav2Pose] = field(default_factory=list)
    planner_id: str = "GridBased"
    controller_id: str = "FollowPath"
    goal_checker_id: str = "general_goal_checker"
    behavior_tree: str = ""
    use_current_robot_pose_as_start: bool = True
    planning_timeout_sec: float = 30.0
    execution_timeout_sec: float = 300.0
    feedback_interval_sec: float = 0.5
    capture_cmd_vel: bool = True
    cmd_vel_topic: str = "/cmd_vel"
    global_plan_topic: str = "/plan"
    local_plan_topic: str = "/local_plan"
    allow_execute: bool = False
    safety_confirmation: SafetyConfirmation = field(default_factory=SafetyConfirmation)
    capability_gate_result: dict[str, Any] = field(default_factory=dict)
    task_context: dict[str, Any] = field(default_factory=dict)
    schema_version: str = SCHEMA_VERSION

    def validate(self) -> None:
        if self.schema_version != SCHEMA_VERSION:
            raise Nav2ValidationError("不支持的 Nav2 schema_version")
        if not self.request_id or "/" in self.request_id or ".." in self.request_id:
            raise Nav2ValidationError("request_id 非法")
        if self.mode not in {Nav2Mode.DISABLED, Nav2Mode.VISUAL_PREVIEW} and self.goal_pose is None:
            raise Nav2ValidationError("启用 Nav2 时必须提供目标位姿")
        if self.goal_pose:
            self.goal_pose.validate(map_frame=self.map_frame)
        if not self.use_current_robot_pose_as_start and self.start_pose is None:
            raise Nav2ValidationError("未使用当前位姿时必须提供起点")
        if self.start_pose:
            self.start_pose.validate(map_frame=self.map_frame)
        if self.planning_timeout_sec <= 0 or self.execution_timeout_sec <= 0:
            raise Nav2ValidationError("超时必须大于 0")
        if self.mode == Nav2Mode.EXECUTE:
            if not self.allow_execute:
                raise Nav2ValidationError("NAV2_EXECUTION_NOT_ALLOWED: allow_execute=false")
            if not self.safety_confirmation.complete:
                raise Nav2ValidationError("NAV2_SAFETY_CONFIRMATION_MISSING: 执行安全门控未全部通过")
            from app.live_robot.navigation_gate import validate_execute_gate_payload

            try:
                validate_execute_gate_payload(self.capability_gate_result)
            except ValueError as exc:
                raise Nav2ValidationError(str(exc)) from exc

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["mode"] = self.mode.value
        value["goal_pose"] = self.goal_pose.to_dict() if self.goal_pose else None
        value["start_pose"] = self.start_pose.to_dict() if self.start_pose else None
        value["waypoints"] = [p.to_dict() for p in self.waypoints]
        return value

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "Nav2Request":
        data = dict(value)
        data["mode"] = Nav2Mode(data["mode"])
        data["goal_pose"] = Nav2Pose.from_dict(data["goal_pose"]) if data.get("goal_pose") else None
        data["start_pose"] = Nav2Pose.from_dict(data["start_pose"]) if data.get("start_pose") else None
        data["waypoints"] = [Nav2Pose.from_dict(p) for p in data.get("waypoints", [])]
        data["safety_confirmation"] = SafetyConfirmation(**data.get("safety_confirmation", {}))
        req = cls(**data)
        req.validate()
        return req


@dataclass
class Nav2Status:
    request_id: str
    state: Nav2JobState
    backend: str
    is_real_nav2_path: bool
    message_zh: str
    error_code: str | None = None
    error_type: str | None = None
    error_message: str | None = None
    updated_at: str | None = None
    finished_at: str | None = None
    current_pose: dict[str, Any] | None = None
    distance_remaining_m: float | None = None
    estimated_time_remaining_sec: float | None = None
    navigation_time_sec: float | None = None
    number_of_recoveries: int = 0
    path_length_m: float | None = None
    progress_ratio: float | None = None
    cmd_vel_samples: int = 0
    replan_count: int = 0
    worker_pid: int | None = None
    nav2_available: bool = False
    schema_version: str = SCHEMA_VERSION

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["state"] = self.state.value
        return value

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "Nav2Status":
        data = dict(value)
        data["state"] = Nav2JobState(data["state"])
        allowed = cls.__dataclass_fields__
        return cls(**{k: v for k, v in data.items() if k in allowed})
