"""Data models for the Go2-W manual WASD+QE web demo.

All models are plain dataclasses with no ROS or HTTP dependency so they can be
unit-tested in isolation and shared between the Web process and the pure
safety/controller modules.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass(frozen=True)
class SceneObject:
    """One main visible object row in the right-hand scene table."""

    name_zh: str
    name_en: str | None = None
    count: int | None = None
    position: str | None = None
    confidence: str = "medium"

    def to_dict(self) -> dict[str, Any]:
        return {
            "name_zh": self.name_zh,
            "name_en": self.name_en,
            "count": self.count,
            "position": self.position,
            "confidence": self.confidence,
        }


@dataclass
class SceneObjectState:
    """Thread-safe-in-use container for the latest scene analysis result."""

    objects: list[SceneObject] = field(default_factory=list)
    scene_summary: str | None = None
    frame_timestamp: float | None = None
    analysis_started_at: float | None = None
    analysis_finished_at: float | None = None
    model: str | None = None
    status: str = "idle"  # idle | running | ok | error
    error: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "objects": [obj.to_dict() for obj in self.objects],
            "scene_summary": self.scene_summary,
            "frame_timestamp": self.frame_timestamp,
            "analysis_started_at": self.analysis_started_at,
            "analysis_finished_at": self.analysis_finished_at,
            "model": self.model,
            "status": self.status,
            "error": self.error,
        }

    def as_json(self) -> dict[str, Any]:
        """Alias used by the API layer to avoid a separate mapping."""
        return self.to_dict()


@dataclass(frozen=True)
class CameraStatus:
    """Freshness snapshot of the camera stream as observed by the Web side."""

    available: bool
    fresh: bool
    age_seconds: float | None = None
    width: int | None = None
    height: int | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "available": self.available,
            "fresh": self.fresh,
            "age_seconds": self.age_seconds,
            "width": self.width,
            "height": self.height,
        }


@dataclass(frozen=True)
class SafetySnapshot:
    """Live safety gate inputs gathered from the ROS worker status."""

    robot_mode: int | None = None
    robot_error_code: int | None = None
    state_fresh: bool = False
    lease_alive: bool = False
    motion_action_available: bool = False
    lidar_fresh: bool | None = None
    front_clearance_m: float | None = None
    left_clearance_m: float | None = None
    right_clearance_m: float | None = None
    rotation_clearance_valid: bool | None = None
    odom_frame: str | None = None
    odom_pose: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class MotionResult:
    """Normalized result of one short pulse returned by the ROS worker."""

    success: bool
    direction: str | None = None
    error_code: str = "none"
    message: str = ""
    elapsed_sec: float = 0.0
    estimated_distance_m: float = 0.0
    actual_relative_yaw_deg: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class ManualDriveState:
    """Runtime state of the manual drive controller (see plan book §14)."""

    control_enabled: bool = False
    pressed_key: str | None = None
    command: str = "stop"
    motion_in_flight: bool = False
    last_heartbeat_monotonic: float = 0.0
    blocked_reason: str | None = None
    last_motion_result: dict[str, Any] | None = None
    status: str = "DISABLED"  # DISABLED READY MOVING STOPPING ESTOP BLOCKED ERROR

    def to_dict(self) -> dict[str, Any]:
        return {
            "control_enabled": self.control_enabled,
            "pressed_key": self.pressed_key,
            "command": self.command,
            "motion_in_flight": self.motion_in_flight,
            "last_heartbeat_monotonic": self.last_heartbeat_monotonic,
            "blocked_reason": self.blocked_reason,
            "last_motion_result": self.last_motion_result,
            "status": self.status,
        }
