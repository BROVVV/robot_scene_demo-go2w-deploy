"""Platform-independent robot backend contract for high-level exploration.

SemanticNavigation, SceneGraph, SemanticMemory and the Exploration Planner must never
depend on Unitree / /go2w/motion / SportModeState / Pandar details.  They talk
to a ``RobotBackend``; the current Go2-W is one implementation
(``Go2WExperimentalBackend``) and a future production dog is another
(metric backend).  See docs/HIGH_LEVEL_AUTONOMOUS_SEMANTIC_EXPLORATION.md.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any, Protocol

from .models import ExplorationGoal


class PoseQuality(str, Enum):
    """Honest pose quality: the high level never fabricates metric pose."""

    UNAVAILABLE = "unavailable"
    RELATIVE = "relative"
    METRIC = "metric"


class NavigationStatus(str, Enum):
    ACCEPTED = "accepted"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"
    TIMEOUT = "timeout"
    OPERATOR_STOP = "operator_stop"
    BACKEND_UNAVAILABLE = "backend_unavailable"
    REJECTED = "rejected"


TERMINAL_NAVIGATION_STATUSES = frozenset(
    {
        NavigationStatus.SUCCEEDED,
        NavigationStatus.FAILED,
        NavigationStatus.CANCELLED,
        NavigationStatus.TIMEOUT,
        NavigationStatus.OPERATOR_STOP,
        NavigationStatus.BACKEND_UNAVAILABLE,
        NavigationStatus.REJECTED,
    }
)


@dataclass(frozen=True)
class RobotCapabilities:
    supports_global_pose: bool = False
    supports_metric_navigation: bool = False
    supports_relative_translation: bool = False
    supports_relative_rotation: bool = False
    supports_heading_control: bool = False
    supports_navigation_cancel: bool = False
    supports_navigation_feedback: bool = False
    supports_platform_obstacle_avoidance: bool = False
    # High-level motion vocabulary exposed to planners.  This is deliberately
    # more specific than supports_relative_translation/rotation: the current
    # Go2-W experiment can move forward, rotate, and execute short operator-
    # supervised backward recovery (BACKWARD_RECOVERY only).  Lateral motion
    # is still not exposed by default.
    allowed_motion_primitives: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["allowed_motion_primitives"] = list(self.allowed_motion_primitives)
        return value

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "RobotCapabilities":
        return cls(
            supports_global_pose=bool(value.get("supports_global_pose", False)),
            supports_metric_navigation=bool(value.get("supports_metric_navigation", False)),
            supports_relative_translation=bool(value.get("supports_relative_translation", False)),
            supports_relative_rotation=bool(value.get("supports_relative_rotation", False)),
            supports_heading_control=bool(value.get("supports_heading_control", False)),
            supports_navigation_cancel=bool(value.get("supports_navigation_cancel", False)),
            supports_navigation_feedback=bool(value.get("supports_navigation_feedback", False)),
            supports_platform_obstacle_avoidance=bool(
                value.get("supports_platform_obstacle_avoidance", False)
            ),
            allowed_motion_primitives=tuple(
                str(item) for item in (value.get("allowed_motion_primitives") or [])
            ),
        )


@dataclass(frozen=True)
class RobotPose:
    x: float
    y: float
    yaw: float
    frame_id: str = "odom"
    quality: PoseQuality = PoseQuality.UNAVAILABLE
    timestamp: float | None = None
    source: str = "unknown"

    def to_dict(self) -> dict[str, Any]:
        return {
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "frame_id": self.frame_id,
            "quality": self.quality.value,
            "timestamp": self.timestamp,
            "source": self.source,
        }


@dataclass
class NavigationHandle:
    """Opaque handle for one executed goal.

    ``result`` is filled by the backend when the goal settles; synchronous
    backends (e.g. the current Go2-W) fill it before ``execute_goal`` returns.
    """

    goal_id: str
    result: "NavigationResult | None" = None

    def settled(self) -> bool:
        return self.result is not None

    def status(self) -> NavigationStatus | None:
        return self.result.status if self.result is not None else None


@dataclass(frozen=True)
class NavigationResult:
    goal_id: str
    status: NavigationStatus
    message: str = ""
    requested_motion: dict[str, Any] = field(default_factory=dict)
    observed_motion: dict[str, Any] = field(default_factory=dict)
    elapsed_sec: float | None = None
    attempt: int = 1
    provenance: dict[str, Any] = field(default_factory=dict)

    @property
    def succeeded(self) -> bool:
        return self.status == NavigationStatus.SUCCEEDED

    @property
    def failed(self) -> bool:
        return self.status in {
            NavigationStatus.FAILED,
            NavigationStatus.TIMEOUT,
            NavigationStatus.BACKEND_UNAVAILABLE,
            NavigationStatus.REJECTED,
        }

    def to_dict(self) -> dict[str, Any]:
        return {
            "goal_id": self.goal_id,
            "status": self.status.value,
            "message": self.message,
            "requested_motion": self.requested_motion,
            "observed_motion": self.observed_motion,
            "elapsed_sec": self.elapsed_sec,
            "attempt": self.attempt,
            "provenance": self.provenance,
        }


@dataclass
class BackendHealth:
    ready: bool
    backend: str = "unknown"
    degraded: list[str] = field(default_factory=list)
    capabilities: RobotCapabilities = field(default_factory=RobotCapabilities)
    pose_quality: PoseQuality = PoseQuality.UNAVAILABLE
    details: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "ready": self.ready,
            "backend": self.backend,
            "degraded": self.degraded,
            "capabilities": self.capabilities.to_dict(),
            "pose_quality": self.pose_quality.value,
            "details": self.details,
        }


class RobotBackend(Protocol):
    """Minimal high-level motion interface shared by every robot platform."""

    def capabilities(self) -> RobotCapabilities: ...

    def get_pose(self) -> RobotPose | None: ...

    def execute_goal(self, goal: ExplorationGoal) -> NavigationHandle: ...

    def get_navigation_status(self, handle: NavigationHandle) -> NavigationResult: ...

    def cancel(self, handle: NavigationHandle | None = None) -> bool: ...

    def stop(self) -> None: ...

    def health(self) -> BackendHealth: ...


def navigation_result(
    goal_id: str,
    status: NavigationStatus | str,
    *,
    message: str = "",
    requested_motion: dict[str, Any] | None = None,
    observed_motion: dict[str, Any] | None = None,
    elapsed_sec: float | None = None,
    attempt: int = 1,
    provenance: dict[str, Any] | None = None,
) -> NavigationResult:
    return NavigationResult(
        goal_id=goal_id,
        status=NavigationStatus(status),
        message=message,
        requested_motion=requested_motion or {},
        observed_motion=observed_motion or {},
        elapsed_sec=elapsed_sec,
        attempt=attempt,
        provenance=provenance or {},
    )
