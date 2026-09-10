"""Stuck classification (plan sections 11, 12, 13).

Before this module the explorer had exactly one explanation for "nothing new
happened": SEARCH_EXHAUSTED.  A dog with its camera against a wall therefore
declared the room searched.  ``detect_stuck`` separates the physical reasons
(sensor unusable / boxed in / camera against a surface / commanded but did not
move) from the genuine "no reachable frontier left" case, so recovery runs
first and exhaustion stays a last resort.

Fail-closed: unusable geometry is reported as SENSOR_DEGRADED, never as FREE.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Sequence

from .safety_models import LocalSafetySnapshot, PerceptionQuality
from .search_policy_config import StuckDetectionConfig


class StuckState(str, Enum):
    FREE = "FREE"
    CONSTRAINED = "CONSTRAINED"
    CAMERA_OCCLUDED = "CAMERA_OCCLUDED"
    FRONT_BLOCKED = "FRONT_BLOCKED"
    CORNERED = "CORNERED"
    NO_PROGRESS = "NO_PROGRESS"
    SENSOR_DEGRADED = "SENSOR_DEGRADED"
    RECOVERY_REQUIRED = "RECOVERY_REQUIRED"


# States that must be handled by the recovery sequence instead of being scored
# as "the area is searched".
RECOVERY_STUCK_STATES = (
    StuckState.CAMERA_OCCLUDED,
    StuckState.FRONT_BLOCKED,
    StuckState.CORNERED,
    StuckState.NO_PROGRESS,
    StuckState.RECOVERY_REQUIRED,
)


def requires_recovery(state: StuckState) -> bool:
    return state in RECOVERY_STUCK_STATES


@dataclass
class StuckDetectorInput:
    pose_history: Sequence[Any] = field(default_factory=tuple)
    command_history: Sequence[dict[str, Any]] = field(default_factory=tuple)
    result_history: Sequence[str] = field(default_factory=tuple)
    local_safety: LocalSafetySnapshot | None = None
    perception_quality: PerceptionQuality | None = None
    repeated_viewpoint_count: int = 0
    repeated_heading_count: int = 0
    navigation_failures: int = 0

    def commanded_progress_m(self, cycles: int) -> float:
        return sum(
            abs(float(item.get("distance_m") or 0.0))
            for item in list(self.command_history)[-cycles:]
        )

    def observed_progress_m(self, cycles: int) -> float:
        poses = [item for item in list(self.pose_history)[-(cycles + 1):] if item is not None]
        if len(poses) < 2:
            return 0.0
        return max(
            _distance(poses[0], poses[index]) for index in range(1, len(poses))
        )


def detect_stuck(
    history: StuckDetectorInput,
    safety: LocalSafetySnapshot | None = None,
    perception: PerceptionQuality | None = None,
    policy: StuckDetectionConfig | None = None,
) -> StuckState:
    """Classify why the robot is not making progress, worst cause first."""
    policy = policy or StuckDetectionConfig()
    safety = safety if safety is not None else history.local_safety
    perception = perception if perception is not None else history.perception_quality
    if not policy.enabled:
        return StuckState.FREE
    if safety is None or not safety.lidar_fresh or not safety.geometry_validated:
        return StuckState.SENSOR_DEGRADED

    forward_open = safety.forward_safe
    can_turn = (
        safety.safe_turn_left_deg >= policy.min_escape_turn_deg
        or safety.safe_turn_right_deg >= policy.min_escape_turn_deg
    )
    if not forward_open and not can_turn:
        return StuckState.CORNERED
    if perception is not None and perception.camera_blocked:
        return StuckState.CAMERA_OCCLUDED
    if not forward_open:
        return StuckState.FRONT_BLOCKED
    if history.navigation_failures >= policy.recovery_required_failures:
        return StuckState.RECOVERY_REQUIRED
    if _commanded_without_moving(history, policy):
        return StuckState.NO_PROGRESS
    if (
        history.repeated_viewpoint_count >= policy.repeated_viewpoint_limit
        or history.repeated_heading_count >= policy.repeated_heading_limit
    ):
        return StuckState.NO_PROGRESS
    clearance = safety.forward_swept_clearance_m
    if clearance is not None and clearance < policy.constrained_clearance_m:
        return StuckState.CONSTRAINED
    return StuckState.FREE


def _commanded_without_moving(
    history: StuckDetectorInput, policy: StuckDetectionConfig
) -> bool:
    cycles = max(1, policy.history_cycles)
    commanded = history.commanded_progress_m(cycles)
    if commanded < policy.min_commanded_progress_m:
        return False
    return history.observed_progress_m(cycles) <= policy.max_observed_progress_m


def _distance(a: Any, b: Any) -> float:
    ax, ay = _xy(a)
    bx, by = _xy(b)
    return ((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5


def _xy(pose: Any) -> tuple[float, float]:
    if isinstance(pose, (tuple, list)):
        return float(pose[0]), float(pose[1])
    return float(getattr(pose, "x", 0.0)), float(getattr(pose, "y", 0.0))
