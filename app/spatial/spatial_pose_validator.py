"""Independent wheel-vs-LIO pose validation for mapping and topology."""

from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass, field
from typing import Any

from app.spatial.models import SpatialPose


HEALTHY = "HEALTHY"
DEGRADED = "DEGRADED"
RECOVERING = "RECOVERING"
REJECTED = "REJECTED"


@dataclass
class MotionEvidence:
    command_type: str = ""
    requested_turn_deg: float = 0.0
    requested_forward_m: float = 0.0
    wheel_delta_xy_m: float | None = None
    wheel_delta_yaw_deg: float | None = None
    wheel_linear_speed_mps: float | None = None
    wheel_yaw_rate_dps: float | None = None
    motion_completed_at: float | None = None

    @property
    def rotation_only(self) -> bool:
        return abs(float(self.requested_turn_deg)) >= 5.0 and abs(float(self.requested_forward_m)) <= 0.03

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class PoseValidationResult:
    accepted: bool
    health: str
    reason_code: str
    raw_pose: SpatialPose | None = None
    accepted_pose: SpatialPose | None = None
    pslam_delta_xy_m: float | None = None
    pslam_delta_yaw_deg: float | None = None
    wheel_delta_xy_m: float | None = None
    raw_pose_age_sec: float | None = None
    details: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["raw_pose"] = self.raw_pose.to_dict() if self.raw_pose else None
        value["accepted_pose"] = self.accepted_pose.to_dict() if self.accepted_pose else None
        return value


class SpatialPoseValidator:
    """Reject implausible LIO poses while retaining the last known good pose."""

    def __init__(
        self,
        *,
        rotation_wheel_translation_max_m: float = 0.08,
        rotation_pslam_translation_max_m: float = 0.20,
        max_pslam_speed_mps: float = 2.0,
        max_pslam_yaw_rate_dps: float = 240.0,
        stale_pose_after_s: float = 2.0,
        recovery_valid_samples: int = 4,
    ) -> None:
        self.rotation_wheel_translation_max_m = float(rotation_wheel_translation_max_m)
        self.rotation_pslam_translation_max_m = float(rotation_pslam_translation_max_m)
        self.max_pslam_speed_mps = float(max_pslam_speed_mps)
        self.max_pslam_yaw_rate_dps = float(max_pslam_yaw_rate_dps)
        self.stale_pose_after_s = max(0.1, float(stale_pose_after_s))
        self.recovery_valid_samples = max(1, int(recovery_valid_samples))
        self._previous_raw: SpatialPose | None = None
        self._previous_time: float | None = None
        self._last_good: SpatialPose | None = None
        self._last_good_time: float | None = None
        self._invalid_streak = 0
        self._valid_streak = 0

    @property
    def last_good_pose(self) -> SpatialPose | None:
        return self._last_good

    def validate(
        self,
        raw_pose: SpatialPose | dict[str, Any] | None,
        motion: MotionEvidence | None = None,
        *,
        timestamp: float | None = None,
    ) -> PoseValidationResult:
        now = time.monotonic() if timestamp is None else float(timestamp)
        pose = _coerce_pose(raw_pose)
        if pose is None:
            self._invalid_streak += 1
            self._valid_streak = 0
            return self._reject("POSE_INVALID", "raw spatial pose missing or non-finite")

        previous = self._previous_raw
        elapsed = None
        delta_xy = None
        delta_yaw = None
        if previous is not None and self._previous_time is not None:
            elapsed = now - self._previous_time
            if elapsed <= 0.0:
                self._remember_raw(pose, now)
                return self._reject("POSE_TIMESTAMP_NONMONOTONIC", "spatial pose timestamp moved backward")
            delta_xy = math.hypot(pose.x - previous.x, pose.y - previous.y)
            delta_yaw = abs(math.degrees(_wrap_pi(pose.yaw - previous.yaw)))
            if delta_xy / max(elapsed, 1e-3) > self.max_pslam_speed_mps:
                self._remember_raw(pose, now)
                return self._reject(
                    "PSLAM_POSE_JUMP", "LIO translation exceeds independent speed bound",
                    raw_pose=pose, pslam_delta_xy_m=delta_xy,
                    pslam_delta_yaw_deg=delta_yaw,
                    wheel_delta_xy_m=_wheel_delta(motion),
                )
            if delta_yaw / max(elapsed, 1e-3) > self.max_pslam_yaw_rate_dps:
                self._remember_raw(pose, now)
                return self._reject(
                    "PSLAM_YAW_JUMP", "LIO yaw exceeds independent rate bound",
                    raw_pose=pose, pslam_delta_xy_m=delta_xy,
                    pslam_delta_yaw_deg=delta_yaw,
                    wheel_delta_xy_m=_wheel_delta(motion),
                )

        wheel_delta = _wheel_delta(motion)
        if (
            motion is not None
            and motion.rotation_only
            and wheel_delta is not None
            and wheel_delta <= self.rotation_wheel_translation_max_m
            and delta_xy is not None
            and delta_xy > self.rotation_pslam_translation_max_m
        ):
            self._remember_raw(pose, now)
            return self._reject(
                "LIO_DRIFT_DURING_ROTATION",
                "rotation-only wheel evidence conflicts with LIO translation",
                raw_pose=pose, pslam_delta_xy_m=delta_xy,
                pslam_delta_yaw_deg=delta_yaw, wheel_delta_xy_m=wheel_delta,
            )

        self._remember_raw(pose, now)
        if self._last_good is None:
            self._last_good = pose
            self._last_good_time = now
            self._invalid_streak = 0
            self._valid_streak = self.recovery_valid_samples
            return self._accept(pose, delta_xy, delta_yaw, wheel_delta, HEALTHY, "INITIAL_VALID_POSE")

        if self._invalid_streak:
            self._valid_streak += 1
            if self._valid_streak < self.recovery_valid_samples:
                return PoseValidationResult(
                    accepted=False, health=RECOVERING,
                    reason_code="LIO_RECOVERY_IN_PROGRESS", raw_pose=pose,
                    accepted_pose=self._last_good,
                    pslam_delta_xy_m=delta_xy,
                    pslam_delta_yaw_deg=delta_yaw,
                    wheel_delta_xy_m=wheel_delta,
                    details={"valid_streak": self._valid_streak,
                             "required": self.recovery_valid_samples},
                )
        self._invalid_streak = 0
        self._valid_streak = max(self._valid_streak, self.recovery_valid_samples)
        self._last_good = pose
        self._last_good_time = now
        return self._accept(pose, delta_xy, delta_yaw, wheel_delta, HEALTHY, "VALID_POSE")

    def _remember_raw(self, pose: SpatialPose, timestamp: float) -> None:
        self._previous_raw = pose
        self._previous_time = timestamp

    def _reject(self, reason: str, detail: str, **fields: Any) -> PoseValidationResult:
        self._valid_streak = 0
        self._invalid_streak = max(1, self._invalid_streak)
        return PoseValidationResult(
            accepted=False,
            health=DEGRADED if self._last_good is not None else REJECTED,
            reason_code=reason,
            accepted_pose=self._last_good,
            details={"detail": detail, "invalid_streak": self._invalid_streak},
            **fields,
        )

    @staticmethod
    def _accept(pose: SpatialPose, delta_xy: float | None, delta_yaw: float | None,
                wheel_delta: float | None, health: str, reason: str) -> PoseValidationResult:
        return PoseValidationResult(
            accepted=True, health=health, reason_code=reason,
            raw_pose=pose, accepted_pose=pose,
            pslam_delta_xy_m=delta_xy,
            pslam_delta_yaw_deg=delta_yaw,
            wheel_delta_xy_m=wheel_delta,
        )


def _wheel_delta(motion: MotionEvidence | None) -> float | None:
    if motion is None or motion.wheel_delta_xy_m is None:
        return None
    try:
        value = float(motion.wheel_delta_xy_m)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) and value >= 0.0 else None


def _coerce_pose(value: SpatialPose | dict[str, Any] | None) -> SpatialPose | None:
    if value is None:
        return None
    try:
        pose = value if isinstance(value, SpatialPose) else SpatialPose.from_dict(value)
        if not all(math.isfinite(float(part)) for part in (pose.x, pose.y, pose.yaw)):
            return None
        return pose
    except (TypeError, ValueError, AttributeError):
        return None


def _wrap_pi(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi
