"""Breadcrumb-safe backward recovery data model.

The first version of autonomous reverse must not rely on a rear sensor.  The
only trustworthy corridor is the one the robot has just successfully walked
forward through, with fresh odometry, bounded yaw drift and no discontinuity.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any


@dataclass
class SafeMotionSegment:
    start_pose: tuple[float, float, float]
    end_pose: tuple[float, float, float]
    signed_distance_m: float
    heading_rad: float
    confirmed: bool = True
    created_monotonic_s: float = field(default_factory=time.monotonic)
    source_step: str = ""
    invalidated: bool = False

    def to_dict(self) -> dict[str, Any]:
        return {
            "start_pose": list(self.start_pose),
            "end_pose": list(self.end_pose),
            "signed_distance_m": round(self.signed_distance_m, 4),
            "heading_rad": round(self.heading_rad, 4),
            "confirmed": self.confirmed,
            "created_monotonic_s": round(self.created_monotonic_s, 4),
            "source_step": self.source_step,
            "invalidated": self.invalidated,
        }


@dataclass
class BackwardSafetyDecision:
    allowed: bool
    distance_m: float = 0.0
    reason: str = ""
    source: str = "none"


def create_forward_breadcrumb(
    before: tuple[float, float, float],
    after: tuple[float, float, float],
    *,
    signed_distance_m: float,
    source_step: str,
    yaw_drift_deg: float,
    max_yaw_drift_deg: float,
    odom_discontinuity: bool = False,
    emergency_stop: bool = False,
) -> SafeMotionSegment | None:
    """Return a confirmed forward breadcrumb, or None when not safe."""
    if odom_discontinuity or emergency_stop:
        return None
    if abs(yaw_drift_deg) > max_yaw_drift_deg:
        return None
    return SafeMotionSegment(
        start_pose=tuple(before),
        end_pose=tuple(after),
        signed_distance_m=float(signed_distance_m),
        heading_rad=float(before[2]),
        source_step=source_step,
    )


def evaluate_backward_safety(
    breadcrumb: SafeMotionSegment | None,
    *,
    current_pose: tuple[float, float, float],
    requested_distance_m: float,
    max_backward_step_m: float,
    min_backward_step_m: float,
    max_age_sec: float,
    heading_tolerance_deg: float,
    now: float | None = None,
) -> BackwardSafetyDecision:
    """Validate a backward recovery command against a breadcrumb corridor."""
    now = now if now is not None else time.monotonic()
    if breadcrumb is None or breadcrumb.invalidated or not breadcrumb.confirmed:
        return BackwardSafetyDecision(
            False, 0.0, "NO_VALID_REVERSE_CORRIDOR: no confirmed forward breadcrumb"
        )
    age = now - breadcrumb.created_monotonic_s
    if age > max_age_sec:
        return BackwardSafetyDecision(
            False,
            0.0,
            f"NO_VALID_REVERSE_CORRIDOR: breadcrumb expired ({age:.1f}s)",
        )
    yaw_error = abs(
        (current_pose[2] - breadcrumb.heading_rad + math.pi)
        % (2.0 * math.pi) - math.pi
    )
    yaw_error_deg = math.degrees(yaw_error)
    if yaw_error_deg > heading_tolerance_deg:
        return BackwardSafetyDecision(
            False,
            0.0,
            f"NO_VALID_REVERSE_CORRIDOR: heading error {yaw_error_deg:.1f}deg"
            f" > {heading_tolerance_deg:.1f}deg",
        )
    allowed_distance = min(
        float(requested_distance_m),
        breadcrumb.signed_distance_m,
        max_backward_step_m,
    )
    if allowed_distance < min_backward_step_m:
        return BackwardSafetyDecision(
            False,
            0.0,
            f"NO_VALID_REVERSE_CORRIDOR: allowed distance {allowed_distance:.3f}m"
            f" below {min_backward_step_m:.2f}m",
        )
    return BackwardSafetyDecision(
        True,
        allowed_distance,
        "breadcrumb_safe",
        "breadcrumb",
    )