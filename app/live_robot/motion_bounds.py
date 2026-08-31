"""Pure geometric gates for operator-scoped small-range motion."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class MotionBoundaryDecision:
    allowed: bool
    reason: str = ""
    predicted_position: tuple[float, float] | None = None


@dataclass(frozen=True)
class MotionObservationDecision:
    """Consistency result for one observed relative-motion primitive."""

    allowed: bool
    reason: str = ""
    code: str = ""
    distance_m: float = 0.0
    yaw_delta_deg: float = 0.0
    signed_progress_m: float = 0.0
    lateral_progress_m: float = 0.0
    motion_direction: str = "unknown"


def _signed_and_lateral_progress(
    before: tuple[float, float, float],
    after: tuple[float, float, float],
) -> tuple[float, float]:
    dx = after[0] - before[0]
    dy = after[1] - before[1]
    signed = dx * math.cos(before[2]) + dy * math.sin(before[2])
    lateral = -dx * math.sin(before[2]) + dy * math.cos(before[2])
    return signed, lateral


def evaluate_motion_observation(
    step: str,
    *,
    before: tuple[float, float, float],
    after: tuple[float, float, float],
    expected_forward_m: float,
    minimum_translation_m: float = 0.03,
    maximum_translation_factor: float = 2.0,
    maximum_forward_yaw_drift_deg: float = 20.0,
    maximum_lateral_drift_m: float = 0.10,
) -> MotionObservationDecision:
    """Reject missing motion and odometry jumps instead of accepting them.

    Translation primitives are verified in signed body coordinates: forward
    steps must produce positive ``signed_progress_m`` and backward recovery
    steps must produce negative ``signed_progress_m``.  Euclidean distance is
    still used as a discontinuity sanity check.
    """

    values = (*before, *after, expected_forward_m)
    if not all(math.isfinite(float(value)) for value in values):
        return MotionObservationDecision(
            False,
            "ODOM_INVALID: odometry contains a non-finite value",
            "ODOM_INVALID",
        )
    dx = after[0] - before[0]
    dy = after[1] - before[1]
    distance = math.hypot(dx, dy)
    signed_progress, lateral_progress = _signed_and_lateral_progress(before, after)
    yaw_delta = abs((after[2] - before[2] + math.pi) % (2.0 * math.pi) - math.pi)
    yaw_delta_deg = math.degrees(yaw_delta)

    if step.startswith("f") or step.startswith("b"):
        direction = "forward" if step.startswith("f") else "backward"
        expected = max(minimum_translation_m, float(expected_forward_m))
        minimum_required = max(minimum_translation_m, expected * 0.5)
        maximum = max(
            minimum_translation_m * 2.0,
            expected * maximum_translation_factor,
        )
        if distance > maximum:
            code = "ODOM_DISCONTINUITY"
            return MotionObservationDecision(
                False,
                (
                    f"{code}: observed {direction} displacement "
                    f"{distance:.3f}m exceeds plausible limit {maximum:.3f}m "
                    f"for requested {expected_forward_m:.3f}m"
                ),
                code,
                distance,
                yaw_delta_deg,
                signed_progress,
                lateral_progress,
                direction,
            )
        if distance < minimum_required:
            code = (
                "FORWARD_NOT_CONFIRMED"
                if direction == "forward"
                else "BACKWARD_NOT_CONFIRMED"
            )
            return MotionObservationDecision(
                False,
                (
                    f"{code}: motion RPC completed but odometry "
                    f"reported only {distance:.3f}m (minimum "
                    f"{minimum_required:.3f}m for requested "
                    f"{expected_forward_m:.3f}m)"
                ),
                code,
                distance,
                yaw_delta_deg,
                signed_progress,
                lateral_progress,
                direction,
            )
        if direction == "forward" and signed_progress <= 0.0:
            return MotionObservationDecision(
                False,
                (
                    "FORWARD_WRONG_DIRECTION: signed body-axis progress "
                    f"{signed_progress:.3f}m is not positive"
                ),
                "FORWARD_WRONG_DIRECTION",
                distance,
                yaw_delta_deg,
                signed_progress,
                lateral_progress,
                direction,
            )
        if direction == "backward" and signed_progress >= 0.0:
            return MotionObservationDecision(
                False,
                (
                    "BACKWARD_WRONG_DIRECTION: signed body-axis progress "
                    f"{signed_progress:.3f}m is not negative"
                ),
                "BACKWARD_WRONG_DIRECTION",
                distance,
                yaw_delta_deg,
                signed_progress,
                lateral_progress,
                direction,
            )
        if abs(lateral_progress) > maximum_lateral_drift_m:
            code = (
                "FORWARD_LATERAL_DRIFT"
                if direction == "forward"
                else "BACKWARD_LATERAL_DRIFT"
            )
            return MotionObservationDecision(
                False,
                (
                    f"{code}: lateral drift {abs(lateral_progress):.3f}m "
                    f"exceeds limit {maximum_lateral_drift_m:.3f}m"
                ),
                code,
                distance,
                yaw_delta_deg,
                signed_progress,
                lateral_progress,
                direction,
            )
        if yaw_delta_deg > maximum_forward_yaw_drift_deg:
            code = (
                "FORWARD_HEADING_DRIFT"
                if direction == "forward"
                else "BACKWARD_HEADING_DRIFT"
            )
            return MotionObservationDecision(
                False,
                (
                    f"{code}: {direction} step changed heading by "
                    f"{yaw_delta_deg:.1f}deg (limit "
                    f"{maximum_forward_yaw_drift_deg:.1f}deg)"
                ),
                code,
                distance,
                yaw_delta_deg,
                signed_progress,
                lateral_progress,
                direction,
            )
        return MotionObservationDecision(
            True,
            distance_m=distance,
            yaw_delta_deg=yaw_delta_deg,
            signed_progress_m=signed_progress,
            lateral_progress_m=lateral_progress,
            motion_direction=direction,
        )

    expected_turn_deg = abs(float(step[1:])) if len(step) > 1 else 0.0
    maximum_turn_deg = max(10.0, expected_turn_deg * 2.0 + 5.0)
    if yaw_delta_deg > maximum_turn_deg:
        return MotionObservationDecision(
            False,
            (
                "ODOM_DISCONTINUITY: observed yaw change "
                f"{yaw_delta_deg:.1f}deg exceeds plausible limit "
                f"{maximum_turn_deg:.1f}deg"
            ),
            "ODOM_DISCONTINUITY",
            distance,
            yaw_delta_deg,
        )
    if yaw_delta_deg <= 3.0:
        return MotionObservationDecision(
            False,
            (
                "TURN_NOT_CONFIRMED: motion RPC completed but odometry "
                f"reported only {yaw_delta_deg:.1f}deg"
            ),
            "TURN_NOT_CONFIRMED",
            distance,
            yaw_delta_deg,
        )
    return MotionObservationDecision(
        True, distance_m=distance, yaw_delta_deg=yaw_delta_deg
    )


def evaluate_lidar_motion_readiness(
    *,
    lidar_fresh: bool | None,
    front_clearance_m: float | None,
    minimum_clearance_m: float,
) -> MotionBoundaryDecision:
    """Fail closed on stale, missing or non-numeric forward safety data."""

    if lidar_fresh is not True:
        return MotionBoundaryDecision(False, "LiDAR clearance is stale or unavailable")
    if front_clearance_m is None or math.isnan(front_clearance_m):
        return MotionBoundaryDecision(False, "front clearance is unavailable")
    if front_clearance_m < minimum_clearance_m:
        return MotionBoundaryDecision(
            False,
            f"front clearance {front_clearance_m:.3f}m < "
            f"{minimum_clearance_m:.3f}m",
        )
    return MotionBoundaryDecision(True)


def evaluate_step_boundary(
    step: str,
    *,
    origin: tuple[float, float, float],
    current: tuple[float, float, float],
    max_radius_m: float,
    front_half_plane_only: bool,
    turn_only: bool,
    forward_distance_m: float,
    tolerance_m: float = 0.05,
) -> MotionBoundaryDecision:
    """Fail closed before a step leaves the operator-authorized region.

    The front half-plane is fixed to the robot's heading at authorization
    time, not its current heading after scan turns.
    """
    current_check = position_within_boundary(
        origin=origin,
        position=current[:2],
        max_radius_m=max_radius_m,
        front_half_plane_only=front_half_plane_only,
        tolerance_m=tolerance_m,
    )
    if not current_check.allowed:
        return current_check
    if not (step.startswith("f") or step.startswith("b")):
        return MotionBoundaryDecision(True, predicted_position=current[:2])
    if turn_only:
        return MotionBoundaryDecision(
            False,
            "translation rejected by operator-scoped turn-only gate",
            predicted_position=current[:2],
        )
    direction = -1.0 if step.startswith("b") else 1.0
    predicted = (
        current[0] + direction * max(0.0, forward_distance_m) * math.cos(current[2]),
        current[1] + direction * max(0.0, forward_distance_m) * math.sin(current[2]),
    )
    return position_within_boundary(
        origin=origin,
        position=predicted,
        max_radius_m=max_radius_m,
        front_half_plane_only=front_half_plane_only,
        tolerance_m=tolerance_m,
    )


def position_within_boundary(
    *,
    origin: tuple[float, float, float],
    position: tuple[float, float],
    max_radius_m: float,
    front_half_plane_only: bool,
    tolerance_m: float = 0.05,
) -> MotionBoundaryDecision:
    dx = position[0] - origin[0]
    dy = position[1] - origin[1]
    radius = math.hypot(dx, dy)
    if max_radius_m > 0.0 and radius > max_radius_m + tolerance_m:
        return MotionBoundaryDecision(
            False,
            f"position radius {radius:.3f}m exceeds {max_radius_m:.3f}m boundary",
            predicted_position=position,
        )
    if front_half_plane_only:
        forward_projection = dx * math.cos(origin[2]) + dy * math.sin(origin[2])
        if forward_projection < -tolerance_m:
            return MotionBoundaryDecision(
                False,
                "position enters the half-plane behind the authorization pose",
                predicted_position=position,
            )
    return MotionBoundaryDecision(True, predicted_position=position)


def evaluate_rotation_clearance(
    step: str,
    *,
    left_clearance_m: float | None,
    right_clearance_m: float | None,
    minimum_clearance_m: float,
    clearance_valid: bool | None = None,
) -> MotionBoundaryDecision:
    """Require both sides of the full-body rotation envelope to be clear."""
    if not (step.startswith("l") or step.startswith("r")):
        return MotionBoundaryDecision(True)
    if clearance_valid is not True:
        return MotionBoundaryDecision(
            False,
            "rotation clearance is not validated for the LiDAR near-field zone",
        )
    if minimum_clearance_m <= 0.0:
        return MotionBoundaryDecision(True)
    if left_clearance_m is None or right_clearance_m is None:
        return MotionBoundaryDecision(
            False, "rotation clearance unavailable on one or both sides"
        )
    if not (
        math.isfinite(left_clearance_m) or math.isinf(left_clearance_m)
    ) or not (
        math.isfinite(right_clearance_m) or math.isinf(right_clearance_m)
    ):
        return MotionBoundaryDecision(False, "rotation clearance is non-numeric")
    minimum = min(left_clearance_m, right_clearance_m)
    if minimum < minimum_clearance_m:
        return MotionBoundaryDecision(
            False,
            f"rotation envelope blocked: {minimum:.3f}m < {minimum_clearance_m:.3f}m",
        )
    return MotionBoundaryDecision(True)


def evaluate_dual_lidar_rotation_gate(
    *,
    fused_state: str | None,
    dual_lidar_enabled: bool,
    unknown_is_clear: bool,
    occupied_sources: list[str] | None = None,
) -> MotionBoundaryDecision:
    """Fail-closed dual-LiDAR rotation gate.

    Only applies when dual-lidar safety fusion is enabled. When disabled the
    gate passes and the existing formal/lease rotation gate continues to
    govern. When enabled:

    * OCCUPIED            -> reject (any occupied wins)
    * UNKNOWN             -> reject unless ``unknown_is_clear`` (never by default)
    * STALE / blind /
      self_occluded /
      unvalidated_geometry -> reject (fail-closed)
    * CLEAR               -> pass

    ``fused_state`` is the string value of
    ``go2w_lidar_preprocessor.lidar_evidence.EvidenceState``.
    """
    if not dual_lidar_enabled:
        return MotionBoundaryDecision(
            True, "dual-lidar safety fusion is disabled; existing gates govern"
        )
    sources = ", ".join(occupied_sources or [])
    if fused_state == "occupied":
        return MotionBoundaryDecision(
            False, f"dual-lidar rotation occupied: {sources}"
        )
    if fused_state == "clear":
        return MotionBoundaryDecision(True, "dual-lidar rotation clear")
    if fused_state == "unknown":
        if unknown_is_clear:
            return MotionBoundaryDecision(
                True, "dual-lidar unknown overridden as clear by operator"
            )
        return MotionBoundaryDecision(
            False, "dual-lidar rotation is unknown; unknown is not clear"
        )
    return MotionBoundaryDecision(
        False, f"dual-lidar rotation not clear ({fused_state or 'no evidence'})"
    )
