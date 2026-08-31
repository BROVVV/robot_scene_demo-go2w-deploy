"""Pure helpers for stationary four-sector rotation-clearance cross-checks."""

from __future__ import annotations

from datetime import datetime, timedelta, timezone
import hashlib
import math
from pathlib import Path
from typing import Iterable

import numpy as np


SECTOR_BEARINGS_RAD = {
    "front": 0.0,
    "right": -math.pi / 2.0,
    "rear": math.pi,
    "left": math.pi / 2.0,
}


def wrapped_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def summarize_sector_frames(
    frames: Iterable[np.ndarray],
    *,
    sector: str,
    expected_distance_m: float,
    bearing_tolerance_deg: float = 15.0,
    distance_tolerance_m: float = 0.15,
    minimum_points_per_frame: int = 2,
) -> dict:
    """Summarize returns near a manually placed direction marker.

    Input points must already be the full-height, self-filtered marker cloud in
    ``base_link``.  This is only a direction/yaw cross-check and is never
    collision-authoritative.  The ROI is deliberately narrow in both bearing
    and range so an unrelated return elsewhere cannot satisfy a sector check.
    """

    if sector not in SECTOR_BEARINGS_RAD:
        raise ValueError(f"unsupported sector: {sector}")
    if not math.isfinite(expected_distance_m) or expected_distance_m <= 0.0:
        raise ValueError("expected target distance must be positive and finite")
    if not 0.0 < bearing_tolerance_deg < 45.0:
        raise ValueError("bearing tolerance must be between 0 and 45 degrees")
    if not math.isfinite(distance_tolerance_m) or distance_tolerance_m <= 0.0:
        raise ValueError("distance tolerance must be positive and finite")
    if minimum_points_per_frame < 1:
        raise ValueError("minimum points per frame must be positive")

    expected_bearing = SECTOR_BEARINGS_RAD[sector]
    bearing_tolerance = math.radians(bearing_tolerance_deg)
    counts: list[int] = []
    selected_ranges: list[float] = []
    selected_errors: list[float] = []
    for frame in frames:
        points = np.asarray(frame, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("each collision cloud must have shape Nx3")
        finite = np.isfinite(points).all(axis=1)
        radial = np.hypot(points[:, 0], points[:, 1])
        angles = np.arctan2(points[:, 1], points[:, 0])
        errors = np.arctan2(
            np.sin(angles - expected_bearing),
            np.cos(angles - expected_bearing),
        )
        mask = (
            finite
            & (np.abs(errors) <= bearing_tolerance)
            & (np.abs(radial - expected_distance_m) <= distance_tolerance_m)
        )
        counts.append(int(np.count_nonzero(mask)))
        selected_ranges.extend(float(value) for value in radial[mask])
        selected_errors.extend(float(value) for value in errors[mask])

    hit_frames = sum(count >= minimum_points_per_frame for count in counts)
    frame_count = len(counts)
    median_error = (
        float(np.median(np.asarray(selected_errors))) if selected_errors else None
    )
    return {
        "sector": sector,
        "expected_bearing_deg": round(math.degrees(expected_bearing), 6),
        "expected_distance_m": float(expected_distance_m),
        "bearing_tolerance_deg": float(bearing_tolerance_deg),
        "distance_tolerance_m": float(distance_tolerance_m),
        "minimum_points_per_frame": int(minimum_points_per_frame),
        "frame_count": frame_count,
        "hit_frames": hit_frames,
        "hit_fraction": hit_frames / frame_count if frame_count else 0.0,
        "median_points_per_frame": float(np.median(counts)) if counts else 0.0,
        "maximum_points_per_frame": max(counts, default=0),
        "selected_points": len(selected_ranges),
        "median_range_m": (
            float(np.median(np.asarray(selected_ranges)))
            if selected_ranges
            else None
        ),
        "median_bearing_deg": (
            round(math.degrees(wrapped_angle(expected_bearing + median_error)), 6)
            if median_error is not None
            else None
        ),
        "absolute_bearing_error_deg": (
            round(abs(math.degrees(median_error)), 6)
            if median_error is not None
            else None
        ),
    }


def compare_sector_capture(
    baseline: dict,
    target: dict,
    *,
    minimum_frames: int = 20,
    minimum_target_hit_fraction: float = 0.60,
    maximum_baseline_hit_fraction: float = 0.20,
    minimum_median_point_gain: float = 2.0,
    minimum_selected_point_gain: int = 60,
    minimum_range_shift_m: float = 0.04,
) -> dict:
    """Require a repeatable before/after target contrast.

    A cramped scene may already contain returns in a sector ROI.  Such a
    baseline is reported but is not automatically fatal.  A placed marker may
    either add returns or occlude a background surface while keeping a similar
    point count.  We therefore accept one of two independent contrasts:
    substantial point-count gain, or a repeatable median-range replacement of
    at least ``minimum_range_shift_m``.  An unchanged pre-existing chair still
    cannot pass the cross-check.
    """

    if baseline.get("sector") != target.get("sector"):
        raise ValueError("baseline and target sectors do not match")
    for key in (
        "expected_bearing_deg",
        "expected_distance_m",
        "bearing_tolerance_deg",
        "distance_tolerance_m",
        "minimum_points_per_frame",
    ):
        if baseline.get(key) != target.get(key):
            raise ValueError(f"baseline and target parameter mismatch: {key}")
    bearing_error = target.get("absolute_bearing_error_deg")
    target_range = target.get("median_range_m")
    baseline_range = baseline.get("median_range_m")
    baseline_median = float(baseline.get("median_points_per_frame", 0.0))
    required_gain = max(minimum_median_point_gain, baseline_median * 0.5)
    baseline_selected = int(baseline.get("selected_points", 0))
    required_selected_gain = max(
        int(minimum_selected_point_gain), int(math.ceil(baseline_selected * 0.5))
    )
    range_shift = (
        abs(float(target_range) - float(baseline_range))
        if target_range is not None and baseline_range is not None
        else None
    )
    point_count_contrast = float(
        target.get("median_points_per_frame", 0.0)
    ) >= baseline_median + required_gain
    selected_point_contrast = int(target.get("selected_points", 0)) >= (
        baseline_selected + required_selected_gain
    )
    range_contrast = range_shift is not None and range_shift >= minimum_range_shift_m
    checks = {
        "minimum_frames": int(baseline.get("frame_count", 0)) >= minimum_frames
        and int(target.get("frame_count", 0)) >= minimum_frames,
        "baseline_roi_clear": float(baseline.get("hit_fraction", 1.0))
        <= maximum_baseline_hit_fraction,
        "target_repeatably_detected": float(target.get("hit_fraction", 0.0))
        >= minimum_target_hit_fraction,
        "target_point_count_contrast": point_count_contrast,
        "target_selected_point_contrast": selected_point_contrast,
        "target_range_contrast": range_contrast,
        "target_contrast": (
            point_count_contrast and selected_point_contrast
        ) or range_contrast,
        "target_bearing_matches": bearing_error is not None
        and float(bearing_error) <= float(target["bearing_tolerance_deg"]),
        "target_range_matches": target_range is not None
        and abs(float(target_range) - float(target["expected_distance_m"]))
        <= float(target["distance_tolerance_m"]),
    }
    diagnostic_checks = {
        "baseline_roi_clear",
        "target_point_count_contrast",
        "target_selected_point_contrast",
        "target_range_contrast",
    }
    required_checks = tuple(key for key in checks if key not in diagnostic_checks)
    return {
        "sector": target["sector"],
        "passed": all(checks[key] for key in required_checks),
        "checks": checks,
        "baseline_roi_clear_is_required": False,
        "required_median_point_gain": required_gain,
        "required_selected_point_gain": required_selected_gain,
        "minimum_range_shift_m": minimum_range_shift_m,
        "observed_range_shift_m": range_shift,
        "baseline": baseline,
        "target": target,
    }


def matching_baseline_summary(baseline_capture: dict, target_summary: dict) -> dict:
    """Select the baseline ROI matching a target's sector and distance."""

    sector = target_summary.get("sector")
    expected_distance = float(target_summary.get("expected_distance_m", math.nan))
    direct = (baseline_capture.get("sector_summaries") or {}).get(sector)
    candidates = [direct] if direct else []
    candidates.extend((baseline_capture.get("sector_profiles") or {}).get(sector) or [])
    for candidate in candidates:
        if candidate and math.isclose(
            float(candidate.get("expected_distance_m", math.nan)),
            expected_distance,
            abs_tol=1e-6,
        ):
            return candidate
    raise ValueError(
        f"baseline has no {sector} ROI at expected distance {expected_distance:.3f}m"
    )


def compare_capture_context(
    baseline_capture: dict,
    target_capture: dict,
    *,
    maximum_age_seconds: int = 900,
    maximum_translation_m: float = 0.03,
    maximum_yaw_change_deg: float = 1.0,
) -> dict:
    """Ensure before/after captures describe the same stationary scene pose."""

    baseline_time = _aware_datetime(
        baseline_capture.get("captured_at"), "baseline captured_at"
    )
    target_time = _aware_datetime(
        target_capture.get("captured_at"), "target captured_at"
    )
    elapsed = (target_time - baseline_time).total_seconds()
    baseline_pose = (
        (baseline_capture.get("stationarity") or {}).get("origin_pose") or {}
    )
    target_pose = (
        (target_capture.get("stationarity") or {}).get("origin_pose") or {}
    )
    try:
        translation = math.hypot(
            float(target_pose["x"]) - float(baseline_pose["x"]),
            float(target_pose["y"]) - float(baseline_pose["y"]),
        )
        yaw_change = abs(
            wrapped_angle(float(target_pose["yaw"]) - float(baseline_pose["yaw"]))
        )
    except (KeyError, TypeError, ValueError):
        translation = math.inf
        yaw_change = math.inf
    checks = {
        "capture_time_separation_within_limit": abs(elapsed)
        <= maximum_age_seconds,
        "translation_within_limit": math.isfinite(translation)
        and translation <= maximum_translation_m,
        "yaw_change_within_limit": math.isfinite(yaw_change)
        and yaw_change <= math.radians(maximum_yaw_change_deg),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "elapsed_seconds": elapsed,
        "baseline_precedes_target": elapsed >= 0.0,
        "translation_m": translation,
        "yaw_change_rad": yaw_change,
        "maximum_age_seconds": maximum_age_seconds,
        "maximum_translation_m": maximum_translation_m,
        "maximum_yaw_change_deg": maximum_yaw_change_deg,
    }


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _aware_datetime(value: object, label: str) -> datetime:
    text = str(value or "").strip().replace("Z", "+00:00")
    try:
        parsed = datetime.fromisoformat(text)
    except ValueError as exc:
        raise ValueError(f"invalid {label}") from exc
    if parsed.tzinfo is None:
        raise ValueError(f"{label} must include a timezone")
    return parsed


def build_rotation_evidence(
    *,
    operator: str,
    configured_envelope_radius_m: float,
    physical_clearance_radius_m: float,
    swept_clearance_confirmed: bool,
    standing_posture_confirmed: bool,
    captures: dict[str, dict],
    capture_paths: dict[str, str | Path],
    validity_seconds: int = 600,
    now: datetime | None = None,
    hardware_binding: dict | None = None,
) -> dict:
    """Build a short-lived, initial-pose-bound evidence object.

    This is intentionally not a permanent calibration.  Human inspection
    covers the LiDAR blind annulus only at the captured pose, so evidence
    expires quickly and is invalid after translation.
    """

    operator = operator.strip()
    if not operator:
        raise ValueError("operator is required")
    if validity_seconds < 60 or validity_seconds > 900:
        raise ValueError("validity must be between 60 and 900 seconds")
    if set(captures) != {"baseline", *SECTOR_BEARINGS_RAD}:
        raise ValueError("baseline plus front/right/rear/left captures are required")
    if set(capture_paths) != set(captures):
        raise ValueError("every capture requires a source path")
    if not math.isfinite(physical_clearance_radius_m):
        raise ValueError("physical clearance radius must be finite")

    current = now or datetime.now(timezone.utc)
    if current.tzinfo is None:
        raise ValueError("evidence timestamp must be timezone-aware")
    capture_times = {
        role: _aware_datetime(capture.get("captured_at"), f"{role} captured_at")
        for role, capture in captures.items()
    }
    earliest_capture = min(capture_times.values())
    latest_capture = max(capture_times.values())
    if latest_capture > current + timedelta(seconds=5):
        raise ValueError("capture timestamp is in the future")
    if current - earliest_capture > timedelta(seconds=900):
        raise ValueError("physical cross-check capture window is older than 15 minutes")
    if latest_capture - earliest_capture > timedelta(seconds=900):
        raise ValueError("physical cross-check captures span more than 15 minutes")

    capture_checks = {}
    for role, capture in captures.items():
        capture_checks[role] = bool(
            capture.get("passed") is True
            and capture.get("robot_motion_commanded") is False
            and capture.get("posture") == "stationary_standing"
            and (capture.get("stationarity") or {}).get("passed") is True
        )

    baseline = captures["baseline"]
    sector_validations = {}
    for sector in SECTOR_BEARINGS_RAD:
        target_summary = (captures[sector].get("sector_summaries") or {})[sector]
        sector_validations[sector] = compare_sector_capture(
            matching_baseline_summary(baseline, target_summary),
            target_summary,
        )

    origin = (
        (captures["baseline"].get("stationarity") or {}).get("origin_pose") or {}
    )
    origins = [
        (captures[role].get("stationarity") or {}).get("origin_pose") or {}
        for role in ("baseline", *SECTOR_BEARINGS_RAD)
    ]
    origin_x = float(origin.get("x", math.nan))
    origin_y = float(origin.get("y", math.nan))
    cross_capture_translation = max(
        (
            math.hypot(
                float(candidate.get("x", math.nan)) - origin_x,
                float(candidate.get("y", math.nan)) - origin_y,
            )
            for candidate in origins
        ),
        default=math.inf,
    )
    origin_yaw = float(origin.get("yaw", math.nan))
    cross_capture_yaw_change = max(
        (
            abs(wrapped_angle(float(candidate.get("yaw", math.nan)) - origin_yaw))
            for candidate in origins
        ),
        default=math.inf,
    )
    origin_consistent = math.isfinite(cross_capture_translation) and (
        cross_capture_translation <= 0.03
    ) and math.isfinite(cross_capture_yaw_change) and (
        cross_capture_yaw_change <= math.radians(1.0)
    )

    all_sectors = all(item["passed"] for item in sector_validations.values())
    physical_sweep = bool(
        swept_clearance_confirmed
        and physical_clearance_radius_m + 1e-9 >= configured_envelope_radius_m
    )
    checks = {
        "horizontal_frame_yaw_validated": all(
            item["checks"]["target_bearing_matches"]
            for item in sector_validations.values()
        ),
        "near_field_blind_zone_mitigated": physical_sweep,
        "self_filter_regions_physically_validated": all_sectors,
        "full_360_sector_detection_validated": all_sectors,
        "standing_posture_validated": bool(
            standing_posture_confirmed
            and all(capture_checks.values())
            and origin_consistent
        ),
    }
    expires = min(
        current + timedelta(seconds=validity_seconds),
        earliest_capture + timedelta(seconds=900),
    )
    if expires - current < timedelta(seconds=60):
        raise ValueError("less than 60 seconds remain in the capture window")
    source_files = {
        role: {
            "path": str(capture_paths[role]),
            "sha256": sha256_file(capture_paths[role]),
        }
        for role in captures
    }
    return {
        "schema_version": "1.0",
        "validation_type": "go2w_rotation_clearance_physical_crosscheck",
        "validation_tool": "validate_rotation_clearance_physical_ros.py",
        "robot_model": "Unitree Go2-W",
        "robot_motion_commanded": False,
        "operator": operator,
        "posture": "stationary_standing",
        "validated_at": current.isoformat(),
        "expires_at": expires.isoformat(),
        "capture_window": {
            "earliest_at": earliest_capture.isoformat(),
            "latest_at": latest_capture.isoformat(),
            "span_seconds": (latest_capture - earliest_capture).total_seconds(),
            "maximum_age_seconds": 900,
        },
        "validated_rotation_envelope_radius_m": float(
            configured_envelope_radius_m
        ),
        "physical_inspection": {
            "swept_clearance_confirmed": bool(swept_clearance_confirmed),
            "measured_clearance_radius_m": float(physical_clearance_radius_m),
        },
        "authorization_scope": {
            "type": "initial_pose_in_place_rotation_only",
            "frame": "odom_wheel",
            "origin_pose": {
                "x": origin_x,
                "y": origin_y,
                "yaw": origin_yaw,
            },
            "maximum_translation_m": 0.03,
            "expires_at": expires.isoformat(),
        },
        "capture_checks": capture_checks,
        "maximum_cross_capture_translation_m": cross_capture_translation,
        "maximum_cross_capture_yaw_change_rad": cross_capture_yaw_change,
        "sector_validations": sector_validations,
        "capture_evidence": source_files,
        "checks": checks,
        "passed": all(checks.values()),
        **(
            {"hardware_binding": dict(hardware_binding)}
            if hardware_binding is not None
            else {}
        ),
    }
