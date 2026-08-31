"""Validation for a short-lived, initial-pose-bound rotation safety lease."""

from __future__ import annotations

from datetime import datetime, timedelta, timezone
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

from app.live_robot.motion_bounds import MotionBoundaryDecision


REQUIRED_CHECKS = (
    "horizontal_frame_yaw_validated",
    "near_field_blind_zone_mitigated",
    "self_filter_regions_physically_validated",
    "full_360_sector_detection_validated",
    "standing_posture_validated",
)
SECTORS = ("front", "right", "rear", "left")
CAPTURE_ROLES = ("baseline", *SECTORS)


@dataclass(frozen=True)
class RotationClearanceSource:
    left_clearance_m: float | None
    right_clearance_m: float | None
    valid: bool
    source: str
    reason: str = ""


def _datetime(value: object, label: str) -> datetime:
    text = str(value or "").strip().replace("Z", "+00:00")
    try:
        parsed = datetime.fromisoformat(text)
    except ValueError as exc:
        raise ValueError(f"rotation lease has invalid {label}") from exc
    if parsed.tzinfo is None:
        raise ValueError(f"rotation lease {label} must include a timezone")
    return parsed


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _validate_hardware_binding(
    payload: dict, expected_binding: dict | None, label: str
) -> None:
    """Verify a lease's hardware binding matches the current rig.

    A lease that declares a hardware binding must satisfy every key the caller
    expects, and must not declare a binding key the caller cannot verify.
    When ``expected_binding`` is None no binding enforcement occurs (legacy
    leases without a binding remain valid), but a Stage-2 consumer always
    passes the current hardware binding for strict enforcement.
    """
    binding = payload.get("hardware_binding") or {}
    if not isinstance(binding, dict):
        raise ValueError(f"{label} hardware_binding must be a mapping")
    if expected_binding is None:
        return
    missing = [key for key in expected_binding if key not in binding]
    if missing:
        raise ValueError(
            f"{label} hardware_binding is missing: " + ", ".join(missing)
        )
    extra = [key for key in binding if key not in expected_binding]
    if extra:
        raise ValueError(
            f"{label} hardware_binding declares unverified keys: "
            + ", ".join(extra)
        )
    for key, expected in expected_binding.items():
        if binding.get(key) != expected:
            raise ValueError(
                f"{label} hardware_binding.{key} mismatch "
                f"(lease={binding.get(key)!r}, expected={expected!r})"
            )


def build_rotation_lease_binding(
    *,
    hardware_state_hash: str,
    geometry_hash: str,
    extrinsic_version: str,
    clock_tier: str,
    dual_lidar_evidence_hash: str | None = None,
) -> dict:
    """Build the hardware/geometry/evidence binding written into a lease."""
    binding = {
        "hardware_state_hash": hardware_state_hash,
        "geometry_hash": geometry_hash,
        "extrinsic_version": extrinsic_version,
        "clock_tier": clock_tier,
    }
    if dual_lidar_evidence_hash is not None:
        binding["dual_lidar_evidence_hash"] = dual_lidar_evidence_hash
    return binding


def load_rotation_lease(
    path: str | Path,
    *,
    required_envelope_radius_m: float,
    project_root: str | Path,
    now: datetime | None = None,
    expected_binding: dict | None = None,
) -> dict:
    """Load and cryptographically link a physical cross-check evidence set.

    ``expected_binding`` optionally binds the lease to the current hardware
    state/geometry/extrinsic/clock so a stale lease from a changed mount
    cannot be reused for motion.
    """

    evidence_path = Path(path)
    payload = json.loads(evidence_path.read_text(encoding="utf-8"))
    required = {
        "schema_version": "1.0",
        "validation_type": "go2w_rotation_clearance_physical_crosscheck",
        "validation_tool": "validate_rotation_clearance_physical_ros.py",
        "robot_model": "Unitree Go2-W",
        "robot_motion_commanded": False,
        "posture": "stationary_standing",
        "passed": True,
    }
    mismatched = [key for key, value in required.items() if payload.get(key) != value]
    if mismatched:
        raise ValueError("rotation lease contract mismatch: " + ", ".join(mismatched))
    _validate_hardware_binding(
        payload, expected_binding, "rotation lease"
    )
    if not str(payload.get("operator") or "").strip():
        raise ValueError("rotation lease operator is missing")
    radius = float(payload.get("validated_rotation_envelope_radius_m", math.nan))
    if not math.isfinite(radius) or radius + 1e-9 < required_envelope_radius_m:
        raise ValueError("rotation lease envelope is insufficient")
    checks = payload.get("checks") or {}
    missing = [key for key in REQUIRED_CHECKS if checks.get(key) is not True]
    if missing:
        raise ValueError("rotation lease checks are incomplete: " + ", ".join(missing))
    cross_translation = float(
        payload.get("maximum_cross_capture_translation_m", math.nan)
    )
    cross_yaw = float(payload.get("maximum_cross_capture_yaw_change_rad", math.nan))
    if not math.isfinite(cross_translation) or cross_translation > 0.03:
        raise ValueError("rotation lease cross-capture translation is excessive")
    if not math.isfinite(cross_yaw) or cross_yaw > math.radians(1.0):
        raise ValueError("rotation lease cross-capture yaw change is excessive")

    validated_at = _datetime(payload.get("validated_at"), "validated_at")
    expires_at = _datetime(payload.get("expires_at"), "expires_at")
    current = now or datetime.now(timezone.utc)
    if current.tzinfo is None:
        raise ValueError("current time must include a timezone")
    validity_seconds = (expires_at - validated_at).total_seconds()
    if validity_seconds < 60.0 or validity_seconds > 900.0:
        raise ValueError("rotation lease lifetime must be between 60 and 900 seconds")
    if validated_at > current or expires_at <= current:
        raise ValueError("rotation lease is not currently valid")
    capture_window = payload.get("capture_window") or {}
    earliest_capture = _datetime(
        capture_window.get("earliest_at"), "capture_window earliest_at"
    )
    latest_capture = _datetime(
        capture_window.get("latest_at"), "capture_window latest_at"
    )
    span_seconds = float(capture_window.get("span_seconds", math.nan))
    if (
        capture_window.get("maximum_age_seconds") != 900
        or not math.isfinite(span_seconds)
        or span_seconds < 0.0
        or span_seconds > 900.0
        or abs(span_seconds - (latest_capture - earliest_capture).total_seconds())
        > 1e-6
    ):
        raise ValueError("rotation lease capture window is invalid")
    if latest_capture > validated_at or expires_at > earliest_capture + timedelta(
        seconds=900
    ):
        raise ValueError("rotation lease exceeds its capture window")

    inspection = payload.get("physical_inspection") or {}
    measured_radius = float(inspection.get("measured_clearance_radius_m", math.nan))
    if (
        inspection.get("swept_clearance_confirmed") is not True
        or not math.isfinite(measured_radius)
        or measured_radius + 1e-9 < required_envelope_radius_m
    ):
        raise ValueError("rotation lease physical swept-clearance proof is insufficient")

    scope = payload.get("authorization_scope") or {}
    if scope.get("type") != "initial_pose_in_place_rotation_only":
        raise ValueError("rotation lease scope is unsupported")
    if scope.get("frame") != "odom_wheel":
        raise ValueError("rotation lease must be bound to odom_wheel")
    if _datetime(scope.get("expires_at"), "scope expires_at") != expires_at:
        raise ValueError("rotation lease scope expiry does not match evidence expiry")
    maximum_translation = float(scope.get("maximum_translation_m", math.nan))
    if not math.isfinite(maximum_translation) or not 0.0 < maximum_translation <= 0.03:
        raise ValueError("rotation lease translation bound is invalid")
    origin = scope.get("origin_pose") or {}
    if not all(math.isfinite(float(origin.get(key, math.nan))) for key in ("x", "y", "yaw")):
        raise ValueError("rotation lease origin pose is invalid")

    validations = payload.get("sector_validations") or {}
    if set(validations) != set(SECTORS) or not all(
        (validations.get(sector) or {}).get("passed") is True for sector in SECTORS
    ):
        raise ValueError("rotation lease four-sector validation is incomplete")
    capture_checks = payload.get("capture_checks") or {}
    if set(capture_checks) != set(CAPTURE_ROLES) or not all(
        capture_checks.get(role) is True for role in CAPTURE_ROLES
    ):
        raise ValueError("rotation lease capture checks are incomplete")
    sources = payload.get("capture_evidence") or {}
    if set(sources) != set(CAPTURE_ROLES):
        raise ValueError("rotation lease capture evidence is incomplete")
    root = Path(project_root)
    source_payloads = {}
    source_capture_times = {}
    for role in CAPTURE_ROLES:
        item = sources[role] or {}
        source_path = Path(str(item.get("path") or ""))
        if not source_path.is_absolute():
            source_path = root / source_path
        if not source_path.is_file() or _sha256(source_path) != item.get("sha256"):
            raise ValueError(f"rotation lease capture hash mismatch: {role}")
        try:
            source_payload = json.loads(source_path.read_text(encoding="utf-8"))
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            raise ValueError(f"rotation lease capture is unreadable: {role}") from exc
        if (
            source_payload.get("validation_type") != "go2w_rotation_sector_capture"
            or source_payload.get("capture_role") != role
            or source_payload.get("robot_model") != "Unitree Go2-W"
            or source_payload.get("posture") != "stationary_standing"
            or source_payload.get("robot_motion_commanded") is not False
            or source_payload.get("passed") is not True
            or (source_payload.get("stationarity") or {}).get("passed") is not True
            or source_payload.get("marker_cloud_topic")
            != "/go2w/lidar/obstacles"
            or source_payload.get("marker_cloud_purpose")
            != "controlled_direction_and_yaw_crosscheck_only"
            or source_payload.get(
                "marker_cloud_safety_authoritative_for_collision"
            ) is not False
            or source_payload.get("collision_cloud_topic")
            != "/go2w/lidar/collision_obstacles"
        ):
            raise ValueError(f"rotation lease capture contract mismatch: {role}")
        source_payloads[role] = source_payload
        source_capture_times[role] = _datetime(
            source_payload.get("captured_at"), f"{role} captured_at"
        )

    if (
        min(source_capture_times.values()) != earliest_capture
        or max(source_capture_times.values()) != latest_capture
    ):
        raise ValueError("rotation lease capture timestamps do not match source files")

    baseline_payload = source_payloads["baseline"]
    required_sector_checks = (
        "minimum_frames",
        "target_repeatably_detected",
        "target_contrast",
        "target_bearing_matches",
        "target_range_matches",
    )
    for sector in SECTORS:
        validation = validations[sector]
        if validation.get("sector") != sector or not all(
            (validation.get("checks") or {}).get(key) is True
            for key in required_sector_checks
        ):
            raise ValueError(f"rotation lease sector checks are incomplete: {sector}")
        target_summary = (
            source_payloads[sector].get("sector_summaries") or {}
        ).get(sector)
        if validation.get("target") != target_summary:
            raise ValueError(f"rotation lease target capture mismatch: {sector}")
        baseline_candidates = []
        direct = (baseline_payload.get("sector_summaries") or {}).get(sector)
        if direct:
            baseline_candidates.append(direct)
        baseline_candidates.extend(
            (baseline_payload.get("sector_profiles") or {}).get(sector) or []
        )
        if validation.get("baseline") not in baseline_candidates:
            raise ValueError(f"rotation lease baseline capture mismatch: {sector}")
    return payload


def evaluate_rotation_lease(
    payload: dict | None,
    *,
    current_pose: tuple[float, float, float],
    current_frame: str,
    now: datetime | None = None,
    expected_binding: dict | None = None,
) -> MotionBoundaryDecision:
    if payload is None:
        return MotionBoundaryDecision(False, "no pose-bound rotation lease")
    if expected_binding is not None:
        try:
            _validate_hardware_binding(
                payload, expected_binding, "rotation lease"
            )
        except ValueError as exc:
            return MotionBoundaryDecision(False, str(exc))
    expires_at = _datetime(payload.get("expires_at"), "expires_at")
    current = now or datetime.now(timezone.utc)
    if current >= expires_at:
        return MotionBoundaryDecision(False, "pose-bound rotation lease expired")
    scope = payload["authorization_scope"]
    if current_frame != scope["frame"]:
        return MotionBoundaryDecision(
            False,
            f"pose-bound rotation lease requires {scope['frame']}, got "
            f"{current_frame or 'missing frame'}",
        )
    origin = scope["origin_pose"]
    translation = math.hypot(
        current_pose[0] - float(origin["x"]),
        current_pose[1] - float(origin["y"]),
    )
    maximum = float(scope["maximum_translation_m"])
    if not math.isfinite(translation) or translation > maximum:
        return MotionBoundaryDecision(
            False,
            f"pose-bound rotation lease invalid after {translation:.3f}m translation",
        )
    return MotionBoundaryDecision(True)


def resolve_rotation_clearance_source(
    *,
    formal_left_clearance_m: float | None,
    formal_right_clearance_m: float | None,
    formal_valid: bool | None,
    lease: dict | None,
    current_pose: tuple[float, float, float],
    current_frame: str,
    diagnostic_left_clearance_m: float | None,
    diagnostic_right_clearance_m: float | None,
    diagnostic_age_seconds: float,
    lidar_fresh: bool | None,
    now: datetime | None = None,
    expected_binding: dict | None = None,
) -> RotationClearanceSource:
    """Select raw side data only when the physical lease is live and local."""

    if formal_valid is True:
        return RotationClearanceSource(
            formal_left_clearance_m,
            formal_right_clearance_m,
            True,
            "persistent_sensor_gate",
        )
    decision = evaluate_rotation_lease(
        lease,
        current_pose=current_pose,
        current_frame=current_frame,
        now=now,
        expected_binding=expected_binding,
    )
    if not decision.allowed:
        return RotationClearanceSource(
            formal_left_clearance_m,
            formal_right_clearance_m,
            False,
            "pose_bound_lease_rejected",
            decision.reason,
        )
    if (
        lidar_fresh is not True
        or not math.isfinite(diagnostic_age_seconds)
        or diagnostic_age_seconds > 0.3
        or diagnostic_left_clearance_m is None
        or diagnostic_right_clearance_m is None
    ):
        return RotationClearanceSource(
            formal_left_clearance_m,
            formal_right_clearance_m,
            False,
            "pose_bound_lease_rejected",
            "diagnostic side clearance is stale or unavailable",
        )
    return RotationClearanceSource(
        diagnostic_left_clearance_m,
        diagnostic_right_clearance_m,
        True,
        "pose_bound_physical_lease_plus_live_raw_lidar",
    )


def evaluate_rotation_lease_step(
    step: str, *, maximum_turn_deg: float = 30.0
) -> MotionBoundaryDecision:
    """Limit a pose-bound lease to one small, explicit turn command."""

    if not isinstance(step, str) or not step.startswith(("l", "r")):
        return MotionBoundaryDecision(
            False, "pose-bound rotation lease permits turn steps only"
        )
    try:
        degrees = float(step[1:])
    except ValueError:
        return MotionBoundaryDecision(False, "rotation step angle is invalid")
    if not math.isfinite(degrees) or degrees <= 0.0 or degrees > maximum_turn_deg:
        return MotionBoundaryDecision(
            False,
            f"pose-bound rotation step {degrees:g}deg exceeds (0, "
            f"{maximum_turn_deg:g}]deg",
        )
    return MotionBoundaryDecision(True)


def rotation_lease_stage2_scope_errors(
    *,
    mode: str,
    semantic_reasoning: bool,
    search_reasoner: str,
    search_reasoner_mode: str,
    turn_only: bool,
    front_half_plane_only: bool,
    max_motion_steps: int,
    max_radius_m: float,
    semantic_allow_forward: bool,
) -> list[str]:
    """Return every CLI-scope violation for the one-step Stage-2 lease."""

    errors = []
    if mode != "state_machine_search":
        errors.append("mode must be state_machine_search")
    if not semantic_reasoning:
        errors.append("--semantic-reasoning is required")
    if search_reasoner not in {"semantic_navigation", "hybrid"}:
        errors.append("reasoner must be semantic_navigation or hybrid")
    if search_reasoner_mode != "active":
        errors.append("reasoner mode must be active")
    if not turn_only:
        errors.append("--turn-only is required")
    if not front_half_plane_only:
        errors.append("--front-half-plane-only is required")
    if max_motion_steps != 1:
        errors.append("--max-motion-steps must equal 1")
    if not math.isfinite(max_radius_m) or not 0.0 < max_radius_m <= 1.5:
        errors.append("--max-radius must be in (0, 1.5]")
    if semantic_allow_forward:
        errors.append("--semantic-allow-forward is forbidden")
    return errors
