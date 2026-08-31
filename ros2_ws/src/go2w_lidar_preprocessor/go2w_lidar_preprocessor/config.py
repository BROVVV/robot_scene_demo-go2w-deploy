from __future__ import annotations

import json
import math
from pathlib import Path

import yaml
from go2w_description.description_config import load_official_reference

from .preprocess_core import PreprocessParameters


_ROTATION_VALIDATION_CHECKS = (
    "horizontal_frame_yaw_validated",
    "near_field_blind_zone_mitigated",
    "self_filter_regions_physically_validated",
    "full_360_sector_detection_validated",
    "standing_posture_validated",
)


def load_safety_ready_config(lidar_path: str, geometry_path: str):
    lidar = yaml.safe_load(Path(lidar_path).read_text(encoding="utf-8")) or {}
    geometry = yaml.safe_load(Path(geometry_path).read_text(encoding="utf-8")) or {}
    if lidar.get("validation_status") != "validated" or not lidar.get("confirmed"):
        raise ValueError("LiDAR preprocessing thresholds are not measured and confirmed")
    if lidar.get("robot_model") != "Unitree Go2-W":
        raise ValueError("LiDAR thresholds are not identified as Unitree Go2-W")

    if geometry.get("reference_status") == "manufacturer_published":
        reference = load_official_reference(geometry_path)
        envelope = (reference.get("dimensions") or {}).get("standing_envelope_m") or {}
        footprint_length = float(envelope.get("length", 0.0))
        footprint_width = float(envelope.get("width", 0.0))
        standing_height = float(envelope.get("height", 0.0))
    else:
        if geometry.get("measurement_status") != "measured" or not geometry.get(
            "confirmed"
        ):
            raise ValueError("Go2-W physical geometry is not measured and confirmed")
        measurements = geometry.get("measurements") or {}

        def measured(key):
            value = (measurements.get(key) or {}).get("value")
            if value is None:
                raise ValueError(f"missing physical measurement: {key}")
            return float(value)

        footprint_length = measured("wheel_outer_envelope_length_m")
        footprint_width = measured("wheel_outer_envelope_width_m")
        standing_height = measured("stationary_standing_height_m")

    required = (
        (lidar.get("height_m") or {}).get("minimum"),
        (lidar.get("height_m") or {}).get("maximum"),
        lidar.get("ground_separation_height_m"),
        (lidar.get("collision_height_m") or {}).get("maximum"),
        lidar.get("self_filter_margin_m"),
        lidar.get("front_corridor_half_width_m"),
        lidar.get("rotation_envelope_radius_m"),
    )
    if any(value is None for value in required):
        raise ValueError("one or more LiDAR geometry thresholds are unmeasured")
    numeric = tuple(float(value) for value in required)
    minimum_range = float(lidar["range_m"]["minimum"])
    maximum_range = float(lidar["range_m"]["maximum"])
    all_numeric = numeric + (
        minimum_range,
        maximum_range,
        footprint_length,
        footprint_width,
        standing_height,
    )
    if not all(math.isfinite(value) for value in all_numeric):
        raise ValueError("LiDAR preprocessing parameters must be finite")
    if (
        minimum_range <= 0.0
        or maximum_range <= minimum_range
        or numeric[1] <= numeric[0]
        or footprint_length <= 0.0
        or footprint_width <= 0.0
        or numeric[3] <= numeric[2]
        or numeric[3] > numeric[1]
        or standing_height <= 0.0
        or numeric[4] < 0.0
        or numeric[5] <= footprint_width / 2.0
        or numeric[6]
        < math.hypot(footprint_length / 2.0, footprint_width / 2.0)
    ):
        raise ValueError("LiDAR preprocessing geometry is unsafe or inconsistent")
    expected_collision_maximum = numeric[2] + standing_height
    if not math.isclose(numeric[3], expected_collision_maximum, abs_tol=1e-6):
        raise ValueError(
            "collision height must equal ground separation height plus "
            "the standing robot height"
        )
    self_regions: list[tuple[float, float, float, float, float, float]] = []
    for region in lidar.get("self_regions") or []:
        if not isinstance(region, dict):
            raise ValueError("self region must be a mapping")
        bounds = tuple(
            float(region.get(key))
            for key in ("x_min", "x_max", "y_min", "y_max", "z_min", "z_max")
        )
        if len(bounds) != 6 or not all(math.isfinite(value) for value in bounds):
            raise ValueError("self region bounds must be finite")
        if not (
            bounds[0] < bounds[1]
            and bounds[2] < bounds[3]
            and bounds[4] < bounds[5]
        ):
            raise ValueError("self region bounds must be ordered min < max")
        self_regions.append(bounds)
    parameters = PreprocessParameters(
        minimum_range=minimum_range,
        maximum_range=maximum_range,
        minimum_height=numeric[0],
        maximum_height=numeric[1],
        ground_height=numeric[2],
        collision_maximum_height=numeric[3],
        self_half_length=footprint_length / 2.0,
        self_half_width=footprint_width / 2.0,
        self_filter_margin=numeric[4],
        front_corridor_half_width=numeric[5],
        rotation_envelope_radius=numeric[6],
        self_regions=tuple(self_regions),
    )
    _validate_rotation_clearance_claim(
        lidar, parameters, lidar_path=Path(lidar_path)
    )
    return lidar, parameters


def _validate_rotation_clearance_claim(
    lidar: dict,
    parameters: PreprocessParameters,
    *,
    lidar_path: Path,
) -> None:
    """Prevent a bare YAML boolean from authorizing rotation clearance."""

    claim = lidar.get("rotation_clearance_validation") or {}
    if not bool(claim.get("valid", False)):
        return
    if claim.get("validation_method") != "physical_360_clearance_with_lidar_crosscheck":
        raise ValueError("rotation clearance validation method is missing or unsupported")
    for key in ("validated_at", "operator", "posture"):
        if not str(claim.get(key) or "").strip():
            raise ValueError(f"rotation clearance validation requires {key}")
    if claim.get("posture") != "stationary_standing":
        raise ValueError("rotation clearance validation posture is unsupported")
    radius = claim.get("validated_rotation_envelope_radius_m")
    if radius is None or not math.isfinite(float(radius)):
        raise ValueError("rotation clearance validation requires a finite envelope radius")
    if float(radius) + 1e-9 < parameters.rotation_envelope_radius:
        raise ValueError("validated rotation envelope is smaller than configured envelope")
    evidence = claim.get("evidence_paths") or []
    if not isinstance(evidence, list) or not evidence or not all(
        isinstance(path, str) and path.strip() for path in evidence
    ):
        raise ValueError("rotation clearance validation requires evidence paths")
    checks = claim.get("checks") or {}
    missing = [key for key in _ROTATION_VALIDATION_CHECKS if checks.get(key) is not True]
    if missing:
        raise ValueError(
            "rotation clearance validation checks are incomplete: " + ", ".join(missing)
        )
    project_root = lidar_path.resolve().parents[2]
    for evidence_path in evidence:
        path = Path(evidence_path)
        if not path.is_absolute():
            path = project_root / path
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            raise ValueError(
                f"rotation clearance evidence is unreadable: {path}"
            ) from exc
        _validate_rotation_evidence_payload(payload, claim, parameters)


def _validate_rotation_evidence_payload(
    payload: dict, claim: dict, parameters: PreprocessParameters
) -> None:
    if not isinstance(payload, dict):
        raise ValueError("rotation clearance evidence must be a JSON object")
    required_values = {
        "validation_type": "go2w_rotation_clearance_physical_crosscheck",
        "robot_model": "Unitree Go2-W",
        "passed": True,
        "robot_motion_commanded": False,
        "operator": claim["operator"],
        "posture": claim["posture"],
    }
    mismatched = [
        key for key, expected in required_values.items()
        if payload.get(key) != expected
    ]
    if mismatched:
        raise ValueError(
            "rotation clearance evidence contract mismatch: "
            + ", ".join(mismatched)
        )
    radius = payload.get("validated_rotation_envelope_radius_m")
    if radius is None or float(radius) + 1e-9 < parameters.rotation_envelope_radius:
        raise ValueError("rotation clearance evidence envelope is insufficient")
    evidence_checks = payload.get("checks") or {}
    missing = [
        key for key in _ROTATION_VALIDATION_CHECKS
        if evidence_checks.get(key) is not True
    ]
    if missing:
        raise ValueError(
            "rotation clearance evidence checks are incomplete: "
            + ", ".join(missing)
        )
    scope = payload.get("authorization_scope") or {}
    if scope.get("type") != "persistent_sensor_coverage":
        raise ValueError(
            "pose-bound rotation evidence cannot enable the persistent "
            "LiDAR preprocessor gate"
        )
    mitigation = payload.get("near_field_mitigation") or {}
    if mitigation.get("sensor_only_observability_complete") is not True:
        raise ValueError(
            "persistent rotation clearance requires complete sensor-only "
            "near-field observability"
        )
    if mitigation.get("method") not in {
        "additional_coverage_sensor",
        "physically_revalidated_lidar_mount",
    }:
        raise ValueError("persistent near-field mitigation method is unsupported")
