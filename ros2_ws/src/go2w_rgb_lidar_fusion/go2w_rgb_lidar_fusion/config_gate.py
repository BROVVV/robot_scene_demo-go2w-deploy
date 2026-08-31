from __future__ import annotations

from pathlib import Path

import yaml

from .fusion_core import FusionParameters


def _fusion_parameters(fusion: dict) -> FusionParameters:
    return FusionParameters(
        maximum_timestamp_delta_ms=float(fusion["maximum_timestamp_delta_ms"]),
        minimum_mask_points=int(fusion["minimum_mask_points"]),
        mask_boundary_margin_px=int(fusion["mask_boundary_margin_px"]),
        depth_mad_multiplier=float(fusion["depth_mad_multiplier"]),
        cluster_tolerance_m=float(fusion["cluster_tolerance_m"]),
        minimum_cluster_points=int(fusion["minimum_cluster_points"]),
        maximum_cluster_extent_m=float(fusion["maximum_cluster_extent_m"]),
    )


def _validate_overlay_evidence(camera: dict, extrinsics: dict) -> None:
    if camera.get("calibration_status") != "calibrated":
        raise ValueError("camera intrinsics are not calibrated")
    if not extrinsics.get("diagnostic_overlay_accepted"):
        raise ValueError("camera-LiDAR diagnostic overlay candidate is not accepted")
    validation = extrinsics.get("validation") or {}
    if int(validation.get("completed_scene_count", 0)) < int(
        validation.get("required_scene_count", 5)
    ):
        raise ValueError("fewer than five extrinsic validation scenes")
    error = validation.get("mean_edge_error_px")
    maximum = validation.get("maximum_allowed_mean_edge_error_px")
    if error is None or maximum is None or float(error) > float(maximum):
        raise ValueError("extrinsic overlay edge error gate failed")


def load_diagnostic_fusion_gate(
    fusion_path: str, camera_path: str, extrinsics_path: str
):
    """Load the stationary diagnostic path without authorizing metric 3D output."""

    fusion = yaml.safe_load(Path(fusion_path).read_text(encoding="utf-8")) or {}
    if not fusion.get("enabled") or not fusion.get("diagnostic_overlay_enabled"):
        raise ValueError("RGB-LiDAR diagnostic overlay is disabled")
    load_diagnostic_extrinsics_gate(camera_path, extrinsics_path)
    return fusion, _fusion_parameters(fusion)


def load_fusion_gate(fusion_path: str, camera_path: str, extrinsics_path: str):
    """Open only for navigation-grade geometry and metric 3D publication."""

    fusion = yaml.safe_load(Path(fusion_path).read_text(encoding="utf-8")) or {}
    if (
        not fusion.get("enabled")
        or fusion.get("validation_status") != "validated"
        or not fusion.get("navigation_geometry_validated")
        or not fusion.get("authorizes_3d_localization")
    ):
        raise ValueError("RGB-LiDAR fusion is disabled or unvalidated")
    load_extrinsics_gate(camera_path, extrinsics_path)
    return fusion, _fusion_parameters(fusion)


def load_diagnostic_extrinsics_gate(camera_path: str, extrinsics_path: str):
    camera = yaml.safe_load(Path(camera_path).read_text(encoding="utf-8")) or {}
    extrinsics = yaml.safe_load(Path(extrinsics_path).read_text(encoding="utf-8")) or {}
    _validate_overlay_evidence(camera, extrinsics)
    return camera, extrinsics


def load_extrinsics_gate(camera_path: str, extrinsics_path: str):
    camera = yaml.safe_load(Path(camera_path).read_text(encoding="utf-8")) or {}
    extrinsics = yaml.safe_load(Path(extrinsics_path).read_text(encoding="utf-8")) or {}
    if camera.get("calibration_status") != "calibrated":
        raise ValueError("camera intrinsics are not calibrated")
    if (
        extrinsics.get("calibration_status") != "calibrated"
        or not extrinsics.get("confirmed")
        or not extrinsics.get("navigation_geometry_validated")
    ):
        raise ValueError(
            "camera-LiDAR extrinsics are not navigation-grade calibrated and confirmed"
        )
    _validate_overlay_evidence(camera, extrinsics)
    validation = extrinsics.get("validation") or {}
    if not validation.get("moved_position_recheck_passed"):
        raise ValueError("moved-position extrinsic recheck has not passed")
    return camera, extrinsics
