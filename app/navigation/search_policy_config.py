"""Typed loader for configs/go2w/search_policy.yaml (plan section 25.1).

schema_version 2 replaces the single ``minimum_front_clearance_m`` knob with
explicit clearance / camera-view / stuck / recovery / viewpoint / target-approach
blocks.  There is exactly one source of truth for every threshold: the loader
never falls back to the deprecated key, so the two cannot disagree.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

DEFAULT_SEARCH_POLICY_CONFIG = "configs/go2w/search_policy.yaml"


@dataclass
class PlatformSafetyCapabilities:
    emergency_collision_stop_supported: bool = False
    clearance_aware_navigation_supported: bool = True
    autonomous_obstacle_bypass_supported: bool = False

    def to_dict(self) -> dict[str, Any]:
        return {
            "emergency_collision_stop_supported": self.emergency_collision_stop_supported,
            "clearance_aware_navigation_supported": self.clearance_aware_navigation_supported,
            "autonomous_obstacle_bypass_supported": self.autonomous_obstacle_bypass_supported,
        }


@dataclass
class ClearanceNavigationConfig:
    enabled: bool = True
    hard_clearance_m: float = 0.40
    preferred_clearance_m: float = 0.80
    footprint_radius_m: float = 0.25
    unknown_is_traversable: bool = False
    stale_is_traversable: bool = False
    path_clearance_weight: float = 3.0
    path_min_clearance_weight: float = 3.0
    path_mean_clearance_weight: float = 1.0
    route_length_weight: float = 1.0
    turning_weight: float = 0.5
    unknown_risk_weight: float = 2.5

    def to_dict(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "hard_clearance_m": self.hard_clearance_m,
            "preferred_clearance_m": self.preferred_clearance_m,
            "footprint_radius_m": self.footprint_radius_m,
            "unknown_is_traversable": self.unknown_is_traversable,
            "stale_is_traversable": self.stale_is_traversable,
            "path_clearance_weight": self.path_clearance_weight,
            "path_min_clearance_weight": self.path_min_clearance_weight,
            "path_mean_clearance_weight": self.path_mean_clearance_weight,
            "route_length_weight": self.route_length_weight,
            "turning_weight": self.turning_weight,
            "unknown_risk_weight": self.unknown_risk_weight,
        }


@dataclass
class CameraViewConfig:
    enabled: bool = True
    fov_half_angle_deg: float = 45.0
    useful_view_min_m: float = 0.30
    useful_view_max_m: float = 2.50
    camera_blocked_front_m: float = 0.45
    near_field_threshold_m: float = 0.50
    near_field_ratio_threshold: float = 0.55
    low_information_threshold: float = 0.20
    clearance_weight: float = 2.0
    visibility_weight: float = 2.5
    information_gain_weight: float = 2.0
    occlusion_penalty_weight: float = 3.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "fov_half_angle_deg": self.fov_half_angle_deg,
            "useful_view_min_m": self.useful_view_min_m,
            "useful_view_max_m": self.useful_view_max_m,
            "camera_blocked_front_m": self.camera_blocked_front_m,
            "near_field_threshold_m": self.near_field_threshold_m,
            "near_field_ratio_threshold": self.near_field_ratio_threshold,
            "low_information_threshold": self.low_information_threshold,
            "clearance_weight": self.clearance_weight,
            "visibility_weight": self.visibility_weight,
            "information_gain_weight": self.information_gain_weight,
            "occlusion_penalty_weight": self.occlusion_penalty_weight,
        }


@dataclass
class StuckDetectionConfig:
    enabled: bool = True
    history_cycles: int = 3
    min_commanded_progress_m: float = 0.10
    max_observed_progress_m: float = 0.03
    repeated_viewpoint_limit: int = 2
    repeated_heading_limit: int = 3
    min_escape_turn_deg: float = 15.0
    constrained_clearance_m: float = 0.80
    recovery_required_failures: int = 2

    def to_dict(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "history_cycles": self.history_cycles,
            "min_commanded_progress_m": self.min_commanded_progress_m,
            "max_observed_progress_m": self.max_observed_progress_m,
            "repeated_viewpoint_limit": self.repeated_viewpoint_limit,
            "repeated_heading_limit": self.repeated_heading_limit,
            "min_escape_turn_deg": self.min_escape_turn_deg,
            "constrained_clearance_m": self.constrained_clearance_m,
            "recovery_required_failures": self.recovery_required_failures,
        }


@dataclass
class RecoveryPolicyConfig:
    enabled: bool = True
    max_backward_step_m: float = 0.12
    min_backward_step_m: float = 0.05
    max_age_sec: float = 8.0
    heading_tolerance_deg: float = 8.0
    max_consecutive: int = 2
    max_total_m: float = 0.36
    escape_turn_candidates_deg: tuple[float, ...] = (
        10.0, -10.0, 15.0, -15.0, 20.0, -20.0, 30.0, -30.0,
    )

    def to_dict(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "max_backward_step_m": self.max_backward_step_m,
            "min_backward_step_m": self.min_backward_step_m,
            "max_age_sec": self.max_age_sec,
            "heading_tolerance_deg": self.heading_tolerance_deg,
            "max_consecutive": self.max_consecutive,
            "max_total_m": self.max_total_m,
            "escape_turn_candidates_deg": list(self.escape_turn_candidates_deg),
        }


@dataclass
class ViewpointPlanningConfig:
    enabled: bool = True
    frontier_standoff_candidates_m: tuple[float, ...] = (0.60, 0.80, 1.00)
    frontier_lateral_offsets_m: tuple[float, ...] = (-0.30, 0.0, 0.30)
    require_path_clearance: bool = True
    require_view_quality: bool = True

    def to_dict(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "frontier_standoff_candidates_m": list(self.frontier_standoff_candidates_m),
            "frontier_lateral_offsets_m": list(self.frontier_lateral_offsets_m),
            "require_path_clearance": self.require_path_clearance,
            "require_view_quality": self.require_view_quality,
        }


@dataclass
class TargetApproachConfig:
    enabled: bool = True
    finish_on_visual_confirmation: bool = False
    preferred_standoff_m: float = 0.85
    minimum_standoff_m: float = 0.70
    maximum_standoff_m: float = 1.10
    approach_ring_angles_deg: tuple[float, ...] = (-90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0)
    max_approach_step_m: float = 0.25
    fine_approach_step_m: float = 0.10
    align_yaw_threshold_deg: float = 8.0
    max_finish_bearing_deg: float = 12.0
    require_fresh_depth: bool = True
    require_final_verification: bool = True
    require_line_of_sight: bool = True

    def to_dict(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "finish_on_visual_confirmation": self.finish_on_visual_confirmation,
            "preferred_standoff_m": self.preferred_standoff_m,
            "minimum_standoff_m": self.minimum_standoff_m,
            "maximum_standoff_m": self.maximum_standoff_m,
            "approach_ring_angles_deg": list(self.approach_ring_angles_deg),
            "max_approach_step_m": self.max_approach_step_m,
            "fine_approach_step_m": self.fine_approach_step_m,
            "align_yaw_threshold_deg": self.align_yaw_threshold_deg,
            "max_finish_bearing_deg": self.max_finish_bearing_deg,
            "require_fresh_depth": self.require_fresh_depth,
            "require_final_verification": self.require_final_verification,
            "require_line_of_sight": self.require_line_of_sight,
        }


@dataclass
class LlmEmbodimentContextConfig:
    enabled: bool = True
    safety_sector_count: int = 24
    max_frontiers: int = 8
    include_recovery_context: bool = True
    include_perception_quality: bool = True

    def to_dict(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "safety_sector_count": self.safety_sector_count,
            "max_frontiers": self.max_frontiers,
            "include_recovery_context": self.include_recovery_context,
            "include_perception_quality": self.include_perception_quality,
        }


@dataclass
class SearchPolicy:
    schema_version: int = 2
    motion_enabled: bool = False
    default_mode: str = "observe_only"
    capabilities: PlatformSafetyCapabilities = field(default_factory=PlatformSafetyCapabilities)
    clearance_navigation: ClearanceNavigationConfig = field(default_factory=ClearanceNavigationConfig)
    camera_view: CameraViewConfig = field(default_factory=CameraViewConfig)
    stuck_detection: StuckDetectionConfig = field(default_factory=StuckDetectionConfig)
    recovery: RecoveryPolicyConfig = field(default_factory=RecoveryPolicyConfig)
    viewpoint_planning: ViewpointPlanningConfig = field(default_factory=ViewpointPlanningConfig)
    target_approach: TargetApproachConfig = field(default_factory=TargetApproachConfig)
    llm_embodiment_context: LlmEmbodimentContextConfig = field(
        default_factory=LlmEmbodimentContextConfig
    )

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "motion_enabled": self.motion_enabled,
            "default_mode": self.default_mode,
            "capabilities": self.capabilities.to_dict(),
            "clearance_navigation": self.clearance_navigation.to_dict(),
            "camera_view": self.camera_view.to_dict(),
            "stuck_detection": self.stuck_detection.to_dict(),
            "recovery": self.recovery.to_dict(),
            "viewpoint_planning": self.viewpoint_planning.to_dict(),
            "target_approach": self.target_approach.to_dict(),
            "llm_embodiment_context": self.llm_embodiment_context.to_dict(),
        }


def load_search_policy(
    path: str | Path | None = None,
    *,
    overrides: dict[str, Any] | None = None,
) -> SearchPolicy:
    source = Path(path or DEFAULT_SEARCH_POLICY_CONFIG)
    payload: dict[str, Any] = {}
    if source.is_file():
        payload = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    payload = _merge(payload, overrides or {})
    return SearchPolicy(
        schema_version=int(payload.get("schema_version", 2)),
        motion_enabled=bool(payload.get("motion_enabled", False)),
        default_mode=str(payload.get("default_mode") or "observe_only"),
        capabilities=_capabilities_from(payload.get("capabilities") or {}),
        clearance_navigation=_clearance_from(payload.get("clearance_navigation") or {}),
        camera_view=_camera_view_from(payload.get("camera_view") or {}),
        stuck_detection=_stuck_from(payload.get("stuck_detection") or {}),
        recovery=_recovery_from(payload.get("recovery") or {}),
        viewpoint_planning=_viewpoint_from(payload.get("viewpoint_planning") or {}),
        target_approach=_target_approach_from(payload.get("target_approach") or {}),
        llm_embodiment_context=_llm_context_from(
            payload.get("llm_embodiment_context") or {}
        ),
    )


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def _capabilities_from(value: dict[str, Any]) -> PlatformSafetyCapabilities:
    return PlatformSafetyCapabilities(
        emergency_collision_stop_supported=bool(
            value.get("emergency_collision_stop_supported", False)
        ),
        clearance_aware_navigation_supported=bool(
            value.get("clearance_aware_navigation_supported", True)
        ),
        autonomous_obstacle_bypass_supported=bool(
            value.get("autonomous_obstacle_bypass_supported", False)
        ),
    )


def _clearance_from(value: dict[str, Any]) -> ClearanceNavigationConfig:
    return ClearanceNavigationConfig(
        enabled=bool(value.get("enabled", True)),
        hard_clearance_m=float(value.get("hard_clearance_m", 0.40)),
        preferred_clearance_m=float(value.get("preferred_clearance_m", 0.80)),
        footprint_radius_m=float(value.get("footprint_radius_m", 0.25)),
        unknown_is_traversable=bool(value.get("unknown_is_traversable", False)),
        stale_is_traversable=bool(value.get("stale_is_traversable", False)),
        path_clearance_weight=float(value.get("path_clearance_weight", 3.0)),
        path_min_clearance_weight=float(value.get("path_min_clearance_weight", 3.0)),
        path_mean_clearance_weight=float(value.get("path_mean_clearance_weight", 1.0)),
        route_length_weight=float(value.get("route_length_weight", 1.0)),
        turning_weight=float(value.get("turning_weight", 0.5)),
        unknown_risk_weight=float(value.get("unknown_risk_weight", 2.5)),
    )


def _camera_view_from(value: dict[str, Any]) -> CameraViewConfig:
    return CameraViewConfig(
        enabled=bool(value.get("enabled", True)),
        fov_half_angle_deg=float(value.get("fov_half_angle_deg", 45.0)),
        useful_view_min_m=float(value.get("useful_view_min_m", 0.30)),
        useful_view_max_m=float(value.get("useful_view_max_m", 2.50)),
        camera_blocked_front_m=float(value.get("camera_blocked_front_m", 0.45)),
        near_field_threshold_m=float(value.get("near_field_threshold_m", 0.50)),
        near_field_ratio_threshold=float(value.get("near_field_ratio_threshold", 0.55)),
        low_information_threshold=float(value.get("low_information_threshold", 0.20)),
        clearance_weight=float(value.get("clearance_weight", 2.0)),
        visibility_weight=float(value.get("visibility_weight", 2.5)),
        information_gain_weight=float(value.get("information_gain_weight", 2.0)),
        occlusion_penalty_weight=float(value.get("occlusion_penalty_weight", 3.0)),
    )


def _stuck_from(value: dict[str, Any]) -> StuckDetectionConfig:
    return StuckDetectionConfig(
        enabled=bool(value.get("enabled", True)),
        history_cycles=int(value.get("history_cycles", 3)),
        min_commanded_progress_m=float(value.get("min_commanded_progress_m", 0.10)),
        max_observed_progress_m=float(value.get("max_observed_progress_m", 0.03)),
        repeated_viewpoint_limit=int(value.get("repeated_viewpoint_limit", 2)),
        repeated_heading_limit=int(value.get("repeated_heading_limit", 3)),
        min_escape_turn_deg=float(value.get("min_escape_turn_deg", 15.0)),
        constrained_clearance_m=float(value.get("constrained_clearance_m", 0.80)),
        recovery_required_failures=int(value.get("recovery_required_failures", 2)),
    )


def _recovery_from(value: dict[str, Any]) -> RecoveryPolicyConfig:
    return RecoveryPolicyConfig(
        enabled=bool(value.get("enabled", True)),
        max_backward_step_m=float(value.get("max_backward_step_m", 0.12)),
        min_backward_step_m=float(value.get("min_backward_step_m", 0.05)),
        max_age_sec=float(value.get("max_age_sec", 8.0)),
        heading_tolerance_deg=float(value.get("heading_tolerance_deg", 8.0)),
        max_consecutive=int(value.get("max_consecutive", 2)),
        max_total_m=float(value.get("max_total_m", 0.36)),
        escape_turn_candidates_deg=_floats(
            value.get("escape_turn_candidates_deg"),
            (10.0, -10.0, 15.0, -15.0, 20.0, -20.0, 30.0, -30.0),
        ),
    )


def _viewpoint_from(value: dict[str, Any]) -> ViewpointPlanningConfig:
    return ViewpointPlanningConfig(
        enabled=bool(value.get("enabled", True)),
        frontier_standoff_candidates_m=_floats(
            value.get("frontier_standoff_candidates_m"), (0.60, 0.80, 1.00)
        ),
        frontier_lateral_offsets_m=_floats(
            value.get("frontier_lateral_offsets_m"), (-0.30, 0.0, 0.30)
        ),
        require_path_clearance=bool(value.get("require_path_clearance", True)),
        require_view_quality=bool(value.get("require_view_quality", True)),
    )


def _target_approach_from(value: dict[str, Any]) -> TargetApproachConfig:
    return TargetApproachConfig(
        enabled=bool(value.get("enabled", True)),
        finish_on_visual_confirmation=bool(
            value.get("finish_on_visual_confirmation", False)
        ),
        preferred_standoff_m=float(value.get("preferred_standoff_m", 0.85)),
        minimum_standoff_m=float(value.get("minimum_standoff_m", 0.70)),
        maximum_standoff_m=float(value.get("maximum_standoff_m", 1.10)),
        approach_ring_angles_deg=_floats(
            value.get("approach_ring_angles_deg"),
            (-90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0),
        ),
        max_approach_step_m=float(value.get("max_approach_step_m", 0.25)),
        fine_approach_step_m=float(value.get("fine_approach_step_m", 0.10)),
        align_yaw_threshold_deg=float(value.get("align_yaw_threshold_deg", 8.0)),
        max_finish_bearing_deg=float(value.get("max_finish_bearing_deg", 12.0)),
        require_fresh_depth=bool(value.get("require_fresh_depth", True)),
        require_final_verification=bool(value.get("require_final_verification", True)),
        require_line_of_sight=bool(value.get("require_line_of_sight", True)),
    )


def _llm_context_from(value: dict[str, Any]) -> LlmEmbodimentContextConfig:
    return LlmEmbodimentContextConfig(
        enabled=bool(value.get("enabled", True)),
        safety_sector_count=int(value.get("safety_sector_count", 24)),
        max_frontiers=int(value.get("max_frontiers", 8)),
        include_recovery_context=bool(value.get("include_recovery_context", True)),
        include_perception_quality=bool(value.get("include_perception_quality", True)),
    )


def _floats(value: Any, default: tuple[float, ...]) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or not value:
        return default
    return tuple(float(item) for item in value)


def _merge(base: dict[str, Any], overlay: dict[str, Any]) -> dict[str, Any]:
    result = dict(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = value
    return result
