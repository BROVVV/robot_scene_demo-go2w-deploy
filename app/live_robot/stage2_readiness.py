"""Machine-readable SemanticNavigation Stage 2/3 readiness.

Every check is fail-closed: a missing or false input makes the stage not
ready.  ``stage2_ready`` requires all twelve checks; ``stage3_ready``
additionally requires an explicit Stage-2 active-turn pass plus the
short-forward policy inputs.  This module is pure and ROS-independent.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


STAGE2_CHECKS = (
    "semantic_v1_ready",
    "pandar_raw_ready",
    "pandar_preprocess_ready",
    "pandar_extrinsics_validated",
    "current_hardware_geometry_loaded",
    "dual_lidar_rotation_observability_valid",
    "current_hardware_four_direction_evidence_valid",
    "pose_bound_rotation_lease_valid",
    "odom_fresh",
    "mode_ok",
    "motion_action_available",
    "no_stage2_error",
)

STAGE3_EXTRA_CHECKS = (
    "stage2_active_turn_only_pass",
    "semantic_forward_enabled",
    "front_clearance_valid",
    "short_forward_scope_valid",
)


@dataclass(frozen=True)
class ReadinessReport:
    checks: dict[str, bool]
    reasons: dict[str, str] = field(default_factory=dict)

    @property
    def ready(self) -> bool:
        return bool(self.checks) and all(self.checks.values())

    def to_dict(self) -> dict[str, Any]:
        return {
            "ready": self.ready,
            "checks": self.checks,
            "reasons": self.reasons,
        }


def _check_value(value: Any) -> bool:
    return value is True


def compute_stage2_readiness(
    *,
    semantic_v1_ready: Any = False,
    pandar_raw_ready: Any = False,
    pandar_preprocess_ready: Any = False,
    pandar_extrinsics_validated: Any = False,
    current_hardware_geometry_loaded: Any = False,
    dual_lidar_rotation_observability_valid: Any = False,
    current_hardware_four_direction_evidence_valid: Any = False,
    pose_bound_rotation_lease_valid: Any = False,
    odom_fresh: Any = False,
    mode_ok: Any = False,
    motion_action_available: Any = False,
    no_stage2_error: Any = True,
    reasons: dict[str, str] | None = None,
) -> ReadinessReport:
    """Compute the unified machine-readable Stage-2 turn-only readiness."""
    supplied = {
        "semantic_v1_ready": semantic_v1_ready,
        "pandar_raw_ready": pandar_raw_ready,
        "pandar_preprocess_ready": pandar_preprocess_ready,
        "pandar_extrinsics_validated": pandar_extrinsics_validated,
        "current_hardware_geometry_loaded": current_hardware_geometry_loaded,
        "dual_lidar_rotation_observability_valid": dual_lidar_rotation_observability_valid,
        "current_hardware_four_direction_evidence_valid": (
            current_hardware_four_direction_evidence_valid
        ),
        "pose_bound_rotation_lease_valid": pose_bound_rotation_lease_valid,
        "odom_fresh": odom_fresh,
        "mode_ok": mode_ok,
        "motion_action_available": motion_action_available,
        "no_stage2_error": no_stage2_error,
    }
    checks = {key: _check_value(value) for key, value in supplied.items()}
    base_reasons = dict(reasons or {})
    # A missing check always fails even if no reason is supplied.
    missing = [key for key in STAGE2_CHECKS if key not in checks]
    for key in missing:
        checks[key] = False
        base_reasons.setdefault(key, "check not supplied")
    report = ReadinessReport(checks=checks, reasons=base_reasons)
    if not report.ready:
        failed = [key for key in STAGE2_CHECKS if not checks.get(key)]
        base_reasons.setdefault("stage2", "not ready: " + ", ".join(failed))
    return report


def compute_stage3_readiness(
    *,
    stage2: ReadinessReport,
    stage2_active_turn_only_pass: Any = False,
    semantic_forward_enabled: Any = False,
    front_clearance_valid: Any = False,
    short_forward_scope_valid: Any = False,
    reasons: dict[str, str] | None = None,
) -> ReadinessReport:
    """Stage 3 short-forward readiness strictly depends on Stage 2 PASS."""
    checks = {
        "stage2_ready": stage2.ready,
        "stage2_active_turn_only_pass": _check_value(stage2_active_turn_only_pass),
        "semantic_forward_enabled": _check_value(semantic_forward_enabled),
        "front_clearance_valid": _check_value(front_clearance_valid),
        "short_forward_scope_valid": _check_value(short_forward_scope_valid),
    }
    base_reasons = dict(reasons or {})
    base_reasons["stage2"] = (
        "stage2_ready" if stage2.ready else "stage2 not ready; forward stays disabled"
    )
    return ReadinessReport(checks=checks, reasons=base_reasons)


def describe_blockers(report: ReadinessReport) -> list[str]:
    """Return human-readable blocker strings for a not-ready report."""
    return [
        f"{key}={value}"
        for key, value in report.checks.items()
        if value is not True
    ]
