"""Fail-closed capability gates for live Go2-W Nav2 planning and execution."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from typing import Any


PLAN_ONLY_REQUIRED = (
    "level_d_passed",
    "physical_geometry_confirmed",
    "footprint_confirmed",
    "scan_frame_valid",
    "lidar_fresh",
    "lio_fresh",
    "map_valid",
    "tf_valid",
    "compute_path_to_pose_ready",
)

EXECUTE_REQUIRED = PLAN_ONLY_REQUIRED + (
    "nav2_allow_execute",
    "collision_monitor_active",
    "velocity_smoother_active",
    "lease_valid",
    "arbiter_active",
    "cmd_vel_bridge_active",
    "cmd_vel_watchdog_active",
    "operator_armed",
    "second_confirmation",
    "emergency_stop_confirmed",
    "remote_override_clear",
    "robot_error_zero",
)


@dataclass(frozen=True)
class NavigationReadiness:
    """Evidence snapshot. Every field defaults to the conservative state."""

    level_d_passed: bool = False
    physical_geometry_confirmed: bool = False
    footprint_confirmed: bool = False
    scan_frame_valid: bool = False
    lidar_fresh: bool = False
    lio_fresh: bool = False
    map_valid: bool = False
    tf_valid: bool = False
    compute_path_to_pose_ready: bool = False
    nav2_allow_execute: bool = False
    collision_monitor_active: bool = False
    velocity_smoother_active: bool = False
    lease_valid: bool = False
    arbiter_active: bool = False
    cmd_vel_bridge_active: bool = False
    cmd_vel_watchdog_active: bool = False
    operator_armed: bool = False
    second_confirmation: bool = False
    emergency_stop_confirmed: bool = False
    remote_override_clear: bool = False
    robot_error_zero: bool = False

    @classmethod
    def from_mapping(cls, value: dict[str, Any] | None) -> "NavigationReadiness":
        raw = value or {}
        known = cls.__dataclass_fields__
        return cls(**{key: raw.get(key) is True for key in known})


@dataclass(frozen=True)
class NavigationGateResult:
    mode: str
    allowed: bool
    blocking_conditions: tuple[str, ...]
    required_conditions: tuple[str, ...]
    evidence: NavigationReadiness
    evaluated_at: str
    schema_version: str = "1.0"

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "mode": self.mode,
            "allowed": self.allowed,
            "blocking_conditions": list(self.blocking_conditions),
            "required_conditions": list(self.required_conditions),
            "evidence": asdict(self.evidence),
            "evaluated_at": self.evaluated_at,
        }


def evaluate_navigation_gate(
    mode: str,
    readiness: NavigationReadiness | dict[str, Any] | None = None,
) -> NavigationGateResult:
    """Evaluate planning/execution without importing ROS or motion code."""

    normalized = str(mode).strip().lower()
    aliases = {
        "plan_only": "nav2_plan_only",
        "nav2_plan_only": "nav2_plan_only",
        "execute": "nav2_execute",
        "nav2_execute": "nav2_execute",
    }
    if normalized not in aliases:
        raise ValueError(f"unsupported navigation gate mode: {mode}")
    selected = aliases[normalized]
    snapshot = (
        readiness
        if isinstance(readiness, NavigationReadiness)
        else NavigationReadiness.from_mapping(readiness)
    )
    required = EXECUTE_REQUIRED if selected == "nav2_execute" else PLAN_ONLY_REQUIRED
    blockers = tuple(name for name in required if not getattr(snapshot, name))
    return NavigationGateResult(
        mode=selected,
        allowed=not blockers,
        blocking_conditions=blockers,
        required_conditions=required,
        evidence=snapshot,
        evaluated_at=datetime.now(UTC).isoformat(),
    )


def validate_execute_gate_payload(payload: dict[str, Any] | None) -> None:
    """Reject absent, stale-format, mismatched, or blocked execution evidence."""

    if not isinstance(payload, dict):
        raise ValueError("NAV2_CAPABILITY_GATE_MISSING: execution gate result is required")
    if payload.get("schema_version") != "1.0":
        raise ValueError("NAV2_CAPABILITY_GATE_INVALID: unsupported gate schema")
    if payload.get("mode") != "nav2_execute":
        raise ValueError("NAV2_CAPABILITY_GATE_INVALID: gate mode is not nav2_execute")
    if payload.get("required_conditions") != list(EXECUTE_REQUIRED):
        raise ValueError("NAV2_CAPABILITY_GATE_INVALID: execution requirements mismatch")
    evidence = payload.get("evidence")
    if not isinstance(evidence, dict) or not all(evidence.get(name) is True for name in EXECUTE_REQUIRED):
        raise ValueError("NAV2_CAPABILITY_GATE_INVALID: required live evidence is incomplete")
    blockers = payload.get("blocking_conditions")
    if payload.get("allowed") is not True or blockers != []:
        detail = ",".join(str(item) for item in (blockers or ["gate_not_allowed"]))
        raise ValueError(f"NAV2_CAPABILITY_GATE_BLOCKED: {detail}")
