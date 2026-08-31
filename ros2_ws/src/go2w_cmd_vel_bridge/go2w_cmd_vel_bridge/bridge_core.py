from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Velocity:
    linear_x: float = 0.0
    angular_z: float = 0.0

    @property
    def is_zero(self) -> bool:
        return self.linear_x == 0.0 and self.angular_z == 0.0


@dataclass(frozen=True)
class Limits:
    maximum_linear_x: float = 0.15
    maximum_angular_z: float = 0.20
    maximum_linear_acceleration: float = 0.20
    maximum_angular_acceleration: float = 0.40
    watchdog_seconds: float = 0.30


@dataclass(frozen=True)
class SafetyState:
    execution_enabled: bool = False
    operator_armed: bool = False
    lease_alive: bool = False
    lidar_fresh: bool = False
    rotation_clearance_valid: bool = False
    lio_fresh: bool = False
    robot_error_zero: bool = False
    emergency_stop: bool = True
    remote_override: bool = True


@dataclass(frozen=True)
class BridgeDecision:
    velocity: Velocity
    allowed: bool
    reason: str
    cancel_active_action: bool


def decide_velocity(
    requested: Velocity,
    previous: Velocity,
    *,
    command_age_seconds: float,
    dt_seconds: float,
    source: str,
    safety: SafetyState,
    limits: Limits = Limits(),
) -> BridgeDecision:
    blockers = []
    if not safety.execution_enabled:
        blockers.append("execution_disabled")
    if not safety.operator_armed:
        blockers.append("operator_not_armed")
    if not safety.lease_alive:
        blockers.append("lease_unavailable")
    if not safety.lidar_fresh:
        blockers.append("lidar_stale")
    if abs(requested.angular_z) > 1e-9 and not safety.rotation_clearance_valid:
        blockers.append("rotation_clearance_unvalidated")
    if source == "nav2" and not safety.lio_fresh:
        blockers.append("lio_stale_for_nav2")
    if not safety.robot_error_zero:
        blockers.append("robot_error_or_unknown")
    if safety.emergency_stop:
        blockers.append("emergency_stop")
    if safety.remote_override:
        blockers.append("remote_override")
    if command_age_seconds < 0.0 or command_age_seconds > limits.watchdog_seconds:
        blockers.append("command_watchdog_expired")
    if not all(math.isfinite(item) for item in (requested.linear_x, requested.angular_z)):
        blockers.append("nonfinite_command")
    if blockers:
        return BridgeDecision(Velocity(), False, ",".join(blockers), True)
    dt = max(0.0, min(float(dt_seconds), limits.watchdog_seconds))
    target_linear = max(-limits.maximum_linear_x, min(limits.maximum_linear_x, requested.linear_x))
    target_angular = max(
        -limits.maximum_angular_z, min(limits.maximum_angular_z, requested.angular_z)
    )
    linear_delta = limits.maximum_linear_acceleration * dt
    angular_delta = limits.maximum_angular_acceleration * dt
    shaped = Velocity(
        _approach(previous.linear_x, target_linear, linear_delta),
        _approach(previous.angular_z, target_angular, angular_delta),
    )
    return BridgeDecision(shaped, True, "allowed", False)


def _approach(current: float, target: float, maximum_delta: float) -> float:
    return current + max(-maximum_delta, min(maximum_delta, target - current))
