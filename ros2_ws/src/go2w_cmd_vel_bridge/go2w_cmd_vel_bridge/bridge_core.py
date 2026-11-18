from __future__ import annotations

import math
from dataclasses import dataclass


def geometry_execution_allowed(*, scope, expected_scope, profile_sha256,
                               expected_sha256, age_seconds, trial_guard,
                               travel_guard, operator_supervised):
    """Bind live posture evidence to the launcher-selected geometry and guard."""
    if (not isinstance(expected_sha256, str) or len(expected_sha256) != 64
            or any(c not in '0123456789abcdef' for c in expected_sha256)
            or profile_sha256 != expected_sha256 or scope != expected_scope
            or not math.isfinite(age_seconds) or not -.05 <= age_seconds <= .3):
        return False
    if trial_guard:
        return scope in ('bounded_trial', 'operator_supervised_field')
    return (scope == 'operator_supervised_field' and bool(travel_guard)
            and bool(operator_supervised))


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


# 计划书 §6.5：来源取自 /go2w/control_source，由控制仲裁器发布。取值集合与
# arbiter_core.PRIORITY 加上仲裁器的几个中止态一致。
KNOWN_CONTROL_SOURCES = (
    "manual",
    "nav2",
    "search",
    "watchdog",
    "emergency_stop",
    "remote_override",
    "disarmed",
)


@dataclass(frozen=True)
class SafetyState:
    execution_enabled: bool = False
    operator_armed: bool = False
    lease_alive: bool = False
    lidar_fresh: bool = False
    rotation_clearance_valid: bool = False
    # 操作员在场的转向放行（2026-10-10 用户批准；方案见
    # docs/GO2W_操作员在场转向放行_方案_20261010.md）。
    #
    # 它是**唯一**的放宽点，且只放宽下面那一条净空判定 —— 武装、租约、急停、
    # 雷达新鲜度、遥控接管、定位新鲜度、看门狗全部照旧。默认 False：
    # 不显式声明"操作员在场"时，本函数的行为与加重这个字段之前**逐位相同**。
    #
    # 为什么可以放宽：净空那条要证明的是"原地转一圈不会撞到东西"，而这只狗的
    # 雷达看不见自身近场（实测 720/720 个采样方位不可观测，最差径向空隙
    # ~0.291 m），所以持久证明在软件上开不了。替代品不是"证明安全"，而是
    # "现场有人手持即停"——那是操作决策，所以只能由启动栈的人声明
    # （`bridge_node` 从启动参数读，不从话题读：话题任何进程都能发）。
    operator_supervised_rotation: bool = False
    # 计划书 §6.5：Nav2 执行不再看 /lio/valid，改看坐标桥发布的
    # /go2w/nav/localization_valid（map -> odom_fused -> base_link 的新鲜度）。
    navigation_localization_fresh: bool = False
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
    turning = abs(requested.angular_z) > 1e-9
    # 三者同时成立才算"用上了操作员豁免"：要转向、没有净空证据、操作员声明在场。
    # 缺任何一条都退回原行为（净空有效时本来就不需要豁免，所以那时也不标它）。
    supervised_turn = (
        turning and not safety.rotation_clearance_valid and safety.operator_supervised_rotation
    )
    if turning and not safety.rotation_clearance_valid and not safety.operator_supervised_rotation:
        blockers.append("rotation_clearance_unvalidated")
    # 计划书 §6.5：控制来源必须是已知取值。来源字符串一旦拼错或为空，下面几条
    # 按来源分流的闸门会全部静默跳过——那正是最危险的一类失效。
    if source not in KNOWN_CONTROL_SOURCES:
        blockers.append("control_source_unknown")
    # Nav2 命令额外要求导航定位新鲜。这条只对 nav2 来源生效：手动遥控不该
    # 因为 Nav2 的坐标桥没起来而失去控制。
    if source == "nav2" and not safety.navigation_localization_fresh:
        blockers.append("navigation_localization_stale")
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
    if requested.is_zero:
        # Stop/cancel is immediate; do not let an existing timed action or
        # acceleration shaping continue movement after an explicit zero.
        return BridgeDecision(Velocity(), True, "zero_command", True)
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
    # 放行理由必须**看得出是靠豁免**，不能与"净空已验证"同形：否则事后从话题上
    # 无法区分哪一次转向有物理依据。这条串会原样发到
    # /go2w/cmd_vel_bridge/status（`bridge_node._tick`）。
    return BridgeDecision(
        shaped,
        True,
        "allowed_operator_supervised_rotation" if supervised_turn else "allowed",
        False,
    )


def _approach(current: float, target: float, maximum_delta: float) -> float:
    return current + max(-maximum_delta, min(maximum_delta, target - current))
