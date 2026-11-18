from go2w_cmd_vel_bridge.bridge_core import Limits, SafetyState, Velocity, decide_velocity


def safe_state(**updates):
    values = {
        "execution_enabled": True,
        "operator_armed": True,
        "lease_alive": True,
        "lidar_fresh": True,
        "rotation_clearance_valid": True,
        "navigation_localization_fresh": True,
        "robot_error_zero": True,
        "emergency_stop": False,
        "remote_override": False,
    }
    values.update(updates)
    return SafetyState(**values)


def test_default_state_and_each_critical_gate_cancel_motion():
    request = Velocity(0.1, 0.1)
    default = decide_velocity(
        request, Velocity(), command_age_seconds=0.0, dt_seconds=0.1, source="search", safety=SafetyState()
    )
    assert not default.allowed and default.velocity.is_zero and default.cancel_active_action
    for update in (
        {"lease_alive": False},
        {"lidar_fresh": False},
        {"rotation_clearance_valid": False},
        {"robot_error_zero": False},
        {"emergency_stop": True},
        {"remote_override": True},
    ):
        decision = decide_velocity(
            request,
            Velocity(),
            command_age_seconds=0.0,
            dt_seconds=0.1,
            source="search",
            safety=safe_state(**update),
        )
        assert not decision.allowed and decision.velocity.is_zero


def test_watchdog_and_nav2_localization_gate():
    stale = decide_velocity(
        Velocity(0.1, 0.0),
        Velocity(),
        command_age_seconds=0.301,
        dt_seconds=0.1,
        source="search",
        safety=safe_state(),
    )
    assert not stale.allowed and "watchdog" in stale.reason
    nav2 = decide_velocity(
        Velocity(0.1, 0.0),
        Velocity(),
        command_age_seconds=0.0,
        dt_seconds=0.1,
        source="nav2",
        safety=safe_state(navigation_localization_fresh=False),
    )
    assert not nav2.allowed and "navigation_localization_stale" in nav2.reason


def test_speed_and_acceleration_limits_are_both_enforced():
    limits = Limits(
        maximum_linear_x=0.15,
        maximum_angular_z=0.20,
        maximum_linear_acceleration=0.20,
        maximum_angular_acceleration=0.40,
        watchdog_seconds=0.30,
    )
    decision = decide_velocity(
        Velocity(10.0, -10.0),
        Velocity(),
        command_age_seconds=0.01,
        dt_seconds=0.1,
        source="search",
        safety=safe_state(),
        limits=limits,
    )
    assert decision.allowed
    assert abs(decision.velocity.linear_x - 0.02) < 1e-9
    assert abs(decision.velocity.angular_z + 0.04) < 1e-9


def test_unvalidated_rotation_still_allows_zero_yaw_forward_request():
    forward = decide_velocity(
        Velocity(0.1, 0.0), Velocity(), command_age_seconds=0.0,
        dt_seconds=0.1, source="search",
        safety=safe_state(rotation_clearance_valid=False),
    )
    assert forward.allowed
    turn = decide_velocity(
        Velocity(0.0, 0.1), Velocity(), command_age_seconds=0.0,
        dt_seconds=0.1, source="search",
        safety=safe_state(rotation_clearance_valid=False),
    )
    assert not turn.allowed
    assert "rotation_clearance_unvalidated" in turn.reason


def _decide(request, **updates):
    return decide_velocity(
        request, Velocity(), command_age_seconds=0.0,
        dt_seconds=0.1, source="search", safety=safe_state(**updates),
    )


# 操作员在场的转向放行（2026-10-10 用户批准）。
#
# 方案见 docs/GO2W_操作员在场转向放行_方案_20261010.md。这只狗雷达看不见自身近场
# （实测 720/720 个采样方位不可观测），持久净空证明在软件上开不了；替代品是
# 「现场有人手持即停」，所以这是一条**操作决策**，只能由启动栈的人声明。
#
# 下面四条是**成对**的：豁免必须生效，且只能用在这一条上。拆成独立用例是为了让
# 反向校准能分别打中 —— 写死豁免时只该 TURN_IS_STILL_BLOCKED 失败，写死不放行时
# 只该 TURN_IS_ALLOWED_AND_MARKED 失败。

TURN = Velocity(0.0, 0.1)


def test_unexempted_turn_is_still_blocked_by_clearance():
    """不声明操作员在场时，带转向的命令仍被净空那条拦死（现状不许被改松）。"""
    decision = _decide(TURN, rotation_clearance_valid=False)
    assert not decision.allowed and decision.velocity.is_zero
    assert decision.reason == "rotation_clearance_unvalidated"


def test_supervised_turn_is_allowed_and_marked():
    """声明在场 + 带转向 ⇒ 放行，且理由**看得出是靠豁免**而不是净空证据。"""
    decision = _decide(
        TURN, rotation_clearance_valid=False, operator_supervised_rotation=True
    )
    assert decision.allowed, decision.reason
    assert decision.reason == "allowed_operator_supervised_rotation"


def test_supervision_does_not_exempt_any_other_gate():
    """豁免不外溢：急停、遥控接管、雷达、武装、租约，一条都不许被放过。"""
    for update in (
        {"emergency_stop": True},
        {"remote_override": True},
        {"lidar_fresh": False},
        {"operator_armed": False},
        {"lease_alive": False},
    ):
        blocked = _decide(
            TURN, rotation_clearance_valid=False, operator_supervised_rotation=True, **update
        )
        assert not blocked.allowed and blocked.velocity.is_zero, update


def test_supervision_is_not_marked_when_the_exemption_was_not_used():
    """豁免没被用上时不许标它 —— 否则事后无法区分「有物理依据的转向」与
    「靠人放行的转向」，而那正是这条通道唯一的安全记录。"""
    forward = _decide(
        Velocity(0.1, 0.0), rotation_clearance_valid=False, operator_supervised_rotation=True
    )
    assert forward.reason == "allowed"
    clearance_ok = _decide(
        TURN, rotation_clearance_valid=True, operator_supervised_rotation=True
    )
    assert clearance_ok.reason == "allowed"


def test_unknown_control_source_is_refused():
    """计划书 §6.5：来源取值必须是已知控制源。

    来源字符串一旦拼错，下面几条按来源分流的闸门会全部静默跳过——那正是最
    危险的一类失效。宁可拒绝，也不要让一个空字符串绕过 nav2 的定位闸门。
    """
    decision = decide_velocity(
        Velocity(0.1, 0.0),
        Velocity(),
        command_age_seconds=0.0,
        dt_seconds=0.1,
        source="",
        safety=safe_state(navigation_localization_fresh=False),
    )
    assert not decision.allowed
    assert "control_source_unknown" in decision.reason
    assert decision.velocity.is_zero


def test_manual_command_does_not_need_navigation_localization():
    """手动遥控不该因为 Nav2 的坐标桥没起来而失去控制。"""
    decision = decide_velocity(
        Velocity(0.1, 0.0),
        Velocity(),
        command_age_seconds=0.0,
        dt_seconds=0.1,
        source="manual",
        safety=safe_state(navigation_localization_fresh=False),
    )
    assert decision.allowed


def test_nav2_command_with_fresh_localization_is_allowed():
    decision = decide_velocity(
        Velocity(0.1, 0.0),
        Velocity(),
        command_age_seconds=0.0,
        dt_seconds=0.1,
        source="nav2",
        safety=safe_state(navigation_localization_fresh=True),
    )
    assert decision.allowed, decision.reason


def test_explicit_stop_bypasses_acceleration_shaping_and_cancels_timed_action():
    decision=decide_velocity(Velocity(),Velocity(.1,.2),command_age_seconds=0.,dt_seconds=.05,
                             source='nav2',safety=safe_state())
    assert decision.velocity.is_zero and decision.cancel_active_action


def test_field_geometry_needs_bound_current_profile_supervision_and_ordinary_guard():
    from go2w_cmd_vel_bridge.bridge_core import geometry_execution_allowed
    fields=dict(scope='operator_supervised_field',expected_scope='operator_supervised_field',
        profile_sha256='a'*64,expected_sha256='a'*64,age_seconds=.05,
        trial_guard=False,travel_guard=True,operator_supervised=True)
    assert geometry_execution_allowed(**fields)
    for update in [dict(profile_sha256='b'*64),dict(expected_sha256=''),
                   dict(age_seconds=.31),dict(age_seconds=float('nan')),
                   dict(age_seconds=-1.),dict(travel_guard=False),
                   dict(operator_supervised=False),dict(scope='bounded_trial'),
                   dict(expected_scope='bounded_trial')]:
        assert not geometry_execution_allowed(**dict(fields, **update)),update


def test_bounded_geometry_cannot_become_a_field_allowance():
    from go2w_cmd_vel_bridge.bridge_core import geometry_execution_allowed
    fields=dict(scope='bounded_trial',expected_scope='bounded_trial',
        profile_sha256='a'*64,expected_sha256='a'*64,age_seconds=.01,
        trial_guard=True,travel_guard=False,operator_supervised=False)
    assert geometry_execution_allowed(**fields)
    assert not geometry_execution_allowed(**dict(fields,trial_guard=False,travel_guard=True,operator_supervised=True))
