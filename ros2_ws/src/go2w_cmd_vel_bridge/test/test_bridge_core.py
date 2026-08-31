from go2w_cmd_vel_bridge.bridge_core import Limits, SafetyState, Velocity, decide_velocity


def safe_state(**updates):
    values = {
        "execution_enabled": True,
        "operator_armed": True,
        "lease_alive": True,
        "lidar_fresh": True,
        "rotation_clearance_valid": True,
        "lio_fresh": True,
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


def test_watchdog_and_nav2_lio_gate():
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
        safety=safe_state(lio_fresh=False),
    )
    assert not nav2.allowed and "lio_stale" in nav2.reason


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
