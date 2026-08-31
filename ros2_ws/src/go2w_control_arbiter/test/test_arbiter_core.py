from go2w_control_arbiter.arbiter_core import CommandSample, Twist2D, select_command


def test_unverified_remote_and_disarmed_states_are_zero():
    commands = {"manual": CommandSample(Twist2D(0.1, 0.0, 0.0), 1.0)}
    remote = select_command(
        commands,
        now=1.1,
        timeout=0.3,
        software_armed=True,
        emergency_stop=False,
        remote_override=True,
    )
    assert remote.command.is_zero
    assert remote.source == "remote_override"
    disarmed = select_command(
        commands,
        now=1.1,
        timeout=0.3,
        software_armed=False,
        emergency_stop=False,
        remote_override=False,
    )
    assert disarmed.command.is_zero


def test_priority_and_linear_y_contract():
    commands = {
        "search": CommandSample(Twist2D(0.1, 0.2, 0.0), 10.0),
        "nav2": CommandSample(Twist2D(0.05, 0.1, 0.1), 10.0),
        "manual": CommandSample(Twist2D(-0.05, 0.3, -0.1), 10.0),
    }
    selected = select_command(
        commands,
        now=10.1,
        timeout=0.3,
        software_armed=True,
        emergency_stop=False,
        remote_override=False,
    )
    assert selected.source == "manual"
    assert selected.command == Twist2D(-0.05, 0.0, -0.1)
    assert selected.blocked_reason == "linear_y_forced_zero"


def test_emergency_and_watchdog_always_publish_zero():
    commands = {"nav2": CommandSample(Twist2D(0.1, 0.0, 0.1), 1.0)}
    emergency = select_command(
        commands,
        now=1.1,
        timeout=0.3,
        software_armed=True,
        emergency_stop=True,
        remote_override=False,
    )
    assert emergency.command.is_zero
    stale = select_command(
        commands,
        now=2.0,
        timeout=0.3,
        software_armed=True,
        emergency_stop=False,
        remote_override=False,
    )
    assert stale.command.is_zero
    assert stale.source == "watchdog"
