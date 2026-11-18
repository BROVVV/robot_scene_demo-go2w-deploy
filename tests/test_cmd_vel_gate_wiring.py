"""Tests for the in-process cmd_vel gate wiring (scripts/go2w/cmd_vel_gate.py).

The gate is `bridge_core.decide_velocity`, which was already covered by its own
unit tests inside the ROS package. What is new here is the *wiring*: which of
the nine SafetyState inputs the search can honestly supply, what the resulting
verdict is on this robot, and the fact that the seam between the search's two
motion primitives and the gate's single Twist is translated rather than assumed.

The central assertion is `test_faithful_inputs_refuse_and_name_every_unsourced_input`:
fed the values the search actually has, the gate refuses, and the refusal is
exactly the four inputs this robot cannot observe. That is the measurement item
11 of the delivery report rests on, so it is pinned here.
"""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "scripts" / "go2w") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts" / "go2w"))

import pytest  # noqa: E402

from cmd_vel_gate import (  # noqa: E402
    COMMAND_AGE_SECONDS,
    GATE_DT_SECONDS,
    GATE_ENV_VAR,
    GATE_SOURCE,
    MODE_AUDIT,
    MODE_ENFORCE,
    MODE_OFF,
    MODES,
    SafetyState,
    Velocity,
    evaluate,
    requested_velocity_for_step,
    resolve_mode,
)


# The two SafetyState fields that have no in-process source on this robot at
# all.  They keep their fail-closed defaults, which is why the gate refuses.
#
# This list used to have four entries.  An adversarial review round found that
# `operator_armed` and `lease_alive` DO have in-process evidence on this host
# (the runner's own arm response, and /tmp/go2w_lease_status/alive); they are
# unwired by choice, not for want of a source.  See the module docstring.
UNSOURCED_BLOCKERS = ["emergency_stop", "remote_override"]


def _gate(safety, requested, command_age_seconds=COMMAND_AGE_SECONDS):
    return evaluate(
        requested=requested,
        previous=Velocity(),
        command_age_seconds=command_age_seconds,
        dt_seconds=GATE_DT_SECONDS,
        source=GATE_SOURCE,
        safety=safety,
    )


def _healthy_live_inputs():
    """The SafetyState the search hook builds from values it really has."""
    return SafetyState(
        execution_enabled=True,
        lidar_fresh=True,
        rotation_clearance_valid=False,
        robot_error_zero=True,
    )


def _all_inputs_supplied():
    return SafetyState(
        execution_enabled=True,
        operator_armed=True,
        lease_alive=True,
        lidar_fresh=True,
        rotation_clearance_valid=True,
        navigation_localization_fresh=True,
        robot_error_zero=True,
        emergency_stop=False,
        remote_override=False,
    )


def _all_inputs_supplied_except_the_unfillable_ones():
    """Every input that has any in-process evidence here, wired.

    The best case this robot could reach without inventing a source: the three
    the hook declines to wire are filled in, the two with no source at all stay
    fail-closed.
    """
    return SafetyState(
        execution_enabled=True,
        operator_armed=True,
        lease_alive=True,
        lidar_fresh=True,
        rotation_clearance_valid=True,
        navigation_localization_fresh=True,
        robot_error_zero=True,
    )


def test_modes_and_default_are_fail_safe():
    assert MODES == (MODE_OFF, MODE_AUDIT, MODE_ENFORCE)
    assert resolve_mode(None) == MODE_OFF
    assert resolve_mode("") == MODE_OFF
    assert resolve_mode(" AUDIT ") == MODE_AUDIT
    # A typo must not silently pick a different behaviour.
    assert resolve_mode("enforc") == MODE_OFF
    assert resolve_mode("true") == MODE_OFF
    assert GATE_ENV_VAR == "GO2W_CMD_VEL_GATE"


def test_source_is_not_nav2_so_the_localization_input_is_inert():
    # navigation_localization_fresh is unsourced here, and only matters for a
    # nav2 source.  Pinning the source keeps that reasoning true.
    assert GATE_SOURCE != "nav2"


def test_requested_velocity_translates_both_primitives():
    rate = 0.40
    forward = SimpleNamespace(vx=0.15, relative_yaw_deg=0.0)
    backward = SimpleNamespace(vx=-0.12, relative_yaw_deg=0.0)
    left = SimpleNamespace(vx=0.0, relative_yaw_deg=30.0)
    right = SimpleNamespace(vx=0.0, relative_yaw_deg=-30.0)

    assert requested_velocity_for_step("f0.15", forward, rate) == Velocity(0.15, 0.0)
    assert requested_velocity_for_step("b0.12", backward, rate) == Velocity(-0.12, 0.0)
    # A turn carries no rate, so the ceiling signed by direction is the only
    # honest angular_z -- and it must be non-zero, because that is what makes
    # the gate consult the rotation-clearance input at all.
    assert requested_velocity_for_step("l30", left, rate) == Velocity(0.0, rate)
    assert requested_velocity_for_step("r30", right, rate) == Velocity(0.0, -rate)


def test_faithful_inputs_refuse_and_name_every_unsourced_input():
    record = _gate(_healthy_live_inputs(), Velocity(0.15, 0.0))

    assert record["allowed"] is False
    # The hook leaves four fields at their fail-closed defaults, but only two of
    # them are unfillable here; the other two are unwired by choice.
    assert record["blockers"] == [
        "operator_not_armed",
        "lease_unavailable",
        *UNSOURCED_BLOCKERS,
    ]
    assert record["reason"] == ",".join(record["blockers"])
    # As a dispatcher the node would cancel whatever goal is active.  That is
    # why the in-process wiring exists instead of a second ROS client.
    assert record["cancel_active_action"] is True


def test_the_refusal_survives_even_if_every_available_source_is_wired():
    # The headline is "enforce refuses every command on this robot".  That must
    # not rest on the two fields we chose not to wire: even filling in all three
    # in-process sources the review found -- operator_armed from the runner's own
    # arm response, lease_alive from /tmp/go2w_lease_status/alive,
    # navigation_localization_fresh from /go2w/nav/localization_valid -- leaves
    # the gate refusing, on the two fields with
    # no source at all.
    record = _gate(_all_inputs_supplied_except_the_unfillable_ones(), Velocity(0.15, 0.0))

    assert record["allowed"] is False
    assert record["blockers"] == UNSOURCED_BLOCKERS
    # A turn too (clearance is valid in this helper, so the rotation input does
    # not add anything -- the two unfillable ones carry the refusal alone).
    turn = _gate(_all_inputs_supplied_except_the_unfillable_ones(), Velocity(0.0, 0.40))
    assert turn["blockers"] == UNSOURCED_BLOCKERS


def test_the_record_says_which_input_the_hook_wrote_without_a_source():
    # The hook hardcodes execution_enabled=True in every mode, so
    # `execution_disabled` can never appear here, while the report's own
    # all-defaults table lists it first.  The two are not comparable, and the
    # record has to admit that rather than read as nine measurements.
    empty = _gate(SafetyState(), Velocity(0.15, 0.0))
    assert empty["blockers"][0] == "execution_disabled"

    live = _gate(_healthy_live_inputs(), Velocity(0.15, 0.0))
    assert live["execution_enabled"] is True
    assert "execution_disabled" not in live["blockers"]


def test_a_turn_additionally_trips_the_rotation_clearance_input():
    straight = _gate(_healthy_live_inputs(), Velocity(0.15, 0.0))
    turning = _gate(_healthy_live_inputs(), Velocity(0.0, 0.40))

    assert "rotation_clearance_unvalidated" not in straight["blockers"]
    assert turning["blockers"] == [
        "operator_not_armed",
        "lease_unavailable",
        "rotation_clearance_unvalidated",
        "emergency_stop",
        "remote_override",
    ]


def test_a_translated_turn_reaches_the_clearance_blocker():
    # The seam itself, i.e. the values _execute_step actually hands over.  A
    # translation that returned a zero rate would silently skip the rotation
    # input without any other test noticing.
    goal = SimpleNamespace(vx=0.0, relative_yaw_deg=30.0)
    requested = requested_velocity_for_step("l30", goal, 0.40)
    record = _gate(_healthy_live_inputs(), requested)

    assert requested == Velocity(0.0, 0.40)
    assert "rotation_clearance_unvalidated" in record["blockers"]


def test_the_watchdog_input_is_reachable_but_never_fires_in_process():
    # The constant is what keeps it quiet, not a dead branch: with a stale age
    # the blocker does appear.
    record = _gate(_healthy_live_inputs(), Velocity(0.15, 0.0), command_age_seconds=1.0)
    assert "command_watchdog_expired" in record["blockers"]
    assert COMMAND_AGE_SECONDS == 0.0


def test_gate_allows_when_all_nine_inputs_are_supplied():
    # Proves the refusal above is caused by the inputs, not by a gate that can
    # only ever refuse.
    record = _gate(_all_inputs_supplied(), Velocity(0.15, 0.0))

    assert record["allowed"] is True
    assert record["reason"] == "allowed"
    assert record["blockers"] == []
    assert record["cancel_active_action"] is False


def test_allowed_shaping_uses_the_supplied_dt_and_limits():
    record = _gate(_all_inputs_supplied(), Velocity(0.15, 0.0))

    # From rest, one tick of the bridge node's acceleration limit: 0.20 * 0.05.
    assert record["shaped"]["linear_x"] == pytest.approx(0.01)
    assert record["shaped"]["angular_z"] == pytest.approx(0.0)
