#!/usr/bin/env python3
"""Put `bridge_core.decide_velocity` on the search's real motion path, in process.

Why in process, not as a ROS node
--------------------------------
`go2w_cmd_vel_bridge` is on no live path today.  `/go2w/cmd_vel_selected`,
`/go2w/control_source` and `/go2w/search_cmd_vel` have no publisher anywhere
(measured: all three are unknown topics), so the whole cmd_vel chain -- arbiter
plus bridge -- is dark.  Beyond that, the node as shipped cannot cancel anything:
`bridge_node._cancel_active` returns while its own `_goal_handle` is None, and
that handle is only ever set by a goal the node itself dispatched, which needs
`decision.allowed` -- unreachable from this repo's wiring, since most of the
eight Bool inputs have no publisher and `execution_enabled` defaults False (in
which case no ActionClient is constructed at all).  So "start the node as a
second `/go2w/motion` client" would today add an inert 20 Hz status publisher.

The reason to wire in process anyway is the *shape* of the coupling, not today's
inertness: a dispatcher and the search would share one owner-blind cancel path.
Any accepted `/go2w/motion` goal can be cancelled by a socket `{'op':'cancel'}`
from either side, the local gateway has a single active slot, and its refusal
message is not one the search's reverse trigger filters -- so contention can end
in a real `b0.12`.  Running the gate in the search's own process keeps one owner:
no new ROS endpoint, no second client, no arbitration to lose.

What the gate can and cannot see here
-------------------------------------
Four fields are wired from what the search already computed:

  execution_enabled          hardcoded True -- see the note below, this one is a
                             substitution, not a reading
  lidar_fresh                `self._lidar_fresh`
  rotation_clearance_valid   `rotation_inputs.valid`, already resolved upstream
  robot_error_zero           `self._sport` mode/error_code

Three more *do* have in-process evidence on this host, and are deliberately NOT
wired -- each because trusting it is a different claim from the field's name:

  operator_armed   the runner's own `_arm(True)` response is the action server's
                   latched `armed_` going true, and it is the same latch the
                   server reads for motion authorisation.  But that is "this
                   process armed the robot", not "an operator authorised this
                   motion"; wiring it would let the search authorise itself.
  lease_alive      `/tmp/go2w_lease_status/alive` is live on this host (written
                   by `hold_sport_lease.py`, bridged to `/go2w/sport_lease/alive`
                   by `lease_status_bridge.py`), and the search shares that
                   filesystem.  Not wired because the search's own lease is a
                   *rotation* lease -- a different object from the sport lease
                   this field names -- so the two would agree by coincidence of
                   deployment, not by meaning.
  navigation_localization_fresh
                   §6.5 renamed this from `lio_fresh` and repointed it at
                   `/go2w/nav/localization_valid`, published by the
                   `plain_slam_nav_frame_bridge`.  Inert here regardless:
                   `decide_velocity` consults it only when the source is "nav2"
                   and this gate's source is not.

Two have no source at all, and keep their fail-closed defaults:

  emergency_stop   `/go2w/emergency_stop` is a Trigger, not a latched state.
  remote_override  the only candidate is the physical remote's stream, and it is
                   silent -- silence cannot be told apart from "remote off".

So a faithfully fed gate reports two of these as blockers, not four.  Feeding the
three unwired ones too would leave `emergency_stop` and `remote_override`, so
`enforce` refuses every command either way -- see
`test_the_refusal_survives_even_if_every_available_source_is_wired`.  The
measured verdicts of the three real-robot runs are in §16 of the delivery report.

`execution_enabled` is the one field this hook writes without a source.  In the
node it gates whether the node does anything at all, and it defaults False; the
hook is the executor, so it passes True in *every* mode, not only in `audit`.  It
is written into each record so the substitution is visible rather than implied.
Consequence worth knowing: `execution_disabled` can therefore never appear in a
record from this hook, while §10.10's table (same `decide_velocity`, all defaults)
lists it first -- the two blocker lists are not directly comparable.

The two non-`SafetyState` arguments are supplied honestly rather than assumed:
`command_age_seconds` is 0 because the command is built and sent in one call
(nothing can go stale between them), and `dt_seconds` is the bridge node's own
tick so the shaped velocity is comparable to what the node would compute.

Modes
-----
`off`      no gate (default; behaviour is byte-for-byte what it was before).
`audit`    compute and record the verdict for every command; never block.
`enforce`  refuse the command when the verdict is not allowed.

`off` is the default because `enforce` would stop the robot from ever moving, and
because with the switch unset this file changes nothing about the real path --
`GO2W_CMD_VEL_GATE` is read here and nowhere else in the repo.
"""

from __future__ import annotations

import sys
from pathlib import Path

_BRIDGE_CORE_DIR = (
    Path(__file__).resolve().parents[2] / "ros2_ws" / "src" / "go2w_cmd_vel_bridge"
)
if str(_BRIDGE_CORE_DIR) not in sys.path:
    sys.path.insert(0, str(_BRIDGE_CORE_DIR))

from go2w_cmd_vel_bridge.bridge_core import (  # noqa: E402
    Limits,
    SafetyState,
    Velocity,
    decide_velocity,
)

MODE_OFF = "off"
MODE_AUDIT = "audit"
MODE_ENFORCE = "enforce"
MODES = (MODE_OFF, MODE_AUDIT, MODE_ENFORCE)

# Read by the search at construction, so a single run can be switched without
# going through the WebUI's parameter plumbing.
GATE_ENV_VAR = "GO2W_CMD_VEL_GATE"


def resolve_mode(raw: str | None) -> str:
    """Map the environment value to a mode, defaulting to `off` when unusable.

    Anything unrecognised disables the gate rather than guessing: an operator
    who typos the mode should get the previous behaviour and a warning, never a
    silently different one.
    """
    if raw is None:
        return MODE_OFF
    mode = raw.strip().lower()
    return mode if mode in MODES else MODE_OFF

# The search is not a nav2 client, so `navigation_localization_stale` is not
# consulted.  The value has to be a real member of the source domain the arbiter
# publishes (`arbiter_core.PRIORITY` plus its abort states) -- §6.5 taught
# `decide_velocity` to refuse a source it does not recognise, and a locally
# invented label like "semantic_search" would now be refused for that reason
# instead of for the reason under test.
GATE_SOURCE = "search"

# The in-process gate has no command queue: the search builds a command and
# sends it in the same call, so the command being judged is brand new.  The
# node's watchdog exists because it reads a topic that can go stale; here there
# is nothing to go stale, and `command_watchdog_expired` cannot fire.
COMMAND_AGE_SECONDS = 0.0

# bridge_node's own tick.  Used only for the acceleration shaping, so that the
# shaped velocity is comparable to what the node would have computed.
GATE_DT_SECONDS = 0.05


def requested_velocity_for_step(step: str, goal, max_yaw_rate: float) -> Velocity:
    """Translate one search command into the Twist the gate reasons about.

    The two sides do not line up, and this is the seam.  The search has two
    motion primitives; the bridge reasons about a Twist.

      `f` / `b`  timed velocity -- the goal's `vx` *is* the requested
                 `linear_x`, and `angular_z` is zero.
      `l` / `r`  a *relative yaw* command.  It carries no rate at all: the
                 action server picks the rate, bounded by `max_yaw_rate`.  So
                 the only honest `angular_z` is that ceiling, signed by the turn
                 direction.  The gate needs no more than that, because it keys
                 the rotation-clearance blocker on whether a rotation was
                 requested at all (`abs(requested.angular_z) > 1e-9`), not on
                 how fast it is.
    """
    if step.startswith("l") or step.startswith("r"):
        degrees = float(goal.relative_yaw_deg)
        rate = max_yaw_rate if degrees >= 0.0 else -max_yaw_rate
        return Velocity(0.0, rate)
    return Velocity(float(goal.vx), 0.0)


def evaluate(
    *,
    requested: Velocity,
    previous: Velocity,
    command_age_seconds: float,
    dt_seconds: float,
    source: str,
    safety: SafetyState,
    limits: Limits = Limits(),
) -> dict:
    """Judge one command and return a JSON-able record of the verdict.

    `blockers` is the reason string split back into the individual codes, which
    is what makes the record readable in the event log; the gate itself returns
    them joined. On the allow path `reason` is the word "allowed", so `blockers`
    is emptied rather than left holding that word as a bogus code.

    Note the coupling: `BridgeDecision` carries no blocker list of its own, so
    this re-derives one by parsing a comma-joined string owned by `bridge_core`.
    All eleven names in `bridge_core` are comma-free today, which is the only
    reason the split round-trips; if a blocker ever gains a comma or any
    human-readable text, this silently inflates the count instead of failing.
    """
    decision = decide_velocity(
        requested,
        previous,
        command_age_seconds=command_age_seconds,
        dt_seconds=dt_seconds,
        source=source,
        safety=safety,
        limits=limits,
    )
    return {
        "allowed": decision.allowed,
        "reason": decision.reason,
        "blockers": (
            [] if decision.allowed else [c for c in decision.reason.split(",") if c]
        ),
        "shaped": {
            "linear_x": decision.velocity.linear_x,
            "angular_z": decision.velocity.angular_z,
        },
        # What the node would do as a dispatcher.  Reported here because it is
        # the field that makes the ROS-node wiring dangerous, and the experiment
        # should show it rather than assert it.
        "cancel_active_action": decision.cancel_active_action,
        # The one input this hook writes without a source, echoed so a record is
        # readable on its own: it is hardcoded True in every mode, which is why
        # `execution_disabled` never appears here even though §10.10's
        # all-defaults table (same `decide_velocity`) lists it first.  Without
        # this the record reads as if all nine inputs were measurements, and the
        # two blocker lists look comparable when they are not.
        "execution_enabled": safety.execution_enabled,
    }
