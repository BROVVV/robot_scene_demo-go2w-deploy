"""Keyboard state-machine tests for the manual drive controller (plan §42)."""

from __future__ import annotations

from dataclasses import replace
from typing import Any, Callable

import pytest

from app.manual_web_demo.config import ManualDemoSettings
from app.manual_web_demo.manual_drive_controller import ManualDriveController
from app.manual_web_demo.models import SafetySnapshot


class FakeClock:
    def __init__(self) -> None:
        self.now = 1000.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class FakeExecutor:
    def __init__(self, available: bool = True) -> None:
        self.available_flag = available
        self.pulses: list[str] = []
        self.stops = 0
        self.estops = 0
        self._stop_requested = False
        self._on_result: Callable[[dict[str, Any]], None] | None = None

    def available(self) -> bool:
        return self.available_flag

    def send_pulse(self, direction: str, on_result: Callable[[dict[str, Any]], None]) -> None:
        self.pulses.append(direction)
        self._stop_requested = False
        self._on_result = on_result

    def stop(self) -> None:
        self.stops += 1
        self._stop_requested = True

    def estop(self) -> None:
        self.estops += 1

    def complete(self, result: dict[str, Any] | None = None) -> None:
        callback = self._on_result
        self._on_result = None
        if callback is None:
            return
        if result is not None:
            callback(result)
            return
        direction = self.pulses[-1] if self.pulses else None
        if self._stop_requested:
            # Model a cancel produced by STOP: success=False, error_code=canceled.
            callback(
                {
                    "success": False,
                    "direction": direction,
                    "error_code": "canceled",
                    "message": "goal canceled",
                    "elapsed_sec": 0.5,
                }
            )
        else:
            # Model a successful hold goal that ran its full duration.
            callback(
                {
                    "success": True,
                    "direction": direction,
                    "error_code": "none",
                    "message": "motion completed",
                    "elapsed_sec": 30.0,
                }
            )


def make_safety(**overrides) -> Callable[[], SafetySnapshot]:
    snapshot = SafetySnapshot(
        lidar_fresh=True,
        front_clearance_m=1.0,
        rotation_clearance_valid=True,
    )
    snapshot = replace(snapshot, **overrides)
    return lambda: snapshot


def make_controller(
    *,
    executor: FakeExecutor | None = None,
    config: ManualDemoSettings | None = None,
    safety: Callable[[], SafetySnapshot] | None = None,
    camera_fresh: Callable[[], bool] | None = None,
    clock: FakeClock | None = None,
) -> tuple[ManualDriveController, FakeExecutor, FakeClock]:
    executor = executor or FakeExecutor()
    clock = clock or FakeClock()
    config = config or ManualDemoSettings()
    controller = ManualDriveController(
        executor=executor,
        config=config,
        safety_provider=safety or make_safety(),
        camera_fresh_provider=camera_fresh or (lambda: True),
        clock=clock,
    )
    assert controller.enable()
    return controller, executor, clock


def config_with(**overrides) -> ManualDemoSettings:
    return replace(ManualDemoSettings(), **overrides)


# ---------------------------------------------------------------------- #
# Single tap / hold                                                       #
# ---------------------------------------------------------------------- #
def test_keydown_w_sends_one_forward_pulse() -> None:
    controller, executor, _ = make_controller()
    controller.on_key_down("w")
    assert executor.pulses == ["forward"]
    executor.complete()


def test_w_held_renews_continuous_goal() -> None:
    """Continuous hold: while W is held, a completed goal is renewed immediately."""
    controller, executor, _ = make_controller()
    controller.on_key_down("w")
    assert executor.pulses == ["forward"]
    executor.complete()  # goal ended at duration boundary; key still held
    assert executor.pulses == ["forward", "forward"]
    executor.complete()
    assert executor.pulses == ["forward", "forward", "forward"]


def test_keyup_w_stops_further_pulses() -> None:
    controller, executor, clock = make_controller()
    controller.on_key_down("w")
    assert executor.pulses == ["forward"]
    controller.on_key_up("w")  # release before the goal completes
    assert executor.stops >= 1
    executor.complete()  # canceled; key not held -> no renewal
    clock.advance(0.40)
    controller.tick()
    assert executor.pulses == ["forward"]


def test_keyup_while_motion_in_flight_requests_stop() -> None:
    controller, executor, _ = make_controller()
    controller.on_key_down("w")
    assert executor.pulses == ["forward"]
    controller.on_key_up("w")
    assert executor.stops >= 1
    executor.complete()


def test_hold_renews_immediately_no_interval_wait() -> None:
    """Continuous hold: no repeat-interval gate, renewal is immediate."""
    controller, executor, clock = make_controller()
    controller.on_key_down("w")
    executor.complete()
    clock.advance(0.05)  # far below the old 250ms interval
    controller.tick()
    assert executor.pulses == ["forward", "forward"]


def test_no_goal_stacking_under_fast_heartbeats() -> None:
    """Plan §44: even with 100ms heartbeats, never more than one goal in flight."""
    controller, executor, clock = make_controller()
    controller.on_key_down("w")
    # Fire many heartbeats/ticks while the first goal is still in flight.
    for _ in range(20):
        clock.advance(0.1)
        controller.on_heartbeat(["w"])
        controller.tick()
    assert executor.pulses == ["forward"]  # exactly one goal started
    executor.complete()
    assert executor.pulses == ["forward", "forward"]  # renewed sequentially


# ---------------------------------------------------------------------- #
# Direction switching                                                     #
# ---------------------------------------------------------------------- #
def test_w_then_a_stops_forward_before_strafe_left() -> None:
    controller, executor, clock = make_controller(
        config=config_with(allow_strafe=True)
    )
    controller.on_key_down("w")
    assert executor.pulses == ["forward"]
    controller.on_key_down("a")
    assert executor.stops >= 1  # cancel W before strafe-left
    executor.complete()
    clock.advance(0.30)
    controller.tick()
    assert executor.pulses == ["forward", "strafe_left"]


def test_a_then_q_stops_strafe_before_turn_left() -> None:
    controller, executor, clock = make_controller(
        config=config_with(allow_strafe=True, allow_turn=True)
    )
    controller.on_key_down("a")
    assert executor.pulses == ["strafe_left"]
    controller.on_key_down("q")
    assert executor.stops >= 1
    executor.complete()
    clock.advance(0.30)
    controller.tick()
    assert executor.pulses == ["strafe_left", "turn_left"]


# ---------------------------------------------------------------------- #
# Conflict pairs resolve to STOP                                          #
# ---------------------------------------------------------------------- #
def test_w_plus_s_conflict_stops() -> None:
    controller, executor, _ = make_controller()
    controller.on_key_down("w")
    controller.on_key_down("s")
    assert executor.stops >= 1
    assert controller.state.command == "stop"
    executor.complete()
    assert executor.pulses == ["forward"]


def test_a_plus_d_conflict_stops() -> None:
    controller, executor, _ = make_controller(
        config=config_with(allow_strafe=True)
    )
    controller.on_key_down("a")
    controller.on_key_down("d")
    assert executor.stops >= 1
    assert controller.state.command == "stop"


def test_q_plus_e_conflict_stops() -> None:
    controller, executor, _ = make_controller()  # allow_turn default True
    controller.on_key_down("q")
    controller.on_key_down("e")
    assert executor.stops >= 1
    assert controller.state.command == "stop"


# ---------------------------------------------------------------------- #
# Gates                                                                   #
# ---------------------------------------------------------------------- #
def test_forward_blocked_when_lidar_not_fresh() -> None:
    controller, executor, _ = make_controller(
        safety=make_safety(lidar_fresh=False)
    )
    controller.on_key_down("w")
    assert executor.pulses == []
    assert controller.state.blocked_reason is not None
    assert controller.state.status == "BLOCKED"


def test_forward_blocked_when_front_clearance_too_small() -> None:
    controller, executor, _ = make_controller(
        safety=make_safety(front_clearance_m=0.05),
        config=config_with(min_front_clearance_m=0.30),
    )
    controller.on_key_down("w")
    assert executor.pulses == []


def test_a_d_blocked_when_strafe_disabled() -> None:
    controller, executor, _ = make_controller()  # allow_strafe default False
    controller.on_key_down("a")
    assert executor.pulses == []
    assert "lateral_motion_not_supported" in (controller.state.blocked_reason or "")
    controller.on_key_up("a")


def test_q_e_blocked_when_rotation_clearance_invalid() -> None:
    controller, executor, _ = make_controller(
        safety=make_safety(rotation_clearance_valid=False)
    )
    controller.on_key_down("q")
    assert executor.pulses == []
    assert "rotation_clearance_invalid" in (controller.state.blocked_reason or "")


def test_s_blocked_when_backward_disabled() -> None:
    controller, executor, _ = make_controller()  # allow_backward default False
    controller.on_key_down("s")
    assert executor.pulses == []
    assert controller.state.blocked_reason == "backward not enabled: rear safety not validated"


def test_camera_stale_blocks_enable() -> None:
    executor = FakeExecutor()
    controller = ManualDriveController(
        executor=executor,
        config=config_with(),
        safety_provider=make_safety(),
        camera_fresh_provider=lambda: False,
    )
    assert controller.enable() is False
    assert controller.state.blocked_reason == "camera_stale"


# ---------------------------------------------------------------------- #
# ESTOP / enable-offline                                                  #
# ---------------------------------------------------------------------- #
def test_enable_refused_when_motion_offline() -> None:
    controller = ManualDriveController(
        executor=FakeExecutor(available=False),
        config=config_with(),
        safety_provider=make_safety(),
        camera_fresh_provider=lambda: True,
    )
    assert controller.enable() is False
    assert controller.state.blocked_reason == "motion_action_offline"


def test_estop_latches_and_disables_control() -> None:
    controller, executor, _ = make_controller()
    controller.on_key_down("w")
    controller.estop()
    assert controller.state.status == "ESTOP"
    assert controller.state.control_enabled is False
    assert executor.estops >= 1
    # Re-enable requires explicit user action; a stale enable is refused.
    assert controller.enable() is False
