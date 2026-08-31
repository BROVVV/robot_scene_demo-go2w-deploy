"""Deadman / disconnect safety tests (plan book §11 / §43)."""

from __future__ import annotations

from dataclasses import replace
from typing import Any, Callable

from app.manual_web_demo.config import ManualDemoSettings
from app.manual_web_demo.manual_drive_controller import ManualDriveController
from app.manual_web_demo.models import SafetySnapshot

from test_manual_drive_controller import FakeClock, FakeExecutor, make_safety


def config_with(**overrides) -> ManualDemoSettings:
    return replace(ManualDemoSettings(), **overrides)


def make_controller(
    *, config: ManualDemoSettings | None = None,
    safety: Callable[[], SafetySnapshot] | None = None,
    camera_fresh: Callable[[], bool] | None = None,
) -> tuple[ManualDriveController, FakeExecutor, FakeClock]:
    executor = FakeExecutor()
    clock = FakeClock()
    controller = ManualDriveController(
        executor=executor,
        config=config or config_with(),
        safety_provider=safety or make_safety(),
        camera_fresh_provider=camera_fresh or (lambda: True),
        clock=clock,
    )
    assert controller.enable()
    return controller, executor, clock


def test_fresh_heartbeat_keeps_control() -> None:
    controller, _, clock = make_controller()
    controller.on_heartbeat(["w"])
    clock.advance(0.20)
    controller.tick()
    assert controller.state.control_enabled is True


def test_stale_heartbeat_stops_and_disables() -> None:
    controller, executor, clock = make_controller()
    controller.on_heartbeat(["w"])
    clock.advance(0.40)  # > 300ms deadman
    controller.tick()
    assert controller.state.control_enabled is False
    assert controller.state.blocked_reason == "heartbeat_timeout"


def test_heartbeat_timeout_while_moving_stops_motion() -> None:
    controller, executor, clock = make_controller()
    controller.on_key_down("w")
    assert executor.pulses == ["forward"]
    controller.on_heartbeat(["w"])
    clock.advance(0.40)
    controller.tick()
    assert executor.stops >= 1
    assert controller.state.control_enabled is False


def test_websocket_disconnect_stops_and_disables() -> None:
    controller, executor, _ = make_controller()
    controller.on_key_down("w")
    controller.on_ws_disconnect()
    assert controller.state.control_enabled is False
    assert executor.stops >= 1
    assert controller.state.blocked_reason == "websocket_disconnected"


def test_release_all_stops_motion() -> None:
    controller, executor, _ = make_controller()
    controller.on_key_down("w")
    controller.on_release_all()
    assert executor.stops >= 1
    assert controller.state.command == "stop"


def test_camera_stale_auto_disables_control() -> None:
    fresh = {"value": True}

    def camera_fresh() -> bool:
        return fresh["value"]

    controller, executor, _ = make_controller(camera_fresh=camera_fresh)
    controller.on_heartbeat(["w"])
    fresh["value"] = False
    controller.tick()
    assert controller.state.control_enabled is False
    assert controller.state.blocked_reason == "camera_stale"
    # Camera recovers but motion needs explicit re-enable (plan §38).
    fresh["value"] = True
    controller.tick()
    assert controller.state.control_enabled is False


def test_enable_again_after_disconnect() -> None:
    controller, executor, _ = make_controller()
    controller.on_ws_disconnect()
    assert controller.enable()
    assert controller.state.control_enabled is True
