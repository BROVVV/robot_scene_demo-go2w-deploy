"""ControlOwner tests (plan book §97, §100: test_control_ownership)."""

from __future__ import annotations

from app.manual_web_demo.control_ownership import ControlOwner, OwnerState


def test_starts_none() -> None:
    owner = ControlOwner()
    assert owner.state() == OwnerState.NONE


def test_manual_take_and_release() -> None:
    owner = ControlOwner()
    ok, _ = owner.try_manual()
    assert ok
    assert owner.is_manual()
    owner.release(OwnerState.MANUAL)
    assert owner.state() == OwnerState.NONE


def test_autonomous_blocks_manual_and_vice_versa() -> None:
    owner = ControlOwner()
    ok, _ = owner.try_autonomous(detail="search_1")
    assert ok
    ok, reason = owner.try_manual()
    assert not ok
    assert "autonomous" in reason
    owner.release(OwnerState.AUTONOMOUS)
    ok, _ = owner.try_manual()
    assert ok
    ok, reason = owner.try_autonomous()
    assert not ok
    assert "manual" in reason


def test_estop_overrides_all_and_latches() -> None:
    owner = ControlOwner()
    owner.try_autonomous()
    owner.estop()
    assert owner.is_estop()
    ok, reason = owner.try_manual()
    assert not ok
    assert reason == "emergency_stop_latched"
    ok, reason = owner.try_autonomous()
    assert not ok
    assert reason == "emergency_stop_latched"
    # release() must never silently lift the estop latch
    owner.release(OwnerState.AUTONOMOUS)
    assert owner.is_estop()


def test_explicit_estop_reset_releases_latch() -> None:
    owner = ControlOwner()
    owner.estop()
    ok, reason = owner.reset_estop()
    assert ok
    assert reason == ""
    assert owner.state() == OwnerState.NONE
    assert owner.try_autonomous()[0] is True


def test_release_only_frees_matching_owner() -> None:
    owner = ControlOwner()
    owner.try_autonomous(detail="search")
    owner.release(OwnerState.MANUAL)
    assert owner.is_autonomous()
    owner.release(OwnerState.AUTONOMOUS)
    assert owner.state() == OwnerState.NONE


def test_snapshot() -> None:
    owner = ControlOwner()
    owner.try_autonomous(detail="autonomous_search")
    snapshot = owner.snapshot()
    assert snapshot["owner"] == "AUTONOMOUS"
    assert snapshot["detail"] == "autonomous_search"
