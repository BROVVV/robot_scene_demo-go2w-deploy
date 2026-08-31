"""Config tests for the manual web demo (plan book §35)."""

from __future__ import annotations

import pytest

from app.manual_web_demo.config import (
    ManualDemoSettings,
    get_manual_demo_settings,
)


def test_defaults_are_fail_closed() -> None:
    settings = ManualDemoSettings()
    assert settings.control_enabled_default is False
    assert settings.allow_backward is False
    assert settings.allow_strafe is False
    assert settings.allow_forward is True
    assert settings.allow_turn is True
    assert settings.deadman_ms == 300
    assert settings.ros_worker_deadman_ms == 500
    assert settings.turn_step_deg == 8.0
    # LLM analysis is OFF by default to save tokens.
    assert settings.llm_enabled is False


def test_derived_seconds() -> None:
    settings = ManualDemoSettings(deadman_ms=300, ros_worker_deadman_ms=500,
                                  repeat_interval_ms=250)
    assert settings.deadman_sec == 0.3
    assert settings.ros_worker_deadman_sec == 0.5
    assert settings.repeat_interval_sec == 0.25


def test_env_overrides(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("MANUAL_DEMO_PORT", "9000")
    monkeypatch.setenv("MANUAL_DEMO_ALLOW_STRAFE", "true")
    monkeypatch.setenv("MANUAL_DEMO_TURN_STEP_DEG", "10")
    monkeypatch.setenv("MANUAL_DEMO_DEADMAN_MS", "400")
    settings = get_manual_demo_settings()
    assert settings.port == 9000
    assert settings.allow_strafe is True
    assert settings.turn_step_deg == 10.0
    assert settings.deadman_ms == 400


def test_env_bool_tolerates_common_forms(monkeypatch: pytest.MonkeyPatch) -> None:
    from app.manual_web_demo import config as config_module

    for value, expected in (("true", True), ("1", True), ("yes", True),
                            ("on", True), ("no", False), ("0", False),
                            ("false", False), ("off", False), ("maybe", False)):
        monkeypatch.setenv("MANUAL_DEMO_TEST_BOOL", value)
        assert config_module._env_bool("MANUAL_DEMO_TEST_BOOL", False) is expected


def test_siliconflow_is_not_redefined() -> None:
    """The demo must not carry a second SiliconFlow configuration."""
    text = ManualDemoSettings.__module__
    assert text  # sanity
    import inspect

    fields = {name for name, _ in ManualDemoSettings.__dataclass_fields__.items()}  # type: ignore[attr-defined]
    assert not any("siliconflow" in name for name in fields)
    assert not any("api_key" in name for name in fields)


def test_hold_mode_params() -> None:
    settings = ManualDemoSettings()
    assert settings.hold_duration_sec == 30.0
    assert settings.turn_yaw_rate == 0.15
    assert settings.pulse_vx == 0.12
    assert settings.pulse_vy == 0.06
