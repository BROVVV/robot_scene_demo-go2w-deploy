"""Tests for the operator-confirmed current hardware geometry and state."""

from __future__ import annotations

from pathlib import Path

import pytest

from app.live_robot.current_hardware import (
    HardwareConfigError,
    geometry_hash,
    load_current_hardware_geometry,
    load_current_hardware_state,
    load_geometry_and_state,
    state_hash,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
GEOMETRY_PATH = PROJECT_ROOT / "configs/go2w/current_hardware_geometry.yaml"
STATE_PATH = PROJECT_ROOT / "configs/go2w/current_hardware_state.yaml"


def test_geometry_confirmed_dimensions() -> None:
    geometry = load_current_hardware_geometry(GEOMETRY_PATH)
    assert geometry.length_m == pytest.approx(0.70)
    assert geometry.width_m == pytest.approx(0.43)
    assert geometry.height_m == pytest.approx(0.70)
    assert geometry.highest_point == "pandarxt16_protective_frame"
    assert geometry.highest_point_fixed_structure is True
    assert geometry.highest_point_is_loose_cable is False


def test_geometry_no_remeasurement_required() -> None:
    geometry = load_current_hardware_geometry(GEOMETRY_PATH)
    assert geometry.remeasurement_required == {
        "length": False,
        "width": False,
        "height": False,
    }


def test_geometry_never_authorizes_motion() -> None:
    geometry = load_current_hardware_geometry(GEOMETRY_PATH)
    assert geometry.authorizes_motion is False


def test_geometry_footprint_half_diagonal() -> None:
    geometry = load_current_hardware_geometry(GEOMETRY_PATH)
    expected = (0.70 ** 2 + 0.43 ** 2) ** 0.5 / 2.0
    assert geometry.horizontal_footprint_half_diagonal_m == pytest.approx(expected)


def test_state_manifest_binds_configs() -> None:
    state = load_current_hardware_state(STATE_PATH)
    assert state.builtin_lidar_present is True
    assert state.pandar_present is True
    assert state.height_m == pytest.approx(0.70)
    assert state.highest_point == "pandarxt16_protective_frame"
    assert state.mount_changed_since_calibration is False
    assert state.motion_authorization == {"rotation": False, "forward": False}


def test_geometry_and_state_load_together() -> None:
    bundle = load_geometry_and_state()
    assert bundle["geometry_hash"]
    assert bundle["state_hash"]
    assert bundle["geometry"].authorizes_motion is False


def test_hashes_are_stable_and_distinct() -> None:
    geometry = load_current_hardware_geometry(GEOMETRY_PATH)
    state = load_current_hardware_state(STATE_PATH)
    first = geometry_hash(geometry)
    assert first == geometry_hash(geometry)  # stable
    assert first != state_hash(state)  # distinct


def test_geometry_rejects_wrong_dimensions(tmp_path) -> None:
    path = tmp_path / "bad_geometry.yaml"
    text = GEOMETRY_PATH.read_text(encoding="utf-8").replace(
        "height: 0.70", "height: 0.50"
    )
    path.write_text(text, encoding="utf-8")
    with pytest.raises(HardwareConfigError, match="dimensions_m.height"):
        load_current_hardware_geometry(path)


def test_geometry_rejects_loose_cable_highest_point(tmp_path) -> None:
    path = tmp_path / "bad_geometry.yaml"
    text = GEOMETRY_PATH.read_text(encoding="utf-8").replace(
        "type: pandarxt16_protective_frame", "type: loose_cable_loop"
    )
    path.write_text(text, encoding="utf-8")
    with pytest.raises(HardwareConfigError, match="highest_point"):
        load_current_hardware_geometry(path)


def test_geometry_rejects_authorizes_motion(tmp_path) -> None:
    path = tmp_path / "bad_geometry.yaml"
    text = GEOMETRY_PATH.read_text(encoding="utf-8").replace(
        "authorizes_motion: false", "authorizes_motion: true"
    )
    path.write_text(text, encoding="utf-8")
    with pytest.raises(HardwareConfigError, match="never authorizes motion"):
        load_current_hardware_geometry(path)


def test_state_rejects_mount_change(tmp_path) -> None:
    path = tmp_path / "bad_state.yaml"
    text = STATE_PATH.read_text(encoding="utf-8").replace(
        "mount_changed_since_calibration: false",
        "mount_changed_since_calibration: true",
    )
    path.write_text(text, encoding="utf-8")
    # A mount change is a legitimate operator signal, so the loader must accept
    # it but the resulting state invalidates any old lease via its hash.
    state = load_current_hardware_state(path)
    assert state.mount_changed_since_calibration is True
    assert state_hash(state) != state_hash(load_current_hardware_state(STATE_PATH))
