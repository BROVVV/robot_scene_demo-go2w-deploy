# Copyright 2026 robot_scene_demo maintainers

"""Unit tests for the automated imu_to_lidar extrinsic derivation.

Covers plan §18.1: the generator must derive T_imu_pandar from the repository
fixtures (never hard-code it), must match the plan's sanity reference within a
small tolerance, and must never change the Pandar safety authorization facts.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_GO2W = PROJECT_ROOT / "scripts" / "go2w"
if str(SCRIPTS_GO2W) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_GO2W))

import generate_plain_slam_pandar_config as gen  # noqa: E402

REF_TRANSLATION = gen.REFERENCE_IMU_TO_PANDAR_TRANSLATION
REF_ROTATION = gen.REFERENCE_IMU_TO_PANDAR_ROTATION


def test_derived_translation_matches_plan_reference() -> None:
    result = gen.derive_imu_to_pandar()
    translation = result["derived_imu_to_pandar"]["translation"]
    assert len(translation) == 3
    for derived, reference in zip(translation, REF_TRANSLATION):
        assert abs(float(derived) - reference) < 1e-3, (
            f"translation {translation} deviates from plan reference {REF_TRANSLATION}"
        )


def test_derived_rotation_matches_plan_reference() -> None:
    result = gen.derive_imu_to_pandar()
    rotation = result["derived_imu_to_pandar"]["rotation_matrix"]
    assert len(rotation) == 9
    for derived, reference in zip(rotation, REF_ROTATION):
        assert abs(float(derived) - reference) < 1e-3, (
            f"rotation deviates from plan reference around element {reference}"
        )


def test_derived_matrix_is_valid_rotation() -> None:
    result = gen.derive_imu_to_pandar()
    r = [float(v) for v in result["derived_imu_to_pandar"]["rotation_matrix"]]
    # R^T @ R == I within tolerance.
    for i in range(3):
        for j in range(3):
            dot = sum(r[i * 3 + k] * r[j * 3 + k] for k in range(3))
            expected = 1.0 if i == j else 0.0
            assert abs(dot - expected) < 1e-8
    # Determinant == 1 (right-handed).
    det = (
        r[0] * (r[4] * r[8] - r[5] * r[7])
        - r[1] * (r[3] * r[8] - r[5] * r[6])
        + r[2] * (r[3] * r[7] - r[4] * r[6])
    )
    assert abs(det - 1.0) < 1e-8


def test_safety_authorizations_never_promoted() -> None:
    result = gen.derive_imu_to_pandar()
    status = result["status"]
    assert status["calibration_status"] == "candidate_unconfirmed"
    assert status["confirmed"] is False
    assert status["authorizes_tf_publication"] is False
    assert status["authorizes_safety_integration"] is False
    assert status["authorizes_motion"] is False


def test_source_files_bytes_unchanged_by_derivation() -> None:
    """The derivation must be read-only with respect to the source YAML."""
    before = {
        path: path.read_bytes() for path in gen.SOURCES.values() if path.exists()
    }
    gen.derive_imu_to_pandar()
    for path, content in before.items():
        assert path.read_bytes() == content, f"generator modified source {path}"


def test_check_mode_exits_zero(monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]) -> None:
    monkeypatch.setattr(sys, "argv", ["generate_plain_slam_pandar_config.py", "--check"])
    assert gen.main() == 0
    captured = capsys.readouterr().out
    assert "Pandar extrinsic: candidate_unconfirmed" in captured
    assert "Mode: mapping_assist_only" in captured
    assert "Derived imu_to_pandar: OK" in captured
    assert "Motion authorization changed: NO" in captured
    assert "Safety authorization changed: NO" in captured
