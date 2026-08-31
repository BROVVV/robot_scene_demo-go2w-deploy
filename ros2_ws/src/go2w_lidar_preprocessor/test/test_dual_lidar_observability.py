from __future__ import annotations

from pathlib import Path

import pytest

from go2w_lidar_preprocessor.dual_lidar_config import (
    DualLidarConfigError,
    load_dual_lidar_safety_config,
    observability_params,
)
from go2w_lidar_preprocessor.dual_lidar_observability import (
    compute_dual_lidar_rotation_observability,
    generate_pandar_unobservable_profile,
)


REPO_ROOT = Path(__file__).resolve().parents[4]
CONFIG_PATH = REPO_ROOT / "configs/go2w/dual_lidar_safety.yaml"


def test_dual_lidar_safety_disabled_by_default() -> None:
    config = load_dual_lidar_safety_config(str(CONFIG_PATH))
    assert config["enabled"] is False
    assert config["require_validated_extrinsics"] is True
    assert config["unknown_is_clear"] is False


def test_enable_requires_validated_extrinsics(tmp_path: Path) -> None:
    path = tmp_path / "enabled.yaml"
    text = CONFIG_PATH.read_text(encoding="utf-8")
    # Enabling dual-lidar safety while dropping the validated-extrinsics
    # requirement must be rejected by the config gate.
    text = text.replace("enabled: false", "enabled: true").replace(
        "require_validated_extrinsics: true",
        "require_validated_extrinsics: false",
    )
    path.write_text(text, encoding="utf-8")
    with pytest.raises(DualLidarConfigError, match="validated extrinsics"):
        load_dual_lidar_safety_config(str(path))


def test_enable_requires_unknown_is_clear_false(tmp_path: Path) -> None:
    path = tmp_path / "enabled.yaml"
    text = CONFIG_PATH.read_text(encoding="utf-8").replace(
        "enabled: false", "enabled: true"
    ).replace(
        "unknown_is_clear: false", "unknown_is_clear: true"
    )
    path.write_text(text, encoding="utf-8")
    with pytest.raises(DualLidarConfigError, match="unknown_is_clear"):
        load_dual_lidar_safety_config(str(path))


def test_pandar_cannot_contribute_clear_without_validated_tier(tmp_path: Path) -> None:
    path = tmp_path / "bad.yaml"
    text = CONFIG_PATH.read_text(encoding="utf-8").replace(
        "transform_tier: candidate_tf", "transform_tier: candidate_tf"
    ).replace(
        "can_contribute_clear: false", "can_contribute_clear: true"
    )
    path.write_text(text, encoding="utf-8")
    with pytest.raises(DualLidarConfigError, match="validated transform"):
        load_dual_lidar_safety_config(str(path))


def test_observability_params() -> None:
    config = load_dual_lidar_safety_config(str(CONFIG_PATH))
    params = observability_params(config)
    assert params["footprint_radius_m"] == 0.350
    assert params["envelope_radius_m"] == 0.511
    assert params["requested_turn_range_deg"] == 30.0


def test_rotation_observability_requires_validated_pandar() -> None:
    builtin_blind = {0.0: [(0.35, 0.5)], 90.0: [(0.35, 0.5)],
                     180.0: [(0.35, 0.5)], 270.0: [(0.35, 0.5)]}
    pandar_clear = {bearing: [] for bearing in builtin_blind}
    unvalidated = compute_dual_lidar_rotation_observability(
        footprint_radius_m=0.35,
        envelope_radius_m=0.511,
        builtin_unobservable=builtin_blind,
        pandar_unobservable=pandar_clear,
        pandar_extrinsics_validated=False,
    )
    assert unvalidated.full_rotation_observability_valid is False
    validated = compute_dual_lidar_rotation_observability(
        footprint_radius_m=0.35,
        envelope_radius_m=0.511,
        builtin_unobservable=builtin_blind,
        pandar_unobservable=pandar_clear,
        pandar_extrinsics_validated=True,
    )
    assert validated.full_rotation_observability_valid is True


def test_pandar_profile_near_field() -> None:
    profile = generate_pandar_unobservable_profile(
        bearings_deg=[0.0],
        footprint_radius_m=0.35,
        envelope_radius_m=0.511,
        pandar_min_range_m=0.40,
    )
    assert profile[0.0] == [(0.35, 0.40)]
