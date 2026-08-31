from __future__ import annotations

import json
import math
import tempfile
from pathlib import Path

import pytest

from go2w_lidar_preprocessor.hesai_pandarxt16_preprocessor import (
    PandarConfigError,
    load_pandar_preprocess_config,
)
from go2w_lidar_preprocessor.hesai_diagnostics import (
    analyze_pandar_frame,
    azimuth_occupied_bins,
    estimate_self_occlusion_fraction,
)


REPO_ROOT = Path(__file__).resolve().parents[4]
CONFIG_PATH = REPO_ROOT / "configs/go2w/hesai_pandarxt16_preprocess.yaml"


def test_config_gate_accepts_diagnostic_only() -> None:
    config = load_pandar_preprocess_config(str(CONFIG_PATH))
    assert config["diagnostic_only"] is True
    assert config["authorizes_motion"] is False
    assert config["authorizes_safety_integration"] is False
    assert config["zero_return"]["maximum_range_m"] == 0.05


def test_config_gate_rejects_authorizes_motion(tmp_path: Path) -> None:
    path = tmp_path / "bad.yaml"
    path.write_text(
        CONFIG_PATH.read_text(encoding="utf-8").replace(
            "authorizes_motion: false", "authorizes_motion: true"
        ),
        encoding="utf-8",
    )
    with pytest.raises(PandarConfigError, match="authorizes motion"):
        load_pandar_preprocess_config(str(path))


def test_config_gate_rejects_non_diagnostic(tmp_path: Path) -> None:
    path = tmp_path / "bad.yaml"
    path.write_text(
        CONFIG_PATH.read_text(encoding="utf-8").replace(
            "diagnostic_only: true", "diagnostic_only: false"
        ),
        encoding="utf-8",
    )
    with pytest.raises(PandarConfigError, match="diagnostic_only"):
        load_pandar_preprocess_config(str(path))


def test_zero_return_points_are_filtered_from_valid() -> None:
    diag = analyze_pandar_frame(
        xyz=[[0.0, 0.0, 0.0], [0.04, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        ring=[0, 1, 2, 3],
        point_timestamp=[0.0, 0.01, 0.02, 0.03],
    )
    assert diag["zero_or_near_zero_points"] == 2
    assert diag["valid_points"] == 2
    assert diag["valid_return_fraction"] == 0.5


def test_self_occlusion_and_azimuth_helpers() -> None:
    body = {
        "x_min": -0.6, "x_max": 0.6,
        "y_min": -0.3, "y_max": 0.3,
        "z_min": -0.6, "z_max": 0.2,
    }
    points = [[0.0, 0.0, -0.4], [2.0, 0.0, 0.0], [0.0, 2.0, 0.0]]
    occlusion = estimate_self_occlusion_fraction(xyz=points, body_region=body)
    assert occlusion["inside_body_region"] == 1
    bins = azimuth_occupied_bins(xyz=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], bearing_bin_count=36)
    assert bins["occupied_bins"] == 2
