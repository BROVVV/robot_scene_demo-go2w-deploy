"""Known-transform checks for the dual-LiDAR calibration math."""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts/go2w"))

from calibrate_pandarxt16_extrinsics import (  # noqa: E402
    estimate_scene_transform,
    icp,
    rotation_z,
    transform_points,
)


def _asymmetric_cloud() -> np.ndarray:
    rng = np.random.default_rng(4)
    return rng.uniform([-1.0, -0.7, -0.3], [2.0, 0.8, 0.9], size=(1500, 3))


def test_icp_recovers_known_rigid_transform() -> None:
    source = _asymmetric_cloud()
    expected_translation = np.array([0.13, -0.02, 0.01])
    expected_yaw = math.radians(12.0)
    target = transform_points(source, expected_translation, rotation_z(expected_yaw))

    translation, rotation, metrics = icp(
        source,
        target,
        np.array([0.12, -0.02, 0.02]),
        rotation_z(math.radians(11.0)),
        max_iterations=8,
    )

    np.testing.assert_allclose(translation, expected_translation, atol=1e-6)
    np.testing.assert_allclose(rotation, rotation_z(expected_yaw), atol=1e-6)
    assert metrics["final_residual_m"] < 1e-6


def test_scene_reports_refined_yaw_not_search_start() -> None:
    source = _asymmetric_cloud()
    expected_yaw = math.radians(12.0)
    translation = np.array([0.13, -0.02, 0.01])
    target = transform_points(source, translation, rotation_z(expected_yaw))

    result = estimate_scene_transform(
        target,
        source,
        init_translation=np.array([0.12, -0.02, 0.02]),
        init_yaw_rad=math.radians(10.0),
        yaw_search_range_rad=math.radians(1.0),
        yaw_search_steps=2,
    )

    assert abs(result["yaw_deg"] - 12.0) < 0.03
    assert result["residual"] < 0.003
