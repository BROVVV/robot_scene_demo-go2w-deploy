from pathlib import Path

import cv2
import numpy as np
import pytest

from go2w_rgb_lidar_fusion.overlay_core import (
    CalibrationBlocked,
    CameraModel,
    LidarToCameraTransform,
    estimate_lidar_to_camera_pnp,
    load_camera_model,
    load_confirmed_transform,
    load_diagnostic_transform,
    project_camera_points,
    render_depth_overlay,
    reprojection_metrics,
    rotation_matrix_from_rpy,
    rpy_from_rotation_matrix,
    select_synchronized_pairs,
    transform_lidar_to_camera,
)


def camera():
    return CameraModel(
        640,
        480,
        "plumb_bob",
        np.asarray([[500.0, 0.0, 320.0], [0.0, 510.0, 240.0], [0.0, 0.0, 1.0]]),
        np.zeros(5),
        "synthetic_test",
    )


def test_project_and_render_use_lidar_child_to_camera_parent_convention():
    transform = LidarToCameraTransform(np.eye(3), np.asarray([0.0, 0.0, 2.0]), "test")
    points = np.asarray([[0.0, 0.0, 0.0], [0.2, 0.0, 0.0], [0.0, 0.2, 0.0]])
    camera_points = transform_lidar_to_camera(points, transform)
    pixels, valid = project_camera_points(camera_points, camera())
    assert valid.all()
    assert pixels[0] == pytest.approx((320.0, 240.0))
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    overlay, summary = render_depth_overlay(image, points, camera(), transform)
    assert summary["projected_inside_count"] == 3
    assert np.count_nonzero(overlay) > 0


def test_pnp_recovers_synthetic_lidar_to_camera_transform():
    rng = np.random.default_rng(8)
    objects = rng.uniform([-0.8, -0.5, -0.2], [0.8, 0.5, 0.6], size=(40, 3))
    expected = LidarToCameraTransform(
        rotation_matrix_from_rpy(0.04, -0.08, 0.03),
        np.asarray([0.12, -0.03, 2.4]),
        "synthetic_truth",
    )
    pixels, front = project_camera_points(
        transform_lidar_to_camera(objects, expected), camera()
    )
    assert front.all()
    solved, metrics = estimate_lidar_to_camera_pnp(objects, pixels, camera())
    assert solved.translation == pytest.approx(expected.translation, abs=1e-6)
    assert solved.rotation == pytest.approx(expected.rotation, abs=1e-6)
    assert metrics.mean_px < 1e-5


def test_reprojection_metrics_and_rpy_round_trip():
    rotation = rotation_matrix_from_rpy(-0.2, 0.3, -0.4)
    assert rotation_matrix_from_rpy(*rpy_from_rotation_matrix(rotation)) == pytest.approx(
        rotation
    )
    transform = LidarToCameraTransform(rotation, np.asarray([0.0, 0.0, 3.0]), "test")
    objects = np.asarray([[0.0, 0.0, 0.0], [0.3, 0.0, 0.1]])
    observed, _ = project_camera_points(
        transform_lidar_to_camera(objects, transform), camera()
    )
    observed[1, 0] += 2.0
    metrics = reprojection_metrics(objects, observed, camera(), transform)
    assert metrics.count == 2
    assert metrics.mean_px == pytest.approx(1.0)
    assert metrics.maximum_px == pytest.approx(2.0)


def test_camera_model_rejects_uncalibrated_config(tmp_path):
    root = Path(__file__).parents[4]
    payload = (root / "configs/go2w/camera_intrinsics.yaml").read_text(encoding="utf-8")
    candidate = tmp_path / "camera_uncalibrated.yaml"
    candidate.write_text(
        payload.replace("calibration_status: calibrated", "calibration_status: uncalibrated"),
        encoding="utf-8",
    )
    with pytest.raises(CalibrationBlocked, match="not calibrated"):
        load_camera_model(candidate)


def test_project_extrinsics_are_diagnostic_only():
    root = Path(__file__).parents[4]
    path = root / "configs/go2w/sensor_extrinsics.yaml"
    transform = load_diagnostic_transform(path)
    assert transform.source.endswith("sensor_extrinsics.yaml")
    with pytest.raises(CalibrationBlocked, match="not navigation-grade"):
        load_confirmed_transform(path)


def test_pair_selection_uses_header_time_limit_and_even_spacing():
    images = [(index * 100_000_000, 1_000 + index) for index in range(20)]
    clouds = [(index * 100_000_000 + 5_000_000, 2_000 + index) for index in range(20)]
    pairs = select_synchronized_pairs(
        images,
        clouds,
        maximum_delta_ns=10_000_000,
        minimum_separation_ns=300_000_000,
        maximum_pairs=4,
    )
    assert len(pairs) == 4
    assert all(item["timestamp_delta_ns"] == 5_000_000 for item in pairs)
    assert pairs[0]["cloud_stamp_ns"] == 5_000_000
    assert pairs[-1]["cloud_stamp_ns"] >= 1_805_000_000
