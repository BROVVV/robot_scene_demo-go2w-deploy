"""评估器单元测试（20260908 计划书 §27 对应项）。"""

from __future__ import annotations

import math

import numpy as np
import pytest

from app.mapping.map_quality_evaluator import (
    duplicate_metrics,
    endpoint_error,
    nn_overlap_metrics,
    object_spread,
    surface_thickness_p95,
)


def _plane(n=400, jitter=0.002, seed=0):
    rng = np.random.default_rng(seed)
    x = rng.uniform(0.0, 2.0, n)
    y = rng.uniform(0.0, 2.0, n)
    z = rng.normal(0.0, jitter, n)
    return np.stack([x, y, z], axis=1)


def test_overlap_identical_clouds_is_zero():
    cloud = _plane()
    metrics = nn_overlap_metrics(cloud, cloud)
    assert metrics["overlap_rmse_m"] == pytest.approx(0.0, abs=1e-9)
    assert metrics["overlap_inlier_ratio"] == pytest.approx(1.0)


def test_overlap_detects_ghosting():
    # 稀疏随机点云：平移后没有"就近的平面邻居"可对齐，才能暴露重影。
    rng = np.random.default_rng(1)
    cloud = rng.uniform(0.0, 2.0, size=(150, 3))
    shifted = cloud + np.array([0.50, 0.0, 0.0])
    metrics = nn_overlap_metrics(cloud, shifted, max_dist_m=0.10)
    assert metrics["overlap_rmse_m"] > 0.20
    assert metrics["overlap_inlier_ratio"] < 0.10


def test_single_wall_thin_double_wall_thick():
    thin = surface_thickness_p95(_plane(jitter=0.002))
    two = np.concatenate([_plane(jitter=0.002), _plane(jitter=0.002) + np.array([0, 0, 0.30])])
    thick = surface_thickness_p95(two)
    assert thin < 0.05
    assert thick > thin


def test_endpoint_error_closed_loop():
    trajectory = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.1], [0.0, 0.0, 0.0]])
    metrics = endpoint_error(trajectory)
    assert metrics["endpoint_translation_error_m"] == pytest.approx(0.0, abs=1e-9)
    assert metrics["endpoint_yaw_error_deg"] == pytest.approx(0.0, abs=1e-9)


def test_endpoint_error_yaw_wraps():
    trajectory = np.array([[0.0, 0.0, math.radians(179.0)],
                           [0.0, 0.0, math.radians(-179.0)]])
    assert endpoint_error(trajectory)["endpoint_yaw_error_deg"] == pytest.approx(2.0, abs=1e-6)


def test_object_spread_stable_and_scattered():
    stable = object_spread([[2.00, 1.00, 0.5], [2.05, 1.03, 0.5], [1.95, 0.98, 0.5]])
    scattered = object_spread([[2.0, 1.0, 0.5], [4.0, 1.0, 0.5]])
    assert stable["centroid_rms_error_m"] < 0.05
    assert scattered["centroid_rms_error_m"] > 0.5


def test_duplicate_metrics_split_and_merge():
    split = duplicate_metrics({"chair": 3}, {"chair": 1})
    assert split["false_split"] == 2
    assert split["false_merge"] == 0
    assert split["duplicate_rate"] == pytest.approx(2.0)

    merged = duplicate_metrics({"chair": 1}, {"chair": 2})
    assert merged["false_merge"] == 1
    assert merged["false_split"] == 0

    exact = duplicate_metrics({"chair": 2}, {"chair": 2})
    assert exact["duplicate_rate"] == 0.0
    assert exact["false_split"] == 0
    assert exact["false_merge"] == 0
