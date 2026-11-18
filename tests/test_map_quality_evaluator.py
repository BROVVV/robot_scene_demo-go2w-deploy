"""评估器单元测试（20260908 计划书 §27 对应项）。"""

from __future__ import annotations

import math

import numpy as np
import pytest

from app.mapping.map_quality_evaluator import (
    acceptance_check,
    duplicate_metrics,
    endpoint_error,
    instance_counts,
    load_object_events,
    loop_overlap_improvement,
    nn_overlap_metrics,
    object_spread,
    post_loop_jitter,
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


def test_object_events_group_by_persistent_id_across_label_drift(tmp_path):
    path = tmp_path / "events.jsonl"
    path.write_text(
        '\n'.join([
            '{"event":"semantic_frame_objects_localized","objects":[{"label":"桌子","persistent_object_id":"obj_2","map_xyz":[1,2,3]}]}',
            '{"event":"semantic_frame_objects_localized","objects":[{"label":"办公桌","persistent_object_id":"obj_2","map_xyz":[1.1,2,3]}]}',
            '{"event":"semantic_frame_objects_localized","objects":[{"label":"办公桌","persistent_object_id":"obj_2","map_xyz":[1.2,2,3]}]}',
        ]),
        encoding="utf-8",
    )
    grouped = load_object_events(str(path))
    assert grouped == {"办公桌#obj_2": [[1.0, 2.0, 3.0], [1.1, 2.0, 3.0], [1.2, 2.0, 3.0]]}


def test_acceptance_covers_all_plan_thresholds():
    report = {
        "map": {"surface_thickness_p95_m": 0.10},
        "trajectory": {
            "endpoint_translation_error_m": 0.10,
            "endpoint_yaw_error_deg": 2.0,
        },
        "overlap": {
            "overlap_rmse_m": 0.07,
            "overlap_p95_m": 0.13,
            "overlap_inlier_ratio": 0.90,
        },
        "loop": {
            "loop_closure_count": 2,
            "post_loop_translation_jitter_m": 0.04,
            "post_loop_yaw_jitter_deg": 1.0,
        },
        "semantic": {
            "object_position_rms_m": 0.09,
            "object_position_p95_m": 0.16,
        },
        "dedup": {"duplicate_rate": 0.0, "false_split": 0, "false_merge": 0},
    }
    acceptance = acceptance_check(report)
    assert acceptance["accepted"] is True
    assert set(acceptance["checks"]) == {
        "surface_thickness_p95_m", "endpoint_translation_error_m",
        "endpoint_yaw_error_deg", "object_position_rms_m",
        "object_position_p95_m", "overlap_rmse_m", "overlap_p95_m",
        "overlap_inlier_ratio", "loop_closure_count",
        "post_loop_translation_jitter_m", "post_loop_yaw_jitter_deg",
        "duplicate_rate", "false_split", "false_merge",
    }


def test_acceptance_reports_missing_plan_metrics_not_verified():
    acceptance = acceptance_check({"overlap": {"overlap_rmse_m": 0.07}})
    assert acceptance["accepted"] is True
    assert acceptance["checks"]["overlap_rmse_m"]["status"] == "PASS"
    assert acceptance["checks"]["overlap_p95_m"]["status"] == "NOT_VERIFIED"
    assert acceptance["checks"]["post_loop_translation_jitter_m"]["status"] == "NOT_VERIFIED"
    assert acceptance["checks"]["duplicate_rate"]["status"] == "NOT_VERIFIED"


def test_acceptance_fails_overlap_inlier_and_false_merge():
    acceptance = acceptance_check({
        "overlap": {"overlap_inlier_ratio": 0.70},
        "dedup": {"false_merge": 1},
    })
    assert acceptance["accepted"] is False
    assert acceptance["failed"] == ["overlap_inlier_ratio", "false_merge"]


def test_post_loop_jitter_static_after_loop_is_small():
    stamps = [float(t) for t in range(11)]
    trajectory = np.array([[0.0, 0.0, 0.0] for _ in stamps])
    metrics = post_loop_jitter(trajectory, stamps, [4.0])
    assert metrics["post_loop_translation_jitter_m"] == pytest.approx(0.0, abs=1e-9)
    assert metrics["post_loop_yaw_jitter_deg"] == pytest.approx(0.0, abs=1e-9)
    assert metrics["post_loop_window"] == "loop+3..5s"


def test_post_loop_jitter_detects_oscillation_and_random_walk():
    stamps = [float(t) for t in range(11)]
    oscillating = np.array([[0.20 * (i % 2), 0.0, math.radians(4.0 * (i % 2))]
                            for i in range(len(stamps))])
    metrics = post_loop_jitter(oscillating, stamps, [4.0])
    assert metrics["post_loop_translation_jitter_m"] > 0.08
    assert metrics["post_loop_yaw_jitter_deg"] > 2.0

    calm = np.array([[0.001 * i, 0.0, 0.0] for i in range(len(stamps))])
    assert post_loop_jitter(calm, stamps, [4.0])["post_loop_translation_jitter_m"] < 0.01


def test_post_loop_jitter_without_loop_is_not_verified():
    trajectory = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    metrics = post_loop_jitter(trajectory, [0.0, 1.0, 2.0], [])
    assert metrics["post_loop_translation_jitter_m"] is None
    assert metrics["post_loop_yaw_jitter_deg"] is None


def test_loop_overlap_improvement_scores_pre_and_post():
    rng = np.random.default_rng(3)
    first = rng.uniform(0.0, 4.0, size=(200, 3))
    pre = first + np.array([0.30, 0.0, 0.0])
    post = first + np.array([0.02, 0.0, 0.0])
    metrics = loop_overlap_improvement((0.0, first, "pslam_odom"),
                                       (1.0, pre, "pslam_odom"),
                                       (2.0, post, "pslam_odom"))
    assert metrics["loop_overlap_improved"] is True
    assert metrics["post_loop_overlap_rmse_m"] < metrics["pre_loop_overlap_rmse_m"]
    assert metrics["overlap_improvement_ratio"] > 0.20


def test_loop_overlap_improvement_requires_loop_and_world_frame():
    cloud = np.zeros((10, 3))
    assert loop_overlap_improvement((0.0, cloud, "pslam_odom"), None, None) is None
    assert loop_overlap_improvement((0.0, cloud, "pandar"),
                                    (1.0, cloud, "pandar"),
                                    (2.0, cloud, "pandar")) is None


def test_instance_counts_keeps_same_label_instances_separate():
    counts = instance_counts({
        "椅子#obj_001": [[1.0, 0.0, 0.0], [1.02, 0.0, 0.0]],
        "椅子#obj_002": [[4.0, 0.0, 0.0], [4.01, 0.0, 0.0]],
        "垃圾桶#obj_003": [[2.0, 0.0, 0.0]],
    })
    assert counts == {"椅子": 2, "垃圾桶": 1}
    dedup = duplicate_metrics(counts, {"椅子": 2, "垃圾桶": 1})
    assert dedup["false_split"] == 0
    assert dedup["false_merge"] == 0
    assert dedup["duplicate_rate"] == 0.0
    over_split = duplicate_metrics(counts, {"椅子": 1, "垃圾桶": 1})
    assert over_split["false_split"] == 1


