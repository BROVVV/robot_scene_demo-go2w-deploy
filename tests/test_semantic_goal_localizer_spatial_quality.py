"""计划书 §19/§20 + §32.10：目标 3D 定位必须按 spatial quality 分级。

钉住的红线（§44）：不许用固定 1m 当真实目标距离，也不许给只有相机局部坐标的
目标伪造一个 global pose。
"""

from __future__ import annotations

from app.navigation.semantic_goal_localizer import (
    APPROACH_LOCAL_VISUAL_SERVO,
    APPROACH_MAP_VIEWPOINTS,
    APPROACH_REOBSERVE_ONLY,
    localize_semantic_goal,
)
from app.navigation.video_pose_estimator import estimate_video_trajectory


def _trajectory():
    return estimate_video_trajectory("missing.mp4", backend="relative")


def _result(evidence: dict) -> dict:
    return {
        "target": "蓝色垃圾桶",
        "target_status": "target_visual_confirmed",
        "best_evidence": evidence,
    }


# METRIC_RGBD：唯一允许生成 map-frame 目标位姿的等级。
def test_metric_rgbd_uses_map_xyz() -> None:
    localization = localize_semantic_goal(
        _result({"frame_id": 10, "bbox": [400, 100, 600, 300], "confidence": 0.8,
                 "spatial_quality": "METRIC_RGBD", "depth_m": 2.4,
                 "bearing_deg": -6.0, "map_xyz": [3.5, 1.25, 0.4]}),
        _trajectory(),
    )
    pose = localization["target_pose"]
    assert pose is not None
    assert (round(pose["x"], 3), round(pose["y"], 3)) == (3.5, 1.25)
    assert pose["scale_status"] == "metric"
    assert pose["provenance"]["scale_verified"] is True
    assert localization["approach_mode"] == APPROACH_MAP_VIEWPOINTS
    assert localization["approach_allowed"] is True
    assert localization["heuristic_only"] is False
    assert localization["local_only"] is False
    assert localization["target_depth_m"] == 2.4
    assert localization["target_bearing_deg"] == -6.0


# CAMERA_LOCAL：有真实深度但没有 map 位姿 -> 绝不伪造 global 1m 目标。
def test_camera_local_never_fabricates_global_pose() -> None:
    for quality in ("CAMERA_LOCAL", "RELATIVE_RGBD"):
        localization = localize_semantic_goal(
            _result({"frame_id": 10, "bbox": [400, 100, 600, 300],
                     "confidence": 0.8, "spatial_quality": quality,
                     "depth_m": 1.8, "bearing_deg": 4.0}),
            _trajectory(),
        )
        assert localization["target_pose"] is None, quality
        assert localization["local_only"] is True
        assert localization["approach_mode"] == APPROACH_LOCAL_VISUAL_SERVO
        # 局部深度是真值，可以驱动接近；只是没有 global 目标点可去。
        assert localization["approach_allowed"] is True
        assert localization["heuristic_only"] is False
        assert localization["target_depth_m"] == 1.8


# METRIC_RGBD 但缺 map_xyz -> 退回 local，仍然不许编造 global pose。
def test_metric_without_map_xyz_falls_back_to_local() -> None:
    localization = localize_semantic_goal(
        _result({"frame_id": 10, "spatial_quality": "METRIC_RGBD",
                 "depth_m": 1.5, "bearing_deg": 0.0, "confidence": 0.9}),
        _trajectory(),
    )
    assert localization["target_pose"] is None
    assert localization["approach_mode"] == APPROACH_REOBSERVE_ONLY
    assert localization["approach_allowed"] is False


# RGB_ONLY：默认既没有 pose 也不允许 approach。
def test_rgb_only_cannot_approach() -> None:
    localization = localize_semantic_goal(
        _result({"frame_id": 10, "bbox": [400, 100, 600, 300], "confidence": 0.8,
                 "spatial_quality": "RGB_ONLY"}),
        _trajectory(),
    )
    assert localization["target_pose"] is None
    assert localization["approach_mode"] == APPROACH_REOBSERVE_ONLY
    assert localization["approach_allowed"] is False
    assert localization["target_depth_m"] is None


# 固定距离启发式即使显式打开，也永远不能算作可接近/可判到达。
def test_fixed_distance_fallback_is_heuristic_only() -> None:
    localization = localize_semantic_goal(
        _result({"frame_id": 10, "bbox": [400, 100, 600, 300], "confidence": 0.8,
                 "spatial_quality": "RGB_ONLY"}),
        _trajectory(),
        allow_fixed_distance_fallback=True,
    )
    assert localization["target_pose"] is not None
    assert localization["heuristic_only"] is True
    assert localization["approach_allowed"] is False
    assert localization["approach_mode"] == APPROACH_REOBSERVE_ONLY
    assert localization["target_pose"]["provenance"]["heuristic_only"] is True
    assert localization["target_pose"]["scale_status"] != "metric"


# 没有任何可定位证据 -> exploration，字段仍必须齐全（WebUI 依赖）。
def test_no_evidence_returns_exploration_goal() -> None:
    localization = localize_semantic_goal(
        {"target": "蓝色垃圾桶", "target_status": "target_not_seen"}, _trajectory())
    assert localization["goal_type"] == "exploration"
    assert localization["target_pose"] is None
    assert localization["approach_allowed"] is False
    for key in ("spatial_quality", "local_only", "target_depth_m",
                "target_bearing_deg", "approach_mode", "heuristic_only"):
        assert key in localization
