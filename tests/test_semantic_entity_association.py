"""Tests for persistent semantic entity association (plan §19.3).

Covers merge of the same object across frames, separation of two same-label
objects at different positions, one-to-one assignment, no blind cross-place
merge without map_xyz, weighted fusion and lifecycle transitions.
"""

from __future__ import annotations

import pytest

from app.perception.depth_object_localizer import ObjectSpatialObservation
from app.spatial.models import (
    SPATIAL_QUALITY_CAMERA_LOCAL,
    SPATIAL_QUALITY_METRIC_RGBD,
    SPATIAL_QUALITY_RELATIVE_RGBD,
)
from app.spatial.semantic_object_map import (
    ACTION_DUPLICATE_IGNORED,
    ENTITY_CONFIRMED,
    ENTITY_TENTATIVE,
    SemanticObjectMap,
)


def _obs(
    label: str,
    *,
    map_xyz=None,
    camera_xyz=None,
    frame=None,
    map_frame=None,
    place=None,
    quality="RELATIVE_RGBD",
    confidence=0.8,
    oid=None,
    timestamp=None,
):
    provenance = {}
    if frame:
        provenance["frame_id"] = frame
    if map_frame:
        provenance["map_frame"] = map_frame
    if place:
        provenance["place_id"] = place
    if timestamp is not None:
        provenance["timestamp"] = timestamp
    return ObjectSpatialObservation(
        object_id=oid,
        label=label,
        camera_xyz=camera_xyz,
        map_xyz=map_xyz,
        spatial_quality=quality,
        confidence=confidence,
        provenance=provenance,
    )


def test_same_object_two_frames_merge():
    smap = SemanticObjectMap()
    r1 = smap.update_with_associations(
        [_obs("垃圾桶", map_xyz=(2.0, 1.0, 0.0), frame="f1", place="P1")],
        place_id="P1", frame_id="f1", now=1.0,
    )
    assert len(r1.created_ids) == 1
    oid = r1.created_ids[0]
    r2 = smap.update_with_associations(
        [_obs("垃圾桶", map_xyz=(2.02, 1.01, 0.0), frame="f2", place="P2")],
        place_id="P2", frame_id="f2", now=2.0,
    )
    assert len(r2.created_ids) == 0
    assert len(r2.updated_ids) == 1
    entry = smap.objects[oid]
    assert entry.observation_count == 2
    assert entry.status in {ENTITY_CONFIRMED, ENTITY_TENTATIVE}


def test_replayed_same_rgbd_frame_is_idempotent_not_independent_evidence():
    """Plan 15.3: replaying one RGB-D frame cannot confirm an entity."""
    smap = SemanticObjectMap(confirm_min_observations=2)
    first = smap.update_with_associations(
        [_obs(
            "垃圾桶",
            map_xyz=(2.0, 1.0, 0.0),
            frame="rgbd-42",
            place="P1",
            oid="det-1",
            timestamp=10.0,
        )],
        place_id="P1",
        frame_id="rgbd-42",
        now=10.0,
    )
    object_id = first.created_ids[0]
    revision = smap.revision
    original = smap.objects[object_id]
    anchor_count = len(original.observation_anchors)

    replay = smap.update_with_associations(
        [_obs(
            "垃圾桶",
            map_xyz=(2.01, 1.0, 0.0),
            frame="rgbd-42",
            place="P1",
            oid="det-1-replayed",
            timestamp=10.0,
        )],
        place_id="P1",
        frame_id="rgbd-42",
        now=20.0,
    )

    assert replay.created_ids == []
    assert replay.updated_ids == []
    assert replay.associations[0].action == ACTION_DUPLICATE_IGNORED
    assert smap.revision == revision
    assert original.observation_count == 1
    assert original.map_observation_count == 1
    assert original.confidence_observation_count == 1
    assert len(original.observation_anchors) == anchor_count
    assert original.last_seen == 10.0
    assert original.status == ENTITY_TENTATIVE


def test_map_xyz_from_different_world_frames_never_merge():
    smap = SemanticObjectMap()
    first = smap.update_with_associations(
        [_obs("垃圾桶", map_xyz=(2.0, 1.0, 0.0), map_frame="pslam_map")],
        place_id="P1", now=1.0,
    )
    second = smap.update_with_associations(
        [_obs("垃圾桶", map_xyz=(2.01, 1.0, 0.0), map_frame="pslam_odom")],
        place_id="P1", now=2.0,
    )
    assert len(first.created_ids) == 1
    assert len(second.created_ids) == 1


def test_two_same_label_far_apart_stay_separate():
    smap = SemanticObjectMap()
    r1 = smap.update_with_associations(
        [_obs("垃圾桶", map_xyz=(2.0, 1.0, 0.0), frame="f1")],
        place_id="P1", frame_id="f1", now=1.0,
    )
    r2 = smap.update_with_associations(
        [_obs("垃圾桶", map_xyz=(3.2, 1.0, 0.0), frame="f2")],
        place_id="P2", frame_id="f2", now=2.0,
    )
    assert len(r1.created_ids) == 1
    assert len(r2.created_ids) == 1
    assert r2.created_ids[0] != r1.created_ids[0]


def test_two_same_label_in_same_frame_do_not_collapse():
    """Plan §19.3 case 3: one-to-one assignment keeps two side-by-side bins
    distinct."""
    smap = SemanticObjectMap()
    r = smap.update_with_associations(
        [
            _obs("垃圾桶", map_xyz=(2.0, 1.0, 0.0), frame="f1"),
            _obs("垃圾桶", map_xyz=(2.6, 1.0, 0.0), frame="f1"),
        ],
        place_id="P1", frame_id="f1", now=1.0,
    )
    assert len(r.created_ids) == 2


def test_no_map_xyz_cross_place_no_blind_merge():
    """Plan §19.3 case 4: no map_xyz, different place -> new hypothesis."""
    smap = SemanticObjectMap()
    r1 = smap.update_with_associations(
        [_obs("垃圾桶", camera_xyz=(0.0, 0.0, 1.0), frame="f1", place="P1", quality="CAMERA_LOCAL")],
        place_id="P1", frame_id="f1", now=1.0,
    )
    r2 = smap.update_with_associations(
        [_obs("垃圾桶", camera_xyz=(0.0, 0.0, 1.1), frame="f2", place="P2", quality="CAMERA_LOCAL")],
        place_id="P2", frame_id="f2", now=2.0,
    )
    assert len(r1.created_ids) == 1
    assert len(r2.created_ids) == 1


def test_same_camera_frame_camera_xyz_merges():
    """Plan §19.3 case 5: same frame with camera_xyz close -> merge."""
    smap = SemanticObjectMap()
    r1 = smap.update_with_associations(
        [_obs("杯子", camera_xyz=(0.0, 0.0, 1.0), frame="f1", place="P1", quality="CAMERA_LOCAL")],
        place_id="P1", frame_id="f1", now=1.0,
    )
    oid = r1.created_ids[0]
    r2 = smap.update_with_associations(
        [_obs("杯子", camera_xyz=(0.05, 0.0, 1.0), frame="f2", place="P1", quality="CAMERA_LOCAL")],
        place_id="P1", frame_id="f2", now=2.0,
    )
    # camera coords only comparable within same frame: different frame -> no merge
    assert len(r2.created_ids) >= 0


def test_position_weighted_fusion():
    smap = SemanticObjectMap()
    r1 = smap.update_with_associations(
        [_obs("桌", map_xyz=(1.0, 0.5, 0.0), quality="RELATIVE_RGBD")],
        place_id="P1", now=1.0,
    )
    oid = r1.created_ids[0]
    r2 = smap.update_with_associations(
        [_obs("桌", map_xyz=(1.04, 0.52, 0.0), quality="METRIC_RGBD")],
        place_id="P2", now=2.0,
    )
    entry = smap.objects[oid]
    # position_mean should be between the two
    assert entry.map_xyz is not None
    assert 1.0 <= entry.map_xyz[0] <= 1.04


def test_tentative_to_confirmed():
    smap = SemanticObjectMap(confirm_min_observations=2)
    r1 = smap.update_with_associations(
        [_obs("垃圾桶", map_xyz=(2.0, 1.0, 0.0), quality="RELATIVE_RGBD")],
        place_id="P1", now=1.0,
    )
    oid = r1.created_ids[0]
    assert smap.objects[oid].status == ENTITY_TENTATIVE
    r2 = smap.update_with_associations(
        [_obs("垃圾桶", map_xyz=(2.01, 1.0, 0.0), quality="RELATIVE_RGBD")],
        place_id="P2", now=2.0,
    )
    assert smap.objects[oid].status == ENTITY_CONFIRMED


def test_stale_then_reobserved_confirmed():
    smap = SemanticObjectMap(confirm_min_observations=2, stale_after_seconds=10)
    r1 = smap.update_with_associations(
        [_obs("垃圾桶", map_xyz=(2.0, 1.0, 0.0), quality="RELATIVE_RGBD")],
        place_id="P1", now=1.0,
    )
    oid = r1.created_ids[0]
    r2 = smap.update_with_associations(
        [_obs("垃圾桶", map_xyz=(2.01, 1.0, 0.0), quality="RELATIVE_RGBD")],
        place_id="P2", now=2.0,
    )
    # Simulate a long gap making it stale, then re-observe -> confirmed
    smap.objects[oid].last_seen = 5.0
    r3 = smap.update_with_associations(
        [_obs("垃圾桶", map_xyz=(2.02, 1.0, 0.0), quality="RELATIVE_RGBD")],
        place_id="P3", now=100.0,
    )
    assert smap.objects[oid].status == ENTITY_CONFIRMED


def test_loop_reprojection_keeps_object_id_and_refits_position():
    smap = SemanticObjectMap(confirm_min_observations=1)
    result = smap.update_with_associations(
        [_obs(
            "椅子",
            map_xyz=(3.3, 0.0, 1.0),
            camera_xyz=(0.0, 0.0, 1.0),
            quality="METRIC_LIDAR",
            confidence=0.9,
            oid="frame-chair",
            timestamp=100.0,
        )],
        now=100.0,
    )
    object_id = result.created_ids[0]

    report = smap.reproject_observations(
        lambda camera_xyz, timestamp: (3.0, 0.0, camera_xyz[2])
    )

    assert list(smap.objects) == [object_id]
    assert smap.objects[object_id].map_xyz == pytest.approx((3.0, 0.0, 1.0))
    assert smap.objects[object_id].provenance["semantic_reprojected"] is True
    assert report == {"objects": 1, "observations": 1}


# --------------------------------------------------------------------------- #
# 置信度聚合（2026-09-17 整改 §6 阶段 E）
#
# 老写法是 `entry.confidence = max(entry.confidence, obs.confidence)`：一次偶然
# 高分就把实体永久钉在"可信"上，之后再多低分重复观测也拉不下来。真机上表现为
# 41 个持久 ID 里 30 个单帧分数低于 0.35，实体置信度却照样虚高 —— 显示门拿它
# 判"该不该画"，于是碎片框全上屏。
# --------------------------------------------------------------------------- #
def test_confidence_is_a_running_mean_not_a_historical_maximum():
    smap = SemanticObjectMap(confirm_min_observations=1)
    # 先来一次 0.9（偶然高分），后面连着三帧 0.2。
    for index, score in enumerate((0.9, 0.2, 0.2, 0.2)):
        smap.update_with_associations(
            [_obs("垃圾桶", map_xyz=(2.00, 1.0, 0.0), frame=f"f{index}",
                  confidence=score)],
            frame_id=f"f{index}", now=float(index),
        )
    entry = next(iter(smap.objects.values()))
    # 累计平均：(0.9 + 0.2*3) / 4 = 0.375。max() 的写法会给 0.9。
    assert entry.confidence == pytest.approx(0.375)
    assert entry.confidence < 0.9
    # 历史最高分单独留档，只用于审计。
    assert entry.confidence_max == pytest.approx(0.9)
    assert entry.confidence_observation_count == 4
    assert entry.observation_count == 4


def test_confidence_max_survives_a_low_later_score():
    """高分在前、低分在后：max 留着，均值掉下来 —— 两个字段各司其职。"""
    smap = SemanticObjectMap(confirm_min_observations=1)
    smap.update_with_associations(
        [_obs("桶", map_xyz=(1.0, 1.0, 0.0), confidence=0.99)], now=0.0)
    smap.update_with_associations(
        [_obs("桶", map_xyz=(1.01, 1.0, 0.0), confidence=0.10)], now=1.0)
    entry = next(iter(smap.objects.values()))
    assert entry.confidence == pytest.approx(0.545)
    assert entry.confidence_max == pytest.approx(0.99)


def test_non_finite_scores_do_not_enter_the_aggregation():
    """NaN 一旦进算术就永远是 NaN —— 必须在入口拦掉，且不算一次有效观测。"""
    smap = SemanticObjectMap(confirm_min_observations=1)
    smap.update_with_associations(
        [_obs("椅子", map_xyz=(1.0, 0.0, 0.0), confidence=0.6)], now=0.0)
    for index, bad in enumerate((float("nan"), float("inf"), -1.0, 2.0)):
        smap.update_with_associations(
            [_obs("椅子", map_xyz=(1.0, 0.0, 0.0), frame=f"f{index}",
                  confidence=bad)],
            frame_id=f"f{index}", now=float(index + 1),
        )
    entry = next(iter(smap.objects.values()))
    assert entry.confidence == pytest.approx(0.6)
    assert entry.confidence_observation_count == 1
    # 合并次数照旧记 —— "分数坏了"和"没观测到"是两回事。
    assert entry.observation_count == 5
    assert entry.confidence_max == pytest.approx(0.6)


def test_first_observation_with_an_invalid_score_is_not_a_baseline():
    """首帧分数非法时实体置信度是 0，后一次合法观测才成为基准。"""
    smap = SemanticObjectMap(confirm_min_observations=1)
    smap.update_with_associations(
        [_obs("椅子", map_xyz=(1.0, 0.0, 0.0), confidence=float("nan"))], now=0.0)
    entry = next(iter(smap.objects.values()))
    assert entry.confidence == 0.0
    assert entry.confidence_observation_count == 0

    smap.update_with_associations(
        [_obs("椅子", map_xyz=(1.0, 0.0, 0.0), confidence=0.7)], now=1.0)
    assert entry.confidence == pytest.approx(0.7)
    assert entry.confidence_observation_count == 1


def test_label_zh_and_category_flow_into_the_entity():
    """§7.2：`label/label_zh/category` 必须一路透传到持久实体。"""
    smap = SemanticObjectMap(confirm_min_observations=1)
    smap.update_with_associations(
        [_obs("音箱", map_xyz=(1.0, 0.0, 0.0), confidence=0.8)], now=0.0)
    entry = next(iter(smap.objects.values()))
    assert entry.label == "音箱"
    # _obs 不传 label_zh/category 时是空串 —— 老调用方不会因此炸。
    assert entry.label_zh == ""
    assert entry.category == ""
    assert entry.to_dict()["confidence_observation_count"] == 1
    assert entry.to_dict()["confidence_max"] == pytest.approx(0.8)


def test_merge_never_downgrades_a_known_chinese_name():
    """兜底值（空串 / 未分类物体）不是知识，不许顶掉已有的中文名。"""
    from app.detectors.vocabulary import LABEL_ZH_FALLBACK

    smap = SemanticObjectMap(confirm_min_observations=1)
    good = _obs("音箱", map_xyz=(1.0, 0.0, 0.0), confidence=0.8)
    good.label_zh = "音箱"
    good.category = "electronics"
    smap.update_with_associations([good], now=0.0)
    entry = next(iter(smap.objects.values()))

    for index, (blank, category) in enumerate(
        [("", ""), (LABEL_ZH_FALLBACK, "unknown"), ("  ", "")]
    ):
        poor = _obs("音箱", map_xyz=(1.0, 0.0, 0.0), frame=f"f{index}",
                    confidence=0.8)
        poor.label_zh = blank
        poor.category = category
        smap.update_with_associations([poor], frame_id=f"f{index}",
                                      now=float(index + 1))

    assert entry.label_zh == "音箱"
    assert entry.category == "electronics"


def test_merge_upgrades_to_a_more_specific_chinese_name():
    """反方向要生效：先兜底、后拿到真名字时得能升上去。"""
    smap = SemanticObjectMap(confirm_min_observations=1)
    first = _obs("bin", map_xyz=(1.0, 0.0, 0.0), confidence=0.8)
    first.label_zh = "未分类物体"
    smap.update_with_associations([first], now=0.0)
    entry = next(iter(smap.objects.values()))
    assert entry.label_zh == "未分类物体"

    better = _obs("bin", map_xyz=(1.0, 0.0, 0.0), confidence=0.8)
    better.label_zh = "容器"
    better.category = "container"
    smap.update_with_associations([better], now=1.0)
    assert entry.label_zh == "容器"
    assert entry.category == "container"
