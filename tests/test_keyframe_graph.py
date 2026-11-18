import json
import math

import pytest

from app.slam.keyframe_graph import KeyframeGraphStore


def _payload(revision, poses, *, epoch="e1"):
    return json.dumps({
        "schema_version": "go2w_keyframe_graph_v1",
        "map_id": "m1", "map_epoch": epoch, "frame_id": "map",
        "sensor_profile_id": "p1", "graph_revision": revision,
        "nodes": [
            {"anchor_id": anchor_id, "source_stamp": stamp, "corrected_pose": pose}
            for anchor_id, stamp, pose in poses
        ],
    })


def test_two_anchors_follow_distinct_loop_closure_corrections():
    store = KeyframeGraphStore()
    assert store.update_json(_payload(1, [
        ("k1", 10.0, [0.0, 0.0, 0.0]),
        ("k2", 20.0, [10.0, 0.0, 0.0]),
    ]))
    first = store.anchor_map_point((1.0, 0.0, 0.2), 10.1)
    second = store.anchor_map_point((11.0, 0.0, 0.3), 20.1)
    assert first["anchor_id"] == "k1"
    assert second["anchor_id"] == "k2"

    assert store.update_json(_payload(2, [
        ("k1", 10.0, [0.5, 0.0, 0.0]),
        ("k2", 20.0, [10.0, 1.0, math.pi / 2]),
    ]))
    p1 = store.reproject("k1", first["point_in_anchor"], map_id="m1", map_epoch="e1")
    p2 = store.reproject("k2", second["point_in_anchor"], map_id="m1", map_epoch="e1")
    assert p1 == pytest.approx((1.5, 0.0, 0.2))
    assert p2 == pytest.approx((10.0, 2.0, 0.3))


def test_old_revision_is_ignored_within_same_epoch():
    store = KeyframeGraphStore()
    assert store.update_json(_payload(2, [("k", 1.0, [0, 0, 0])]))
    assert not store.update_json(_payload(1, [("k", 1.0, [9, 9, 0])]))
    assert store.snapshot().graph_revision == 2


def test_epoch_mismatch_refuses_reprojection():
    store = KeyframeGraphStore()
    store.update_json(_payload(1, [("k", 1.0, [0, 0, 0])], epoch="new"))
    assert store.reproject("k", (1, 2, 3), map_id="m1", map_epoch="old") is None


def test_far_away_frame_is_not_attached_to_latest_anchor():
    store = KeyframeGraphStore()
    store.update_json(_payload(1, [("k", 1.0, [0, 0, 0])]))
    assert store.anchor_map_point((1, 2, 3), 100.0) is None


def test_stale_time_can_anchor_only_with_matching_acquisition_pose():
    store = KeyframeGraphStore()
    store.update_json(_payload(1, [
        ("k1", 1.0, [0.0, 0.0, 0.0]),
        ("k2", 2.0, [5.0, 0.0, math.pi / 2]),
    ]))

    anchored = store.anchor_map_point(
        (1.0, 0.0, 0.2), 100.0, observer_pose=(0.1, 0.0, 0.02)
    )
    assert anchored["anchor_id"] == "k1"
    assert anchored["anchor_association_method"] == "acquisition_pose_gate"
    assert anchored["anchor_observer_distance_m"] == pytest.approx(0.1)
    assert anchored["anchor_time_delta_s"] == pytest.approx(99.0)

    assert store.anchor_map_point(
        (1.0, 0.0, 0.2), 100.0, observer_pose=(2.0, 0.0, 0.0)
    ) is None
    assert store.anchor_map_point(
        (1.0, 0.0, 0.2), 100.0, observer_pose=(0.1, 0.0, math.pi / 2)
    ) is None
