from app.perception.depth_object_localizer import ObjectSpatialObservation
from app.spatial.semantic_object_map import SemanticObjectMap


def _observation(object_id, point, anchor_id, point_in_anchor, stamp):
    return ObjectSpatialObservation(
        object_id=object_id,
        label="backpack",
        label_zh="背包",
        camera_xyz=(0.0, 0.0, 1.0),
        map_xyz=point,
        spatial_quality="METRIC_LIDAR",
        confidence=0.9,
        provenance={
            "timestamp": stamp,
            "frame_id": f"rgb_{object_id}",
            "timestamp_quality": "measured_clock_mapping",
            "source_sensor": "rgbd",
            "depth_method": "mask_robust",
            "map_id": "map-a",
            "map_epoch": "epoch-a",
            "anchor_id": anchor_id,
            "anchor_graph_revision": 1,
            "point_in_anchor": point_in_anchor,
            "sensor_profile_id": "profile-a",
            "anchor_status": "anchored",
        },
    )


def test_anchor_contract_is_persisted_and_single_observation_has_unknown_variance():
    object_map = SemanticObjectMap()
    object_map.update([_observation("frame-o1", (1.0, 2.0, 0.3), "k1", (0.1, 0.2, 0.3), 10.0)])
    entry = object_map.objects["obj_001"]
    anchor = entry.observation_anchors[0]
    assert anchor.map_id == "map-a"
    assert anchor.map_epoch == "epoch-a"
    assert anchor.anchor_id == "k1"
    assert anchor.point_in_anchor == (0.1, 0.2, 0.3)
    assert anchor.depth_method == "mask_robust"
    assert anchor.transform_valid is True
    assert entry.position_variance_xyz is None


def test_reprojection_prefers_persistent_anchor_without_historical_tf():
    object_map = SemanticObjectMap(merge_distance_m=2.0)
    object_map.update([_observation("frame-o1", (1.0, 0.0, 0.2), "k1", (1.0, 0.0, 0.2), 10.0)])
    calls = {"tf": 0, "anchor": 0}

    def missing_tf(_camera_xyz, _stamp):
        calls["tf"] += 1
        return None

    def anchor_lookup(anchor_id, point, **identity):
        calls["anchor"] += 1
        assert anchor_id == "k1"
        assert identity == {"map_id": "map-a", "map_epoch": "epoch-a"}
        return (point[0] + 2.0, point[1] + 3.0, point[2])

    report = object_map.reproject_observations(missing_tf, anchor_lookup=anchor_lookup)
    assert report == {"objects": 1, "observations": 1}
    assert calls == {"tf": 0, "anchor": 1}
    assert object_map.objects["obj_001"].map_xyz == (3.0, 3.0, 0.2)


def test_wrong_or_missing_anchor_degrades_to_capture_time_tf_lookup():
    object_map = SemanticObjectMap()
    object_map.update([_observation("frame-o1", (1.0, 0.0, 0.2), "gone", (1, 0, 0.2), 10.0)])
    report = object_map.reproject_observations(
        lambda camera, stamp: (camera[0] + stamp, camera[1], camera[2]),
        anchor_lookup=lambda *args, **kwargs: None,
    )
    assert report["observations"] == 1
    assert object_map.objects["obj_001"].map_xyz == (10.0, 0.0, 1.0)
