from __future__ import annotations

import json

from app.mapping.map_quality_evaluator import build_report, load_object_events
from app.perception.depth_object_localizer import (
    ObjectSpatialObservation,
    enrich_scene_objects_with_spatial,
)


def _localized(object_id: str, xyz: tuple[float, float, float]):
    return ObjectSpatialObservation(
        object_id=object_id,
        label="椅子",
        camera_xyz=(0.1, 0.2, 1.0),
        map_xyz=xyz,
        spatial_quality="METRIC_LIDAR",
        provenance={"map_frame": "pslam_map"},
    )


def test_spatial_enrichment_matches_source_id_not_label_or_order():
    scene_objects = [
        {"frame_object_id": "chair_b", "label": "椅子"},
        {"frame_object_id": "chair_a", "label": "椅子"},
        {"frame_object_id": "chair_missing", "label": "椅子"},
    ]
    enriched = enrich_scene_objects_with_spatial(
        scene_objects,
        [_localized("chair_a", (1.0, 0.0, 0.5)),
         _localized("chair_b", (2.0, 0.0, 0.5))],
        image_timestamp=123.5,
    )

    assert enriched[0]["map_xyz"] == [2.0, 0.0, 0.5]
    assert enriched[1]["map_xyz"] == [1.0, 0.0, 0.5]
    assert "map_xyz" not in enriched[2]
    assert enriched[0]["source_object_id"] == "chair_b"
    assert enriched[0]["map_frame"] == "pslam_map"
    assert enriched[0]["image_timestamp"] == 123.5


def test_event_loader_groups_by_persistent_id_and_never_by_label(tmp_path):
    events = tmp_path / "events.jsonl"
    rows = [
        {"event": "other", "objects": []},
        {"event": "semantic_frame_objects_localized", "objects": [
            {"label": "椅子", "source_object_id": "a", "persistent_object_id": "obj_1",
             "map_xyz": [1, 0, 0]},
            {"label": "椅子", "source_object_id": "b", "persistent_object_id": "obj_2",
             "map_xyz": [2, 0, 0]},
            {"label": "椅子", "source_object_id": "c", "map_xyz": [3, 0, 0]},
        ]},
        {"event": "semantic_frame_objects_localized", "objects": [
            {"label": "椅子", "source_object_id": "a2", "persistent_object_id": "obj_1",
             "map_xyz": [1.1, 0, 0]},
        ]},
    ]
    events.write_text("".join(json.dumps(row, ensure_ascii=False) + "\n" for row in rows))

    grouped = load_object_events(str(events))

    assert grouped == {
        "椅子#obj_1": [[1.0, 0.0, 0.0], [1.1, 0.0, 0.0]],
        "椅子#obj_2": [[2.0, 0.0, 0.0]],
    }


def test_single_frame_object_is_not_reported_as_zero_error_pass(tmp_path):
    events = tmp_path / "events.jsonl"
    events.write_text(json.dumps({
        "event": "semantic_frame_objects_localized",
        "objects": [{
            "label": "椅子", "persistent_object_id": "obj_1",
            "map_xyz": [1.0, 0.0, 0.0],
        }],
    }, ensure_ascii=False) + "\n")

    report = build_report(
        map_path=None, trajectory_path=None, objects_path=None,
        expected_instances=None, overlap_max_dist_m=0.1,
        object_events_path=str(events),
    )

    assert report["semantic"]["repeated_object_count"] == 0
    assert report["semantic"]["object_position_rms_m"] != report["semantic"]["object_position_rms_m"]
