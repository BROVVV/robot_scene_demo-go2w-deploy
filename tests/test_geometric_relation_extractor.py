from __future__ import annotations

from app.spatial.geometric_relation_extractor import extract_geometric_relations


def test_same_frame_geometry_emits_id_bound_relations() -> None:
    relations = extract_geometric_relations([
        {"id": "a", "label": "桌", "bbox_2d": [0.1, 0.1, 0.3, 0.4], "camera_xyz": [0.0, 0.0, 1.0]},
        {"id": "b", "label": "桶", "bbox_2d": [0.65, 0.5, 0.85, 0.8], "camera_xyz": [0.2, 0.1, 1.5]},
    ])
    keys = {(item["subject_id"], item["object_id"], item["relation"]) for item in relations}
    assert ("a", "b", "left_of") in keys
    assert ("a", "b", "above") in keys
    assert ("a", "b", "in_front_of") in keys
    assert ("a", "b", "near") in keys
    assert all(item["provenance"].startswith("geometric_same_frame") for item in relations)


def test_overlapping_boxes_do_not_fabricate_order() -> None:
    relations = extract_geometric_relations([
        {"id": "a", "bbox_2d": [0.4, 0.4, 0.6, 0.6]},
        {"id": "b", "bbox_2d": [0.45, 0.45, 0.65, 0.65]},
    ])
    assert not any(item["relation"] in {"left_of", "above"} for item in relations)
