from scripts.go2w.remote_control_semantic_mapping_worker import (
    _extract_teleop_geometric_relations,
)


def test_teleop_worker_restores_direction_relations_without_duplicate_near():
    objects = [
        {
            "frame_object_id": "f1",
            "label_zh": "椅子",
            "bbox_2d": [0.10, 0.10, 0.30, 0.30],
            "camera_xyz": [-0.4, 0.0, 1.0],
            "confidence": 0.9,
        },
        {
            "frame_object_id": "f2",
            "label_zh": "纸箱",
            "bbox_2d": [0.60, 0.55, 0.80, 0.80],
            "camera_xyz": [0.4, 0.0, 1.6],
            "confidence": 0.9,
        },
    ]

    relations = _extract_teleop_geometric_relations(objects, {})
    keys = {(item["subject_id"], item["object_id"], item["relation"]) for item in relations}

    assert ("f1", "f2", "left_of") in keys
    assert ("f1", "f2", "above") in keys
    assert ("f1", "f2", "in_front_of") in keys
    assert not any(item["relation"] == "near" for item in relations)
