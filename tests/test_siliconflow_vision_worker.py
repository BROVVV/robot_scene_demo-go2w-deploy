"""Unit tests for the SiliconFlow quick-detection worker helpers."""

from __future__ import annotations

import unittest

from app.detectors.siliconflow_vision_worker import (
    _matched_objects,
    _normalize_quick_bbox,
)


class QuickBboxNormalizationTest(unittest.TestCase):
    def test_normalized_coordinates_are_kept(self) -> None:
        bbox = _normalize_quick_bbox([0.1, 0.2, 0.4, 0.6], 1280, 720)
        self.assertEqual(bbox, [0.1, 0.2, 0.4, 0.6])

    def test_pixel_coordinates_are_normalized(self) -> None:
        bbox = _normalize_quick_bbox([251, 477, 373, 753], 1280, 720)
        self.assertAlmostEqual(bbox[0], 251 / 1280)
        self.assertAlmostEqual(bbox[1], 477 / 720)
        self.assertAlmostEqual(bbox[2], 373 / 1280)
        self.assertAlmostEqual(bbox[3], 1.0)

    def test_dict_bbox_is_accepted(self) -> None:
        bbox = _normalize_quick_bbox(
            {"x1": 0.1, "y1": 0.2, "x2": 0.4, "y2": 0.6}, 1280, 720
        )
        self.assertEqual(bbox, [0.1, 0.2, 0.4, 0.6])

    def test_degenerate_bbox_is_rejected(self) -> None:
        self.assertIsNone(_normalize_quick_bbox([0.5, 0.5, 0.5, 0.5], 1280, 720))
        self.assertIsNone(_normalize_quick_bbox([0.9, 0.2, 0.4, 0.6], 1280, 720))
        self.assertIsNone(_normalize_quick_bbox("not-a-bbox", 1280, 720))


class MatchedObjectsTest(unittest.TestCase):
    def test_only_matched_objects_are_returned(self) -> None:
        result = {
            "objects": [
                {
                    "id": "obj_001",
                    "name": "backpack",
                    "name_zh": "书包",
                    "confidence": 0.85,
                    "bbox_2d": {"x1": 0.1, "y1": 0.2, "x2": 0.4, "y2": 0.6},
                },
                {
                    "id": "obj_002",
                    "name": "chair",
                    "name_zh": "椅子",
                    "confidence": 0.9,
                    "bbox_2d": {"x1": 0.0, "y1": 0.0, "x2": 1.0, "y2": 1.0},
                },
            ],
            "target_decision": {
                "matched_object_ids": ["obj_001"],
            },
        }
        objects = _matched_objects(result, "黑色书包")
        self.assertEqual(len(objects), 1)
        self.assertEqual(objects[0]["label"], "backpack 书包")
        self.assertEqual(objects[0]["score"], 0.85)
        self.assertEqual(objects[0]["bbox_2d"], [0.1, 0.2, 0.4, 0.6])

    def test_no_match_returns_empty(self) -> None:
        result = {
            "objects": [],
            "target_decision": {"matched_object_ids": []},
        }
        self.assertEqual(_matched_objects(result, "手机"), [])


if __name__ == "__main__":
    unittest.main()
