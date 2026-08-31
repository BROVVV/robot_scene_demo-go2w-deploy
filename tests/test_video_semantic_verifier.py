from __future__ import annotations

import unittest

from app.video.semantic_verifier import _verification_region
from app.video.target_profile import TargetProfile


class VideoSemanticVerifierTests(unittest.TestCase):
    def test_relation_target_crop_includes_nearest_explicit_anchor(self) -> None:
        target = {
            "object_id": "bin",
            "label": "trash_bin",
            "label_zh": "垃圾桶",
            "bbox": [0.30, 0.50, 0.40, 0.80],
        }
        objects = [
            target,
            {
                "object_id": "water",
                "label": "water_cooler",
                "label_zh": "饮水机",
                "bbox": [0.10, 0.45, 0.25, 0.85],
            },
            {
                "object_id": "chair",
                "label": "chair",
                "label_zh": "椅子",
                "bbox": [0.45, 0.20, 0.70, 0.80],
            },
        ]
        profile = TargetProfile(
            raw_query="饮水机旁边的蓝色垃圾桶",
            canonical_name_zh="蓝色垃圾桶",
            relation_constraints=["next to water dispenser"],
            context_labels_en=["water dispenser"],
            context_labels_zh=["饮水机"],
        )

        bbox, context = _verification_region(target, objects, profile)

        self.assertEqual(bbox, [0.10, 0.45, 0.40, 0.85])
        self.assertEqual(context[0]["object_id"], "water")

    def test_plain_target_keeps_tight_candidate_region(self) -> None:
        target = {"object_id": "phone", "bbox": [0.2, 0.2, 0.3, 0.4]}
        profile = TargetProfile(raw_query="手机", canonical_name_zh="手机")
        bbox, context = _verification_region(target, [target], profile)
        self.assertEqual(bbox, target["bbox"])
        self.assertEqual(context, [])


if __name__ == "__main__":
    unittest.main()
