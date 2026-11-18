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

    def test_plain_chair_alone_does_not_confirm_a_relation_target(self) -> None:
        # §64：只看到普通 chair 不能确认 relation target。
        # 画面里没有「挂着书包的椅子」的关联物体时，裁剪框不许被撑大，
        # context 也必须是空的 —— 否则 VLM 会拿到一个「看起来包含关系」的框，
        # 把一张普通椅子判成 relation target。
        chair = {
            "object_id": "det_000",
            "label": "office chair",
            "label_zh": "办公椅",
            "bbox": [0.30, 0.20, 0.45, 0.70],
        }
        profile = TargetProfile(
            raw_query="挂着书包的椅子",
            canonical_name_zh="挂着书包的椅子",
            relation_constraints=["backpack on chair"],
            context_labels_en=["backpack"],
            context_labels_zh=["书包"],
        )

        bbox, context = _verification_region(chair, [chair], profile)

        self.assertEqual(bbox, chair["bbox"])
        self.assertEqual(context, [])

    def test_reasoning_profile_drives_the_video_crop_region(self) -> None:
        # 真机链路的真实组合：run_autonomous_loop 用 app.reasoning 的 resolver
        # 解析目标，却把画像交给 app.video 的 _verification_region 裁剪。
        # 这两个模块里各有一份逐字节相同的 TargetProfile，锚点字段只补了一份时，
        # 生产线就是在这里 AttributeError 崩的 —— 这条测试锁住那个组合。
        from app.reasoning.target_profile import TargetProfile as ReasoningProfile

        chair = {"object_id": "c0", "label": "office chair", "bbox": [0.30, 0.20, 0.45, 0.70]}
        backpack = {"object_id": "c1", "label": "backpack", "bbox": [0.46, 0.25, 0.55, 0.60]}
        glass_door = {"object_id": "c2", "label": "glass door", "bbox": [0.47, 0.10, 0.60, 0.90]}
        profile = ReasoningProfile(
            raw_query="挂着书包的椅子",
            canonical_name_zh="挂着书包的椅子",
            primary_labels_en=["chair"],
            relation_constraints=["backpack hanging on chair"],
            relation_anchor_labels_en=["backpack"],
            relation_anchor_labels_zh=["书包"],
        )

        bbox, context = _verification_region(chair, [chair, backpack, glass_door], profile)

        # 并进的是背包，不是更近/更大的玻璃门。
        self.assertEqual(bbox, [0.30, 0.20, 0.55, 0.70])
        self.assertEqual([item["object_id"] for item in context], ["c1"])

    def test_plain_target_keeps_tight_candidate_region(self) -> None:
        target = {"object_id": "phone", "bbox": [0.2, 0.2, 0.3, 0.4]}
        profile = TargetProfile(raw_query="手机", canonical_name_zh="手机")
        bbox, context = _verification_region(target, [target], profile)
        self.assertEqual(bbox, target["bbox"])
        self.assertEqual(context, [])


if __name__ == "__main__":
    unittest.main()
