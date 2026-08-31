import unittest

from app.video.models import FrameAnalysisResult, VideoMetadata
from app.video.object_tracker import track_objects
from app.video.target_search import search_target_in_video, target_label_match_score
from app.video.target_profile import TargetProfile


def _frame(objects):
    return FrameAnalysisResult(
        frame_id=30,
        timestamp_sec=1.0,
        image_path="frame.jpg",
        annotated_frame_path="annotated.jpg",
        scene_summary="测试场景",
        objects=objects,
        relations=[],
    )


class TargetSearchTest(unittest.TestCase):
    def setUp(self) -> None:
        self.meta = VideoMetadata(
            video_path="walk.mp4",
            fps=30.0,
            duration_sec=2.0,
            frame_count=60,
            width=640,
            height=480,
            sampled_keyframes=1,
        )

    def test_matches_chinese_target_to_english_synonym(self) -> None:
        self.assertEqual(target_label_match_score("手机", "cell phone"), 1.0)
        self.assertEqual(target_label_match_score("找到桌子上的手机", "phone"), 1.0)
        self.assertEqual(target_label_match_score("找到厕所", "restroom sign"), 1.0)

    def test_selects_direct_evidence_and_nearby_reference(self) -> None:
        frame = _frame(
            [
                {
                    "object_id": "phone",
                    "label": "cell phone",
                    "label_zh": "手机",
                    "confidence": 0.9,
                    "bbox": [0.6, 0.65, 0.75, 0.8],
                },
                {
                    "object_id": "table",
                    "label": "table",
                    "label_zh": "桌子",
                    "confidence": 0.8,
                    "bbox": [0.3, 0.4, 0.9, 0.9],
                },
            ]
        )
        tracks = track_objects([frame])
        result = search_target_in_video("手机", self.meta, [frame], tracks, detector="mock")

        self.assertTrue(result["target_found"])
        self.assertEqual(result["best_evidence"]["image_position"], "lower_right")
        self.assertEqual(result["best_evidence"]["nearby_objects"][0]["label"], "table")
        self.assertFalse(
            result["navigation_interpretation"]["can_generate_real_navigation"]
        )

    def test_does_not_invent_target_without_evidence(self) -> None:
        frame = _frame(
            [
                {
                    "object_id": "wall",
                    "label": "wall",
                    "label_zh": "墙",
                    "confidence": 0.8,
                    "bbox": [0.0, 0.0, 1.0, 1.0],
                }
            ]
        )
        tracks = track_objects([frame])
        result = search_target_in_video(
            "钥匙",
            self.meta,
            [frame],
            tracks,
            detector="mock",
            enable_knowledge=True,
        )

        self.assertFalse(result["target_found"])
        self.assertIsNone(result["best_evidence"])
        self.assertEqual(
            result["reason"], "target_not_observed_and_no_strong_contextual_clue"
        )

    def test_restroom_task_uses_door_and_sign_as_candidate_region(self) -> None:
        frame = _frame(
            [
                {
                    "object_id": "door",
                    "label": "door",
                    "label_zh": "门",
                    "confidence": 0.9,
                    "bbox": [0.2, 0.2, 0.4, 0.9],
                },
                {
                    "object_id": "sign",
                    "label": "sign",
                    "label_zh": "指示牌",
                    "confidence": 0.85,
                    "bbox": [0.4, 0.1, 0.55, 0.3],
                },
            ]
        )
        tracks = track_objects([frame])
        result = search_target_in_video(
            "找到厕所",
            self.meta,
            [frame],
            tracks,
            detector="llm",
            enable_knowledge=True,
        )

        self.assertFalse(result["target_found"])
        self.assertEqual(result["task"]["canonical_target"], "厕所")
        self.assertEqual(
            result["reason"], "target_not_observed_but_contextual_clues_exist"
        )
        self.assertEqual(result["candidate_regions"][0]["priority"], "medium")
        self.assertIn("门", result["candidate_regions"][0]["nearby_objects"])
        self.assertIn("指示牌", result["candidate_regions"][0]["nearby_objects"])

    def test_frame_reasoner_can_match_unlisted_natural_language_target(self) -> None:
        frame = _frame(
            [
                {
                    "object_id": "frame_obj",
                    "source_object_id": "obj_001",
                    "label": "machine",
                    "label_zh": "设备",
                    "confidence": 0.88,
                    "bbox": [0.2, 0.2, 0.6, 0.8],
                }
            ]
        )
        frame.metadata["target_decision"] = {
            "is_present": True,
            "matched_object_ids": ["obj_001"],
            "match_reason_zh": "设备具有纸盒和打印结构，符合 A3 打印设备。",
        }
        profile = TargetProfile(
            raw_query="找到能打印 A3 的设备",
            canonical_name_zh="A3打印机",
            primary_labels_en=["printer", "A3 printer"],
            attributes=["supports A3 paper"],
        )
        tracks = track_objects([frame])
        result = search_target_in_video(
            profile.raw_query,
            self.meta,
            [frame],
            tracks,
            detector="llm",
            target_profile=profile,
        )

        self.assertTrue(result["target_found"])
        self.assertEqual(
            result["best_evidence"]["evidence_source"],
            "frame_target_decision",
        )
        self.assertIn("A3", result["best_evidence"]["match_reason"])

    def test_grounded_sam_rejects_one_frame_low_confidence_open_vocab_hit(self) -> None:
        frame = _frame(
            [
                {
                    "object_id": "printer_false_positive",
                    "label": "large format printer",
                    "label_zh": "A3打印机",
                    "confidence": 0.29,
                    "bbox": [0.1, 0.1, 0.8, 0.8],
                }
            ]
        )
        profile = TargetProfile(
            raw_query="请帮我找一台能打印 A3 纸的设备",
            canonical_name_zh="A3打印机",
            primary_labels_en=["printer", "A3 printer"],
            aliases_en=["large format printer"],
        )
        tracks = track_objects([frame])
        result = search_target_in_video(
            profile.raw_query,
            self.meta,
            [frame],
            tracks,
            detector="grounded_sam",
            target_profile=profile,
        )

        self.assertFalse(result["target_found"])
        self.assertIsNone(result["best_evidence"])

    def test_grounded_sam_requires_semantic_verifier_for_constraints(self) -> None:
        frame = _frame(
            [
                {
                    "object_id": "generic_door",
                    "label": "white cabinet door with red handle",
                    "label_zh": "带红色把手的白色柜门",
                    "confidence": 0.82,
                    "bbox": [0.1, 0.1, 0.6, 0.9],
                }
            ]
        )
        frame.metadata["semantic_verification"] = {
            "attempted": True,
            "is_present": False,
            "reason": "把手实际为黑色，不满足红色约束。",
        }
        profile = TargetProfile(
            raw_query="找到红色把手的白色柜门",
            canonical_name_zh="带红色把手的白色柜门",
            primary_labels_en=["white cabinet door"],
            aliases_en=["white cabinet door with red handle"],
            attributes=["white", "red handle"],
        )
        tracks = track_objects([frame])
        result = search_target_in_video(
            profile.raw_query,
            self.meta,
            [frame],
            tracks,
            detector="grounded_sam",
            target_profile=profile,
        )
        self.assertFalse(result["target_found"])

        frame.metadata["semantic_verification"] = {
            "attempted": True,
            "is_present": True,
            "confidence": 0.9,
            "reason": "白色柜门具有清晰红色把手。",
            "matched_bboxes": [[0.1, 0.1, 0.6, 0.9]],
        }
        result = search_target_in_video(
            profile.raw_query,
            self.meta,
            [frame],
            tracks,
            detector="grounded_sam",
            target_profile=profile,
        )
        self.assertTrue(result["target_found"])
        self.assertIn("红色把手", result["best_evidence"]["match_reason"])


if __name__ == "__main__":
    unittest.main()
