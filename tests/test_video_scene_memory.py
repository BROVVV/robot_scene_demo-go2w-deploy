import unittest

from app.video.models import FrameAnalysisResult
from app.video.target_profile import TargetProfile
from app.video.video_memory_builder import VideoMemoryBuilder
from app.video.video_scene_reasoner import VideoSceneReasoner


class VideoSceneMemoryTest(unittest.TestCase):
    def test_target_not_found_still_builds_environment_memory(self) -> None:
        frame = FrameAnalysisResult(
            frame_id=30,
            timestamp_sec=1.0,
            image_path="frame.jpg",
            annotated_frame_path=None,
            scene_summary="机器狗经过一段走廊，前方可见地面和墙面。",
            objects=[
                {
                    "object_id": "floor",
                    "label": "floor",
                    "label_zh": "地面",
                    "category": "surface",
                    "confidence": 0.9,
                    "bbox": [0.0, 0.5, 1.0, 1.0],
                    "image_position": "lower_center",
                    "is_target_candidate": False,
                },
                {
                    "object_id": "wall",
                    "label": "wall",
                    "label_zh": "墙",
                    "category": "structure",
                    "confidence": 0.8,
                    "bbox": [0.0, 0.0, 1.0, 0.7],
                    "image_position": "center",
                    "is_target_candidate": False,
                },
            ],
            relations=[],
        )
        profile = TargetProfile(
            raw_query="手机",
            canonical_name_zh="手机",
            primary_labels_en=["phone"],
            likely_regions_zh=["桌面", "沙发"],
        )
        result = VideoSceneReasoner("手机", profile).reason("video_001", frame)
        memories = VideoMemoryBuilder().build([result])

        self.assertFalse(result.target_evidence["directly_found"])
        self.assertTrue(result.memory_update["should_write"])
        self.assertGreater(len(result.negative_evidence), 0)
        self.assertGreater(len(result.psg_hypotheses), 0)
        self.assertEqual(memories[0]["memory_kind"], "environment_observation")
        self.assertFalse(memories[0]["target_context"]["found"])


if __name__ == "__main__":
    unittest.main()
