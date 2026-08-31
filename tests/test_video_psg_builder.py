import json
import tempfile
import unittest
from pathlib import Path

from app.video.video_psg_builder import build_video_psg
from app.video.video_scene_reasoner import FrameSceneReasoningResult


class VideoPSGBuilderTest(unittest.TestCase):
    def test_exports_graphml_and_hypotheses(self) -> None:
        result = FrameSceneReasoningResult(
            video_id="video_001",
            frame_id=0,
            timestamp_sec=0.0,
            image_path="frame.jpg",
            annotated_frame_path=None,
            scene_understanding={
                "room_type": "corridor",
                "scene_summary": "走廊",
            },
            landmarks=[],
            objects=[],
            regions=[],
            target_evidence={
                "target": "手机",
                "directly_found": False,
                "candidate_found": False,
            },
            negative_evidence=["未发现手机。"],
            psg_hypotheses=[
                {
                    "hypothesis": "走廊对手机搜索优先级较低。",
                    "supporting_evidence": ["未发现手机。"],
                    "relation_type": "low_priority_for_target",
                    "confidence": 0.7,
                }
            ],
            memory_update={
                "should_write": True,
                "memory_kind": "environment_observation",
                "importance": "low",
                "summary": "走廊中未发现手机。",
                "tags": [],
            },
            reasoning_summary={},
        )
        memory = {
            "memory_id": "mem_001",
            "room_type": "corridor",
            "time_range": [0.0, 0.0],
            "frame_ids": [0],
            "scene_summary": "走廊中未发现手机。",
            "place_signature": {"stable_landmarks": []},
            "regions": [],
            "target_context": {"found": False},
            "importance": "low",
        }
        with tempfile.TemporaryDirectory() as tmpdir:
            payload, paths = build_video_psg("手机", [result], [memory], tmpdir)
            hypotheses = json.loads(
                paths["video_hypotheses"].read_text(encoding="utf-8")
            )

            self.assertTrue(paths["video_predictive_scene_graph"].is_file())
            self.assertTrue(paths["video_hypotheses"].is_file())
            self.assertGreater(len(hypotheses["hypotheses"]), 0)
            self.assertTrue(
                any(
                    edge["type"] == "target_not_found_in"
                    for edge in payload["edges"]
                )
            )


if __name__ == "__main__":
    unittest.main()
