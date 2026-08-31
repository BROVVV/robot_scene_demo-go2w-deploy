import tempfile
import unittest
from pathlib import Path

from app.video.video_search_summarizer import (
    enrich_search_result,
    write_video_reasoning_report,
)
from app.video.video_scene_reasoner import FrameSceneReasoningResult


class VideoNoTargetNotEmptyTest(unittest.TestCase):
    def test_no_target_result_contains_memory_negative_evidence_and_report(self) -> None:
        frame = FrameSceneReasoningResult(
            video_id="video_001",
            frame_id=0,
            timestamp_sec=0.0,
            image_path="frame.jpg",
            annotated_frame_path=None,
            scene_understanding={"room_type": "corridor"},
            landmarks=[],
            objects=[],
            regions=[],
            target_evidence={"directly_found": False},
            negative_evidence=["当前走廊未发现手机。"],
            psg_hypotheses=[],
            memory_update={"should_write": True},
            reasoning_summary={},
        )
        memory = {
            "memory_kind": "environment_observation",
            "room_type": "corridor",
            "time_range": [0.0, 0.0],
            "scene_summary": "走廊中未发现手机。",
            "place_signature": {"stable_landmarks": ["墙"]},
            "target_context": {"found": False, "candidate_found": False},
            "importance": "low",
        }
        search_result = {
            "task": {
                "target": "手机",
                "video_path": "walk.mp4",
                "detector": "mock",
            },
            "video_meta": {"sampled_keyframes": 1},
            "target_found": False,
            "best_evidence": None,
            "timeline": [],
            "candidate_regions": [],
            "navigation_interpretation": {
                "suggestion": "继续搜索。",
                "reason": "无位姿。",
            },
        }
        psg = {
            "hypotheses": [
                {
                    "hypothesis_id": "hyp_001",
                    "type": "low_priority_for_target",
                    "summary": "走廊优先级较低。",
                    "related_place": "corridor",
                    "supporting_evidence": ["未发现手机。"],
                    "confidence": 0.7,
                }
            ]
        }
        result = enrich_search_result(
            search_result,
            [frame],
            [memory],
            psg,
            memory_written_count=1,
            memory_store_path="memory.jsonl",
            retrieved_memory_context={},
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            report = write_video_reasoning_report(
                result, Path(tmpdir) / "video_reasoning_report.md"
            )
            text = report.read_text(encoding="utf-8")

        self.assertFalse(result["target_found"])
        self.assertGreater(result["environment_memories_written"], 0)
        self.assertGreater(result["negative_evidence_count"], 0)
        self.assertGreater(len(result["psg_hypotheses"]), 0)
        self.assertIn("已观察环境记忆", text)
        self.assertIn("负目标证据", text)
        self.assertIn("PSG", text)
        self.assertIn("后续搜索建议", text)


if __name__ == "__main__":
    unittest.main()
