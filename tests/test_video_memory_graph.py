import json
import tempfile
import unittest
from pathlib import Path

import networkx as nx

from app.video.models import FrameAnalysisResult
from app.video.video_memory_graph import build_video_memory_graph


class VideoMemoryGraphTest(unittest.TestCase):
    def test_exports_json_and_graphml(self) -> None:
        frame = FrameAnalysisResult(
            frame_id=0,
            timestamp_sec=0.0,
            image_path="frame.jpg",
            annotated_frame_path="annotated.jpg",
            scene_summary="测试",
            objects=[
                {
                    "object_id": "frame_000000_obj_001",
                    "label": "phone",
                    "label_zh": "手机",
                    "confidence": 0.9,
                    "bbox": [0.4, 0.4, 0.6, 0.6],
                    "track_id": "track_001",
                    "is_target_candidate": True,
                }
            ],
            relations=[],
        )
        result = {
            "task": {
                "target": "手机",
                "canonical_target": "手机",
                "video_path": "walk.mp4",
                "detector": "mock",
            },
            "video_meta": {
                "video_path": "walk.mp4",
                "fps": 30.0,
                "duration_sec": 1.0,
                "frame_count": 30,
                "width": 640,
                "height": 480,
                "sampled_keyframes": 1,
            },
            "target_found": True,
            "candidate_regions": [],
            "best_evidence": {
                "timestamp_sec": 0.0,
                "frame_id": 0,
                "confidence": 0.9,
                "evidence_score": 0.8,
                "description": "手机位于中央。",
            },
        }
        with tempfile.TemporaryDirectory() as tmpdir:
            paths = build_video_memory_graph(result, [frame], tmpdir)
            payload = json.loads(
                paths["video_memory_graph_json"].read_text(encoding="utf-8")
            )
            graph = nx.read_graphml(paths["video_memory_graph_graphml"])

        self.assertTrue(any(node["type"] == "TargetNode" for node in payload["nodes"]))
        self.assertIn("target_best_evidence", graph.nodes)


if __name__ == "__main__":
    unittest.main()
