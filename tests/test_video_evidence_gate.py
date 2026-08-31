from __future__ import annotations

import unittest

from app.config import Settings
from app.video.models import FrameAnalysisResult
from app.video.pipeline import _gate_video_search_result


class VideoEvidenceGateTest(unittest.TestCase):
    def test_gate_checks_all_candidates_not_only_best_evidence(self) -> None:
        frame = FrameAnalysisResult(
            frame_id=12,
            timestamp_sec=2.4,
            image_path="frame_000012.jpg",
            annotated_frame_path="annotated_000012.jpg",
            scene_summary="桌面上有多个小物体。",
            objects=[
                {
                    "object_id": "high_unverified",
                    "label": "phone",
                    "label_zh": "手机",
                    "confidence": 0.95,
                    "bbox": [0.1, 0.1, 0.25, 0.25],
                    "is_target_candidate": True,
                },
                {
                    "object_id": "verified_lower_score",
                    "label": "phone",
                    "label_zh": "手机",
                    "confidence": 0.75,
                    "bbox": [0.55, 0.55, 0.7, 0.72],
                    "is_target_candidate": True,
                    "crop_verify": {
                        "is_target": True,
                        "target_match_score": 0.8,
                    },
                },
            ],
            relations=[],
        )
        search_result = {
            "task": {
                "target": "手机",
                "canonical_target": "手机",
                "detector": "grounded_sam",
            },
            "best_evidence": {
                "object_id": "high_unverified",
                "frame_id": 12,
                "timestamp_sec": 2.4,
                "confidence": 0.95,
                "bbox": [0.1, 0.1, 0.25, 0.25],
            },
        }

        report = _gate_video_search_result(
            search_result,
            [frame],
            Settings(
                siliconflow_api_key="test-key",
                evidence_gating_enabled=True,
                target_confirmation_require_visual_evidence=True,
                target_confirmation_require_bbox=True,
                target_confirmation_require_crop_verify=True,
                target_confirmation_min_score=0.72,
            ),
        )

        self.assertTrue(report["target_found"])
        self.assertEqual(report["candidate_id"], "verified_lower_score")
        self.assertEqual(report["best_evidence"]["object_id"], "verified_lower_score")
        self.assertGreaterEqual(len(report["candidates"]), 2)


if __name__ == "__main__":
    unittest.main()
