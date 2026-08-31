from __future__ import annotations

import unittest

from app.config import Settings
from app.vision.schema import CandidateObject
from app.vision.score_fusion import apply_score_fusion, fuse_score


class ScoreFusionTest(unittest.TestCase):
    def setUp(self) -> None:
        self.settings = Settings(siliconflow_api_key="")
        self.candidate = CandidateObject(
            object_id="obj_001",
            label="phone",
            score=0.6,
            detector_score=0.6,
        )

    def test_vlm_score_increases_final_score(self) -> None:
        baseline = fuse_score(self.candidate, None, self.settings)
        verified = fuse_score(
            self.candidate,
            {
                "is_target": True,
                "target_match_score": 0.95,
                "attribute_score": 0.9,
                "context_score": 0.8,
            },
            self.settings,
        )
        self.assertGreater(verified, baseline)

    def test_decisions_cover_all_threshold_bands(self) -> None:
        apply_score_fusion(
            self.candidate,
            {
                "is_target": True,
                "target_match_score": 0.95,
                "attribute_score": 0.95,
                "context_score": 0.95,
            },
            self.settings,
        )
        self.assertEqual(self.candidate.decision, "confirmed")
        rejected = CandidateObject("obj_002", "remote", score=0.1, detector_score=0.1)
        apply_score_fusion(
            rejected,
            {
                "is_target": False,
                "target_match_score": 0.05,
                "attribute_score": 0.0,
                "context_score": 0.0,
                "rejection_reason": "not a phone",
            },
            self.settings,
        )
        self.assertEqual(rejected.decision, "rejected")


if __name__ == "__main__":
    unittest.main()
