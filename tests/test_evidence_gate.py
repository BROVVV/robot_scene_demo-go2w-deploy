from __future__ import annotations

import json
import unittest
from pathlib import Path

from app.config import Settings
from app.reasoning.evidence_gate import (
    EvidenceGateConfig,
    evaluate_candidate,
    gate_scene_target,
)
from app.schemas import SceneAnalysisResult


ROOT = Path(__file__).resolve().parents[1]


class EvidenceGateTest(unittest.TestCase):
    def test_llm_commonsense_cannot_confirm(self) -> None:
        report = evaluate_candidate(
            {
                "candidate_id": "hyp",
                "source": "llm_commonsense",
                "has_visual_evidence": False,
                "bbox": None,
                "detector_score": 0.99,
            },
            EvidenceGateConfig(require_crop_verify=False),
        )

        self.assertFalse(report["target_found"])
        self.assertIn("LLM_COMMONSENSE_CANNOT_CONFIRM", report["blocking_rules"])

    def test_visual_candidate_requires_crop_verify_and_score(self) -> None:
        report = evaluate_candidate(
            {
                "candidate_id": "obj_001",
                "source": "visual_detector",
                "has_visual_evidence": True,
                "bbox": [0.1, 0.1, 0.2, 0.2],
                "detector_score": 0.85,
                "crop_verify_score": 0.82,
            },
            EvidenceGateConfig(require_crop_verify=True, min_score=0.72),
        )

        self.assertTrue(report["target_found"])
        self.assertEqual(report["target_status"], "visual_confirmed")

    def test_missing_api_key_does_not_require_crop_verify_for_visual_bbox(self) -> None:
        scene = SceneAnalysisResult.model_validate(
            json.loads(
                (ROOT / "examples" / "mock_scene_result.json").read_text(
                    encoding="utf-8"
                )
            )
        )

        gated, report = gate_scene_target(
            scene,
            Settings(
                siliconflow_api_key="",
                evidence_gating_enabled=True,
                target_confirmation_require_visual_evidence=True,
                target_confirmation_require_bbox=True,
                target_confirmation_require_crop_verify=True,
                target_confirmation_min_score=0.72,
            ),
        )

        self.assertTrue(gated.target_decision.is_present)
        self.assertTrue(report["target_found"])
        self.assertIn(
            "TARGET_CONFIRMATION_CROP_VERIFY_UNAVAILABLE",
            report["passed_rules"],
        )


if __name__ == "__main__":
    unittest.main()
