from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from scripts.evaluate_live_search_reasoners import evaluate


class ReasonerReplayEvaluatorTests(unittest.TestCase):
    def _session(self, root: Path) -> Path:
        session = root / "session"
        session.mkdir()
        graph = {
            "nodes": [
                {
                    "node_id": "trash", "node_type": "object",
                    "label": "trash bin", "label_zh": "垃圾桶",
                    "category": "container", "source": "observed",
                    "confidence": 0.9, "evidence_level": "observed_confirmed",
                    "attributes": {"attributes": ["blue"], "stable_position_2d": "left"},
                },
                {
                    "node_id": "water", "node_type": "object",
                    "label": "water cooler", "label_zh": "饮水机",
                    "category": "appliance", "source": "observed",
                    "confidence": 0.9, "evidence_level": "observed_confirmed",
                    "attributes": {"stable_position_2d": "left"},
                },
            ],
            "edges": [
                {
                    "edge_id": "near", "source_node_id": "trash",
                    "target_node_id": "water", "relation": "near",
                    "source": "observed", "confidence": 0.9,
                    "evidence_level": "observed_confirmed",
                }
            ],
        }
        (session / "scene_graph.json").write_text(json.dumps(graph), encoding="utf-8")
        observations = [{"frame_id": value} for value in (1, 2, 3)]
        (session / "frame_observations.json").write_text(
            json.dumps(observations), encoding="utf-8"
        )
        (session / "target_profile.json").write_text(
            json.dumps({"resolver_source": "llm"}), encoding="utf-8"
        )
        (session / "crop_verify_results.json").write_text(
            json.dumps({"attempted": 2}), encoding="utf-8"
        )
        (session / "search_directive.json").write_text(
            json.dumps({"kind": "reobserve_sector"}), encoding="utf-8"
        )
        (session / "target_search.json").write_text(json.dumps({
            "task": {"detector": "llm"},
            "best_evidence": {"timestamp_sec": 1.2},
            "timeline": [
                {"type": "direct_detection", "timestamp_sec": 1.2}
            ],
        }), encoding="utf-8")
        (session / "evidence_gating_report.json").write_text(json.dumps({
            "target_found": False,
            "candidates": [
                {"blocking_rules": ["crop"]}, {"blocking_rules": []}
            ],
        }), encoding="utf-8")
        return session

    def test_reports_real_artifact_metrics_and_shadow_invariant(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            session = self._session(root)
            report = evaluate(
                session=str(session),
                target="饮水机旁边的蓝色垃圾桶",
                output_dir=str(root / "report"),
            )
            self.assertEqual(report["detector_calls"], 3)
            self.assertEqual(report["recorded_llm_calls_inferred"], 6)
            self.assertEqual(report["verify_reject_count"], 1)
            self.assertEqual(report["time_to_candidate"], 1.2)
            self.assertIsNone(report["time_to_target"])
            self.assertTrue(report["success_proxy"])
            self.assertFalse(report["target_confirmed"])
            self.assertTrue(report["actual_shadow_behavior_matches_legacy"])
            self.assertEqual(report["dangerous_forward_request_count"], 0)
            self.assertTrue(all(
                row["shadow_executed_step"] == row["legacy_step"]
                for row in report["results"]
            ))
            self.assertTrue((root / "report/reasoner_comparison.json").is_file())
            self.assertTrue((root / "report/reasoner_metrics.json").is_file())
            self.assertTrue((root / "report/shadow_comparison.json").is_file())

    def test_event_log_counts_only_verified_physical_progress(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            session = self._session(root)
            event_log = root / "events.jsonl"
            event_log.write_text("\n".join([
                json.dumps({"event": "step_start", "step": "l20"}),
                json.dumps({"event": "step_verified", "step": "l20", "distance_m": 0.02}),
                json.dumps({"event": "step_start", "step": "f"}),
                json.dumps({"event": "step_verified", "step": "f", "distance_m": 0.18}),
            ]) + "\n", encoding="utf-8")
            report = evaluate(
                session=str(session), target="手机",
                output_dir=str(root / "report"), event_log=str(event_log),
            )
            self.assertEqual(report["search_steps"], 2)
            self.assertEqual(report["total_turn_deg"], 20.0)
            self.assertAlmostEqual(report["estimated_distance"], 0.2)
            self.assertIsNone(report["negative_revisit_count"])


if __name__ == "__main__":
    unittest.main()
