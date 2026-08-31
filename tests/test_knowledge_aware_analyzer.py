from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from app.schemas import KnowledgeAwareSceneResult, SceneAnalysisResult
from app.services.knowledge_aware_analyzer import KnowledgeAwareAnalyzer
from app.services.knowledge_output_writer import write_knowledge_aware_outputs


ROOT = Path(__file__).resolve().parents[1]


class KnowledgeAwareAnalyzerTest(unittest.TestCase):
    def test_enrich_base_scene_builds_full_result(self) -> None:
        scene = _base_scene()

        result = KnowledgeAwareAnalyzer(update_kb=False).enrich_base_scene(
            scene,
            "找到桌子上的手机",
        )

        self.assertIsInstance(result, KnowledgeAwareSceneResult)
        self.assertEqual(result.parsed_task.task_type, "find_object")
        self.assertGreaterEqual(len(result.predictive_scene_graph.nodes), 1)
        self.assertGreaterEqual(len(result.task_plan.steps), 1)
        self.assertIn("任务计划", result.final_answer_zh)

    def test_writes_knowledge_aware_outputs(self) -> None:
        scene = _base_scene()
        result = KnowledgeAwareAnalyzer(update_kb=False).enrich_base_scene(
            scene,
            "找到桌子上的手机",
        )

        with tempfile.TemporaryDirectory() as tmpdir:
            paths = write_knowledge_aware_outputs(result, tmpdir)

            expected = {
                "knowledge_aware_result",
                "parsed_task",
                "retrieved_knowledge",
                "predictive_scene_graph_graphml",
                "hypotheses",
                "knowledge_updates",
                "reasoning_report",
                "llm_search_hypotheses",
                "quadruped_search_plan",
                "reasoned_predictive_scene_graph",
                "reasoned_predictive_scene_graph_graphml",
                "actionability_report",
                "quadruped_ros2_motion_plan",
                "visual_grounding_report",
                "motion_horizon_decision",
            }
            self.assertEqual(set(paths), expected)
            self.assertTrue(all(path.is_file() for path in paths.values()))
            payload = json.loads(paths["knowledge_aware_result"].read_text(encoding="utf-8"))
            KnowledgeAwareSceneResult.model_validate(payload)
            motion_payload = json.loads(
                paths["motion_horizon_decision"].read_text(encoding="utf-8")
            )
            self.assertIn("recommended_distance_m", motion_payload)


def _base_scene() -> SceneAnalysisResult:
    data = json.loads((ROOT / "examples" / "mock_scene_result.json").read_text(encoding="utf-8"))
    return SceneAnalysisResult.model_validate(data)


if __name__ == "__main__":
    unittest.main()
