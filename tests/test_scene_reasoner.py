from __future__ import annotations

import json
import unittest
from pathlib import Path
from unittest import mock

from app.knowledge.retrieval import retrieve_relevant_knowledge
from app.reasoning.scene_reasoner import reason_about_scene
from app.reasoning.task_parser import parse_robot_task
from app.schemas import KnowledgeAwareSceneResult, SceneAnalysisResult
from app.services.psg_builder import build_predictive_scene_graph
from app.task_understanding import llm_task_interpreter as _llm_interpreter


ROOT = Path(__file__).resolve().parents[1]


class SceneReasonerTest(unittest.TestCase):
    def setUp(self) -> None:
        # §14-10 离线桩，与 tests/test_task_planner.py 那处同一个机制：
        # 本文件两处 `parse_robot_task` 都不带 llm_client ⇒ 会打真实
        # SiliconFlow（实测 16.4 秒/条）。断言的是**场景推理**结果，
        # 与 LLM 怎么解析任务无关。
        patcher = mock.patch.object(
            _llm_interpreter,
            "_default_openai_client",
            side_effect=RuntimeError("offline stub: real LLM disabled in tests"),
        )
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_generates_ranked_hypotheses_for_invisible_phone(self) -> None:
        scene = _knowledge_mock().base_scene
        task = parse_robot_task("找到手机")
        knowledge = retrieve_relevant_knowledge(
            target_text=task.raw_text,
            room_type="office",
            kb_dir=ROOT / "data" / "scene_kb",
        )
        psg = build_predictive_scene_graph(scene, knowledge, task)

        reasoning = reason_about_scene(scene, task, knowledge, psg)

        self.assertGreaterEqual(len(reasoning.hypotheses), 1)
        self.assertEqual(reasoning.hypotheses[0].status, "proposed")
        self.assertGreater(reasoning.hypotheses[0].probability, 0.5)
        self.assertIn("当前画面没有直接确认", reasoning.reasoning_summary_zh)
        self.assertIn("重新观察", reasoning.recommended_action_zh)

    def test_visible_target_returns_verified_hypothesis(self) -> None:
        scene = _base_mock()
        task = parse_robot_task(scene.target_decision.target_text)
        psg = build_predictive_scene_graph(scene, [], task)

        reasoning = reason_about_scene(scene, task, [], psg)

        self.assertEqual(len(reasoning.hypotheses), 1)
        self.assertEqual(reasoning.hypotheses[0].status, "verified")
        self.assertIn("已经匹配到任务目标", reasoning.reasoning_summary_zh)


def _knowledge_mock() -> KnowledgeAwareSceneResult:
    data = json.loads(
        (ROOT / "examples" / "mock_knowledge_aware_result.json").read_text(
            encoding="utf-8"
        )
    )
    return KnowledgeAwareSceneResult.model_validate(data)


def _base_mock() -> SceneAnalysisResult:
    data = json.loads((ROOT / "examples" / "mock_scene_result.json").read_text(encoding="utf-8"))
    return SceneAnalysisResult.model_validate(data)


if __name__ == "__main__":
    unittest.main()
