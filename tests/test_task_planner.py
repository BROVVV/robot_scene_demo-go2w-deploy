from __future__ import annotations

import json
import unittest
from pathlib import Path
from unittest import mock

from app.knowledge.retrieval import retrieve_relevant_knowledge
from app.planning.task_planner import plan_task
from app.reasoning.scene_reasoner import reason_about_scene
from app.reasoning.task_parser import parse_robot_task
from app.schemas import KnowledgeAwareSceneResult, SceneAnalysisResult
from app.services.psg_builder import build_predictive_scene_graph
from app.task_understanding import llm_task_interpreter as _llm_interpreter


ROOT = Path(__file__).resolve().parents[1]


class TaskPlannerTest(unittest.TestCase):
    def setUp(self) -> None:
        # §14-10 离线桩。`parse_robot_task` 走 `run_task_understanding_pipeline`
        # 时不带 llm_client，而 `llm_task_interpreter.py:161` 是
        # `client = llm_client or _default_openai_client()` ⇒ 每条用例都去建真实
        # SiliconFlow client 打网络（实测 10~18 秒/条），结果随 LLM 输出漂移：
        # 本文件的 `test_plan_count_objects_has_count_state` 就会因为 LLM 把
        # 「数数这个房间里有几个椅子」判成 `inspect_area` 而红。
        # 桩把出口钉死在「不可用」，链路落到 `app/reasoning/task_parser.py:124`
        # 的 `_offline_compat_task`（确定性关键词匹配），用例因此可复现。
        # 本文件断言的是**规划层**行为（plan_type / steps），与 LLM 解析质量无关，
        # 所以钉死出口不损失任何分辨力。
        patcher = mock.patch.object(
            _llm_interpreter,
            "_default_openai_client",
            side_effect=RuntimeError("offline stub: real LLM disabled in tests"),
        )
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_plan_visible_find_object(self) -> None:
        scene = _base_mock()
        task = parse_robot_task(scene.target_decision.target_text)

        plan = plan_task(scene, task)

        self.assertEqual(plan.plan_type, "find_object")
        self.assertEqual(plan.steps[0].action_type, "move")
        self.assertIsNone(plan.count_state)

    def test_plan_invisible_find_object_uses_hypotheses(self) -> None:
        scene = _knowledge_mock().base_scene
        task = parse_robot_task("找到手机")
        knowledge = retrieve_relevant_knowledge(
            target_text=task.raw_text,
            room_type="office",
            kb_dir=ROOT / "data" / "scene_kb",
        )
        psg = build_predictive_scene_graph(scene, knowledge, task)
        reasoning = reason_about_scene(scene, task, knowledge, psg)

        plan = plan_task(scene, task, reasoning.hypotheses, psg)

        self.assertEqual(plan.plan_type, "find_object")
        self.assertGreaterEqual(len(plan.steps), 1)
        self.assertGreaterEqual(len(plan.fallback_steps), 1)
        self.assertIn("目标当前不可见", plan.summary_zh)

    def test_plan_count_objects_has_count_state(self) -> None:
        scene = _base_mock()
        task = parse_robot_task("数数这个房间里有几个椅子")

        plan = plan_task(scene, task)

        self.assertEqual(plan.plan_type, "count_objects")
        self.assertIsNotNone(plan.count_state)
        self.assertIn("obj_003", plan.count_state.counted_object_ids)
        self.assertGreaterEqual(len(plan.count_state.uncertain_regions), 1)

    def test_plan_check_door_state(self) -> None:
        task = parse_robot_task("看看 503 是不是开着门")
        plan = plan_task(_base_mock(), task)

        self.assertEqual(plan.plan_type, "check_door_state")
        self.assertEqual(plan.steps[1].action_type, "verify")

    def test_plan_navigate_to_location(self) -> None:
        task = parse_robot_task("去走廊尽头的房间")
        plan = plan_task(_base_mock(), task)

        self.assertEqual(plan.plan_type, "navigate_to_location")
        self.assertEqual(plan.steps[0].action_type, "observe")


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
