from __future__ import annotations

import json
import unittest
from pathlib import Path

from app.knowledge.retrieval import retrieve_relevant_knowledge
from app.planning.task_planner import plan_task
from app.reasoning.scene_reasoner import reason_about_scene
from app.reasoning.task_parser import parse_robot_task
from app.schemas import KnowledgeAwareSceneResult, SceneAnalysisResult
from app.services.psg_builder import build_predictive_scene_graph


ROOT = Path(__file__).resolve().parents[1]


class TaskPlannerTest(unittest.TestCase):
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
