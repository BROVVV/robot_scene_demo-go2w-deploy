from __future__ import annotations

import json
import shutil
import tempfile
import unittest
from pathlib import Path

from app.knowledge import kb_store
from app.knowledge.kb_updater import (
    extract_candidate_facts,
    update_knowledge_from_scene,
)
from app.reasoning.task_parser import parse_robot_task
from app.schemas import KnowledgeAwareSceneResult, SceneAnalysisResult


ROOT = Path(__file__).resolve().parents[1]


class KBUpdaterTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tmpdir = Path(tempfile.mkdtemp())
        shutil.copytree(ROOT / "data" / "scene_kb", self.tmpdir / "scene_kb")
        self.kb_dir = self.tmpdir / "scene_kb"

    def tearDown(self) -> None:
        shutil.rmtree(self.tmpdir)

    def test_extracts_stable_and_temporary_facts(self) -> None:
        scene = _knowledge_mock().base_scene
        task = parse_robot_task("找到手机")

        facts = extract_candidate_facts(scene, task)
        fact_types = {fact.fact_type for fact in facts}

        self.assertIn("room_type_prior", fact_types)
        self.assertIn("task_memory", fact_types)
        phone_fact = next(fact for fact in facts if fact.fact_type == "task_memory")
        self.assertFalse(phone_fact.stable)

    def test_update_appends_observation_and_ignores_temporary_state(self) -> None:
        scene = _knowledge_mock().base_scene
        task = parse_robot_task("找到手机")
        before = kb_store.load_kb(self.kb_dir)

        updates = update_knowledge_from_scene(scene, task, kb_dir=self.kb_dir)
        after = kb_store.load_kb(self.kb_dir)

        self.assertEqual(len(after.observations), len(before.observations) + 1)
        self.assertTrue(any(update.update_type == "confirmed" for update in updates))
        self.assertTrue(
            any(
                update.knowledge_type == "task_memory"
                and update.update_type == "ignored"
                for update in updates
            )
        )

    def test_door_state_is_temporary(self) -> None:
        scene = _base_mock()
        task = parse_robot_task("看看 503 是不是开着门")

        facts = extract_candidate_facts(scene, task)

        self.assertTrue(
            any(
                fact.fact_type == "temporary_state" and not fact.stable
                for fact in facts
            )
        )


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
