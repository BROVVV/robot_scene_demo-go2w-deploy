from __future__ import annotations

import json
import unittest
from pathlib import Path

from app.knowledge.retrieval import retrieve_relevant_knowledge
from app.reasoning.hypothesis_generator import generate_scene_hypotheses
from app.reasoning.task_parser import parse_robot_task
from app.schemas import KnowledgeAwareSceneResult
from app.services.psg_builder import build_predictive_scene_graph


ROOT = Path(__file__).resolve().parents[1]


class HypothesisGeneratorTest(unittest.TestCase):
    def test_generates_phone_locations_from_psg_and_knowledge(self) -> None:
        scene = _knowledge_mock().base_scene
        task = parse_robot_task("找到手机")
        knowledge = retrieve_relevant_knowledge(
            target_text=task.raw_text,
            room_type="office",
            kb_dir=ROOT / "data" / "scene_kb",
        )
        psg = build_predictive_scene_graph(scene, knowledge, task)

        hypotheses = generate_scene_hypotheses(scene, task, knowledge, psg)

        locations = [hypothesis.possible_location for hypothesis in hypotheses]
        self.assertIn("键盘旁", locations)
        self.assertTrue(all(hypothesis.verification_action for hypothesis in hypotheses))
        self.assertGreaterEqual(hypotheses[0].probability, hypotheses[-1].probability)


def _knowledge_mock() -> KnowledgeAwareSceneResult:
    data = json.loads(
        (ROOT / "examples" / "mock_knowledge_aware_result.json").read_text(
            encoding="utf-8"
        )
    )
    return KnowledgeAwareSceneResult.model_validate(data)


if __name__ == "__main__":
    unittest.main()
