from __future__ import annotations

import shutil
import tempfile
import unittest
from pathlib import Path

from app.knowledge import kb_store
from app.knowledge.retrieval import retrieve_relevant_knowledge
from app.knowledge.scene_kb import SceneKnowledgeBase


ROOT = Path(__file__).resolve().parents[1]


class SceneKBTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tmpdir = Path(tempfile.mkdtemp())
        shutil.copytree(ROOT / "data" / "scene_kb", self.tmpdir / "scene_kb")
        self.kb_dir = self.tmpdir / "scene_kb"

    def tearDown(self) -> None:
        shutil.rmtree(self.tmpdir)

    def test_search_relevant_knowledge(self) -> None:
        items = retrieve_relevant_knowledge(
            target_text="找到手机",
            room_type="office",
            location_hint="floor_5",
            kb_dir=self.kb_dir,
        )

        item_types = {item.knowledge_type for item in items}
        self.assertIn("object_location_prior", item_types)
        self.assertIn("room_type_prior", item_types)
        self.assertIn("environment_layout", item_types)

    def test_get_floor_layout(self) -> None:
        layout = SceneKnowledgeBase(self.kb_dir).get_floor_layout("floor_5")

        self.assertIsNotNone(layout)
        self.assertEqual(layout.floor_id, "floor_5")
        self.assertGreaterEqual(len(layout.doors), 1)

    def test_append_observation_and_update_confidence(self) -> None:
        kb_store.append_observation(
            {
                "observation_id": "obs_test_001",
                "timestamp": "2026-06-16T11:00:00+08:00",
                "location_hint": "floor_5",
                "summary_zh": "测试观察记录。",
                "confidence": 0.6,
            },
            kb_dir=self.kb_dir,
        )

        self.assertTrue(kb_store.update_confidence("phone", 0.9, kb_dir=self.kb_dir))
        data = kb_store.load_kb(self.kb_dir)
        phone_prior = next(
            prior for prior in data.object_location_priors if prior.object_name == "phone"
        )

        self.assertEqual(phone_prior.confidence, 0.9)
        self.assertEqual(data.observations[-1].observation_id, "obs_test_001")


if __name__ == "__main__":
    unittest.main()
