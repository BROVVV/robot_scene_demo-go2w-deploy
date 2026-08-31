from __future__ import annotations

import json
import unittest
from pathlib import Path

from pydantic import ValidationError

from app.schemas import (
    EnvironmentKnowledge,
    KnowledgeAwareSceneResult,
    SceneAnalysisResult,
)


ROOT = Path(__file__).resolve().parents[1]


class KnowledgeSchemaTest(unittest.TestCase):
    def test_mock_knowledge_aware_result_validates(self) -> None:
        data = json.loads(
            (ROOT / "examples" / "mock_knowledge_aware_result.json").read_text(
                encoding="utf-8"
            )
        )

        result = KnowledgeAwareSceneResult.model_validate(data)

        self.assertEqual(result.parsed_task.task_type, "find_object")
        self.assertEqual(result.parsed_task.target_object, "phone")
        self.assertFalse(result.base_scene.target_decision.is_present)
        self.assertGreaterEqual(len(result.predictive_scene_graph.inferred_node_ids), 1)
        self.assertEqual(result.task_plan.plan_type, "find_object")

    def test_existing_scene_analysis_result_still_validates(self) -> None:
        data = json.loads(
            (ROOT / "examples" / "mock_scene_result.json").read_text(encoding="utf-8")
        )

        result = SceneAnalysisResult.model_validate(data)

        self.assertTrue(result.target_decision.is_present)
        self.assertGreaterEqual(len(result.objects), 1)

    def test_new_schemas_keep_extra_fields_forbidden(self) -> None:
        with self.assertRaises(ValidationError):
            EnvironmentKnowledge.model_validate(
                {
                    "building_id": "building_a",
                    "floor_id": "floor_5",
                    "known_rooms": [],
                    "known_doors": [],
                    "corridor_layout": "east-west",
                    "room_type_priors": [],
                    "object_location_priors": [],
                    "last_updated_at": "2026-06-16T10:00:00+08:00",
                    "confidence": 0.8,
                    "unexpected": "must fail",
                }
            )


if __name__ == "__main__":
    unittest.main()
