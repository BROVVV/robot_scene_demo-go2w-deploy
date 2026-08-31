from __future__ import annotations

import unittest

from app.llm_clients.siliconflow_client import (
    _build_fast_user_prompt,
    _normalize_fast_result,
)
from app.schemas import SceneAnalysisResult


class LLMPromptContractTest(unittest.TestCase):
    def test_prompt_mentions_structured_reasoning_hints(self) -> None:
        prompt = _build_fast_user_prompt("找到手机")

        self.assertIn("task_understanding", prompt)
        self.assertIn("scene_reasoning_hints", prompt)
        self.assertIn("不要写完整思维链", prompt)
        self.assertIn("bbox_2d", prompt)

    def test_normalizer_ignores_optional_reasoning_fields(self) -> None:
        normalized = _normalize_fast_result(
            {
                "scene_summary_zh": "办公室场景。",
                "objects": [
                    {
                        "name": "desk",
                        "name_zh": "桌子",
                        "category": "furniture",
                        "relative_to_robot": "front",
                        "confidence": 0.8,
                    }
                ],
                "relations": [],
                "target_decision": {
                    "is_present": False,
                    "matched_indices": [],
                    "match_reason_zh": "没有看到手机。",
                    "confidence": 0.7,
                },
                "route_plan": {"route_type": "explore_likely_location", "steps": []},
                "task_understanding": {
                    "task_type": "find_object",
                    "entities": ["phone"],
                    "constraints": [],
                    "uncertainty": "桌面有遮挡。",
                },
                "scene_reasoning_hints": {
                    "scene_type": "office",
                    "candidate_locations": ["desk surface"],
                    "supporting_evidence": ["visible desk"],
                    "recommended_next_observation": "靠近桌面。",
                },
            },
            "找到手机",
        )

        result = SceneAnalysisResult.model_validate(normalized)
        self.assertFalse(result.target_decision.is_present)
        self.assertEqual(result.route_plan.route_type, "explore_likely_location")
        self.assertEqual(result.objects[0].name, "desk")

    def test_normalizer_accepts_bbox_and_maps_model_enum_drift(self) -> None:
        normalized = _normalize_fast_result(
            {
                "objects": [
                    {
                        "name": "cup",
                        "name_zh": "水杯",
                        "bbox_2d": [0.2, 0.3, 0.5, 0.8],
                    },
                    {
                        "name": "person",
                        "name_zh": "人",
                        "bbox_2d": {"x1": 0.0, "y1": 0.0, "x2": 0.8, "y2": 1.0},
                    },
                ],
                "relations": [
                    {
                        "source_index": 2,
                        "target_index": 1,
                        "relation_type": "holding",
                    }
                ],
                "target_decision": {
                    "is_present": True,
                    "matched_indices": [1],
                },
                "route_plan": {
                    "steps": [{"action": "approach"}],
                },
            },
            "水杯",
        )

        result = SceneAnalysisResult.model_validate(normalized)
        self.assertEqual(result.objects[0].bbox_2d.x1, 0.2)
        self.assertEqual(result.relations[0].relation_type, "near")
        self.assertEqual(result.route_plan.steps[0].action, "stop")

    def test_zero_based_index_signal_applies_to_all_relations_and_target_matches(self) -> None:
        normalized = _normalize_fast_result(
            {
                "objects": [
                    {"name": "bin", "name_zh": "垃圾桶"},
                    {"name": "dispenser", "name_zh": "饮水机"},
                    {"name": "chair", "name_zh": "椅子"},
                ],
                "relations": [
                    {"source_index": 0, "target_index": 1, "relation_type": "next_to"},
                    {"source_index": 1, "target_index": 2, "relation_type": "behind"},
                ],
                "target_decision": {"is_present": True, "matched_indices": [1]},
            },
            "饮水机",
        )

        result = SceneAnalysisResult.model_validate(normalized)
        self.assertEqual(result.relations[0].source_id, "obj_001")
        self.assertEqual(result.relations[1].source_id, "obj_002")
        self.assertEqual(result.relations[1].target_id, "obj_003")
        self.assertEqual(result.target_decision.matched_object_ids, ["obj_002"])


if __name__ == "__main__":
    unittest.main()
