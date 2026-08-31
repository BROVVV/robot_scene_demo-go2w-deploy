from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from app.memory.llm_experience_memory import LLMExperienceMemory
from app.schemas import SpatialExperienceMemory


class LLMExperienceMemoryTest(unittest.TestCase):
    def test_retrieves_similar_positive_and_negative_experience(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            store = LLMExperienceMemory(Path(tmpdir) / "memory.jsonl")
            positive = _memory("mem_found", "found", "桌子右侧")
            negative = _memory("mem_not_found", "not_found", "走廊左侧")
            self.assertTrue(store.append_if_novel(positive))
            self.assertTrue(store.append_if_novel(negative))
            self.assertFalse(store.append_if_novel(negative))

            result = store.retrieve(
                target_text="找手机",
                scene_type="office",
                visible_anchor_labels=["桌子", "门"],
                top_k=5,
            )

        outcomes = {item["outcome"] for item in result}
        self.assertEqual(outcomes, {"found", "not_found"})
        self.assertGreaterEqual(result[0]["retrieval_score"], result[-1]["retrieval_score"])


def _memory(
    memory_id: str,
    outcome: str,
    region: str,
) -> SpatialExperienceMemory:
    return SpatialExperienceMemory(
        memory_id=memory_id,
        created_at="2026-06-25T00:00:00+00:00",
        target_text="找手机",
        target_normalized="找手机",
        scene_type="office",
        visible_anchor_labels=["桌子", "门"],
        hypothesis_region_zh=region,
        hypothesis_rationale_zh="历史搜索经验",
        action_taken=["turn_right", "stop_and_reobserve"],
        outcome=outcome,  # type: ignore[arg-type]
        visual_evidence_summary_zh="历史视觉结果",
        negative_evidence_zh=[],
        confidence_after_outcome=0.7,
    )


if __name__ == "__main__":
    unittest.main()
