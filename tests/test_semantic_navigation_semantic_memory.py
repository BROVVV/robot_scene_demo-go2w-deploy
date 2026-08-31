from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from app.reasoning.semantic_navigation.semantic_memory import SemanticSearchMemory


class SemanticMemoryTests(unittest.TestCase):
    def test_sector_penalty_ttl_and_release(self):
        clock = [100.0]
        memory = SemanticSearchMemory(default_ttl_sec=10.0, now=lambda: clock[0])
        memory.add_negative(
            target_key="垃圾桶", heading_sector=2, reason="not seen",
            source_event_id="evt_1", confidence=0.8,
        )
        penalty, refs = memory.sector_penalty("垃圾桶", 2)
        self.assertGreater(penalty, 0.0)
        self.assertEqual(refs, ["evt_1"])
        self.assertEqual(memory.release(target_key="垃圾桶", heading_sector=2), 1)
        memory.add_negative(
            target_key="垃圾桶", heading_sector=2, reason="not seen",
            source_event_id="evt_2", confidence=0.8,
        )
        clock[0] = 111.0
        self.assertEqual(memory.active(), [])

    def test_session_memory_does_not_touch_existing_jsonl(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "memory.jsonl"
            path.write_text('{"keep": true}\n', encoding="utf-8")
            before = path.read_bytes()
            memory = SemanticSearchMemory()
            memory.add_negative(
                target_key="phone", heading_sector=0, reason="not seen",
                source_event_id="evt",
            )
            self.assertEqual(path.read_bytes(), before)

    def test_reads_existing_observation_store_without_writing(self):
        class Store:
            def __init__(self):
                self.calls = []

            def retrieve(self, target, top_k=None):
                self.calls.append((target, top_k))
                return [{"memory_id": "existing_visual_memory"}]

        store = Store()
        memory = SemanticSearchMemory(observation_store=store)
        self.assertEqual(
            memory.retrieve_long_term("手机", top_k=3),
            [{"memory_id": "existing_visual_memory"}],
        )
        self.assertEqual(store.calls, [("手机", 3)])


if __name__ == "__main__":
    unittest.main()
