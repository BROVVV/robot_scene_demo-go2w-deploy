from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from app.config import Settings
from app.memory.observation_memory_store import ObservationMemoryStore


class ObservationMemoryStoreTest(unittest.TestCase):
    def test_rejects_memory_without_visual_provenance(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            store = ObservationMemoryStore(
                Path(tmpdir) / "memory.jsonl",
                settings=Settings(siliconflow_api_key=""),
            )
            with self.assertRaises(ValueError):
                store.append(
                    {
                        "memory_id": "mem_001",
                        "memory_type": "object_observation",
                        "label": "phone",
                        "evidence": {},
                    }
                )

    def test_appends_visual_observation(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            store = ObservationMemoryStore(
                Path(tmpdir) / "memory.jsonl",
                settings=Settings(siliconflow_api_key=""),
            )
            store.append(
                {
                    "memory_id": "mem_001",
                    "memory_type": "object_observation",
                    "label": "possible phone",
                    "evidence": {
                        "frame_id": "frame_001",
                        "bbox": [0.1, 0.1, 0.2, 0.2],
                    },
                }
            )

            self.assertEqual(len(store.load_all()), 1)


if __name__ == "__main__":
    unittest.main()
