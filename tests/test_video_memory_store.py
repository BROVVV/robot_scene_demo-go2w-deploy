import tempfile
import unittest
from pathlib import Path

from app.memory.video_memory_store import VideoMemoryStore


class VideoMemoryStoreTest(unittest.TestCase):
    def test_jsonl_round_trip_and_negative_search(self) -> None:
        memory = {
            "memory_id": "mem_001",
            "video_id": "video_001",
            "room_type": "corridor",
            "scene_summary": "走廊中未发现手机。",
            "place_signature": {"stable_landmarks": ["墙", "地面"]},
            "regions": [{"name": "front_floor"}],
            "target_context": {"target": "手机", "found": False},
            "importance": "medium",
            "tags": ["corridor", "手机"],
            "created_at": "2026-06-24T12:00:00+08:00",
        }
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "memory.jsonl"
            store = VideoMemoryStore(path)
            store.append(memory)
            items = store.load_all()
            negative = store.search_negative_evidence("手机")

        self.assertEqual(items[-1]["memory_id"], "mem_001")
        self.assertEqual(negative[0]["room_type"], "corridor")


if __name__ == "__main__":
    unittest.main()
