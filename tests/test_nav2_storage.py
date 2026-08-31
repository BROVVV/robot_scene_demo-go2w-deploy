import tempfile, unittest
from pathlib import Path
from app.navigation.nav2_storage import *
class StorageTest(unittest.TestCase):
    def test_utf8_atomic_and_jsonl(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/"x.json"; atomic_write_json(p,{"中文":"正常"}); self.assertEqual(read_json(p)["中文"],"正常")
            j=Path(d)/"x.jsonl"; append_jsonl(j,{"n":1}); append_jsonl(j,{"n":2}); self.assertEqual(read_jsonl(j,1),[{"n":2}])
