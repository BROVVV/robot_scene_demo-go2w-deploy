import tempfile, unittest
from pathlib import Path
from app.navigation.nav2_result_adapter import build_webui_payload
from app.navigation.nav2_storage import atomic_write_json
class PayloadTest(unittest.TestCase):
    def test_payload_has_contract(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d); atomic_write_json(p/"status.json",{"request_id":"x"}); atomic_write_json(p/"request.json",{})
            value=build_webui_payload(p)
            self.assertIn("cmd_vel_summary",value); self.assertIn("artifacts",value)
