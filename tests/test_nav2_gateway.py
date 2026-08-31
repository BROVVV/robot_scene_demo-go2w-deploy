import tempfile, unittest
from pathlib import Path
from app.navigation.nav2_config import Nav2Settings
from app.navigation.nav2_gateway import Nav2Gateway
from app.navigation.nav2_models import Nav2Mode, Nav2Pose
from app.navigation.nav2_request_builder import make_request
class GatewayTest(unittest.TestCase):
    def test_no_silent_fallback(self):
        with tempfile.TemporaryDirectory() as d:
            settings=Nav2Settings(output_dir="out",setup_bash="/definitely/missing")
            req=make_request(mode=Nav2Mode.PLAN_ONLY,goal=Nav2Pose(),settings=settings)
            gateway=Nav2Gateway(settings,d); h=gateway.plan(req); status=gateway.get_status(h.request_id)
            self.assertEqual(status.state.value,"unavailable"); self.assertEqual(status.error_code,"NAV2_ROS_SETUP_NOT_FOUND")
    def test_visual_preview_never_spawns_worker(self):
        with tempfile.TemporaryDirectory() as d:
            settings=Nav2Settings(output_dir="out",setup_bash="/definitely/missing")
            req=make_request(mode=Nav2Mode.VISUAL_PREVIEW,goal=None,settings=settings)
            gateway=Nav2Gateway(settings,d); h=gateway.plan(req); status=gateway.get_status(h.request_id)
            self.assertEqual(status.state.value,"unavailable"); self.assertEqual(status.error_code,"NAV2_VISUAL_PREVIEW_ONLY")
            self.assertFalse((Path(d)/"out"/"jobs"/h.request_id/"worker.pid").exists())
