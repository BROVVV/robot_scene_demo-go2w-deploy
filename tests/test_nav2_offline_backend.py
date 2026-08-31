import tempfile, unittest
from pathlib import Path
from app.navigation.nav2_config import Nav2Settings
from app.navigation.nav2_gateway import Nav2Gateway
from app.navigation.nav2_models import Nav2Mode, Nav2Pose
from app.navigation.nav2_request_builder import make_request
class OfflineTest(unittest.TestCase):
    def test_explicit_offline_preview(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d); (root/"data/nav2_demo").mkdir(parents=True)
            source=Path("data/nav2_demo/offline_path_fixture.json")
            (root/source).write_text(source.read_text(encoding="utf-8"),encoding="utf-8")
            settings=Nav2Settings(output_dir="out",offline_fixture=str(source))
            req=make_request(mode=Nav2Mode.OFFLINE_PREVIEW,goal=Nav2Pose(x=2,y=1),settings=settings)
            handle=Nav2Gateway(settings,root).plan(req)
            path=Nav2Gateway(settings,root).get_path(handle.request_id)
            self.assertFalse(path["is_real_nav2_path"]); self.assertEqual(path["backend"],"offline_preview")
