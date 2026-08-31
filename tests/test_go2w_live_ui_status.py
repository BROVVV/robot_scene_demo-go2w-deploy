import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path

from app.live_robot.ui_status import blockers_for_mode, load_live_ui_status
from streamlit.testing.v1 import AppTest


class Go2wLiveUiStatusTest(unittest.TestCase):
    def test_missing_runtime_is_fail_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            status = load_live_ui_status(spool_root=root / "spool", acceptance_root=root / "gates")
        self.assertIn("camera", blockers_for_mode("observe_only", status))
        self.assertIn("plan_gate_unavailable", blockers_for_mode("nav2_plan_only", status))
        self.assertIn("execute_gate_unavailable", blockers_for_mode("nav2_execute", status))

    def test_step_search_cannot_inherit_observe_permission(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            status = load_live_ui_status(spool_root=root / "spool", acceptance_root=root / "gates")
        blockers = blockers_for_mode("step_search", status)
        self.assertIn("motion_execution_disabled", blockers)
        self.assertIn("operator_not_armed", blockers)

    def test_streamlit_live_mode_shows_fail_closed_controls(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch.dict(
                "os.environ",
                {
                    "SILICONFLOW_API_KEY": "",
                    "GO2W_FRAME_SPOOL_DIR": str(Path(directory) / "spool"),
                },
            ):
                app = AppTest.from_file("streamlit_app.py", default_timeout=30).run()
                app.radio[0].set_value("Go2-W 实时目标搜索").run(timeout=30)
        self.assertEqual(len(app.exception), 0)
        self.assertIn("实时搜索模式", [item.label for item in app.selectbox])
        buttons = {item.label: item for item in app.button}
        self.assertTrue(buttons["只观察"].disabled)
        self.assertTrue(buttons["暂停"].disabled)
        self.assertTrue(buttons["取消"].disabled)
        self.assertTrue(buttons["紧急停止"].disabled)


if __name__ == "__main__":
    unittest.main()
