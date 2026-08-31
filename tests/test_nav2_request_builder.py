import os, unittest
from unittest.mock import patch
from app.navigation.nav2_config import Nav2Settings, env_bool, resolve_setup_bash
from app.navigation.nav2_models import Nav2Mode, Nav2Pose
from app.navigation.nav2_request_builder import make_request
from app.live_robot.navigation_gate import EXECUTE_REQUIRED, evaluate_navigation_gate
class RequestBuilderTest(unittest.TestCase):
    def test_plan_only_needs_no_execution_permission(self):
        r=make_request(mode=Nav2Mode.PLAN_ONLY,goal=Nav2Pose(),settings=Nav2Settings())
        self.assertFalse(r.allow_execute)
    def test_execute_requires_environment_and_operator(self):
        settings=Nav2Settings(allow_execute=True,footprint_confirmed=True,emergency_stop_confirmed=True)
        gate=evaluate_navigation_gate("execute",{name: True for name in EXECUTE_REQUIRED}).to_dict()
        r=make_request(mode="execute",goal=Nav2Pose(),settings=settings,allow_execute=True,
            operator_confirmed=True,footprint_confirmed=True,estop_confirmed=True,
            capability_gate_result=gate)
        self.assertTrue(r.safety_confirmation.complete)
    def test_bool_parser(self):
        with patch.dict(os.environ,{"X_BOOL":"yes"}): self.assertTrue(env_bool("X_BOOL"))
    def test_invalid_setup_path_is_not_silently_accepted(self):
        value=resolve_setup_bash("/missing/setup.bashe/setup.bas","not-installed")
        self.assertEqual(value,"/missing/setup.bashe/setup.bas")
