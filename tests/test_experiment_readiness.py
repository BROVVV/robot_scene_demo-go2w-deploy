"""Unit tests for the automatic-only ExperimentReadiness (plan section 27)."""

from __future__ import annotations

import unittest
from unittest import mock

from app.live_robot.experiment_readiness import compute_experiment_readiness
from app.navigation.robot_backend import RobotCapabilities


class TestExperimentReadiness(unittest.TestCase):
    def setUp(self) -> None:
        self.env_patcher = mock.patch.dict(
            "os.environ",
            {"SILICONFLOW_API_KEY": "test-key"},
            clear=False,
        )
        self.env_patcher.start()

    def tearDown(self) -> None:
        self.env_patcher.stop()

    def test_all_ok_ready(self) -> None:
        readiness = compute_experiment_readiness(
            camera_fresh=True, bundle_fresh=True, llm_available=True,
            motion_action_available=True, robot_mode_ok=True,
            emergency_stop_available=True, backend_healthy=True,
        )
        self.assertTrue(readiness.ready)
        self.assertIn("metric_pose_unavailable", readiness.degraded)

    def test_camera_missing_degrades(self) -> None:
        readiness = compute_experiment_readiness(
            camera_fresh=False, bundle_fresh=True, llm_available=True,
            motion_action_available=True, robot_mode_ok=True,
            emergency_stop_available=True,
        )
        self.assertFalse(readiness.ready)
        self.assertIn("camera_unavailable", readiness.degraded)

    def test_motion_unavailable_fails_closed(self) -> None:
        readiness = compute_experiment_readiness(
            camera_fresh=True, bundle_fresh=True, llm_available=True,
            motion_action_available=False, robot_mode_ok=True,
            emergency_stop_available=True,
        )
        self.assertFalse(readiness.ready)
        self.assertIn("motion_action_unavailable", readiness.degraded)

    def test_llm_key_missing_fails(self) -> None:
        with mock.patch.dict("os.environ", {}, clear=True):
            readiness = compute_experiment_readiness(
                camera_fresh=True, bundle_fresh=True, llm_available=True,
                motion_action_available=True, robot_mode_ok=True,
                emergency_stop_available=True,
            )
        self.assertFalse(readiness.ready)
        self.assertIn("llm_api_key_missing", readiness.degraded)

    def test_pose_stale_is_degraded_not_fatal(self) -> None:
        readiness = compute_experiment_readiness(
            camera_fresh=True, bundle_fresh=True, llm_available=True,
            motion_action_available=True, robot_mode_ok=True,
            emergency_stop_available=True,
            pose_freshness_if_available=False,
        )
        self.assertIn("pose_stale", readiness.degraded)

    def test_capabilities_reported(self) -> None:
        caps = RobotCapabilities(supports_relative_rotation=True)
        readiness = compute_experiment_readiness(
            camera_fresh=True, bundle_fresh=True, llm_available=True,
            motion_action_available=True, robot_mode_ok=True,
            emergency_stop_available=True,
            capabilities=caps,
        )
        self.assertTrue(readiness.capabilities["supports_relative_rotation"])
        self.assertFalse(readiness.capabilities["supports_global_pose"])

    def test_serializable(self) -> None:
        readiness = compute_experiment_readiness(
            camera_fresh=True, bundle_fresh=True, llm_available=True,
            motion_action_available=True, robot_mode_ok=True,
            emergency_stop_available=True,
        )
        payload = readiness.to_dict()
        self.assertTrue(payload["ready"])
        self.assertEqual(payload["backend"], "go2w_experimental")


if __name__ == "__main__":
    unittest.main()
