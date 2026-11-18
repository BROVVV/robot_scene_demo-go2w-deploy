"""Unit tests for the automatic-only ExperimentReadiness (plan section 27)."""

from __future__ import annotations

from types import SimpleNamespace
import unittest
from unittest import mock

from app.live_robot.experiment_readiness import compute_experiment_readiness
from app.navigation.go2w_experimental_backend import (
    GO2W_EXPERIMENTAL_CAPABILITIES,
)
from app.navigation.nav2_search_backend import NAV2_CAPABILITIES
from app.navigation.robot_backend import RobotCapabilities
from scripts.go2w.run_semantic_exploration import _probe_readiness


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

    # ---- §P3-4：degraded 必须按能力实报，不能无条件追加 --------------------

    def test_metric_pose_degraded_only_when_backend_lacks_it(self) -> None:
        """相对后端：确实没有地图系导航 -> 如实报降级。"""
        readiness = compute_experiment_readiness(
            camera_fresh=True, bundle_fresh=True, llm_available=True,
            motion_action_available=True, robot_mode_ok=True,
            emergency_stop_available=True,
            capabilities=GO2W_EXPERIMENTAL_CAPABILITIES,
        )
        self.assertIn("metric_pose_unavailable", readiness.degraded)
        self.assertFalse(readiness.checks["metric_pose_available"])

    def test_metric_pose_not_degraded_for_nav2(self) -> None:
        """2026-10-15 11:05 的假降级：backend=nav2 却报没有地图位姿。

        同一次会话的 `backend_health` 写着 supports_metric_navigation=true，
        `experiment_readiness` 写着 false —— 两份报告互相打脸。
        """
        readiness = compute_experiment_readiness(
            camera_fresh=True, bundle_fresh=True, llm_available=True,
            motion_action_available=True, robot_mode_ok=True,
            emergency_stop_available=True,
            backend="nav2", capabilities=NAV2_CAPABILITIES,
        )
        self.assertNotIn("metric_pose_unavailable", readiness.degraded)
        self.assertTrue(readiness.checks["metric_pose_available"])
        self.assertTrue(readiness.capabilities["supports_metric_navigation"])

    # ---- §P3-4：启动自检报告的能力必须是所选后端的 ------------------------

    def _probe(self, backend: str):
        """用最省事的桩跑一遍真实的 `_probe_readiness`。

        只桩掉 ROS 那几件东西（action client / 服务 / sport / odom）；能力表的
        判定是这条测试真正要看的那一段，必须跑真代码。
        """
        node = SimpleNamespace(
            _client=SimpleNamespace(server_is_ready=lambda: True),
            _arm_client=SimpleNamespace(service_is_ready=lambda: True),
            _stop_srv=SimpleNamespace(service_is_ready=lambda: True),
            _sport=None,
            _odom=None,
        )
        args = SimpleNamespace(backend=backend, spool_root="/nonexistent/spool")
        return _probe_readiness(args, node)

    def test_probe_reports_nav2_capabilities(self) -> None:
        with mock.patch.dict(
            "os.environ", {"SILICONFLOW_API_KEY": "test-key"}, clear=False
        ):
            readiness = self._probe("nav2")
        self.assertEqual(readiness.backend, "nav2")
        self.assertTrue(readiness.capabilities["supports_metric_navigation"])
        self.assertFalse(readiness.capabilities["supports_relative_translation"])
        self.assertEqual(
            tuple(readiness.capabilities["allowed_motion_primitives"]),
            ("NAVIGATE_POSE", "STOP"),
        )
        self.assertNotIn("metric_pose_unavailable", readiness.degraded)

    def test_probe_reports_relative_capabilities_for_legacy_backend(self) -> None:
        with mock.patch.dict(
            "os.environ", {"SILICONFLOW_API_KEY": "test-key"}, clear=False
        ):
            readiness = self._probe("go2w_experimental")
        self.assertFalse(readiness.capabilities["supports_metric_navigation"])
        self.assertTrue(readiness.capabilities["supports_relative_translation"])
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
        # §4.3：go2w_experimental 不得再出现在生产路径的任何默认值里，
        # 这份报告也不例外——不传就是生产默认的 nav2。
        self.assertEqual(payload["backend"], "nav2")
        self.assertEqual(
            compute_experiment_readiness(
                camera_fresh=True, bundle_fresh=True, llm_available=True,
                motion_action_available=True, robot_mode_ok=True,
                emergency_stop_available=True, backend="mock_metric",
            ).to_dict()["backend"],
            "mock_metric",
        )


if __name__ == "__main__":
    unittest.main()
