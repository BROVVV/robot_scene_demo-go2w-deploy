"""Unit tests for navigation-result recovery / replan semantics (plan 13)."""

from __future__ import annotations

import types
import unittest

from app.live_robot.autonomous_explorer import AutonomousExplorer
from app.live_robot.mock_observation_scene import (
    MockObservationScene,
    MockSceneStep,
    scenario_target_appears_after,
)
from app.navigation.backend_factory import MockBackend
from app.navigation.exploration_config import (
    ExplorationBudget,
    RecoveryConfig,
    load_exploration_policy,
)
from app.navigation.exploration_graph import ExplorationGraph
from app.navigation.robot_backend import (
    BackendHealth,
    NavigationStatus,
    navigation_result,
)


class TestExplorationRecovery(unittest.TestCase):
    def test_timeout_cancels_and_replans(self) -> None:
        scene = scenario_target_appears_after(2)
        backend = MockBackend(outcome_sequence=[NavigationStatus.TIMEOUT])
        explorer = AutonomousExplorer(
            target="t", observer=scene.observer(), matcher=scene.matcher(),
            verifier=scene.verifier(), backend=backend,
            graph=ExplorationGraph(session_id="t"),
            policy=load_exploration_policy(),
        )
        result = explorer.run()
        self.assertEqual(result.result, "TARGET_FOUND")
        self.assertTrue(backend._cancel_called)
        self.assertGreaterEqual(result.replans, 1)

    def test_backend_unavailable_retries_then_fails(self) -> None:
        scene = MockObservationScene(scenes=[
            MockSceneStep(objects=["desk"]),
        ])
        backend = MockBackend(
            outcome_sequence=[NavigationStatus.BACKEND_UNAVAILABLE] * 10
        )
        policy = load_exploration_policy()
        policy.recovery = RecoveryConfig(
            replan_after_failure=True, timeout_retry_count=0,
            backend_reconnect_attempts=2,
            backend_reconnect_delay_seconds=0.0,
        )
        explorer = AutonomousExplorer(
            target="t", observer=scene.observer(), matcher=scene.matcher(),
            verifier=scene.verifier(), backend=backend,
            graph=ExplorationGraph(session_id="t"), policy=policy,
        )
        result = explorer.run()
        # If the first navigation is BACKEND_UNAVAILABLE the loop replans;
        # with a permanently failing backend the budget eventually stops it.
        self.assertIn(result.result,
                      {"BACKEND_FAILURE", "SEARCH_EXHAUSTED", "MAX_STEPS_REACHED"})

    def test_operator_stop_during_wait(self) -> None:
        scene = scenario_target_appears_after(5)
        backend = MockBackend()
        policy = load_exploration_policy()
        policy.budget = ExplorationBudget(max_motion_steps=20)
        explorer = AutonomousExplorer(
            target="t", observer=scene.observer(), matcher=scene.matcher(),
            verifier=scene.verifier(), backend=backend,
            graph=ExplorationGraph(session_id="t"), policy=policy,
        )
        original = explorer._wait_result

        def stop_then_wait(handle, *, timeout_sec):
            explorer.request_stop()
            return original(handle, timeout_sec=timeout_sec)

        explorer._wait_result = stop_then_wait
        result = explorer.run()
        self.assertEqual(result.result, "OPERATOR_STOP")

    def test_failed_goal_marks_node_failure_count(self) -> None:
        scene = scenario_target_appears_after(3)
        backend = MockBackend(outcome_sequence=[NavigationStatus.FAILED])
        graph = ExplorationGraph(session_id="t")
        explorer = AutonomousExplorer(
            target="t", observer=scene.observer(), matcher=scene.matcher(),
            verifier=scene.verifier(), backend=backend, graph=graph,
            policy=load_exploration_policy(),
        )
        explorer.run()
        any_failures = any(
            node.navigation_fail_count > 0 or graph.sector_failure_count(sector) > 0
            for sector in range(12)
            for node in [graph.get_node(f"node_obs_00{i}") for i in range(1, 5)]
        )
        self.assertTrue(any_failures)


class _SubscriptionFedBackend:
    """就绪判据由订阅回调喂的后端 —— nav2 后端的形状。

    ``health()`` 在 ``spin_once`` 被调用够次数之前恒判不通过，且缓存初值是
    「未收到」。这正是 ``create_nav2_search_backend`` 里 ``localization_valid``
    的语义：不 spin 就永远是 ``False``。

    ``pumpable=False`` 时故意不暴露 ``spin_once``，用来证明"能恢复"确实是
    ``_pump_backend`` 带来的，不是重试次数自己等出来的。
    """

    def __init__(self, inner: object, *, spins_needed: int, pumpable: bool = True) -> None:
        self._inner = inner
        self._spins_needed = spins_needed
        self._spins = 0
        self.health_calls = 0
        if pumpable:
            self.spin_once = self._spin_once  # type: ignore[assignment]

    def _spin_once(self, timeout_sec: float = 0.1) -> None:
        self._spins += 1

    def __getattr__(self, name: str):
        return getattr(self._inner, name)

    def health(self):
        self.health_calls += 1
        healthy = self._inner.health()
        if self._spins >= self._spins_needed:
            return healthy
        return BackendHealth(
            ready=False,
            backend=healthy.backend,
            degraded=["NO_LOCALIZATION"],
            capabilities=healthy.capabilities,
            pose_quality=healthy.pose_quality,
            details={"reject_reason": "NO_LOCALIZATION", "localization_valid": False},
        )


class TestBackendBootstrapPump(unittest.TestCase):
    """bootstrap 重试必须把后端的 ROS 订阅转起来（nav2 后端靠订阅喂就绪）。"""

    def test_retry_pumps_backend_and_recovers(self) -> None:
        scene = scenario_target_appears_after(2)
        backend = _SubscriptionFedBackend(
            MockBackend(), spins_needed=1
        )
        explorer = AutonomousExplorer(
            target="t", observer=scene.observer(), matcher=scene.matcher(),
            verifier=scene.verifier(), backend=backend,
            graph=ExplorationGraph(session_id="t"),
            policy=load_exploration_policy(),
        )
        result = explorer.run()
        self.assertNotEqual(result.result, "BACKEND_FAILURE")
        self.assertGreaterEqual(backend._spins, 1)
        self.assertGreaterEqual(backend.health_calls, 1)

    def test_without_pump_the_same_backend_fails(self) -> None:
        # 对照组：同一个"靠订阅喂"的后端，只是没有 spin_once。
        # 重试次数照旧，仍然必然 BACKEND_FAILURE —— 证明恢复来自 pump 本身。
        scene = MockObservationScene(scenes=[MockSceneStep(objects=["desk"])])
        backend = _SubscriptionFedBackend(
            MockBackend(), spins_needed=1, pumpable=False
        )
        policy = load_exploration_policy()
        policy.recovery = RecoveryConfig(
            backend_reconnect_attempts=3,
            backend_reconnect_delay_seconds=0.0,
        )
        explorer = AutonomousExplorer(
            target="t", observer=scene.observer(), matcher=scene.matcher(),
            verifier=scene.verifier(), backend=backend,
            graph=ExplorationGraph(session_id="t"), policy=policy,
        )
        self.assertEqual(explorer.run().result, "BACKEND_FAILURE")
        self.assertEqual(backend.health_calls, 3)


class _StatusGatedBackend:
    """导航状态由回调喂的后端 —— nav2 后端的形状（与上面那条同类，另一处缝）。

    ``RosActionClientPort.poll()`` 的文档写着「必须有人持续 spin 这个 node，
    否则状态永远停在 UNKNOWN」：``get_navigation_status`` 读的是回调写进去的
    缓存，不 spin 就永远是 ``RUNNING``。这里复刻这条语义：``spin_once`` 够次数
    之前恒 ``RUNNING``，之后才给终态。

    ``pumpable=False`` 时故意不暴露 ``spin_once``，用来证明「等出结果」确实是
    ``_pump_backend`` 带来的，不是等待窗口自己熬出来的。
    """

    def __init__(self, inner: object, *, spins_needed: int, pumpable: bool = True) -> None:
        self._inner = inner
        self._spins_needed = spins_needed
        self._spins = 0
        self.status_calls = 0
        if pumpable:
            self.spin_once = self._spin_once  # type: ignore[assignment]

    def _spin_once(self, timeout_sec: float = 0.1) -> None:
        self._spins += 1

    def __getattr__(self, name: str):
        return getattr(self._inner, name)

    def get_navigation_status(self, handle):
        self.status_calls += 1
        goal_id = getattr(handle, "goal_id", "goal")
        if self._spins >= self._spins_needed:
            return navigation_result(
                goal_id, NavigationStatus.SUCCEEDED, message="mock succeeded"
            )
        return navigation_result(goal_id, NavigationStatus.RUNNING, message="mock running")


class _StepClock:
    """每读一次前进 ``step`` 秒的假钟：让等待窗口的轮数可数、不依赖真实时间。"""

    def __init__(self, *, start: float = 1000.0, step: float = 1.0) -> None:
        self._t = start
        self._step = step

    def __call__(self) -> float:
        value = self._t
        self._t += self._step
        return value


class TestNavigationWaitPump(unittest.TestCase):
    """等待导航结果时也必须把后端的 ROS 订阅转起来（``_wait_result``）。"""

    def _explorer(self, backend: object, clock: _StepClock) -> AutonomousExplorer:
        scene = MockObservationScene(scenes=[MockSceneStep(objects=["desk"])])
        return AutonomousExplorer(
            target="t", observer=scene.observer(), matcher=scene.matcher(),
            verifier=scene.verifier(), backend=backend,
            graph=ExplorationGraph(session_id="t"),
            policy=load_exploration_policy(), now=clock,
        )

    def test_wait_result_pumps_backend_and_sees_terminal_status(self) -> None:
        handle = types.SimpleNamespace(goal_id="local_001")
        backend = _StatusGatedBackend(MockBackend(), spins_needed=2)
        explorer = self._explorer(backend, _StepClock(step=1.0))
        result = explorer._wait_result(handle, timeout_sec=5.0)
        self.assertEqual(result.status, NavigationStatus.SUCCEEDED)
        self.assertGreaterEqual(backend._spins, 2)

    def test_without_pump_the_same_backend_times_out(self) -> None:
        # 对照组：同一个「靠回调喂」的后端，只是没有 spin_once。
        # 窗口照旧，仍然必然 TIMEOUT —— 证明结果不是等待熬出来的。
        handle = types.SimpleNamespace(goal_id="local_001")
        backend = _StatusGatedBackend(MockBackend(), spins_needed=2, pumpable=False)
        explorer = self._explorer(backend, _StepClock(step=1.0))
        result = explorer._wait_result(handle, timeout_sec=5.0)
        self.assertEqual(result.status, NavigationStatus.TIMEOUT)
        self.assertGreater(backend.status_calls, 0)


if __name__ == "__main__":
    unittest.main()
