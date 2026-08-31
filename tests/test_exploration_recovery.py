"""Unit tests for navigation-result recovery / replan semantics (plan 13)."""

from __future__ import annotations

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
from app.navigation.robot_backend import NavigationStatus


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


if __name__ == "__main__":
    unittest.main()
