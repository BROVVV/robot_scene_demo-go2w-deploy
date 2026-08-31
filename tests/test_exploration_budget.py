"""Unit tests for the exploration search budget (plan section 14)."""

from __future__ import annotations

import unittest

from app.live_robot.autonomous_explorer import AutonomousExplorer
from app.live_robot.mock_observation_scene import (
    MockObservationScene,
    MockSceneStep,
    scenario_no_target,
)
from app.navigation.backend_factory import MockBackend
from app.navigation.exploration_config import (
    ExplorationBudget,
    ExplorationPolicy,
    load_exploration_policy,
)
from app.navigation.exploration_graph import ExplorationGraph


class TestExplorationBudget(unittest.TestCase):
    def test_budget_remaining(self) -> None:
        budget = ExplorationBudget(max_search_seconds=10.0,
                                   max_planning_cycles=3,
                                   max_motion_steps=2)
        self.assertTrue(budget.remaining(elapsed_sec=1.0, planning_cycles=0,
                                          motion_steps=0))
        self.assertFalse(budget.remaining(elapsed_sec=10.0, planning_cycles=0,
                                          motion_steps=0))
        self.assertFalse(budget.remaining(elapsed_sec=1.0, planning_cycles=3,
                                          motion_steps=0))
        self.assertFalse(budget.remaining(elapsed_sec=1.0, planning_cycles=0,
                                          motion_steps=2))

    def test_motion_step_limit_terminates(self) -> None:
        scene = scenario_no_target(empty_scenes=2)
        policy = load_exploration_policy()
        policy.budget = ExplorationBudget(max_motion_steps=3,
                                          max_planning_cycles=100)
        explorer = AutonomousExplorer(
            target="t", observer=scene.observer(), matcher=scene.matcher(),
            verifier=scene.verifier(), backend=MockBackend(),
            graph=ExplorationGraph(session_id="t"), policy=policy,
        )
        result = explorer.run()
        self.assertEqual(result.result, "MAX_STEPS_REACHED")
        self.assertEqual(result.motion_steps, 3)

    def test_planning_cycle_limit_terminates(self) -> None:
        scene = MockObservationScene(scenes=[
            MockSceneStep(objects=["desk"]),
        ])
        policy = load_exploration_policy()
        policy.budget = ExplorationBudget(max_motion_steps=0,
                                          max_planning_cycles=4)
        explorer = AutonomousExplorer(
            target="t", observer=scene.observer(), matcher=scene.matcher(),
            verifier=scene.verifier(), backend=MockBackend(),
            graph=ExplorationGraph(session_id="t"), policy=policy,
        )
        result = explorer.run()
        self.assertEqual(result.result, "MAX_PLANNING_CYCLES_REACHED")
        self.assertEqual(result.planning_cycles, 4)

    def test_time_limit_terminates(self) -> None:
        scene = scenario_no_target(empty_scenes=2)
        policy = ExplorationPolicy(
            budget=ExplorationBudget(max_search_seconds=0.0,
                                     max_planning_cycles=1000,
                                     max_motion_steps=1000),
        )
        explorer = AutonomousExplorer(
            target="t", observer=scene.observer(), matcher=scene.matcher(),
            verifier=scene.verifier(), backend=MockBackend(),
            graph=ExplorationGraph(session_id="t"), policy=policy,
        )
        result = explorer.run()
        self.assertEqual(result.result, "TIMEOUT")

    def test_no_information_exhausts_only_when_stuck(self) -> None:
        """机器人走不动（每次运动失败）且无新信息时，按阈值判 SEARCH_EXHAUSTED。"""
        scene = scenario_no_target(empty_scenes=1)
        policy = load_exploration_policy()
        policy.budget = ExplorationBudget(
            max_motion_steps=0, max_planning_cycles=1000,
            max_consecutive_no_information_cycles=3,
        )
        explorer = AutonomousExplorer(
            target="t", observer=scene.observer(), matcher=scene.matcher(),
            verifier=scene.verifier(),
            backend=MockBackend(outcome_sequence=["failed"]),
            graph=ExplorationGraph(session_id="t"), policy=policy,
        )
        result = explorer.run()
        self.assertEqual(result.result, "SEARCH_EXHAUSTED")

    def test_moving_without_new_info_does_not_early_exhaust(self) -> None:
        """关键回归：即使连续多轮没有新信息，只要机器狗还在成功移动探索，
        就不得过早判 SEARCH_EXHAUSTED（黑色沙发被 8 轮提前终止的修复）。"""
        scene = scenario_no_target(empty_scenes=1)
        policy = load_exploration_policy()
        policy.budget = ExplorationBudget(
            max_motion_steps=30, max_planning_cycles=30,
            max_consecutive_no_information_cycles=3,  # 低阈值也压不住移动中的探索
        )
        explorer = AutonomousExplorer(
            target="t", observer=scene.observer(), matcher=scene.matcher(),
            verifier=scene.verifier(), backend=MockBackend(),
            graph=ExplorationGraph(session_id="t"), policy=policy,
        )
        result = explorer.run()
        self.assertNotEqual(result.result, "SEARCH_EXHAUSTED")
        self.assertIn(result.result, {"MAX_STEPS_REACHED", "MAX_PLANNING_CYCLES_REACHED"})
        self.assertGreaterEqual(result.planning_cycles, 3)

    def test_default_policy_from_yaml(self) -> None:
        policy = load_exploration_policy()
        self.assertEqual(policy.budget.max_search_seconds, 600.0)
        self.assertEqual(policy.scoring.semantic_relevance, 0.35)
        self.assertEqual(policy.candidates.heading_sectors, 12)


if __name__ == "__main__":
    unittest.main()
