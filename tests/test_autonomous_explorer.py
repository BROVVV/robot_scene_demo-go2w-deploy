"""End-to-end tests of the AutonomousExplorer against mock backends.

Covers plan section 34.2 scenarios A-I: first-frame target, absent target,
semantic anchor boost, navigation failure replan, oscillation, SemanticNavigation
exception fallback, operator stop, pose unavailable, future metric backend.
"""

from __future__ import annotations

import threading
import time
import unittest

from app.live_robot.autonomous_explorer import (
    AutonomousExplorer,
    PerceptionFailure,
    SemanticMatch,
    VerificationOutcome,
)
from app.live_robot.mock_observation_scene import (
    MockObservationScene,
    MockSceneStep,
    scenario_anchor_then_target,
    scenario_no_target,
    scenario_target_appears_after,
)
from app.navigation.backend_factory import MockBackend, MockMetricBackend
from app.navigation.exploration_config import (
    ExplorationBudget,
    ExplorationPolicy,
    load_exploration_policy,
)
from app.navigation.exploration_graph import ExplorationGraph
from app.navigation.models import (
    GOAL_NAVIGATE_POSE,
    GOAL_RELATIVE_MOVE,
    GOAL_ROTATE_VIEW,
    LiveObservation,
)
from app.navigation.robot_backend import NavigationStatus, RobotPose, PoseQuality


def _explorer(scene: MockObservationScene, backend=None, *, policy=None,
              graph=None, **kwargs) -> AutonomousExplorer:
    backend = backend or MockBackend()
    return AutonomousExplorer(
        target="蓝色垃圾桶",
        observer=scene.observer(),
        matcher=scene.matcher(),
        verifier=scene.verifier(),
        backend=backend,
        policy=policy or load_exploration_policy(),
        graph=graph or ExplorationGraph(session_id="test"),
        negative_target_key="蓝色垃圾桶",
        **kwargs,
    )


class TestAutonomousExplorerE2E(unittest.TestCase):
    # Scenario A: target appears in the first frame -> TARGET_FOUND, 0 motion.
    def test_scenario_a_first_frame_target(self) -> None:
        scene = MockObservationScene(scenes=[
            MockSceneStep(objects=["blue trash bin"], target_present=True,
                          target_score=0.95),
        ])
        explorer = _explorer(scene)
        result = explorer.run()
        self.assertEqual(result.result, "TARGET_FOUND")
        self.assertEqual(result.motion_steps, 0)
        self.assertEqual(result.observations, 1)

    # Scenario B: target never appears -> exhaustion, no infinite loop.
    def test_scenario_b_no_target_exhausts(self) -> None:
        scene = scenario_no_target(empty_scenes=4)
        policy = load_exploration_policy()
        policy.budget = ExplorationBudget(max_motion_steps=40,
                                          max_planning_cycles=40)
        explorer = _explorer(scene, policy=policy)
        result = explorer.run()
        # 机器狗仍在成功移动探索：应持续找直到预算，而不是在 8 轮左右被
        # “连续无新信息”提前判穷尽。
        self.assertIn(result.result, {"SEARCH_EXHAUSTED", "MAX_STEPS_REACHED",
                                       "MAX_PLANNING_CYCLES_REACHED", "TIMEOUT"})
        self.assertGreaterEqual(result.planning_cycles, 8)

    # Scenario C: semantic anchor raises candidate priority.
    def test_scenario_c_anchor_boosts_selection(self) -> None:
        scene = MockObservationScene(scenes=[
            MockSceneStep(objects=["water dispenser"],
                          anchor_labels=["water dispenser"]),
            MockSceneStep(objects=["water dispenser", "blue trash bin"],
                          target_present=True, anchor_labels=["water dispenser"],
                          target_score=0.95),
        ])
        explorer = _explorer(scene)
        result = explorer.run()
        self.assertEqual(result.result, "TARGET_FOUND")
        inspect_goals = [
            event for event in explorer.events
            if event.get("event") == "selected_goal"
            and event["goal"]["goal_type"] == "INSPECT_ANCHOR"
        ]
        self.assertTrue(inspect_goals, "expected an INSPECT_ANCHOR goal")
        self.assertGreaterEqual(inspect_goals[0]["goal"]["semantic_relevance"], 0.4)

    # Scenario D: navigation failure -> replan to a different goal.
    def test_scenario_d_navigation_failure_replans(self) -> None:
        scene = scenario_target_appears_after(3)
        backend = MockBackend(outcome_sequence=[NavigationStatus.FAILED])
        explorer = _explorer(scene, backend=backend)
        result = explorer.run()
        self.assertEqual(result.result, "TARGET_FOUND")
        self.assertGreaterEqual(result.replans, 1)
        self.assertGreaterEqual(result.navigation_failures, 1)
        selected = [event["goal"]["heading_sector"] for event in explorer.events
                    if event.get("event") == "selected_goal"]
        self.assertGreaterEqual(len(set(selected)), 2,
                                "planner should move on to a different goal")

    # Scenario E: repeated scene -> no long A-B-A-B oscillation.
    def test_scenario_e_oscillation_avoided(self) -> None:
        scene = scenario_no_target(empty_scenes=2)
        policy = load_exploration_policy()
        policy.budget = ExplorationBudget(max_motion_steps=30)
        explorer = _explorer(scene, policy=policy)
        explorer.run()
        selected = [event["goal"]["heading_sector"] for event in explorer.events
                    if event.get("event") == "selected_goal"]
        # No 2-cycle run longer than one repeat: the tabu/oscillation logic
        # must force a new direction.
        for index in range(2, len(selected)):
            self.assertFalse(
                selected[index] == selected[index - 2]
                and selected[index] != selected[index - 1],
                f"2-cycle oscillation detected: {selected}",
            )
        self.assertGreaterEqual(len(set(selected)), 3,
                                f"exploration should cover directions: {selected}")

    # Scenario F: SemanticNavigation exception -> legacy/fallback candidates continue.
    def test_scenario_f_matcher_exception_falls_back(self) -> None:
        scene = scenario_target_appears_after(2)
        explorer = _explorer(scene)
        original = explorer._matcher
        calls = {"count": 0}

        def flaky_matcher(observation: LiveObservation) -> SemanticMatch:
            calls["count"] += 1
            if calls["count"] == 1:
                raise RuntimeError("LLM timeout")
            return original(observation)

        explorer._matcher = flaky_matcher
        result = explorer.run()
        self.assertEqual(result.result, "TARGET_FOUND")
        self.assertGreaterEqual(calls["count"], 2)

    # Scenario G: operator stop -> OPERATOR_STOP, no further motion.
    def test_scenario_g_operator_stop(self) -> None:
        scene = scenario_target_appears_after(10)
        explorer = _explorer(scene)
        explorer.request_stop()
        result = explorer.run()
        self.assertEqual(result.result, "OPERATOR_STOP")
        self.assertLessEqual(result.motion_steps, 1)

    # Scenario H: pose unavailable -> topological exploration still runs.
    def test_scenario_h_pose_unavailable(self) -> None:
        scene = scenario_no_target(empty_scenes=3)
        backend = MockBackend()
        real_pose = backend.get_pose

        def no_pose() -> None:
            return None

        backend.get_pose = no_pose
        policy = load_exploration_policy()
        policy.budget = ExplorationBudget(max_motion_steps=5)
        explorer = _explorer(scene, backend=backend, policy=policy)
        result = explorer.run()
        self.assertGreaterEqual(result.planning_cycles, 1)
        self.assertGreaterEqual(result.motion_steps, 1)

    # Scenario I: future metric backend gets NAVIGATE_POSE with same explorer.
    def test_scenario_i_metric_backend_navigate_pose(self) -> None:
        scene = MockObservationScene(scenes=[
            MockSceneStep(objects=["desk"]),
            MockSceneStep(objects=["desk"]),
        ])
        backend = MockMetricBackend()
        explorer = _explorer(scene, backend=backend)
        result = explorer.run()
        self.assertGreaterEqual(result.planning_cycles, 1)
        selected = [event["goal"] for event in explorer.events
                    if event.get("event") == "selected_goal"]
        self.assertTrue(selected)
        self.assertEqual(selected[0]["goal_type"], GOAL_NAVIGATE_POSE)

    def test_perception_failure_first_cycle_fails_cleanly(self) -> None:
        def observer() -> LiveObservation:
            raise PerceptionFailure("camera stale")

        backend = MockBackend()
        explorer = AutonomousExplorer(
            target="x", observer=observer,
            matcher=lambda obs: SemanticMatch(has_candidate=False),
            verifier=lambda obs, match: VerificationOutcome(confirmed=False, attempts=1),
            backend=backend,
            graph=ExplorationGraph(session_id="t"),
            policy=load_exploration_policy(),
        )
        result = explorer.run()
        self.assertEqual(result.result, "PERCEPTION_FAILURE")

    def test_session_summary_fields(self) -> None:
        scene = scenario_anchor_then_target()
        explorer = _explorer(scene)
        result = explorer.run()
        payload = result.to_dict()
        for key in ("result", "planning_cycles", "motion_steps", "observations",
                    "unique_nodes", "replans", "verify_attempts"):
            self.assertIn(key, payload)
        self.assertGreaterEqual(result.unique_nodes, 1)


if __name__ == "__main__":
    unittest.main()
