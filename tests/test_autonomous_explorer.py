"""End-to-end tests of the AutonomousExplorer against mock backends.

Covers plan section 34.2 scenarios A-I: first-frame target, absent target,
semantic anchor boost, navigation failure replan, oscillation, SemanticNavigation
exception fallback, operator stop, pose unavailable, future metric backend.
"""

from __future__ import annotations

import threading
import time
import unittest
from dataclasses import replace

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
from app.navigation.robot_backend import (
    NavigationHandle,
    NavigationResult,
    NavigationStatus,
    RobotPose,
    PoseQuality,
)


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
    def test_navigation_target_monitor_cancels_long_goal_before_reobserve(self) -> None:
        class PendingBackend(MockBackend):
            def __init__(self):
                super().__init__()
                self.cancelled = False
                self.stopped = False

            def get_navigation_status(self, handle):
                return NavigationResult(handle.goal_id, NavigationStatus.RUNNING)

            def cancel(self, handle=None):
                self.cancelled = True
                return True

            def stop(self):
                self.stopped = True

        scene = scenario_no_target(empty_scenes=1)
        backend = PendingBackend()
        explorer = _explorer(
            scene,
            backend=backend,
            navigation_target_monitor=lambda: True,
        )
        result = explorer._wait_result(NavigationHandle("long_goal"), timeout_sec=2.)
        self.assertEqual(result.status, NavigationStatus.CANCELLED)
        self.assertTrue(result.provenance["target_candidate_interrupt"])
        self.assertTrue(backend.cancelled)
        self.assertTrue(backend.stopped)

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

    def test_slow_observation_cannot_start_motion_after_time_budget(self) -> None:
        scene = scenario_no_target(empty_scenes=2)
        backend = MockBackend()
        policy = load_exploration_policy()
        policy.budget = ExplorationBudget(
            max_search_seconds=1.0, max_planning_cycles=2,
            max_motion_steps=1,
        )
        clock = [100.0]
        observe = scene.observer()

        def slow_observer():
            result = observe()
            clock[0] = 102.0
            return result

        explorer = AutonomousExplorer(
            target="蓝色垃圾桶", observer=slow_observer,
            matcher=scene.matcher(), verifier=scene.verifier(),
            backend=backend, policy=policy,
            graph=ExplorationGraph(session_id="expired-before-motion"),
            now=lambda: clock[0],
        )
        result = explorer.run()
        self.assertEqual(result.result, "TIMEOUT")
        self.assertEqual(result.motion_steps, 0)
        self.assertEqual(backend._executed, [])
        self.assertTrue(any(
            event.get("event") == "motion_budget_expired"
            for event in explorer.events
        ))

    def test_navigation_goal_cap_counts_attempts_not_only_successes(self) -> None:
        scene = scenario_no_target(empty_scenes=4)
        backend = MockBackend(outcome_sequence=[NavigationStatus.FAILED])
        policy = load_exploration_policy()
        policy.budget = ExplorationBudget(
            max_search_seconds=600, max_planning_cycles=20,
            max_motion_steps=0,
        )
        explorer = _explorer(
            scene, backend=backend, policy=policy,
            max_navigation_goal_attempts=3,
        )
        result = explorer.run()
        self.assertEqual(len(backend._executed), 3)
        self.assertTrue(any(
            event.get("event") == "navigation_goal_budget_reached"
            for event in explorer.events
        ))
        self.assertEqual(result.motion_steps, 0)

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


class TestConfirmedTargetObjectId(unittest.TestCase):
    """确认标记必须落在本帧的目标候选节点上，而不是 label 碰巧沾边的物体。"""

    def _observation(self) -> LiveObservation:
        return LiveObservation(
            bundle_id="bundle_1",
            timestamp=1.0,
            heading_sector=0,
            pose={"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
            scene_objects=[
                # VLM 的 scene_objects_light 先给了远处另一个桶，label 与目标
                # 存在包含关系 -> label 猜测会命中它。
                {"frame_object_id": "scene_obj_001", "label": "垃圾桶",
                 "category": "object", "bbox_2d": [0.1, 0.1, 0.2, 0.3],
                 "score": 0.6},
                {"frame_object_id": "target_obj_001", "label": "蓝色垃圾桶 深蓝色网格桶",
                 "category": "target", "bbox_2d": [0.5, 0.4, 0.7, 0.8],
                 "score": 0.9},
            ],
        )

    def test_confirmation_uses_the_frame_target_candidate(self):
        explorer = _explorer(MockObservationScene(scenes=[
            MockSceneStep(objects=["垃圾桶"], target_present=True,
                          target_score=0.9),
        ]))
        observation = self._observation()
        spatial_update = explorer.semantic_graph.update_observation(
            observation_id=observation.bundle_id,
            heading_sector=observation.heading_sector,
            scene_objects=observation.scene_objects,
            scene_relations=[],
            pose=observation.pose,
            timestamp=observation.timestamp,
            target_candidate=True,
        )
        mapping = spatial_update["frame_object_ids"]
        self.assertIn("target_obj_001", mapping)
        object_id = explorer._confirmed_target_object_id(observation, spatial_update)
        self.assertEqual(object_id, mapping["target_obj_001"])
        # label 猜测会命中另一个桶，精确映射不会。
        self.assertNotEqual(object_id, mapping["scene_obj_001"])
        self.assertEqual(explorer._target_object_id(), mapping["scene_obj_001"])
        explorer.semantic_graph.mark_target_confirmed(
            object_id=object_id, observation_id=observation.bundle_id)
        objects = explorer.semantic_graph.object_map.objects
        self.assertTrue(objects[object_id].provenance.get("target_confirmed"))
        self.assertNotIn(
            "target_confirmed", objects[mapping["scene_obj_001"]].provenance)


class _ProvenanceBackend(MockBackend):
    """MockBackend + 一份可控的 provenance。

    用来钉住「后端 provenance → ``navigation_result`` 事件顶层」这一跳。
    """

    def __init__(self, provenance, **kwargs):
        super().__init__(**kwargs)
        self._provenance = provenance

    def execute_goal(self, goal):
        handle = super().execute_goal(goal)
        if handle.result is not None:
            handle.result = replace(handle.result,
                                    provenance=dict(self._provenance))
        return handle


class TestNavigationResultCarriesCandidateDecision(unittest.TestCase):
    """§13.2 第 7 项：候选决策记录必须落在 ``navigation_result`` 事件的**顶层**。

    这条链有三跳，此前只有两跳有测试：后端把 §14 的候选记录塞进
    ``provenance``（``test_nav2_search_backend``）、WebUI 快照读
    ``robot.last_motion_result.candidate_decision``（``test_search_state_store``）。
    中间这一跳 —— explorer 把它抬到事件顶层、写进 ``events.jsonl`` —— 没有任何
    用例守着，而 §13.2 第 7 项要现场人员去 ``events.jsonl`` 里翻的正是它。
    改坏了只有实机跑才现形，所以在这里钉死。
    """

    RECORD = {
        "candidate_id": "g1",
        "goal_x": 1.0,
        "goal_y": 0.0,
        "goal_yaw": 0.0,
        "path_length_m": 1.2,
        "min_left_body_gap_m": 0.62,
        "min_right_body_gap_m": 0.55,
        "min_side_gap_m": 0.55,
        "corridor_decision": "OK",
        "speed_limit_mps": 0.2,
        "center_score": 0.91,
        "accepted": True,
        "reject_reason": "",
    }

    def test_candidate_decision_reaches_the_event_top_level(self) -> None:
        policy = load_exploration_policy()
        policy.budget = ExplorationBudget(max_motion_steps=3, max_planning_cycles=3)
        explorer = _explorer(
            scenario_no_target(empty_scenes=6),
            backend=_ProvenanceBackend({"candidate_decision": dict(self.RECORD)}),
            policy=policy,
        )
        explorer.run()

        results = [e for e in explorer.events if e.get("event") == "navigation_result"]
        self.assertTrue(results, "这个场景一条 navigation_result 都没发，断言前提已变")
        for event in results:
            # 顶层，不是嵌在 provenance / graph / message 里的。
            self.assertEqual(event.get("candidate_decision"), self.RECORD)

    def test_missing_candidate_decision_is_null_not_absent(self) -> None:
        """后端没给候选记录时，字段必须存在且为 null。

        ``jq 'select(.event=="navigation_result" and .candidate_decision != null)'``
        是现场查法；字段整个缺席与值为 null 在 jq 下都是 false，但在
        ``json.loads`` 之后 ``row["candidate_decision"]`` 会 KeyError。
        §13.2 的脚本按后者读，所以缺席和 null 不能混。
        """
        policy = load_exploration_policy()
        policy.budget = ExplorationBudget(max_motion_steps=3, max_planning_cycles=3)
        explorer = _explorer(
            scenario_no_target(empty_scenes=6),
            backend=_ProvenanceBackend({"backend": "mock"}),
            policy=policy,
        )
        explorer.run()

        results = [e for e in explorer.events if e.get("event") == "navigation_result"]
        self.assertTrue(results, "这个场景一条 navigation_result 都没发，断言前提已变")
        for event in results:
            self.assertIn("candidate_decision", event)
            self.assertIsNone(event["candidate_decision"])
