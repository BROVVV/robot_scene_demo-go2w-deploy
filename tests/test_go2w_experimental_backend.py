"""Unit tests for the Go2-W experimental backend (injected hardware)."""

from __future__ import annotations

import unittest

from app.navigation.go2w_experimental_backend import (
    Go2WBackendConfig,
    Go2WExperimentalBackend,
)
from app.navigation.models import (
    GOAL_INSPECT_ANCHOR,
    GOAL_NAVIGATE_POSE,
    GOAL_RELATIVE_MOVE,
    GOAL_ROTATE_VIEW,
    GOAL_STOP,
    ExplorationGoal,
)
from app.navigation.robot_backend import NavigationStatus, PoseQuality


class _FakeMotion:
    def __init__(self) -> None:
        self.steps: list[str] = []
        self.odom = [0.0, 0.0, 0.0]
        self.fail_steps: set[str] = set()

    def execute(self, step: str) -> tuple[bool, str, dict]:
        self.steps.append(step)
        if step in self.fail_steps:
            return False, f"motion rejected: {step}", {"step": step}
        if step.startswith("f"):
            self.odom[0] += float(step[1:] or 0.18)
        elif step.startswith("b"):
            self.odom[0] -= float(step[1:] or 0.10)
        else:
            degrees = float(step[1:])
            if step.startswith("r"):
                degrees = -degrees
            self.odom[2] += degrees * 3.141592653589793 / 180.0
        return True, "ok", {"step": step}

    def odometry(self) -> tuple[float, float, float]:
        return tuple(self.odom)


def _backend(motion: _FakeMotion, **config) -> Go2WExperimentalBackend:
    return Go2WExperimentalBackend(
        execute_step=motion.execute,
        odometry=motion.odometry,
        config=Go2WBackendConfig(**config),
    )


class TestGo2WExperimentalBackend(unittest.TestCase):
    def test_capabilities_relative(self) -> None:
        backend = _backend(_FakeMotion())
        caps = backend.capabilities()
        self.assertFalse(caps.supports_global_pose)
        self.assertFalse(caps.supports_metric_navigation)
        self.assertTrue(caps.supports_relative_rotation)
        self.assertTrue(caps.supports_relative_translation)
        self.assertEqual(
            caps.allowed_motion_primitives,
            ("FORWARD", "BACKWARD_RECOVERY", "ROTATE_LEFT", "ROTATE_RIGHT"),
        )

    def test_pose_quality_relative(self) -> None:
        backend = _backend(_FakeMotion())
        pose = backend.get_pose()
        self.assertEqual(pose.quality, PoseQuality.RELATIVE)
        self.assertEqual(pose.frame_id, "odom")

    def test_rotate_view_clamps_to_max_turn(self) -> None:
        motion = _FakeMotion()
        backend = _backend(motion, max_turn_deg_per_action=30.0)
        goal = ExplorationGoal(goal_id="g1", goal_type=GOAL_ROTATE_VIEW,
                               relative_dyaw=90.0)
        result = backend.execute_goal(goal).result
        self.assertTrue(result.succeeded)
        self.assertEqual(motion.steps, ["l30"])
        self.assertIn("yaw_delta_deg", result.observed_motion)

    def test_rotate_right_uses_r_step(self) -> None:
        motion = _FakeMotion()
        backend = _backend(motion)
        goal = ExplorationGoal(goal_id="g1", goal_type=GOAL_ROTATE_VIEW,
                               relative_dyaw=-20.0)
        backend.execute_goal(goal)
        self.assertEqual(motion.steps, ["r20"])

    def test_forward_clamps_to_max(self) -> None:
        motion = _FakeMotion()
        backend = _backend(motion, max_forward_step_m=0.30)
        goal = ExplorationGoal(goal_id="g1", goal_type=GOAL_RELATIVE_MOVE,
                               relative_dx=5.0)
        result = backend.execute_goal(goal).result
        self.assertTrue(result.succeeded)
        self.assertEqual(motion.steps, ["f0.200", "f0.100"])
        self.assertLessEqual(result.requested_motion["distance_m"], 0.30)
        self.assertTrue(result.requested_motion["safety_limited"])

    def test_long_forward_is_stopped_and_verified_in_segments(self) -> None:
        motion = _FakeMotion()
        backend = _backend(
            motion, forward_step_m=0.30, max_forward_step_m=1.50
        )
        goal = ExplorationGoal(
            goal_id="g-segmented",
            goal_type=GOAL_RELATIVE_MOVE,
            relative_dx=1.5,
        )
        result = backend.execute_goal(goal).result
        self.assertTrue(result.succeeded)
        self.assertEqual(motion.steps, ["f0.300"] * 5)
        self.assertEqual(result.requested_motion["segment_count"], 5)
        self.assertAlmostEqual(result.observed_motion["displacement_m"], 1.5)

    def test_segmented_forward_stops_at_first_failed_segment(self) -> None:
        motion = _FakeMotion()
        calls = 0

        def fail_second(step: str) -> tuple[bool, str, dict]:
            nonlocal calls
            calls += 1
            if calls == 2:
                return False, "FORWARD_NOT_CONFIRMED: no chassis motion", {
                    "error_type": "FORWARD_NOT_CONFIRMED"
                }
            return motion.execute(step)

        backend = Go2WExperimentalBackend(
            execute_step=fail_second,
            odometry=motion.odometry,
            config=Go2WBackendConfig(
                forward_step_m=0.30, max_forward_step_m=1.50
            ),
        )
        result = backend.execute_goal(ExplorationGoal(
            goal_id="g-fail",
            goal_type=GOAL_RELATIVE_MOVE,
            relative_dx=1.5,
        )).result
        self.assertEqual(result.status, NavigationStatus.FAILED)
        self.assertEqual(calls, 2)
        self.assertIn("FORWARD_NOT_CONFIRMED", result.message)

    def test_lateral_rejected_by_default(self) -> None:
        motion = _FakeMotion()
        backend = _backend(motion)
        goal = ExplorationGoal(goal_id="g1", goal_type=GOAL_RELATIVE_MOVE,
                               relative_dx=0.0, relative_dy=0.2)
        result = backend.execute_goal(goal).result
        self.assertEqual(result.status, NavigationStatus.REJECTED)
        self.assertEqual(motion.steps, [])
        self.assertTrue(result.provenance["replan_required"])

    def test_reverse_recovery_executes_bounded_backward(self) -> None:
        motion = _FakeMotion()
        backend = _backend(motion)
        goal = ExplorationGoal(goal_id="g1", goal_type=GOAL_RELATIVE_MOVE,
                               relative_dx=-0.2)
        result = backend.execute_goal(goal).result
        self.assertTrue(result.succeeded)
        self.assertEqual(motion.steps, ["b0.120"])
        self.assertEqual(result.requested_motion["primitive"], "BACKWARD_RECOVERY")
        self.assertAlmostEqual(result.requested_motion["distance_m"], 0.12)
        self.assertLess(result.observed_motion["signed_progress_m"], 0.0)

    def test_reverse_recovery_disabled_when_config_off(self) -> None:
        motion = _FakeMotion()
        backend = _backend(motion, allow_backward_recovery=False)
        goal = ExplorationGoal(goal_id="g1", goal_type=GOAL_RELATIVE_MOVE,
                               relative_dx=-0.2)
        result = backend.execute_goal(goal).result
        self.assertEqual(result.status, NavigationStatus.REJECTED)
        self.assertEqual(motion.steps, [])
        self.assertIn("disabled", result.message)

    def test_metric_goal_rejected(self) -> None:
        backend = _backend(_FakeMotion())
        goal = ExplorationGoal(goal_id="g1", goal_type=GOAL_NAVIGATE_POSE,
                               position=(1.0, 1.0))
        result = backend.execute_goal(goal).result
        self.assertEqual(result.status, NavigationStatus.REJECTED)

    def test_motion_failure_maps_to_failed(self) -> None:
        motion = _FakeMotion()
        motion.fail_steps.add("l10")
        backend = _backend(motion)
        goal = ExplorationGoal(goal_id="g1", goal_type=GOAL_ROTATE_VIEW,
                               relative_dyaw=10.0)
        result = backend.execute_goal(goal).result
        self.assertEqual(result.status, NavigationStatus.FAILED)

    def test_motion_transport_timeout_maps_to_timeout(self) -> None:
        motion = _FakeMotion()

        def timed_out(step: str) -> tuple[bool, str, dict]:
            motion.steps.append(step)
            return (
                False,
                "MOTION_RESULT_TIMEOUT: no result within 20s",
                {"step": step, "error_type": "MOTION_RESULT_TIMEOUT"},
            )

        backend = Go2WExperimentalBackend(
            execute_step=timed_out,
            odometry=motion.odometry,
        )
        goal = ExplorationGoal(
            goal_id="g-timeout",
            goal_type=GOAL_ROTATE_VIEW,
            relative_dyaw=20.0,
        )
        result = backend.execute_goal(goal).result
        self.assertEqual(result.status, NavigationStatus.TIMEOUT)
        self.assertIn("MOTION_RESULT_TIMEOUT", result.message)

    def test_duplicate_motion_servers_fail_health_closed(self) -> None:
        motion = _FakeMotion()
        backend = Go2WExperimentalBackend(
            execute_step=motion.execute,
            odometry=motion.odometry,
            health_probe=lambda: {
                "motion_action_available": True,
                "motion_action_server_count": 2,
            },
        )
        health = backend.health()
        self.assertFalse(health.ready)
        self.assertIn("duplicate_motion_action_servers", health.degraded)

    def test_duplicate_motion_server_processes_fail_health_closed(self) -> None:
        motion = _FakeMotion()
        backend = Go2WExperimentalBackend(
            execute_step=motion.execute,
            odometry=motion.odometry,
            health_probe=lambda: {
                "motion_action_available": True,
                "motion_action_server_count": 1,
                "motion_action_server_process_count": 2,
            },
        )
        health = backend.health()
        self.assertFalse(health.ready)
        self.assertIn(
            "duplicate_motion_action_server_processes", health.degraded
        )

    def test_duplicate_odom_publishers_fail_health_closed(self) -> None:
        motion = _FakeMotion()
        backend = Go2WExperimentalBackend(
            execute_step=motion.execute,
            odometry=motion.odometry,
            health_probe=lambda: {"odom_publisher_count": 2},
        )
        health = backend.health()
        self.assertFalse(health.ready)
        self.assertIn("duplicate_odom_publishers", health.degraded)

        result = backend.execute_goal(ExplorationGoal(
            goal_id="g-blocked",
            goal_type=GOAL_RELATIVE_MOVE,
            relative_dx=0.3,
        )).result
        self.assertEqual(result.status, NavigationStatus.BACKEND_UNAVAILABLE)
        self.assertEqual(motion.steps, [])

    def test_stop_calls_injected_stop(self) -> None:
        stopped = []
        motion = _FakeMotion()
        backend = Go2WExperimentalBackend(
            execute_step=motion.execute, odometry=motion.odometry,
            stop=lambda: stopped.append(True),
        )
        goal = ExplorationGoal(goal_id="g1", goal_type=GOAL_STOP)
        result = backend.execute_goal(goal).result
        self.assertTrue(result.succeeded)
        self.assertEqual(stopped, [True])

    def test_health_reports_metric_pose_degraded(self) -> None:
        motion = _FakeMotion()
        backend = _backend(motion)
        health = backend.health()
        self.assertTrue(health.ready)
        self.assertIn("metric_pose_unavailable", health.degraded)

    def test_health_fails_closed_on_motion_unavailable(self) -> None:
        motion = _FakeMotion()
        backend = Go2WExperimentalBackend(
            execute_step=motion.execute, odometry=motion.odometry,
            health_probe=lambda: {"motion_action_available": False},
        )
        health = backend.health()
        self.assertFalse(health.ready)
        self.assertIn("motion_action_unavailable", health.degraded)

    def test_opportunistic_correction_learning(self) -> None:
        motion = _FakeMotion()
        backend = _backend(motion, correction_min_samples=2,
                           correction_min_confidence=0.6)
        for _ in range(4):
            goal = ExplorationGoal(goal_id="g1", goal_type=GOAL_ROTATE_VIEW,
                                   relative_dyaw=30.0)
            backend.execute_goal(goal)
        correction = backend.correction()
        self.assertEqual(correction.samples, 4)
        # Fake executes exactly 30 deg for a 30 deg request -> scale ~1.0
        self.assertAlmostEqual(correction.rotation_scale, 1.0, places=3)

    def test_inspect_anchor_uses_heading(self) -> None:
        motion = _FakeMotion()
        backend = _backend(motion)
        goal = ExplorationGoal(goal_id="g1", goal_type=GOAL_INSPECT_ANCHOR,
                               relative_dyaw=25.0, semantic_anchor="water dispenser")
        result = backend.execute_goal(goal).result
        self.assertTrue(result.succeeded)
        self.assertEqual(motion.steps, ["l25"])


if __name__ == "__main__":
    unittest.main()
