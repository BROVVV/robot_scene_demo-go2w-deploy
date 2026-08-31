"""Unit tests for the platform-independent RobotBackend contract and mocks."""

from __future__ import annotations

import unittest

from app.navigation.backend_factory import MockBackend, MockMetricBackend, create_backend
from app.navigation.models import (
    GOAL_INSPECT_ANCHOR,
    GOAL_NAVIGATE_POSE,
    GOAL_RELATIVE_MOVE,
    GOAL_ROTATE_VIEW,
    ExplorationGoal,
)
from app.navigation.robot_backend import (
    NavigationStatus,
    PoseQuality,
    RobotCapabilities,
)


def _goal(goal_type: str, **kwargs) -> ExplorationGoal:
    return ExplorationGoal(goal_id="g1", goal_type=goal_type, **kwargs)


class TestRobotCapabilities(unittest.TestCase):
    def test_defaults_are_false(self) -> None:
        caps = RobotCapabilities()
        self.assertFalse(caps.supports_global_pose)
        self.assertFalse(caps.supports_metric_navigation)

    def test_roundtrip(self) -> None:
        caps = RobotCapabilities(supports_metric_navigation=True)
        restored = RobotCapabilities.from_dict(caps.to_dict())
        self.assertTrue(restored.supports_metric_navigation)
        self.assertFalse(restored.supports_relative_rotation)


class TestMockBackend(unittest.TestCase):
    def test_relative_capabilities(self) -> None:
        backend = MockBackend()
        caps = backend.capabilities()
        self.assertFalse(caps.supports_metric_navigation)
        self.assertTrue(caps.supports_relative_rotation)

    def test_pose_quality_relative(self) -> None:
        backend = MockBackend()
        pose = backend.get_pose()
        self.assertIsNotNone(pose)
        self.assertEqual(pose.quality, PoseQuality.RELATIVE)

    def test_turn_changes_yaw(self) -> None:
        backend = MockBackend()
        goal = _goal(GOAL_ROTATE_VIEW, relative_dyaw=30.0)
        handle = backend.execute_goal(goal)
        self.assertTrue(handle.result.succeeded)
        self.assertAlmostEqual(backend.get_pose().yaw, 30.0 * 3.141592653589793 / 180.0, places=6)

    def test_forward_changes_position(self) -> None:
        backend = MockBackend()
        goal = _goal(GOAL_RELATIVE_MOVE, relative_dx=0.2)
        handle = backend.execute_goal(goal)
        self.assertTrue(handle.result.succeeded)
        pose = backend.get_pose()
        self.assertGreater(pose.x, 0.19)

    def test_outcome_sequence_replays(self) -> None:
        backend = MockBackend(outcome_sequence=[NavigationStatus.FAILED, NavigationStatus.TIMEOUT])
        goal = _goal(GOAL_ROTATE_VIEW, relative_dyaw=10.0)
        first = backend.execute_goal(goal).result
        second = backend.execute_goal(goal).result
        third = backend.execute_goal(goal).result
        self.assertEqual(first.status, NavigationStatus.FAILED)
        self.assertEqual(second.status, NavigationStatus.TIMEOUT)
        self.assertEqual(third.status, NavigationStatus.TIMEOUT)

    def test_operator_stop(self) -> None:
        backend = MockBackend()
        backend.stop()
        self.assertTrue(backend._stop_called)


class TestMockMetricBackend(unittest.TestCase):
    def test_metric_capabilities(self) -> None:
        backend = MockMetricBackend()
        caps = backend.capabilities()
        self.assertTrue(caps.supports_global_pose)
        self.assertTrue(caps.supports_metric_navigation)

    def test_navigate_pose_moves(self) -> None:
        backend = MockMetricBackend(start_pose=(0.0, 0.0, 0.0))
        goal = _goal(GOAL_NAVIGATE_POSE, position=(1.0, 2.0), yaw=0.5)
        result = backend.execute_goal(goal).result
        self.assertTrue(result.succeeded)
        pose = backend.get_pose()
        self.assertAlmostEqual(pose.x, 1.0, places=6)
        self.assertAlmostEqual(pose.y, 2.0, places=6)

    def test_metric_pose_quality(self) -> None:
        backend = MockMetricBackend()
        self.assertEqual(backend.get_pose().quality, PoseQuality.METRIC)


class TestBackendFactory(unittest.TestCase):
    def test_factory_kinds(self) -> None:
        self.assertIsInstance(create_backend("go2w_experimental",
                                             execute_step=lambda step: (True, "", {}),
                                             odometry=lambda: (0.0, 0.0, 0.0)),
                              object)
        self.assertIsInstance(create_backend("mock"), MockBackend)
        self.assertIsInstance(create_backend("mock_metric"), MockMetricBackend)

    def test_unknown_kind_rejected(self) -> None:
        with self.assertRaises(ValueError):
            create_backend("hoverboard")


class TestExplorationGoal(unittest.TestCase):
    def test_invalid_type_rejected(self) -> None:
        with self.assertRaises(ValueError):
            ExplorationGoal(goal_id="g", goal_type="TELEPORT")

    def test_roundtrip(self) -> None:
        goal = _goal(GOAL_INSPECT_ANCHOR, semantic_anchor="water dispenser",
                     relative_dyaw=25.0, heading_sector=3)
        restored = ExplorationGoal.from_dict(goal.to_dict())
        self.assertEqual(restored.goal_type, GOAL_INSPECT_ANCHOR)
        self.assertEqual(restored.semantic_anchor, "water dispenser")
        self.assertEqual(restored.relative_dyaw, 25.0)


if __name__ == "__main__":
    unittest.main()
