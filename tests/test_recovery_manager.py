from __future__ import annotations

import unittest

from app.live_robot.recovery_motion import SafeMotionSegment
from app.navigation.recovery_manager import RecoveryManager, RecoveryManagerConfig
from app.navigation.robot_backend import NavigationResult, NavigationStatus


def _crumb() -> SafeMotionSegment:
    return SafeMotionSegment(
        start_pose=(0.0, 0.0, 0.0),
        end_pose=(0.10, 0.0, 0.0),
        signed_distance_m=0.10,
        heading_rad=0.0,
        source_step="f0.10",
    )


def _result(status=NavigationStatus.FAILED, message="FORWARD_NOT_CONFIRMED: no motion"):
    return NavigationResult(
        goal_id="g",
        status=status,
        message=message,
    )


class TestRecoveryManager(unittest.TestCase):
    def test_front_blocked_with_valid_breadcrumb_allows_backward(self) -> None:
        manager = RecoveryManager()
        decision = manager.decide(
            result=_result(),
            breadcrumb=_crumb(),
            current_pose=(0.10, 0.0, 0.0),
            front_clearance_m=0.1,
            forward_min_clearance_m=0.3,
            goal_id="recover",
        )
        self.assertTrue(decision.backward_allowed)
        self.assertIsNotNone(decision.backward_goal)
        self.assertLess(decision.backward_goal.relative_dx, 0.0)

    def test_without_breadcrumb_rejects(self) -> None:
        manager = RecoveryManager()
        decision = manager.decide(
            result=_result(),
            breadcrumb=None,
            current_pose=(0.0, 0.0, 0.0),
            goal_id="recover",
        )
        self.assertFalse(decision.backward_allowed)
        self.assertIsNone(decision.backward_goal)

    def test_budget_blocks_consecutive_loop(self) -> None:
        manager = RecoveryManager(config=RecoveryManagerConfig(max_consecutive=1))
        first = manager.decide(
            result=_result(), breadcrumb=_crumb(),
            current_pose=(0.10, 0.0, 0.0), goal_id="r",
        )
        self.assertTrue(first.backward_allowed)
        second = manager.decide(
            result=_result(), breadcrumb=_crumb(),
            current_pose=(0.10, 0.0, 0.0), goal_id="r",
        )
        self.assertFalse(second.backward_allowed)


if __name__ == "__main__":
    unittest.main()