from __future__ import annotations

import math
import unittest

from app.live_robot.motion_bounds import (
    evaluate_motion_observation,
    evaluate_step_boundary,
)


class TestBackwardMotionBounds(unittest.TestCase):
    def test_backward_observation_accepts_negative_signed_progress(self) -> None:
        result = evaluate_motion_observation(
            "b0.10",
            before=(0.0, 0.0, 0.0),
            after=(-0.09, 0.01, 0.02),
            expected_forward_m=0.10,
        )
        self.assertTrue(result.allowed)
        self.assertLess(result.signed_progress_m, 0.0)
        self.assertEqual(result.motion_direction, "backward")

    def test_backward_observation_rejects_wrong_direction(self) -> None:
        result = evaluate_motion_observation(
            "b0.10",
            before=(0.0, 0.0, 0.0),
            after=(0.09, 0.0, 0.0),
            expected_forward_m=0.10,
        )
        self.assertFalse(result.allowed)
        self.assertEqual(result.code, "BACKWARD_WRONG_DIRECTION")

    def test_backward_observation_yaw_90_uses_body_axis(self) -> None:
        # Robot faces +Y, so backward is world -Y.
        result = evaluate_motion_observation(
            "b0.10",
            before=(0.0, 0.0, math.pi / 2.0),
            after=(0.0, -0.09, math.pi / 2.0),
            expected_forward_m=0.10,
        )
        self.assertTrue(result.allowed)
        self.assertLess(result.signed_progress_m, 0.0)

    def test_backward_observation_rejects_no_motion(self) -> None:
        result = evaluate_motion_observation(
            "b0.10",
            before=(0.0, 0.0, 0.0),
            after=(0.005, 0.0, 0.0),
            expected_forward_m=0.10,
        )
        self.assertFalse(result.allowed)
        self.assertEqual(result.code, "BACKWARD_NOT_CONFIRMED")

    def test_step_boundary_predicts_backward(self) -> None:
        decision = evaluate_step_boundary(
            "b0.10",
            origin=(0.0, 0.0, 0.0),
            current=(0.10, 0.0, 0.0),
            max_radius_m=1.0,
            front_half_plane_only=True,
            turn_only=False,
            forward_distance_m=0.10,
            tolerance_m=0.01,
        )
        self.assertTrue(decision.allowed)
        self.assertAlmostEqual(decision.predicted_position[0], 0.0, places=2)


if __name__ == "__main__":
    unittest.main()