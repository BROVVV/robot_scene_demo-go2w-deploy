from __future__ import annotations

import math
import unittest

from app.live_robot.motion_bounds import (
    evaluate_lidar_motion_readiness,
    evaluate_motion_observation,
    evaluate_rotation_clearance,
    evaluate_step_boundary,
    position_within_boundary,
)


class MotionBoundsTests(unittest.TestCase):
    def test_forward_observation_accepts_plausible_motion(self) -> None:
        result = evaluate_motion_observation(
            "f0.300",
            before=(0.0, 0.0, 0.0),
            after=(0.27, 0.01, 0.02),
            expected_forward_m=0.30,
        )
        self.assertTrue(result.allowed)

    def test_forward_observation_rejects_no_motion(self) -> None:
        result = evaluate_motion_observation(
            "f0.300",
            before=(0.0, 0.0, 0.0),
            after=(0.01, 0.0, 0.0),
            expected_forward_m=0.30,
        )
        self.assertFalse(result.allowed)
        self.assertEqual(result.code, "FORWARD_NOT_CONFIRMED")

    def test_forward_observation_rejects_duplicate_odom_jump(self) -> None:
        result = evaluate_motion_observation(
            "f0.300",
            before=(4.50, 2.55, 0.9),
            after=(0.40, 1.07, 1.1),
            expected_forward_m=0.30,
        )
        self.assertFalse(result.allowed)
        self.assertEqual(result.code, "ODOM_DISCONTINUITY")

    def test_lidar_readiness_rejects_stale_missing_and_nan(self) -> None:
        self.assertFalse(evaluate_lidar_motion_readiness(
            lidar_fresh=False, front_clearance_m=1.0,
            minimum_clearance_m=0.3,
        ).allowed)
        self.assertFalse(evaluate_lidar_motion_readiness(
            lidar_fresh=True, front_clearance_m=None,
            minimum_clearance_m=0.3,
        ).allowed)
        self.assertFalse(evaluate_lidar_motion_readiness(
            lidar_fresh=True, front_clearance_m=math.nan,
            minimum_clearance_m=0.3,
        ).allowed)

    def test_lidar_readiness_accepts_fresh_infinite_no_return(self) -> None:
        self.assertTrue(evaluate_lidar_motion_readiness(
            lidar_fresh=True, front_clearance_m=math.inf,
            minimum_clearance_m=0.3,
        ).allowed)

    def test_turn_only_rejects_forward_but_allows_turn(self) -> None:
        common = {
            "origin": (1.0, 2.0, 0.0),
            "current": (1.0, 2.0, 0.0),
            "max_radius_m": 1.5,
            "front_half_plane_only": True,
            "turn_only": True,
            "forward_distance_m": 0.2,
        }
        self.assertTrue(evaluate_step_boundary("r10", **common).allowed)
        self.assertFalse(evaluate_step_boundary("f", **common).allowed)

    def test_front_half_disk_is_fixed_to_initial_heading(self) -> None:
        origin = (0.0, 0.0, math.pi / 2.0)
        self.assertTrue(position_within_boundary(
            origin=origin, position=(0.0, 1.4), max_radius_m=1.5,
            front_half_plane_only=True,
        ).allowed)
        self.assertFalse(position_within_boundary(
            origin=origin, position=(0.0, -0.2), max_radius_m=1.5,
            front_half_plane_only=True, tolerance_m=0.01,
        ).allowed)
        self.assertFalse(position_within_boundary(
            origin=origin, position=(0.0, 1.6), max_radius_m=1.5,
            front_half_plane_only=True, tolerance_m=0.01,
        ).allowed)

    def test_forward_prediction_uses_current_heading_inside_initial_boundary(self) -> None:
        decision = evaluate_step_boundary(
            "f",
            origin=(0.0, 0.0, 0.0),
            current=(0.0, 0.0, math.pi),
            max_radius_m=1.5,
            front_half_plane_only=True,
            turn_only=False,
            forward_distance_m=0.2,
            tolerance_m=0.01,
        )
        self.assertFalse(decision.allowed)
        self.assertIn("behind", decision.reason)

    def test_rotation_requires_both_sides_outside_full_body_envelope(self) -> None:
        blocked = evaluate_rotation_clearance(
            "r10",
            left_clearance_m=0.38,
            right_clearance_m=0.41,
            minimum_clearance_m=0.511,
            clearance_valid=True,
        )
        self.assertFalse(blocked.allowed)
        self.assertIn("rotation envelope blocked", blocked.reason)
        self.assertTrue(evaluate_rotation_clearance(
            "r10",
            left_clearance_m=0.60,
            right_clearance_m=0.55,
            minimum_clearance_m=0.511,
            clearance_valid=True,
        ).allowed)
        self.assertFalse(evaluate_rotation_clearance(
            "l10",
            left_clearance_m=None,
            right_clearance_m=0.80,
            minimum_clearance_m=0.511,
            clearance_valid=True,
        ).allowed)
        invalid = evaluate_rotation_clearance(
            "l10",
            left_clearance_m=math.inf,
            right_clearance_m=math.inf,
            minimum_clearance_m=0.511,
            clearance_valid=False,
        )
        self.assertFalse(invalid.allowed)
        self.assertIn("not validated", invalid.reason)
        self.assertFalse(evaluate_rotation_clearance(
            "r10",
            left_clearance_m=math.inf,
            right_clearance_m=math.inf,
            minimum_clearance_m=0.0,
            clearance_valid=None,
        ).allowed)


if __name__ == "__main__":
    unittest.main()
