from __future__ import annotations

import unittest

from app.live_robot.recovery_motion import (
    SafeMotionSegment,
    create_forward_breadcrumb,
    evaluate_backward_safety,
)


def _crumb(now: float, *, heading: float = 0.0) -> SafeMotionSegment:
    return SafeMotionSegment(
        start_pose=(0.0, 0.0, heading),
        end_pose=(0.10, 0.0, heading),
        signed_distance_m=0.10,
        heading_rad=heading,
        created_monotonic_s=now,
        source_step="f0.10",
    )


class TestBackwardBreadcrumbSafety(unittest.TestCase):
    def test_no_breadcrumb_fails_closed(self) -> None:
        decision = evaluate_backward_safety(
            None,
            current_pose=(0.0, 0.0, 0.0),
            requested_distance_m=0.10,
            max_backward_step_m=0.12,
            min_backward_step_m=0.05,
            max_age_sec=8.0,
            heading_tolerance_deg=8.0,
            now=100.0,
        )
        self.assertFalse(decision.allowed)
        self.assertIn("NO_VALID_REVERSE_CORRIDOR", decision.reason)

    def test_expired_breadcrumb_fails_closed(self) -> None:
        decision = evaluate_backward_safety(
            _crumb(now=90.0),
            current_pose=(0.10, 0.0, 0.0),
            requested_distance_m=0.10,
            max_backward_step_m=0.12,
            min_backward_step_m=0.05,
            max_age_sec=8.0,
            heading_tolerance_deg=8.0,
            now=100.0,
        )
        self.assertFalse(decision.allowed)

    def test_yaw_mismatch_fails_closed(self) -> None:
        decision = evaluate_backward_safety(
            _crumb(now=100.0, heading=0.0),
            current_pose=(0.10, 0.0, 0.5),
            requested_distance_m=0.10,
            max_backward_step_m=0.12,
            min_backward_step_m=0.05,
            max_age_sec=8.0,
            heading_tolerance_deg=8.0,
            now=101.0,
        )
        self.assertFalse(decision.allowed)

    def test_valid_breadcrumb_allows_bounded_distance(self) -> None:
        decision = evaluate_backward_safety(
            _crumb(now=100.0),
            current_pose=(0.10, 0.0, 0.0),
            requested_distance_m=0.12,
            max_backward_step_m=0.12,
            min_backward_step_m=0.05,
            max_age_sec=8.0,
            heading_tolerance_deg=8.0,
            now=100.5,
        )
        self.assertTrue(decision.allowed)
        self.assertAlmostEqual(decision.distance_m, 0.10)
        self.assertEqual(decision.source, "breadcrumb")

    def test_create_forward_breadcrumb_rejects_discontinuity(self) -> None:
        crumb = create_forward_breadcrumb(
            (0.0, 0.0, 0.0),
            (0.1, 0.0, 0.0),
            signed_distance_m=0.1,
            source_step="f0.1",
            yaw_drift_deg=0.0,
            max_yaw_drift_deg=8.0,
            odom_discontinuity=True,
        )
        self.assertIsNone(crumb)


if __name__ == "__main__":
    unittest.main()