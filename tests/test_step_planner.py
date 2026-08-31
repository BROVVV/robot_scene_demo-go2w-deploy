"""Unit tests for the pure short-step planner."""

from __future__ import annotations

import unittest

from app.live_robot.step_planner import (
    PlanKind,
    describe_step,
    plan_approach_step,
    plan_scan_step,
    scan_sequence,
    verify_rejection_step,
)


class ScanSequenceTest(unittest.TestCase):
    def test_net_heading_returns_to_zero(self):
        sequence = scan_sequence(scan_turn_deg=30, scan_span=3)
        net = sum(
            (30 if step.startswith("l") else -30 if step.startswith("r") else 0)
            for step in sequence
        )
        self.assertEqual(net, 0)
        self.assertEqual(len(sequence), 14)
        self.assertEqual(sequence[:3], ["r30", "r30", "r30"])


class ApproachPlanTest(unittest.TestCase):
    def test_centered_target_plans_forward(self):
        plan = plan_approach_step(center_x=0.5, area_ratio=0.05)
        self.assertEqual(plan.kind, PlanKind.MOVE)
        self.assertEqual(plan.step, "f")

    def test_left_target_plans_left_turn(self):
        plan = plan_approach_step(center_x=0.3, area_ratio=0.05)
        self.assertEqual(plan.kind, PlanKind.MOVE)
        self.assertTrue(plan.step.startswith("l"))

    def test_right_target_plans_right_turn(self):
        plan = plan_approach_step(center_x=0.8, area_ratio=0.05)
        self.assertEqual(plan.kind, PlanKind.MOVE)
        self.assertTrue(plan.step.startswith("r"))

    def test_reach_threshold_requests_verification(self):
        plan = plan_approach_step(center_x=0.5, area_ratio=0.20)
        self.assertEqual(plan.kind, PlanKind.VERIFY)

    def test_forward_beyond_radius_aborts(self):
        plan = plan_approach_step(
            center_x=0.5, area_ratio=0.05,
            distance_m=0.95, max_radius_m=1.0,
        )
        self.assertEqual(plan.kind, PlanKind.ABORT_RADIUS)


class ScanPlanTest(unittest.TestCase):
    def test_scan_step_is_from_sequence(self):
        plan = plan_scan_step(0, scan_turn_deg=30, scan_span=2)
        self.assertEqual(plan.kind, PlanKind.SCAN)
        self.assertEqual(plan.step, "r30")

    def test_forward_near_radius_becomes_turn(self):
        plan = plan_scan_step(
            4, scan_turn_deg=30, scan_span=2,
            distance_m=0.95, max_radius_m=1.0,
        )
        self.assertTrue(plan.step.startswith("r"))


class MiscTest(unittest.TestCase):
    def test_describe_step(self):
        self.assertEqual(describe_step("f"), "前进 0.12 m/s × 2s")
        self.assertEqual(describe_step("l30"), "左转 30°")
        self.assertEqual(describe_step("r45"), "右转 45°")

    def test_rejection_step(self):
        plan = verify_rejection_step(15.0)
        self.assertEqual(plan.step, "r15")
        self.assertEqual(plan.kind, PlanKind.SCAN)


if __name__ == "__main__":
    unittest.main()
