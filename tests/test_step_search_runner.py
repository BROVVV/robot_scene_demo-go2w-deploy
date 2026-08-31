"""Unit tests for the framework-level step-search runner with fakes."""

from __future__ import annotations

import unittest

from app.live_robot.search_state_machine import SearchState, SensorSnapshot
from app.live_robot.step_search_runner import (
    Detection,
    StepSearchConfig,
    StepSearchRunner,
    VerificationResult,
)


def _snapshot() -> SensorSnapshot:
    return SensorSnapshot(
        camera_fresh=True, lidar_fresh=True, robot_stationary=True
    )


def _odometry() -> tuple[float, float, float]:
    return (0.0, 0.0, 0.0)


class StepSearchRunnerTest(unittest.TestCase):
    def _runner(self, config, detect, verify, execute_steps):
        def execute(step: str):
            execute_steps.append(step)
            return True, ""

        return StepSearchRunner(
            config,
            detect=detect,
            verify=verify,
            execute_step=execute,
            snapshot=_snapshot,
            odometry=_odometry,
        )

    def test_reach_with_verification_confirms_target(self):
        calls = {"n": 0}

        def detect():
            calls["n"] += 1
            return [
                Detection(
                    "灰色书包", 0.9, (0.4, 0.1, 0.6, 0.9)
                )
            ]

        def verify(_bbox):
            return VerificationResult(
                "灰色书包", True, 0.92, "挂在办公椅上的灰色书包"
            )

        steps: list[str] = []
        runner = self._runner(
            StepSearchConfig(target="灰色书包", reach_area_ratio=0.15),
            detect,
            verify,
            steps,
        )
        result = runner.run()
        self.assertEqual(result["status"], "target_reached")
        self.assertEqual(result["steps_executed"], 0)
        self.assertEqual(result["final_state"], SearchState.FINISH.value)
        self.assertEqual(steps, [])
        self.assertTrue(
            any(
                e["event"] == "target_verification"
                and e["verification"]["is_target"]
                for e in result["events"]
            )
        )

    def test_off_center_target_aligns_before_approach(self):
        detections = [
            [Detection("灰色书包", 0.85, (0.3, 0.2, 0.4, 0.7))],
            [Detection("灰色书包", 0.9, (0.4, 0.1, 0.6, 0.9))],
        ]

        def detect():
            return detections.pop(0)

        def verify(_bbox):
            return VerificationResult("灰色书包", True, 0.9, "ok")

        steps: list[str] = []
        runner = self._runner(
            StepSearchConfig(target="灰色书包", reach_area_ratio=0.15),
            detect,
            verify,
            steps,
        )
        result = runner.run()
        self.assertEqual(result["status"], "target_reached")
        self.assertEqual(result["steps_executed"], 1)
        self.assertTrue(steps[0].startswith("l"))
        self.assertEqual(steps[0], "l7")

    def test_verification_rejection_turns_and_retries(self):
        verify_results = [
            VerificationResult("黑色椅子", False, 0.9, "不是书包"),
            VerificationResult("灰色书包", True, 0.9, "是书包"),
        ]

        def detect():
            return [Detection("灰色书包", 0.85, (0.4, 0.1, 0.6, 0.9))]

        def verify(_bbox):
            return verify_results.pop(0)

        steps: list[str] = []
        runner = self._runner(
            StepSearchConfig(target="灰色书包", reach_area_ratio=0.15),
            detect,
            verify,
            steps,
        )
        result = runner.run()
        self.assertEqual(result["status"], "target_reached")
        self.assertEqual(steps, ["r15"])
        self.assertTrue(
            any(
                e["event"] == "verification_rejected"
                for e in result["events"]
            )
        )

    def test_no_target_ends_by_time_limit(self):
        def detect():
            return []

        def verify(_bbox):
            return VerificationResult("", False, 0.0)

        steps: list[str] = []
        runner = self._runner(
            StepSearchConfig(target="灰色书包", max_seconds=0.0),
            detect,
            verify,
            steps,
        )
        result = runner.run()
        self.assertEqual(result["status"], "time_limit")
        self.assertEqual(steps, [])

    def test_operator_motion_step_limit_stops_after_one_successful_step(self):
        steps: list[str] = []
        runner = self._runner(
            StepSearchConfig(
                target="灰色书包",
                max_seconds=30.0,
                max_motion_steps=1,
                scan_turn_deg=10.0,
            ),
            lambda: [],
            lambda _bbox: VerificationResult("", False, 0.0),
            steps,
        )
        result = runner.run()
        self.assertEqual(result["status"], "motion_step_limit")
        self.assertEqual(result["steps_executed"], 1)
        self.assertEqual(steps, ["r10"])


if __name__ == "__main__":
    unittest.main()
