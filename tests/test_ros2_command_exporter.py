from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from app.schemas import (
    QuadrupedActionPrimitive,
    QuadrupedSearchPlan,
    RoutePlan,
    SceneAnalysisResult,
    ViewpointStep,
)
from app.services.ros2_command_exporter import (
    build_quadruped_ros2_motion_plan,
    build_ros2_motion_plan,
    export_ros2_motion_plan,
)


ROOT = Path(__file__).resolve().parents[1]


class Ros2CommandExporterTest(unittest.TestCase):
    def test_quadruped_plan_only_exports_motion_whitelist(self) -> None:
        plan = QuadrupedSearchPlan(
            plan_id="plan_001",
            target_text="手机",
            target_found=False,
            plan_type="situated_viewpoint_search",
            steps=[
                ViewpointStep(
                    step_id="view_001",
                    primitive=QuadrupedActionPrimitive.TURN_LEFT,
                    reason_zh="向左转后观察。",
                    expected_information_gain=0.8,
                    safety_level="conservative",
                ),
                ViewpointStep(
                    step_id="view_002",
                    primitive=QuadrupedActionPrimitive.ASK_HUMAN_FOR_OCCLUDED_AREA,
                    reason_zh="请求人工查看封闭区域。",
                    expected_information_gain=0.4,
                    safety_level="human_required",
                ),
            ],
        )
        output = build_quadruped_ros2_motion_plan(
            plan,
            generated_at="2026-06-25T00:00:00+00:00",
        )
        self.assertEqual(
            [item.source_action for item in output.commands],
            ["turn_left", "stop"],
        )
        self.assertIn(
            "ask_human_for_occluded_area",
            " ".join(output.safety_notes_zh),
        )

    def test_builds_cmd_vel_commands_from_route_steps(self) -> None:
        result = _mock_scene()

        plan = build_ros2_motion_plan(
            result,
            generated_at="2026-06-17T00:00:00+00:00",
        )

        self.assertTrue(plan.dry_run)
        self.assertEqual(plan.topic, "/cmd_vel")
        self.assertEqual(plan.message_type, "geometry_msgs/msg/Twist")
        self.assertEqual(plan.frame_id, "base_link")
        self.assertEqual(len(plan.commands), 6)

        move = plan.commands[0]
        self.assertEqual(move.source_action, "move_forward")
        self.assertEqual(move.route_step_id, 1)
        self.assertEqual(move.twist.linear.x, 0.25)
        self.assertEqual(move.twist.angular.z, 0.0)
        self.assertEqual(move.distance_m, 0.8)
        self.assertEqual(move.duration_sec, 3.2)
        self.assertTrue(move.platform_obstacle_avoidance_assumed)
        self.assertTrue(move.interruptible_by_platform)
        self.assertTrue(plan.dynamic_motion_horizon_enabled)
        self.assertEqual(
            plan.motion_horizon_decision["motion_policy"],
            "target_candidate_confirm",
        )

        self.assertEqual(plan.commands[1].source_action, "stop")

        turn = plan.commands[2]
        self.assertEqual(turn.source_action, "turn_right")
        self.assertEqual(turn.twist.linear.x, 0.0)
        self.assertEqual(turn.twist.angular.z, -0.5)
        self.assertAlmostEqual(turn.duration_sec, 0.698, places=3)

        stop = plan.commands[-1]
        self.assertEqual(stop.source_action, "stop")
        self.assertEqual(stop.twist.linear.x, 0.0)
        self.assertEqual(stop.twist.angular.z, 0.0)
        self.assertEqual(stop.duration_sec, 1.0)
        for index, command in enumerate(plan.commands[:-1]):
            if command.source_action != "stop":
                self.assertEqual(plan.commands[index + 1].source_action, "stop")

    def test_empty_route_exports_safe_stop_command(self) -> None:
        result = _mock_scene().model_copy(
            update={
                "route_plan": RoutePlan(
                    route_type="explore_likely_location",
                    summary_zh="无安全路线。",
                    steps=[],
                    safety_notes_zh=[],
                )
            }
        )

        plan = build_ros2_motion_plan(
            result,
            generated_at="2026-06-17T00:00:00+00:00",
        )

        self.assertEqual(len(plan.commands), 1)
        self.assertEqual(plan.commands[0].source_action, "stop")
        self.assertIsNone(plan.commands[0].route_step_id)
        self.assertEqual(plan.commands[0].twist.linear.x, 0.0)
        self.assertIn("dry_run", " ".join(plan.safety_notes_zh))

    def test_export_writes_json_file(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            output_path = Path(tmp_dir) / "ros2_motion_plan.json"

            returned_path = export_ros2_motion_plan(_mock_scene(), output_path)

            self.assertEqual(returned_path, output_path)
            data = json.loads(output_path.read_text(encoding="utf-8"))
            self.assertEqual(data["topic"], "/cmd_vel")
            self.assertEqual(data["commands"][0]["message_type"], "geometry_msgs/msg/Twist")
            self.assertIn("motion_horizon_decision", data)


def _mock_scene() -> SceneAnalysisResult:
    data = json.loads((ROOT / "examples" / "mock_scene_result.json").read_text(encoding="utf-8"))
    return SceneAnalysisResult.model_validate(data)


if __name__ == "__main__":
    unittest.main()
