from __future__ import annotations

import json
import unittest
from dataclasses import replace
from pathlib import Path

from app.config import Settings
from app.schemas import RoutePlan, RouteStep, SceneAnalysisResult, TargetDecision
from app.services.ros2_command_exporter import build_ros2_motion_plan


ROOT = Path(__file__).resolve().parents[1]


def _settings(**updates) -> Settings:
    return replace(Settings(siliconflow_api_key=""), **updates)


class Ros2DynamicMotionHorizonTest(unittest.TestCase):
    def test_open_area_exports_more_than_half_meter(self) -> None:
        scene = _open_area_scene(target_present=False, distance=5.0)
        settings = _settings(motion_horizon_profile="platform_assisted_open_area")

        plan = build_ros2_motion_plan(
            scene,
            generated_at="2026-06-26T00:00:00+00:00",
            settings=settings,
        )

        move = plan.commands[0]
        self.assertEqual(move.source_action, "move_forward")
        self.assertGreater(move.distance_m, 0.5)
        self.assertLessEqual(move.distance_m, settings.motion_platform_open_max_step_m)
        self.assertEqual(move.duration_sec, round(move.distance_m / 0.25, 3))
        self.assertTrue(move.platform_obstacle_avoidance_assumed)
        self.assertTrue(move.interruptible_by_platform)
        self.assertTrue(plan.platform_obstacle_avoidance_assumed)
        self.assertEqual(
            plan.motion_horizon_decision["motion_policy"],
            "platform_assisted_open_area",
        )

    def test_target_candidate_still_shortens_exported_distance(self) -> None:
        scene = _open_area_scene(target_present=True, distance=5.0)
        settings = _settings(motion_horizon_profile="platform_assisted_open_area")

        plan = build_ros2_motion_plan(
            scene,
            generated_at="2026-06-26T00:00:00+00:00",
            settings=settings,
        )

        move = plan.commands[0]
        self.assertLessEqual(move.distance_m, settings.motion_target_confirm_max_step_m)
        self.assertEqual(
            plan.motion_horizon_decision["motion_policy"],
            "target_candidate_confirm",
        )

    def test_strict_safe_profile_restores_half_meter_limit(self) -> None:
        scene = _open_area_scene(target_present=False, distance=5.0)
        settings = _settings(
            motion_horizon_profile="strict_safe",
            platform_obstacle_avoidance_assumed=False,
        )

        plan = build_ros2_motion_plan(
            scene,
            generated_at="2026-06-26T00:00:00+00:00",
            settings=settings,
        )

        self.assertLessEqual(plan.commands[0].distance_m, 0.5)
        self.assertFalse(plan.commands[0].platform_obstacle_avoidance_assumed)
        self.assertEqual(plan.motion_horizon_decision["motion_policy"], "strict_safe")


def _open_area_scene(*, target_present: bool, distance: float) -> SceneAnalysisResult:
    base = SceneAnalysisResult.model_validate(
        json.loads((ROOT / "examples" / "mock_scene_result.json").read_text(encoding="utf-8"))
    )
    return base.model_copy(
        update={
            "scene_summary_zh": "室外开放区域，前方道路宽阔。",
            "target_decision": TargetDecision(
                target_text="红色消防器材",
                is_present=target_present,
                matched_object_ids=["obj_001"] if target_present else [],
                match_reason_zh=(
                    "目标候选已出现，需要短距离确认。"
                    if target_present
                    else "当前画面未确认目标。"
                ),
                confidence=0.8,
            ),
            "candidate_objects": (
                [{"decision": "candidate", "final_score": 0.8}]
                if target_present
                else []
            ),
            "candidate_summary": {"num_candidate": 1 if target_present else 0},
            "route_plan": RoutePlan(
                route_type="explore_likely_location",
                summary_zh="沿开放区域向前搜索。",
                steps=[
                    RouteStep(
                        step_id=1,
                        action="move_forward",
                        distance_m=distance,
                        description_zh="向前移动到下一观察点。",
                    )
                ],
                safety_notes_zh=[],
            ),
        }
    )


if __name__ == "__main__":
    unittest.main()
