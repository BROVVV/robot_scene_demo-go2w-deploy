from __future__ import annotations

import unittest
from dataclasses import replace

from app.config import Settings
from app.planning.motion_horizon import estimate_motion_horizon


def _settings(**updates) -> Settings:
    return replace(Settings(siliconflow_api_key=""), **updates)


class MotionHorizonPlannerTest(unittest.TestCase):
    def test_strict_safe_mode_clips_to_half_meter(self) -> None:
        decision = estimate_motion_horizon(
            requested_distance_m=3.0,
            scene_type="open_area",
            task_phase="search",
            target_candidate_visible=False,
            target_confirming=False,
            llm_recommended_horizon_m=None,
            settings=_settings(
                motion_horizon_profile="strict_safe",
                platform_obstacle_avoidance_assumed=False,
            ),
        )

        self.assertLessEqual(decision.recommended_distance_m, 0.5)
        self.assertEqual(decision.motion_policy, "strict_safe")

    def test_platform_assisted_indoor_allows_more_than_half_meter(self) -> None:
        settings = _settings(motion_horizon_profile="platform_assisted_indoor")

        decision = estimate_motion_horizon(
            requested_distance_m=3.0,
            scene_type="office",
            task_phase="search",
            target_candidate_visible=False,
            target_confirming=False,
            llm_recommended_horizon_m=None,
            settings=settings,
        )

        self.assertLessEqual(
            decision.recommended_distance_m,
            settings.motion_platform_indoor_max_step_m,
        )
        self.assertGreater(decision.recommended_distance_m, 0.5)

    def test_platform_assisted_open_area_exports_longer_segment(self) -> None:
        settings = _settings(motion_horizon_profile="platform_assisted_open_area")

        decision = estimate_motion_horizon(
            requested_distance_m=5.0,
            scene_type="open_area",
            task_phase="search",
            target_candidate_visible=False,
            target_confirming=False,
            llm_recommended_horizon_m=None,
            settings=settings,
        )

        self.assertGreaterEqual(decision.recommended_distance_m, 2.0)
        self.assertLessEqual(
            decision.recommended_distance_m,
            settings.motion_platform_open_max_step_m,
        )

    def test_target_candidate_shortens_distance(self) -> None:
        settings = _settings()

        decision = estimate_motion_horizon(
            requested_distance_m=5.0,
            scene_type="open_area",
            task_phase="approach_candidate",
            target_candidate_visible=True,
            target_confirming=False,
            llm_recommended_horizon_m=None,
            settings=settings,
        )

        self.assertLessEqual(
            decision.recommended_distance_m,
            settings.motion_target_confirm_max_step_m,
        )

    def test_llm_recommendation_cannot_break_absolute_max(self) -> None:
        settings = _settings(motion_absolute_max_step_m=6.0)

        decision = estimate_motion_horizon(
            requested_distance_m=20.0,
            scene_type="open_area",
            task_phase="search",
            target_candidate_visible=False,
            target_confirming=False,
            llm_recommended_horizon_m=20.0,
            settings=settings,
        )

        self.assertLessEqual(decision.recommended_distance_m, 6.0)
        self.assertLessEqual(decision.max_allowed_distance_m, 6.0)

    def test_llm_unavailable_with_platform_uses_fallback_over_half_meter(self) -> None:
        settings = _settings(platform_obstacle_avoidance_assumed=True)

        decision = estimate_motion_horizon(
            requested_distance_m=None,
            scene_type="unknown",
            task_phase="search",
            target_candidate_visible=False,
            target_confirming=False,
            llm_recommended_horizon_m=None,
            settings=settings,
        )

        self.assertAlmostEqual(
            decision.recommended_distance_m,
            settings.motion_platform_fallback_step_m,
        )
        self.assertGreater(decision.recommended_distance_m, 0.5)
        self.assertEqual(decision.source, "fallback")


if __name__ == "__main__":
    unittest.main()
