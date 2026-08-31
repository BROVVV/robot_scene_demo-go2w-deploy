import unittest

from app.navigation.exploration_planner import generate_exploration_candidates
from app.navigation.nav2_adapter import adapt_visual_plan_to_nav2_goal
from app.navigation.video_navigation_map import build_video_navigation_map
from app.navigation.video_pose_estimator import estimate_video_trajectory
from app.navigation.visual_path_planner import plan_visual_path


class Nav2AdapterTest(unittest.TestCase):
    def test_rejects_relative_visual_plan(self):
        navigation_map = build_video_navigation_map(estimate_video_trajectory("missing.mp4", backend="relative"))
        plan = plan_visual_path(navigation_map, generate_exploration_candidates(navigation_map)[0])
        result = adapt_visual_plan_to_nav2_goal(plan)
        self.assertFalse(result.allowed)
        self.assertIn("No metric scale", result.reason)

    def test_metric_video_plan_requires_and_uses_map_transform(self):
        navigation_map = build_video_navigation_map(estimate_video_trajectory("missing.mp4", backend="metric"))
        plan = plan_visual_path(navigation_map, generate_exploration_candidates(navigation_map)[0])
        rejected = adapt_visual_plan_to_nav2_goal(plan)
        self.assertFalse(rejected.allowed)
        accepted = adapt_visual_plan_to_nav2_goal(plan, transform={"x": 1.0, "y": 2.0, "yaw": 0.0})
        self.assertTrue(accepted.allowed)
        self.assertEqual(accepted.goal_pose.frame_id, "map")
        self.assertGreater(accepted.goal_pose.x, 1.0)


if __name__ == "__main__":
    unittest.main()
