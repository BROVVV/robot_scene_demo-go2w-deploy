import unittest

from app.navigation.exploration_planner import generate_exploration_candidates
from app.navigation.video_navigation_map import build_video_navigation_map
from app.navigation.video_pose_estimator import estimate_video_trajectory
from app.navigation.visual_path_planner import plan_visual_path


class VisualPathPlannerTest(unittest.TestCase):
    def test_relative_plan_is_not_executable(self):
        navigation_map = build_video_navigation_map(estimate_video_trajectory("missing.mp4", backend="relative"))
        goal = generate_exploration_candidates(navigation_map)[0]
        plan = plan_visual_path(navigation_map, goal, navigation_strategy="exploration")
        self.assertFalse(plan.executable)
        self.assertIn("No metric scale", plan.executable_reason)
        self.assertGreaterEqual(len(plan.path), 2)


if __name__ == "__main__":
    unittest.main()
