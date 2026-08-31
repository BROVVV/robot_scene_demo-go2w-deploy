import unittest

from app.navigation.exploration_planner import generate_exploration_candidates
from app.navigation.navigation_instruction_generator import generate_navigation_instructions
from app.navigation.video_navigation_map import build_video_navigation_map
from app.navigation.video_pose_estimator import estimate_video_trajectory
from app.navigation.visual_path_planner import plan_visual_path


class NavigationInstructionGeneratorTest(unittest.TestCase):
    def test_relative_instruction_uses_relative_units(self):
        navigation_map = build_video_navigation_map(estimate_video_trajectory("missing.mp4", backend="relative"))
        plan = plan_visual_path(navigation_map, generate_exploration_candidates(navigation_map)[0])
        instructions = generate_navigation_instructions(plan)
        self.assertTrue(any("相对单位" in item["instruction"] for item in instructions))


if __name__ == "__main__":
    unittest.main()
