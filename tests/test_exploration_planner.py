import unittest

from app.navigation.exploration_planner import generate_exploration_candidates
from app.navigation.video_navigation_map import build_video_navigation_map
from app.navigation.video_pose_estimator import estimate_video_trajectory


class ExplorationPlannerTest(unittest.TestCase):
    def test_generates_frontiers_without_target(self):
        navigation_map = build_video_navigation_map(estimate_video_trajectory("missing.mp4", backend="mock"))
        candidates = generate_exploration_candidates(navigation_map, {"target_status": "target_not_seen"})
        self.assertTrue(candidates)
        self.assertEqual(candidates[0].waypoint_type, "frontier")


if __name__ == "__main__":
    unittest.main()
