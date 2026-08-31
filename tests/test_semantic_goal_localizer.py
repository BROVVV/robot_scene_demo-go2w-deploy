import unittest

from app.navigation.semantic_goal_localizer import localize_semantic_goal
from app.navigation.video_navigation_map import build_video_navigation_map
from app.navigation.video_pose_estimator import estimate_video_trajectory


class SemanticGoalLocalizerTest(unittest.TestCase):
    def test_confirmed_target_gets_relative_pose(self):
        trajectory = estimate_video_trajectory("missing.mp4", backend="relative")
        result = {
            "target": "红色背包",
            "target_status": "target_visual_confirmed",
            "best_evidence": {"frame_id": 10, "bbox": [400, 100, 600, 300], "confidence": 0.8},
        }
        localization = localize_semantic_goal(result, trajectory, build_video_navigation_map(trajectory))
        self.assertEqual(localization["goal_type"], "target")
        self.assertEqual(localization["target_pose"]["scale_status"], "relative")


if __name__ == "__main__":
    unittest.main()
