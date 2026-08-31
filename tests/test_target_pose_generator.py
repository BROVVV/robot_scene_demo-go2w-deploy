import unittest

from app.navigation.semantic_goal_localizer import localize_semantic_goal
from app.navigation.target_pose_generator import generate_observation_goal
from app.navigation.video_pose_estimator import estimate_video_trajectory


class TargetPoseGeneratorTest(unittest.TestCase):
    def test_observation_goal_is_not_raw_bbox(self):
        trajectory = estimate_video_trajectory("missing.mp4", backend="relative")
        localization = localize_semantic_goal(
            {
                "target": "背包",
                "target_status": "target_visual_confirmed",
                "best_evidence": {"frame_id": 5, "bbox": [0.4, 0.1, 0.6, 0.3], "confidence": 0.9},
            },
            trajectory,
        )
        goal = generate_observation_goal(localization)
        self.assertIsNotNone(goal)
        self.assertEqual(goal.waypoint_type, "observation")
        self.assertEqual(goal.pose.scale_status, "relative")


if __name__ == "__main__":
    unittest.main()
