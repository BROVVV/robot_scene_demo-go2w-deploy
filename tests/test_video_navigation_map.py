import unittest

from app.navigation.video_navigation_map import build_video_navigation_map
from app.navigation.video_pose_estimator import estimate_video_trajectory


class VideoNavigationMapTest(unittest.TestCase):
    def test_map_has_start_nodes_and_edges(self):
        trajectory = estimate_video_trajectory("missing.mp4", backend="mock")
        navigation_map = build_video_navigation_map(trajectory)
        self.assertEqual(navigation_map["nodes"][0]["frame_id"], 0)
        self.assertEqual(navigation_map["metadata"]["scale_status"], "relative")
        self.assertEqual(len(navigation_map["edges"]), len(navigation_map["nodes"]) - 1)


if __name__ == "__main__":
    unittest.main()
