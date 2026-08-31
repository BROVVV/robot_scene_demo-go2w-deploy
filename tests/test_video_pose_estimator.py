import unittest

from app.navigation.video_pose_estimator import estimate_video_trajectory


class VideoPoseEstimatorTest(unittest.TestCase):
    def test_relative_backend_never_marks_metric(self):
        trajectory = estimate_video_trajectory("missing.mp4", backend="relative", max_frames=10)
        self.assertEqual(trajectory[0].pose.x, 0.0)
        self.assertEqual(trajectory[0].pose.scale_status, "relative")
        self.assertGreater(len(trajectory), 1)

    def test_metric_backend_marks_metric(self):
        trajectory = estimate_video_trajectory("missing.mp4", backend="metric", max_frames=10)
        self.assertEqual(trajectory[0].pose.scale_status, "metric")
        self.assertTrue(trajectory[0].pose.provenance["scale_verified"])


if __name__ == "__main__":
    unittest.main()
