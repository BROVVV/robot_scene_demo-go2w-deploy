import tempfile
import unittest
from pathlib import Path

from app.navigation.navigation_planning_pipeline import run_video_navigation_planning


class VideoNavigationPipelineTest(unittest.TestCase):
    def test_no_target_still_creates_exploration_plan(self):
        with tempfile.TemporaryDirectory() as directory:
            result = run_video_navigation_planning(
                video_path="missing.mp4",
                target_search_result={
                    "target": "红色背包",
                    "video_meta": {"duration_sec": 1.0, "sampled_keyframes": 3},
                    "target_status": "target_not_seen",
                    "target_found": False,
                    "candidate_regions": [],
                },
                output_root=directory,
            )
            plan = result["visual_navigation_plan"]
            self.assertEqual(plan["navigation_strategy"], "exploration")
            self.assertEqual(plan["scale_status"], "relative")
            self.assertTrue(Path(result["output_files"]["nav2_plan"]).is_file())

    def test_lost_after_seen_reobserves_last_known_location(self):
        with tempfile.TemporaryDirectory() as directory:
            result = run_video_navigation_planning(
                video_path="missing.mp4",
                target_search_result={
                    "target": "红色背包",
                    "video_meta": {"duration_sec": 2.0, "sampled_keyframes": 6},
                    "target_status": "target_lost_after_seen",
                    "target_found": False,
                    "best_evidence": {"frame_id": 10, "bbox": [0.4, 0.2, 0.6, 0.4], "confidence": 0.7},
                },
                output_root=directory,
            )
            self.assertEqual(
                result["visual_navigation_plan"]["navigation_strategy"],
                "last_known_reobserve",
            )

    def test_metric_plan_with_map_transform_prepares_nav2_goal(self):
        with tempfile.TemporaryDirectory() as directory:
            result = run_video_navigation_planning(
                video_path="missing.mp4",
                target_search_result={
                    "target": "红色背包",
                    "video_meta": {"duration_sec": 2.0, "sampled_keyframes": 6},
                    "target_status": "target_visual_confirmed",
                    "target_found": True,
                    "best_evidence": {"frame_id": 10, "bbox": [0.4, 0.2, 0.6, 0.4], "confidence": 0.9},
                },
                output_root=directory,
                mode="metric_preview",
                pose_backend="metric",
                calibration={"T_map_video_map": {"x": 1.0, "y": 2.0, "yaw": 0.0}},
            )
            self.assertTrue(result["nav2_adapter"]["allowed"])
            self.assertEqual(result["nav2_adapter"]["goal_pose"]["frame_id"], "map")

    def test_string_priority_candidate_region_is_supported(self):
        with tempfile.TemporaryDirectory() as directory:
            result = run_video_navigation_planning(
                video_path="missing.mp4",
                target_search_result={
                    "target": "红色背包",
                    "video_meta": {"duration_sec": 2.0, "sampled_keyframes": 6},
                    "target_status": "target_candidate",
                    "target_found": False,
                    "candidate_regions": [
                        {
                            "frame_id": 10,
                            "timestamp_sec": 1.0,
                            "priority": "high",
                            "reason": "疑似背包区域",
                        }
                    ],
                },
                output_root=directory,
            )
            self.assertEqual(
                result["visual_navigation_plan"]["navigation_strategy"],
                "candidate_navigation",
            )
            self.assertIsInstance(result["visual_navigation_plan"]["confidence"], float)


if __name__ == "__main__":
    unittest.main()
