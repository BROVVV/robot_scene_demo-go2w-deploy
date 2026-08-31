from __future__ import annotations

import unittest

from app.live_robot.semantic_observer import (
    LiveSemanticObserver,
    semantic_payload_from_quick_target_absence,
)
from app.video.target_profile import TargetProfile


class LiveSemanticObserverTests(unittest.TestCase):
    def test_reuses_explicit_quick_negative_without_fabricating_objects(self):
        result = semantic_payload_from_quick_target_absence(
            {
                "objects": [],
                "scene_summary_zh": "办公室中未看到紫色三角锥",
                "target_decision": {
                    "is_present": False,
                    "confidence": 0.8,
                },
            },
            image_path="frame.jpg",
            frame_id="42",
        )
        self.assertIsNotNone(result)
        self.assertEqual(result["scene_objects"], [])
        self.assertEqual(result["scene_relations"], [])
        self.assertEqual(
            result["source"], "siliconflow_quick_explicit_target_absence"
        )

    def test_does_not_reuse_positive_or_ambiguous_quick_decision(self):
        for decision in ({"is_present": True}, {"is_present": None}, {}):
            self.assertIsNone(
                semantic_payload_from_quick_target_absence(
                    {
                        "scene_summary_zh": "办公室",
                        "target_decision": decision,
                    },
                    image_path="frame.jpg",
                    frame_id="42",
                )
            )

    def test_reuses_observed_scene_graph_and_cache_without_metric_position(self):
        clock = [10.0]
        calls = []
        payload = {
            "frame_id": "12",
            "scene_objects": [{
                "id": "water", "name": "water dispenser", "name_zh": "饮水机",
                "category": "appliance", "bbox_2d": [0.1, 0.1, 0.3, 0.8],
                "confidence": 0.9, "position": {"horizontal": "left"},
            }],
            "scene_relations": [],
        }
        observer = LiveSemanticObserver(
            lambda frame, profile: calls.append((frame, profile)) or payload,
            ttl_seconds=10.0, now=lambda: clock[0],
        )
        profile = TargetProfile(
            raw_query="找垃圾桶", canonical_name_zh="垃圾桶",
            primary_labels_en=["trash can"],
        )
        first = observer.observe(
            target_profile=profile, frame_or_bundle="frame.jpg",
            robot_pose={"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
        )
        second = observer.observe(
            target_profile=profile, frame_or_bundle="frame.jpg",
            robot_pose={"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
        )
        self.assertEqual(len(calls), 1)
        self.assertTrue(second.cache_hit)
        object_node = next(node for node in first.scene_graph.nodes if node.label == "water dispenser")
        self.assertEqual(object_node.attributes["position_status"], "observation_pose_only")
        self.assertNotIn("map_x", object_node.attributes)

    def test_translation_after_motion_forces_a_fresh_observation(self):
        calls = []
        observer = LiveSemanticObserver(
            lambda frame, _profile: calls.append(frame) or {
                "frame_id": str(len(calls)),
                "scene_objects": [],
                "scene_relations": [],
            },
            ttl_seconds=30.0,
            translation_refresh_m=0.05,
        )
        profile = TargetProfile(
            raw_query="找手机", canonical_name_zh="手机",
            primary_labels_en=["phone"],
        )
        observer.observe(
            target_profile=profile, frame_or_bundle="frame_1.jpg",
            robot_pose={"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
        )
        moved = observer.observe(
            target_profile=profile, frame_or_bundle="frame_2.jpg",
            robot_pose={"x": 0.08, "y": 0.0, "yaw_deg": 0.0},
        )
        self.assertEqual(calls, ["frame_1.jpg", "frame_2.jpg"])
        self.assertFalse(moved.cache_hit)


if __name__ == "__main__":
    unittest.main()
