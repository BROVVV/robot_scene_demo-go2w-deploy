import tempfile
import unittest
from pathlib import Path

from app.live_robot.frame_bundle_reader import FrameBundle
from app.live_robot.live_search_pipeline import (
    _video_frames,
    enforce_relation_evidence_gate,
    run_live_bundle_search,
    sensor_snapshot_from_health,
)
from app.video.schemas import SceneGraph, SceneGraphEdge, SceneGraphNode
from app.video.target_profile import TargetProfile


class LiveSearchPipelineTests(unittest.TestCase):
    @staticmethod
    def _relation_profile() -> TargetProfile:
        return TargetProfile(
            raw_query="饮水机旁边的蓝色垃圾桶",
            canonical_name_zh="蓝色垃圾桶",
            primary_labels_en=["trash bin"],
            colors=["blue"],
            relation_constraints=["next to water cooler"],
            context_labels_en=["water cooler"],
            context_labels_zh=["饮水机"],
        )

    @staticmethod
    def _node(node_id, label, attributes=None):
        return SceneGraphNode(
            node_id=node_id, node_type="object", label=label,
            label_zh=label, category="object", source="observed",
            confidence=0.9, evidence_level="observed_confirmed",
            attributes=attributes or {},
        )

    def test_relation_target_requires_strong_observed_relation(self):
        gate = {
            "target_found": True,
            "best_evidence": {"candidate_id": "bin"},
            "passed_rules": ["TARGET_CONFIRMATION_REQUIRE_CROP_VERIFY"],
            "blocking_rules": [],
        }
        without_relation = SceneGraph(nodes=[
            self._node("bin", "trash bin", {"attributes": ["blue"]}),
            self._node("water", "water cooler"),
        ])
        rejected = enforce_relation_evidence_gate(
            gate, self._relation_profile(), without_relation
        )
        self.assertFalse(rejected["target_found"])
        self.assertIsNone(rejected["best_evidence"])
        self.assertIn(
            "TARGET_CONFIRMATION_REQUIRE_RELATION_EVIDENCE",
            rejected["blocking_rules"],
        )

        with_relation = SceneGraph(
            nodes=without_relation.nodes,
            edges=[SceneGraphEdge(
                edge_id="near", source_node_id="bin", target_node_id="water",
                relation="near", source="observed", confidence=0.9,
                evidence_level="observed_confirmed",
            )],
        )
        accepted = enforce_relation_evidence_gate(
            gate, self._relation_profile(), with_relation
        )
        self.assertTrue(accepted["target_found"])
        self.assertIn(
            "TARGET_CONFIRMATION_REQUIRE_RELATION_EVIDENCE",
            accepted["passed_rules"],
        )

    def test_video_frames_snapshot_survives_spool_bundle_pruning(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bundle_dir = root / "spool" / "bundle-1"
            bundle_dir.mkdir(parents=True)
            image = bundle_dir / "image.jpg"
            image.write_bytes(b"stable-live-frame")
            bundle = FrameBundle(
                directory=bundle_dir,
                image_path=image,
                payload={
                    "frame_id": 42,
                    "image_receive_time_ns": 123456789,
                    "camera_info": {"width": 1920, "height": 1080},
                },
            )

            frames = _video_frames([bundle], snapshot_dir=root / "output" / "input_frames")
            image.unlink()

            self.assertEqual(len(frames), 1)
            self.assertNotEqual(frames[0].image_path, image)
            self.assertEqual(frames[0].image_path.read_bytes(), b"stable-live-frame")

    def test_camera_intrinsics_never_imply_rgb_lidar_extrinsics(self):
        snapshot = sensor_snapshot_from_health(
            {
                "camera": True,
                "camera_info_calibrated": True,
                "rgb_lidar_extrinsics": False,
                "lidar": True,
                "lio": False,
                "tf": True,
            }
        )
        self.assertTrue(snapshot.camera_fresh)
        self.assertFalse(snapshot.extrinsics_ready)

    def test_unhealthy_lidar_writes_complete_blocked_session_without_inference(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            image = root / "image.jpg"
            image.write_bytes(b"not-decoded-because-gate-closes-first")
            bundle = FrameBundle(
                directory=root,
                image_path=image,
                payload={
                    "session_id": "test_session",
                    "frame_id": 1,
                    "image_receive_time_ns": 1,
                    "camera_info": {"width": 1920, "height": 1080},
                    "sensor_health": {
                        "camera": True,
                        "camera_info_calibrated": False,
                        "rgb_lidar_extrinsics": False,
                        "rgb_lidar_fusion": False,
                        "lidar": False,
                        "lio": False,
                        "tf": False,
                    },
                },
            )
            output = root / "session"
            result = run_live_bundle_search(
                [bundle], target="手机", detector="grounded_sam", output_dir=output
            )
            self.assertEqual(result["status"], "blocked_wait_for_sensors")
            required = {
                "target_profile.json",
                "target_search.json",
                "target_timeline.json",
                "target_candidates.json",
                "object_tracks.json",
                "track_summary.json",
                "crop_verify_results.json",
                "evidence_gating_report.json",
                "frame_observations.json",
                "scene_graph.json",
                "scene_graph.graphml",
                "navigation_topology.json",
                "navigation_topology.graphml",
                "search_trace.json",
                "sensor_health.json",
                "safety_events.jsonl",
                "report.md",
                "final_report.md",
                "task.json",
                "parsed_task.json",
                "capability_gate_result.json",
                "grounding_prompt_plan.json",
                "memory_provenance.json",
                "motion_commands.jsonl",
                "nav2_requests.jsonl",
                "sensor_health.jsonl",
            }
            self.assertTrue(required.issubset({item.name for item in output.iterdir()}))
            self.assertEqual((output / "motion_commands.jsonl").read_text(), "")
            self.assertEqual((output / "nav2_requests.jsonl").read_text(), "")


if __name__ == "__main__":
    unittest.main()
