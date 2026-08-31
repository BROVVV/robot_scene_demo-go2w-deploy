from __future__ import annotations

import json
import tempfile
import unittest

from app.video.full_scene_mapper import VideoFullSceneMapper


class VideoFullSceneMapperTest(unittest.TestCase):
    def test_mock_missing_video_exports_required_files(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            result, paths = VideoFullSceneMapper(output_dir=tmpdir).run(
                video_path="dummy.mp4",
                detector="mock",
                sample_fps=1.0,
                max_frames=1,
                enable_video_psg=True,
                enable_navigation_topology=True,
            )

            self.assertEqual(result.to_dict()["mode"], "full_scene_map")
            for key in [
                "video_frame_observations",
                "video_all_objects",
                "video_place_segments",
                "video_observed_scene_graph_json",
                "video_observed_scene_graph_graphml",
                "video_psg_layer",
                "video_hybrid_scene_graph_json",
                "video_hybrid_scene_graph_graphml",
                "video_navigation_topology",
                "video_navigation_topology_graphml",
                "video_navigation_topology_png",
                "video_navigation_topology_debug",
                "video_navigation_map",
                "video_full_scene_report",
            ]:
                self.assertIn(key, paths)
                self.assertTrue(paths[key].is_file(), key)

            psg = json.loads(paths["video_psg_layer"].read_text(encoding="utf-8"))
            for node in psg["predicted_nodes"]:
                self.assertEqual(node["source"], "predicted")
                self.assertFalse(node["can_confirm_target"])


if __name__ == "__main__":
    unittest.main()
