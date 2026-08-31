import json
import os
import tempfile
import unittest
from pathlib import Path

from app.live_robot.frame_bundle_reader import FrameBundleReader, FrameBundleUnavailable


class FrameBundleReaderTests(unittest.TestCase):
    def _payload(self):
        return {
            "schema_version": "1.0",
            "session_id": "session_test",
            "frame_id": 7,
            "image_path": "image.jpg",
            "image_receive_time_ns": 123,
            "image_capture_time_trusted": False,
            "camera_frame": "front_camera_optical_frame",
            "camera_info": {},
            "robot_pose": {"available": False},
            "clearance": {"lidar_fresh": False},
            "sensor_health": {
                "camera": True,
                "camera_info_calibrated": False,
                "rgb_lidar_extrinsics": False,
                "rgb_lidar_fusion": False,
                "lidar": False,
                "lio": False,
                "tf": False,
            },
        }

    def test_reads_only_complete_latest_bundle(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bundle = root / "bundles/frame"
            bundle.mkdir(parents=True)
            (bundle / "image.jpg").write_bytes(b"jpeg")
            (bundle / "frame_bundle.json").write_text(json.dumps(self._payload()))
            os.symlink(Path("bundles/frame"), root / "latest")
            reader = FrameBundleReader(root)
            with self.assertRaises(FrameBundleUnavailable):
                reader.read_latest()
            (bundle / "READY").write_text("ready\n")
            result = reader.read_latest()
            self.assertEqual(result.frame_id, 7)
            self.assertEqual(result.directory, bundle.resolve())

    def test_rejects_claimed_trusted_camera_capture_time(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bundle = root / "bundles/frame"
            bundle.mkdir(parents=True)
            payload = self._payload()
            payload["image_capture_time_trusted"] = True
            (bundle / "image.jpg").write_bytes(b"jpeg")
            (bundle / "frame_bundle.json").write_text(json.dumps(payload))
            (bundle / "READY").write_text("ready\n")
            os.symlink(Path("bundles/frame"), root / "latest")
            with self.assertRaises(FrameBundleUnavailable):
                FrameBundleReader(root).read_latest()


if __name__ == "__main__":
    unittest.main()
