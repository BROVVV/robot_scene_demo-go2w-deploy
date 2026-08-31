import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from app.video.video_reader import read_and_sample_video


class _FakeImage:
    shape = (48, 64, 3)


class _FakeCapture:
    def __init__(self, path: str) -> None:
        self.path = path
        self.position = 0

    def isOpened(self) -> bool:
        return True

    def get(self, key: int) -> float:
        return {
            1: 10.0,
            2: 25,
            3: 64,
            4: 48,
        }[key]

    def set(self, key: int, value: int) -> None:
        self.position = value

    def read(self):
        return True, _FakeImage()

    def release(self) -> None:
        pass


class _FakeCv2:
    CAP_PROP_FPS = 1
    CAP_PROP_FRAME_COUNT = 2
    CAP_PROP_FRAME_WIDTH = 3
    CAP_PROP_FRAME_HEIGHT = 4
    CAP_PROP_POS_FRAMES = 5

    VideoCapture = _FakeCapture

    @staticmethod
    def imwrite(path: str, image) -> bool:
        Path(path).write_bytes(b"fake-jpeg")
        return True


class VideoReaderTest(unittest.TestCase):
    def test_reads_metadata_and_saves_sampled_frames(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            video = Path(tmpdir) / "walk.mp4"
            video.write_bytes(b"fake-video")
            output = Path(tmpdir) / "frames"
            with patch("app.video.video_reader._load_cv2", return_value=_FakeCv2):
                metadata, frames = read_and_sample_video(
                    video,
                    sample_fps=1.0,
                    max_frames=3,
                    output_dir=output,
                )

            self.assertEqual(metadata.fps, 10.0)
            self.assertEqual(metadata.sampled_keyframes, 3)
            self.assertEqual([frame.frame_id for frame in frames], [0, 10, 20])
            self.assertTrue(all(frame.image_path.is_file() for frame in frames))


if __name__ == "__main__":
    unittest.main()
