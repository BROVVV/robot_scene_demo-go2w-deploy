import unittest

from app.video.keyframe_selector import select_frame_indices


class KeyframeSelectorTest(unittest.TestCase):
    def test_samples_by_time_and_respects_limit(self) -> None:
        self.assertEqual(
            select_frame_indices(
                frame_count=300,
                source_fps=30.0,
                sample_fps=2.0,
                max_frames=4,
            ),
            [0, 15, 30, 45],
        )

    def test_rejects_invalid_sampling_values(self) -> None:
        with self.assertRaises(ValueError):
            select_frame_indices(30, 30.0, sample_fps=0)
        with self.assertRaises(ValueError):
            select_frame_indices(30, 0, sample_fps=1)


if __name__ == "__main__":
    unittest.main()
