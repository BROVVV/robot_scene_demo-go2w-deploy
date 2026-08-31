from __future__ import annotations

import unittest

from app.video.models import FrameAnalysisResult
from app.video.tracking import track_objects


def _frame(frame_id: int, bbox: list[float], score: float = 0.7) -> FrameAnalysisResult:
    return FrameAnalysisResult(
        frame_id=frame_id,
        timestamp_sec=float(frame_id),
        image_path=f"{frame_id}.jpg",
        annotated_frame_path=None,
        scene_summary="",
        relations=[],
        objects=[
            {
                "object_id": f"phone_{frame_id}",
                "label": "phone",
                "bbox": bbox,
                "confidence": score,
            }
        ],
    )


class VideoTrackingTest(unittest.TestCase):
    def test_nearby_boxes_join_and_multi_frame_track_confirms(self) -> None:
        tracks = track_objects(
            [
                _frame(1, [0.1, 0.1, 0.3, 0.4]),
                _frame(2, [0.11, 0.1, 0.31, 0.4]),
                _frame(3, [0.12, 0.1, 0.32, 0.4]),
            ],
            confirm_min_frames=3,
            confirm_score=0.65,
        )
        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracks[0]["decision"], "confirmed")

    def test_single_frame_false_positive_is_rejected(self) -> None:
        tracks = track_objects([_frame(1, [0.1, 0.1, 0.3, 0.4], 0.95)])
        self.assertEqual(tracks[0]["decision"], "rejected")


if __name__ == "__main__":
    unittest.main()
