"""OpenCV-backed video metadata reading and keyframe extraction."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from app.video.keyframe_selector import select_frame_indices
from app.video.models import VideoFrame, VideoMetadata


class VideoReadError(RuntimeError):
    """Raised when a video cannot be opened or decoded."""


def read_and_sample_video(
    video_path: str | Path,
    sample_fps: float = 1.0,
    max_frames: int = 120,
    output_dir: str | Path = "outputs/video_frames",
) -> tuple[VideoMetadata, list[VideoFrame]]:
    cv2 = _load_cv2()
    source = Path(video_path).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"Video file not found: {source}")

    capture = cv2.VideoCapture(str(source))
    if not capture.isOpened():
        capture.release()
        raise VideoReadError(f"Unable to open video: {source}")

    try:
        fps = float(capture.get(cv2.CAP_PROP_FPS))
        frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if fps <= 0 or frame_count <= 0:
            raise VideoReadError(
                f"Video has invalid metadata: fps={fps}, frame_count={frame_count}"
            )

        metadata = VideoMetadata(
            video_path=str(source),
            fps=fps,
            duration_sec=round(frame_count / fps, 3),
            frame_count=frame_count,
            width=width,
            height=height,
        )
        indices = select_frame_indices(frame_count, fps, sample_fps, max_frames)
        frame_dir = Path(output_dir)
        frame_dir.mkdir(parents=True, exist_ok=True)

        frames: list[VideoFrame] = []
        for frame_index in indices:
            capture.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
            ok, image = capture.read()
            if not ok or image is None:
                continue
            actual_height, actual_width = image.shape[:2]
            image_path = frame_dir / f"frame_{frame_index:06d}.jpg"
            if not cv2.imwrite(str(image_path), image):
                raise VideoReadError(f"Unable to save extracted frame: {image_path}")
            frames.append(
                VideoFrame(
                    frame_id=frame_index,
                    timestamp_sec=round(frame_index / fps, 3),
                    image_path=image_path,
                    width=int(actual_width),
                    height=int(actual_height),
                )
            )
    finally:
        capture.release()

    if not frames:
        raise VideoReadError("Video opened successfully but no frames could be decoded.")
    return replace(metadata, sampled_keyframes=len(frames)), frames


def _load_cv2():
    try:
        import cv2
    except ImportError as exc:
        raise ImportError(
            "Video mode requires OpenCV. Install dependencies with "
            "`pip install -r requirements.txt`."
        ) from exc
    return cv2
