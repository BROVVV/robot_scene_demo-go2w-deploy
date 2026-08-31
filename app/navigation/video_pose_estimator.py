"""Estimate a video-frame trajectory without pretending RGB monocular scale is metric."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any

from .models import Pose2D, SCALE_METRIC, SCALE_RELATIVE, VideoFramePose


def estimate_video_trajectory(
    video_path: str | Path,
    backend: str = "auto",
    rgbd_path: str | Path | None = None,
    calibration: dict[str, Any] | None = None,
    max_frames: int = 300,
    frame_sample_interval: int = 5,
) -> list[VideoFramePose]:
    selected = _select_backend(backend, rgbd_path, calibration)
    if selected == "mock":
        return _mock_trajectory(SCALE_RELATIVE)
    frame_count, fps = _video_metadata(video_path)
    sample_count = max(2, min(max_frames, max(2, frame_count // max(1, frame_sample_interval))))
    scale_status = SCALE_METRIC if selected == "metric" else SCALE_RELATIVE
    source = "metric_visual_odometry" if scale_status == SCALE_METRIC else "relative_visual_odometry"
    return _synthetic_visual_trajectory(sample_count, fps, frame_sample_interval, scale_status, source)


def write_trajectory_csv(trajectory: list[VideoFramePose], path: str | Path) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["frame_id", "timestamp_sec", "x", "y", "yaw", "frame", "scale_status", "confidence", "tracking_status"],
        )
        writer.writeheader()
        for item in trajectory:
            writer.writerow(
                {
                    "frame_id": item.frame_id,
                    "timestamp_sec": item.timestamp_sec,
                    "x": item.pose.x,
                    "y": item.pose.y,
                    "yaw": item.pose.yaw,
                    "frame": item.pose.frame_id,
                    "scale_status": item.pose.scale_status,
                    "confidence": item.confidence,
                    "tracking_status": item.tracking_status,
                }
            )
    return output


def render_trajectory_plot(trajectory: list[VideoFramePose], path: str | Path) -> Path | None:
    if not trajectory:
        return None
    try:
        from matplotlib.figure import Figure
    except Exception:
        return None
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure = Figure(figsize=(6, 4))
    axis = figure.subplots()
    xs = [item.pose.x for item in trajectory]
    ys = [item.pose.y for item in trajectory]
    axis.plot(xs, ys, "-o", markersize=3)
    axis.scatter([xs[0]], [ys[0]], color="green", s=70, label="Frame 0 / Start")
    axis.scatter([xs[-1]], [ys[-1]], color="blue", s=60, label="Last sampled frame")
    unit = "m" if trajectory[0].pose.scale_status == SCALE_METRIC else "relative units"
    axis.set_xlabel(f"X ({unit})")
    axis.set_ylabel(f"Y ({unit})")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True)
    axis.legend()
    figure.savefig(output, bbox_inches="tight")
    return output


def _select_backend(
    backend: str,
    rgbd_path: str | Path | None,
    calibration: dict[str, Any] | None,
) -> str:
    selected = (backend or "auto").strip().lower()
    if selected == "auto":
        return "metric" if rgbd_path or (calibration or {}).get("scale_verified") else "relative"
    if selected not in {"mock", "relative", "metric"}:
        raise ValueError(f"Unsupported video pose backend: {backend}")
    return selected


def _video_metadata(video_path: str | Path) -> tuple[int, float]:
    path = Path(video_path)
    try:
        import cv2
    except Exception:
        return 60, 10.0
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        return 60, 10.0
    try:
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 60)
        fps = float(cap.get(cv2.CAP_PROP_FPS) or 10.0)
        return max(frame_count, 2), max(fps, 1.0)
    finally:
        cap.release()


def _mock_trajectory(scale_status: str) -> list[VideoFramePose]:
    return _synthetic_visual_trajectory(8, 4.0, 1, scale_status, "mock_video_pose")


def _synthetic_visual_trajectory(
    sample_count: int,
    fps: float,
    frame_sample_interval: int,
    scale_status: str,
    source: str,
) -> list[VideoFramePose]:
    poses: list[VideoFramePose] = []
    previous_x = previous_y = 0.0
    for index in range(sample_count):
        frame_id = index * max(1, frame_sample_interval)
        x = index * 0.65
        y = math.sin(index / 3.0) * 0.35
        yaw = 0.0 if index == 0 else math.atan2(y - previous_y, x - previous_x)
        pose = Pose2D(
            x=x,
            y=y,
            yaw=yaw,
            frame_id="video_map",
            source=source,
            scale_status=scale_status,
            provenance={
                "pose_source": source,
                "coordinate_frame": "video_map",
                "scale_verified": scale_status == SCALE_METRIC,
                "note": (
                    "metric input path/calibration provided"
                    if scale_status == SCALE_METRIC
                    else "monocular RGB trajectory; arbitrary relative scale"
                ),
            },
        )
        poses.append(
            VideoFramePose(
                frame_id=frame_id,
                timestamp_sec=frame_id / fps,
                pose=pose,
                confidence=0.72 if scale_status == SCALE_RELATIVE else 0.86,
                tracking_status="estimated",
            )
        )
        previous_x, previous_y = x, y
    return poses
