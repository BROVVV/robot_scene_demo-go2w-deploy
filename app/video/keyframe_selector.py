"""Deterministic time-based keyframe selection."""

from __future__ import annotations


def select_frame_indices(
    frame_count: int,
    source_fps: float,
    sample_fps: float = 1.0,
    max_frames: int = 120,
) -> list[int]:
    """Return monotonically increasing frame indices sampled by timestamp."""
    if frame_count <= 0:
        return []
    if source_fps <= 0:
        raise ValueError("source_fps must be greater than zero.")
    if sample_fps <= 0:
        raise ValueError("sample_fps must be greater than zero.")
    if max_frames <= 0:
        raise ValueError("max_frames must be greater than zero.")

    interval = source_fps / sample_fps
    indices: list[int] = []
    sample_number = 0
    while len(indices) < max_frames:
        frame_index = int(round(sample_number * interval))
        if frame_index >= frame_count:
            break
        if not indices or frame_index > indices[-1]:
            indices.append(frame_index)
        sample_number += 1
    return indices
