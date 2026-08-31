"""Shared data structures for video target search."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class VideoMetadata:
    video_path: str
    fps: float
    duration_sec: float
    frame_count: int
    width: int
    height: int
    sampled_keyframes: int = 0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class VideoFrame:
    frame_id: int
    timestamp_sec: float
    image_path: Path
    width: int
    height: int

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["image_path"] = str(self.image_path)
        return payload


@dataclass
class FrameAnalysisResult:
    frame_id: int
    timestamp_sec: float
    image_path: str
    annotated_frame_path: str | None
    scene_summary: str
    objects: list[dict[str, Any]]
    relations: list[dict[str, Any]]
    raw_result_path: str | None = None
    error: str | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)
