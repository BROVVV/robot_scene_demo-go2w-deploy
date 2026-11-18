"""Playback clock: source time → video time, key segments, chapters.

原始记录是真实时间序列；倍速只改变"展示映射"，绝不改写原始记录。关键片段
（首次移动、目标确认、人工停止等）默认 1× 播放，其余默认 10×；重叠的片段
合并，避免同一秒被重复放慢。结尾停留单独加在时间轴末端。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, Sequence


@dataclass(frozen=True)
class SpeedSegment:
    src_start: float
    src_end: float
    speed: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "source_start": round(self.src_start, 3),
            "source_end": round(self.src_end, 3),
            "speed": self.speed,
        }


@dataclass(frozen=True)
class KeyMoment:
    t: float
    label: str


KIND_LABELS = {
    "first_motion": "首次移动",
    "first_turn": "首次转向",
    "target_suspected": "疑似目标",
    "target_confirmed": "确认目标",
    "operator_stop": "人工停止",
    "search_finished": "搜索结束",
    "error": "异常",
    "map_origin": "地图起点",
}


def build_speed_plan(
    first_t: float,
    last_t: float,
    key_moments: Sequence[KeyMoment],
    *,
    default_speed: float,
    key_speed: float = 1.0,
    pad_seconds: float = 6.0,
) -> list[SpeedSegment]:
    """Piecewise-constant speed over the whole recorded interval.

    ``key_moments`` are merged when their padded windows overlap.  The result
    always covers ``[first_t, last_t]`` exactly once and in order.
    """
    start = float(first_t)
    end = float(last_t)
    if end <= start:
        return [SpeedSegment(start, start + 0.05, max(1.0, default_speed))]
    pad = max(0.0, float(pad_seconds))
    windows: list[tuple[float, float]] = []
    for moment in sorted(key_moments, key=lambda item: item.t):
        t = float(moment.t)
        if t < start or t > end:
            continue
        windows.append((max(start, t - pad), min(end, t + pad)))
    merged: list[tuple[float, float]] = []
    for window_start, window_end in windows:
        if merged and window_start <= merged[-1][1] + 1e-6:
            merged[-1] = (merged[-1][0], max(merged[-1][1], window_end))
        else:
            merged.append((window_start, window_end))

    segments: list[SpeedSegment] = []
    cursor = start
    for window_start, window_end in merged:
        if window_start > cursor + 1e-6:
            segments.append(SpeedSegment(cursor, window_start, default_speed))
        segments.append(SpeedSegment(window_start, window_end, key_speed))
        cursor = max(cursor, window_end)
    if cursor < end - 1e-6:
        segments.append(SpeedSegment(cursor, end, default_speed))
    if not segments:
        segments = [SpeedSegment(start, end, default_speed)]
    return segments


class PlaybackTimemap:
    """Map between source seconds and video seconds.

    Video time is 0 at ``source_start``; the closing hold has no source time
    (``video_to_source`` clamps to ``source_end`` and ``is_end_hold`` is true).
    """

    def __init__(
        self,
        segments: Sequence[SpeedSegment],
        *,
        end_hold_seconds: float = 3.0,
        chapters: Sequence[dict[str, Any]] | None = None,
    ) -> None:
        self.segments = list(segments)
        self.end_hold_seconds = max(0.0, float(end_hold_seconds))
        self.chapters = list(chapters or [])
        self._video: list[tuple[float, float, float, float, float]] = []
        cursor = 0.0
        for segment in self.segments:
            length = max(0.0, segment.src_end - segment.src_start)
            speed = max(1e-6, float(segment.speed))
            duration = length / speed
            self._video.append(
                (cursor, cursor + duration, segment.src_start,
                 segment.src_end, speed)
            )
            cursor += duration
        self.content_duration = cursor
        self.duration = cursor + self.end_hold_seconds
        self.source_start = self.segments[0].src_start if self.segments else 0.0
        self.source_end = self.segments[-1].src_end if self.segments else 0.0

    # -- mapping ------------------------------------------------------- #
    def video_to_source(self, video_t: float) -> float:
        value = max(0.0, min(float(video_t), self.content_duration))
        for v0, v1, s0, s1, speed in self._video:
            if value <= v1 + 1e-9:
                return min(s1, s0 + (value - v0) * speed)
        return self.source_end

    def source_to_video(self, source_t: float) -> float:
        value = max(self.source_start, min(float(source_t), self.source_end))
        for v0, _v1, s0, s1, speed in self._video:
            if value <= s1 + 1e-9:
                return v0 + max(0.0, value - s0) / speed
        return self.content_duration

    def is_end_hold(self, video_t: float) -> bool:
        return float(video_t) >= self.content_duration - 1e-9

    def speed_at_source(self, source_t: float) -> float:
        value = max(self.source_start, min(float(source_t), self.source_end))
        for _v0, _v1, s0, s1, speed in self._video:
            if value <= s1 + 1e-9:
                return speed
        return self.segments[-1].speed if self.segments else 1.0

    def key_windows(self) -> list[dict[str, Any]]:
        windows: list[dict[str, Any]] = []
        for _v0, v1, s0, s1, speed in self._video:
            if speed <= 1.0 + 1e-9 and s1 > s0:
                windows.append({
                    "source_start": round(s0, 3),
                    "source_end": round(s1, 3),
                    "video_start": round(self.source_to_video(s0), 3),
                    "video_end": round(self.source_to_video(s1), 3),
                })
        return windows

    def to_dict(self) -> dict[str, Any]:
        return {
            "segments": [segment.to_dict() for segment in self.segments],
            "source_start": round(self.source_start, 3),
            "source_end": round(self.source_end, 3),
            "content_duration": round(self.content_duration, 3),
            "end_hold_seconds": self.end_hold_seconds,
            "duration": round(self.duration, 3),
            "key_windows": self.key_windows(),
            "chapters": self.chapters,
        }


def split_chapters(
    timemap: PlaybackTimemap,
    *,
    chapter_seconds: float = 1800.0,
    title_prefix: str = "第",
) -> list[dict[str, Any]]:
    """Split very long sessions at speed-plan boundaries for indexing.

    Chapters never truncate the recording; they only index it.  Each chapter
    lists its own source/video range and speed segments.
    """
    if chapter_seconds <= 0 or timemap.duration <= chapter_seconds:
        return []
    chapters: list[dict[str, Any]] = []
    chapter_index = 1
    chapter_source_start = timemap.source_start
    chapter_video_start = 0.0
    last_source = chapter_source_start
    speeds: list[dict[str, Any]] = []
    for segment in timemap.segments:
        segment_video_end = timemap.source_to_video(segment.src_end)
        if speeds and (segment_video_end - chapter_video_start) > chapter_seconds:
            chapters.append({
                "index": chapter_index,
                "title": f"{title_prefix}{chapter_index}章",
                "source_start": round(chapter_source_start, 3),
                "source_end": round(last_source, 3),
                "video_start": round(chapter_video_start, 3),
                "video_end": round(segment_video_end, 3),
                "speeds": speeds,
            })
            chapter_index += 1
            chapter_video_start = segment_video_end
            chapter_source_start = segment.src_start
            speeds = []
        speeds.append(segment.to_dict())
        last_source = segment.src_end
    chapters.append({
        "index": chapter_index,
        "title": f"{title_prefix}{chapter_index}章",
        "source_start": round(chapter_source_start, 3),
        "source_end": round(last_source, 3),
        "video_start": round(chapter_video_start, 3),
        "video_end": round(timemap.duration, 3),
        "speeds": speeds,
    })
    return chapters


def key_moments_from_events(
    events: Iterable[dict[str, Any]],
    *,
    first_motion_t: float | None = None,
) -> list[KeyMoment]:
    """Extract the moments that deserve normal-speed playback.

    Events are the replay-side rows (``t`` monotonic + ``event_type``); a
    suspected target is deliberately **not** mapped to the confirmation label
    and vice versa.
    """
    moments: list[KeyMoment] = []
    seen: set[str] = set()
    if first_motion_t is not None and first_motion_t > 0:
        moments.append(KeyMoment(first_motion_t, "first_motion"))
        seen.add("first_motion")
    for event in events:
        event_type = str(event.get("event_type") or "")
        t = event.get("t")
        if not isinstance(t, (int, float)):
            continue
        t = float(t)
        label: str | None = None
        dedupe_key: str | None = None
        if event_type == "TARGET_CANDIDATE":
            label, dedupe_key = "target_suspected", "target_suspected"
        elif event_type == "TARGET_CONFIRMED":
            label, dedupe_key = "target_confirmed", "target_confirmed"
        elif event_type in {"OPERATOR_STOP"}:
            label, dedupe_key = "operator_stop", "operator_stop"
        elif event_type in {"SEARCH_FINISHED"}:
            label, dedupe_key = "search_finished", "search_finished"
        elif event_type == "ERROR":
            label, dedupe_key = "error", "error"
        if label is None or (dedupe_key and dedupe_key in seen):
            # 只保留每类第一个事件作为倍速锚点；后面的同类事件仍会出现在
            # 事件列表里，但不会把整段末尾都拖成慢放。
            continue
        if dedupe_key:
            seen.add(dedupe_key)
        moments.append(KeyMoment(t, label))
    return moments


def label_for(kind: str) -> str:
    return KIND_LABELS.get(kind, kind)
