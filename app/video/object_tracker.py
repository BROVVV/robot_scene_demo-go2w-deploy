"""Object tracking helpers for target search and full-scene mapping."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from difflib import SequenceMatcher
from statistics import mean
from typing import Any

from app.video.tracking import (
    aggregate_track_score,
    track_objects,
    track_observation_counts,
)
from app.video.schemas import FrameObject, FrameObservation, ObjectTrack


TRACK_MATCH_THRESHOLD = 0.55


@dataclass
class _TrackState:
    track_id: str
    observations: list[tuple[FrameObservation, FrameObject]] = field(default_factory=list)

    @property
    def last_frame(self) -> FrameObservation:
        return self.observations[-1][0]

    @property
    def last_object(self) -> FrameObject:
        return self.observations[-1][1]


class VideoObjectTracker:
    """Simple label/position/bbox tracker for full-scene observations."""

    def __init__(
        self,
        match_threshold: float = TRACK_MATCH_THRESHOLD,
        max_time_gap_sec: float = 3.0,
    ) -> None:
        self.match_threshold = match_threshold
        self.max_time_gap_sec = max_time_gap_sec

    def build_tracks(self, observations: list[FrameObservation]) -> list[ObjectTrack]:
        states: list[_TrackState] = []
        counters: Counter[str] = Counter()
        for frame in sorted(observations, key=lambda item: item.timestamp_sec):
            for obj in frame.objects:
                best_state = None
                best_score = 0.0
                for state in states:
                    score = self._match_score(state, frame, obj)
                    if score > best_score:
                        best_score = score
                        best_state = state
                if best_state is None or best_score < self.match_threshold:
                    slug = _slug(obj.label)
                    counters[slug] += 1
                    best_state = _TrackState(f"obj_{slug}_{counters[slug]:03d}")
                    states.append(best_state)
                best_state.observations.append((frame, obj))
        return [self._finalize(state) for state in states]

    def _match_score(
        self,
        state: _TrackState,
        frame: FrameObservation,
        obj: FrameObject,
    ) -> float:
        last = state.last_object
        time_gap = max(0.0, frame.timestamp_sec - state.last_frame.timestamp_sec)
        if time_gap > self.max_time_gap_sec:
            return 0.0
        label_similarity = _label_similarity(last.label, obj.label)
        # Spatial overlap alone must never merge different nearby objects. This
        # is especially important for semantic relations such as bin-near-water-
        # dispenser, where both boxes may overlap heavily in image space.
        if label_similarity < 0.55:
            return 0.0
        score = 0.0
        if label_similarity >= 0.82:
            score += 0.45
        iou = _bbox_iou(last.bbox, obj.bbox)
        if iou > 0.3:
            score += 0.25
        if last.position_2d == obj.position_2d:
            score += 0.15
        if time_gap <= self.max_time_gap_sec:
            score += 0.10
        if set(last.attributes).intersection(obj.attributes):
            score += 0.05
        if obj.bbox is None and last.bbox is None and score >= 0.55:
            return score
        return score

    def _finalize(self, state: _TrackState) -> ObjectTrack:
        frames = [frame for frame, _ in state.observations]
        objects = [obj for _, obj in state.observations]
        best_index = max(range(len(objects)), key=lambda index: objects[index].confidence)
        label = _most_common([obj.label for obj in objects])
        label_zh = _most_common([obj.label_zh for obj in objects])
        category = _most_common([obj.category for obj in objects])
        role = _most_common([obj.navigation_role for obj in objects])
        attributes = sorted({attr for obj in objects for attr in obj.attributes})
        return ObjectTrack(
            track_id=state.track_id,
            label=label,
            label_zh=label_zh,
            category=category,
            navigation_role=role,
            first_seen_sec=frames[0].timestamp_sec,
            last_seen_sec=frames[-1].timestamp_sec,
            seen_frame_ids=[frame.frame_id for frame in frames],
            best_frame_id=frames[best_index].frame_id,
            best_frame_path=frames[best_index].frame_path,
            stable_position_2d=_most_common([obj.position_2d for obj in objects]),
            representative_bbox=objects[best_index].bbox,
            confidence=round(mean([obj.confidence for obj in objects]), 4),
            attributes=attributes,
            source="observed",
        )


def _label_similarity(left: str, right: str) -> float:
    left = left.lower().replace("_", " ").strip()
    right = right.lower().replace("_", " ").strip()
    if left == right:
        return 1.0
    if left in right or right in left:
        return 0.9
    return SequenceMatcher(None, left, right).ratio()


def _bbox_iou(left: list[float] | None, right: list[float] | None) -> float:
    if not left or not right or len(left) != 4 or len(right) != 4:
        return 0.0
    x_left = max(left[0], right[0])
    y_top = max(left[1], right[1])
    x_right = min(left[2], right[2])
    y_bottom = min(left[3], right[3])
    if x_right <= x_left or y_bottom <= y_top:
        return 0.0
    intersection = (x_right - x_left) * (y_bottom - y_top)
    left_area = max(0.0, left[2] - left[0]) * max(0.0, left[3] - left[1])
    right_area = max(0.0, right[2] - right[0]) * max(0.0, right[3] - right[1])
    union = left_area + right_area - intersection
    return intersection / union if union > 0 else 0.0


def _most_common(values: list[str]) -> str:
    return Counter(values).most_common(1)[0][0] if values else "unknown"


def _slug(value: Any) -> str:
    text = "".join(character.lower() if character.isalnum() else "_" for character in str(value))
    return "_".join(part for part in text.split("_") if part) or "object"


__all__ = [
    "VideoObjectTracker",
    "aggregate_track_score",
    "track_objects",
    "track_observation_counts",
]
