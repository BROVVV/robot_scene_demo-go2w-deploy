"""Rule-based place segmentation for first-person navigation videos."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from typing import Iterable

from app.video.navigation_topology_schema import PlaceSegment
from app.video.schemas import FrameObservation, ObjectTrack


PLACE_MIN_DURATION_SEC = 2.0
PLACE_MAX_DURATION_SEC = 8.0
PLACE_LABEL_JACCARD_SPLIT_THRESHOLD = 0.45

PASSAGE_TERMS = {
    "door",
    "doorway",
    "door frame",
    "room entrance",
    "corridor entrance",
    "hallway",
    "exit",
    "stairs",
    "staircase",
    "elevator",
    "gate",
    "opening",
    "archway",
    "门",
    "门框",
    "门口",
    "房间入口",
    "走廊口",
    "出口",
    "楼梯",
    "电梯",
    "通道",
    "开口",
}


@dataclass
class _OpenSegment:
    place_id: str
    frames: list[FrameObservation]
    split_reason: str

    @property
    def start_time(self) -> float:
        return self.frames[0].timestamp_sec

    @property
    def end_time(self) -> float:
        return self.frames[-1].timestamp_sec

    @property
    def duration(self) -> float:
        return max(0.0, self.end_time - self.start_time)

    @property
    def scene_type(self) -> str:
        return _most_common([frame.scene_type or "unknown" for frame in self.frames])

    @property
    def labels(self) -> set[str]:
        labels: set[str] = set()
        for frame in self.frames:
            labels.update(_frame_labels(frame))
        return labels

    @property
    def passage_labels(self) -> set[str]:
        labels: set[str] = set()
        for frame in self.frames:
            labels.update(_passage_labels(frame))
        return labels

    @property
    def free_space_hint(self) -> str | None:
        hints = [
            item.position_2d
            for frame in self.frames
            for item in frame.free_space
            if item.position_2d and item.position_2d != "unknown"
        ]
        return _most_common(hints) if hints else None


class PlaceSegmenter:
    """Split frame observations into navigation places without metric SLAM."""

    def __init__(
        self,
        min_duration_sec: float = PLACE_MIN_DURATION_SEC,
        max_duration_sec: float = PLACE_MAX_DURATION_SEC,
        label_jaccard_threshold: float = PLACE_LABEL_JACCARD_SPLIT_THRESHOLD,
    ) -> None:
        self.min_duration_sec = min_duration_sec
        self.max_duration_sec = max_duration_sec
        self.label_jaccard_threshold = label_jaccard_threshold

    def segment(
        self,
        observations: list[FrameObservation],
        object_tracks: list[ObjectTrack] | None = None,
    ) -> list[PlaceSegment]:
        frames = sorted(observations, key=lambda item: (item.timestamp_sec, item.frame_id))
        if not frames:
            return []

        segments: list[_OpenSegment] = []
        current = _OpenSegment("place_001", [frames[0]], "initial_segment")
        for frame in frames[1:]:
            should_split, reasons = self._should_split(current, frame)
            if should_split and current.duration >= self.min_duration_sec:
                segments.append(current)
                current = _OpenSegment(
                    f"place_{len(segments) + 1:03d}",
                    [frame],
                    "+".join(reasons) or "segment_change",
                )
            else:
                current.frames.append(frame)
        segments.append(current)

        place_segments = [self._finalize(item) for item in segments]
        if object_tracks:
            assign_tracks_to_places(place_segments, object_tracks)
        return place_segments

    def _should_split(self, current: _OpenSegment, frame: FrameObservation) -> tuple[bool, list[str]]:
        reasons: list[str] = []
        if frame.timestamp_sec - current.start_time >= self.max_duration_sec:
            reasons.append("max_duration")
        frame_labels = _frame_labels(frame)
        if jaccard(frame_labels, current.labels) < self.label_jaccard_threshold:
            reasons.append("label_distribution_change")
        frame_scene_type = frame.scene_type or "unknown"
        if frame_scene_type != current.scene_type and frame_scene_type != "unknown":
            reasons.append("scene_type_change")
        if _passage_labels(frame) - current.passage_labels:
            reasons.append("new_passage")
        frame_free_space = _free_space_hint(frame)
        if frame_free_space and current.free_space_hint and frame_free_space != current.free_space_hint:
            reasons.append("free_space_change")
        motion_hint = str(frame.raw_model_output.get("camera_motion_hint", "")).lower()
        if "turn" in motion_hint or "转向" in motion_hint:
            reasons.append("camera_turn")
        return bool(reasons), reasons

    def _finalize(self, segment: _OpenSegment) -> PlaceSegment:
        labels = sorted(segment.labels)
        confidences = [
            obj.confidence
            for frame in segment.frames
            for obj in frame.objects
            if obj.confidence > 0
        ]
        confidence = sum(confidences) / len(confidences) if confidences else 0.65
        return PlaceSegment(
            place_id=segment.place_id,
            start_time=round(segment.start_time, 3),
            end_time=round(segment.end_time, 3),
            start_frame=segment.frames[0].frame_id,
            end_frame=segment.frames[-1].frame_id,
            scene_type=segment.scene_type,
            dominant_labels=labels[:20],
            passage_candidates=sorted(segment.passage_labels),
            free_space_hint=segment.free_space_hint,
            split_reason=segment.split_reason,
            confidence=round(max(0.0, min(1.0, confidence)), 4),
            evidence_frames=[frame.frame_id for frame in segment.frames],
        )


def assign_tracks_to_places(
    place_segments: list[PlaceSegment],
    object_tracks: list[ObjectTrack],
) -> dict[str, list[str]]:
    """Assign tracks to all overlapping places and return track -> place ids."""

    assignments: dict[str, list[str]] = {}
    for track in object_tracks:
        scored = [
            (place, _track_place_overlap(track, place))
            for place in place_segments
        ]
        positives = [(place, score) for place, score in scored if score > 0]
        if not positives and scored:
            positives = [max(scored, key=lambda item: item[1])]
        positives.sort(key=lambda item: item[1], reverse=True)
        place_ids = [place.place_id for place, _ in positives if place.place_id]
        if not place_ids and place_segments:
            place_ids = [place_segments[0].place_id]
        assignments[track.track_id] = place_ids
        for place_id in place_ids:
            place = next(item for item in place_segments if item.place_id == place_id)
            if track.track_id not in place.object_track_ids:
                place.object_track_ids.append(track.track_id)
            if _is_passage_track(track) and track.track_id not in place.passage_candidates:
                place.passage_candidates.append(track.track_id)
        for place in place_segments:
            place.object_track_ids.sort()
            place.passage_candidates = sorted(set(place.passage_candidates))
    return assignments


def jaccard(a: set[str], b: set[str]) -> float:
    if not a and not b:
        return 1.0
    return len(a & b) / max(1, len(a | b))


def _track_place_overlap(track: ObjectTrack, place: PlaceSegment) -> float:
    overlap = min(track.last_seen_sec, place.end_time) - max(track.first_seen_sec, place.start_time)
    frame_overlap = len(set(track.seen_frame_ids) & set(place.evidence_frames))
    if overlap <= 0 and frame_overlap > 0:
        return float(frame_overlap)
    return max(0.0, overlap) + float(frame_overlap)


def _frame_labels(frame: FrameObservation) -> set[str]:
    labels = {frame.scene_type or "unknown"}
    for obj in frame.objects:
        labels.add(_norm(obj.label))
        labels.add(_norm(obj.label_zh))
        labels.add(_norm(obj.navigation_role))
        labels.add(_norm(obj.category))
    for item in frame.free_space:
        labels.add("free_space")
        labels.add(_norm(item.position_2d))
    for item in frame.hazards:
        labels.add("obstacle")
        labels.add(_norm(item.position_2d))
    return {label for label in labels if label and label != "unknown"}


def _passage_labels(frame: FrameObservation) -> set[str]:
    labels: set[str] = set()
    for obj in frame.objects:
        if obj.navigation_role == "passage" or _contains_term([obj.label, obj.label_zh], PASSAGE_TERMS):
            labels.add(obj.frame_object_id)
    return labels


def _free_space_hint(frame: FrameObservation) -> str | None:
    hints = [item.position_2d for item in frame.free_space if item.position_2d]
    return hints[0] if hints else None


def _is_passage_track(track: ObjectTrack) -> bool:
    return track.navigation_role == "passage" or _contains_term([track.label, track.label_zh], PASSAGE_TERMS)


def _contains_term(values: Iterable[str], terms: set[str]) -> bool:
    text = " ".join(values).lower()
    return any(term in text for term in terms)


def _most_common(values: list[str]) -> str:
    return Counter(values).most_common(1)[0][0] if values else "unknown"


def _norm(value: str | None) -> str:
    return str(value or "").lower().replace("_", " ").strip()
