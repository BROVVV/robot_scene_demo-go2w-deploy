"""Lightweight label-aware IoU tracking and track-level voting."""

from __future__ import annotations

from collections import Counter
from difflib import SequenceMatcher
from statistics import mean
from typing import Any

from app.video.spatial_context import bbox_iou


def track_objects(
    frame_results: list[Any],
    iou_threshold: float = 0.35,
    max_missing_frames: int = 8,
    min_hits: int = 2,
    confirm_min_frames: int = 3,
    confirm_score: float = 0.65,
) -> list[dict[str, Any]]:
    active: dict[str, dict[str, Any]] = {}
    tracks: dict[str, dict[str, Any]] = {}
    next_track = 1

    for frame_order, frame in enumerate(
        sorted(frame_results, key=lambda item: item.timestamp_sec)
    ):
        used_tracks: set[str] = set()
        for track_id, state in list(active.items()):
            if frame_order - state["last_order"] > max_missing_frames:
                active.pop(track_id, None)
        for obj in frame.objects:
            best_track_id = None
            best_affinity = 0.0
            for track_id, state in active.items():
                if track_id in used_tracks:
                    continue
                iou = bbox_iou(state["last_bbox"], obj.get("bbox", []))
                label_similarity = _label_similarity(
                    state["label"], _object_label(obj)
                )
                if iou < iou_threshold or label_similarity < 0.45:
                    continue
                affinity = 0.75 * iou + 0.25 * label_similarity
                if affinity > best_affinity:
                    best_affinity = affinity
                    best_track_id = track_id
            if best_track_id is None:
                best_track_id = f"track_{next_track:04d}"
                next_track += 1
                tracks[best_track_id] = {
                    "track_id": best_track_id,
                    "label": obj.get("label"),
                    "label_zh": obj.get("label_zh"),
                    "observations": [],
                    "missing_count": 0,
                }
            active[best_track_id] = {
                "last_bbox": obj.get("bbox", []),
                "label": _object_label(obj),
                "last_order": frame_order,
            }
            used_tracks.add(best_track_id)
            obj["track_id"] = best_track_id
            tracks[best_track_id]["observations"].append(
                {
                    "frame_id": frame.frame_id,
                    "timestamp_sec": frame.timestamp_sec,
                    "object_id": obj.get("object_id"),
                    "bbox": obj.get("bbox"),
                    "confidence": float(obj.get("final_score") or obj.get("confidence") or 0.0),
                    "decision": obj.get("decision", "candidate"),
                    "crop_verify": obj.get("crop_verify"),
                }
            )

    for track in tracks.values():
        observations = track["observations"]
        track["first_seen_sec"] = observations[0]["timestamp_sec"]
        track["last_seen_sec"] = observations[-1]["timestamp_sec"]
        track["observation_count"] = len(observations)
        track["frame_count"] = len({item["frame_id"] for item in observations})
        track["frames"] = [item["frame_id"] for item in observations]
        track["timestamps"] = [item["timestamp_sec"] for item in observations]
        track["bboxes"] = [item["bbox"] for item in observations]
        track["scores"] = [item["confidence"] for item in observations]
        track["final_score"] = aggregate_track_score(track)
        track["decision"] = (
            "confirmed"
            if track["frame_count"] >= confirm_min_frames
            and track["final_score"] >= confirm_score
            else "candidate"
            if track["frame_count"] >= min_hits
            else "rejected"
        )
        best = max(observations, key=lambda item: item["confidence"])
        track["best_frame"] = best["frame_id"]
        track["best_bbox"] = best["bbox"]
        positive_verifications = sum(
            bool((item.get("crop_verify") or {}).get("is_target"))
            for item in observations
        )
        track["evidence"] = [
            f"same object associated across {track['frame_count']} frames",
            f"{positive_verifications} crop verifications supported the target",
        ]
        track["possible_confusions"] = _track_confusions(observations)
    return list(tracks.values())


def aggregate_track_score(track: dict[str, Any]) -> float:
    scores = sorted(
        [float(value) for value in track.get("scores", [])],
        reverse=True,
    )[:5]
    if not scores:
        return 0.0
    base = mean(scores)
    frame_count = int(track.get("frame_count") or len(track.get("observations", [])))
    hit_bonus = min(frame_count / 10.0, 0.15)
    observations = track.get("observations", [])
    attempted = [
        item.get("crop_verify")
        for item in observations
        if item.get("crop_verify")
        and not item["crop_verify"].get("verification_failed")
    ]
    positive_ratio = (
        sum(bool(item.get("is_target")) for item in attempted) / len(attempted)
        if attempted
        else 0.0
    )
    verify_bonus = positive_ratio * 0.15
    return round(min(1.0, base + hit_bonus + verify_bonus), 4)


def track_observation_counts(tracks: list[dict[str, Any]]) -> Counter:
    return Counter(
        {
            str(track["track_id"]): int(track.get("observation_count", 0))
            for track in tracks
        }
    )


def _object_label(obj: dict[str, Any]) -> str:
    return str(obj.get("label") or obj.get("label_zh") or "unknown").lower().replace("_", " ").strip()


def _label_similarity(left: str, right: str) -> float:
    if left == right:
        return 1.0
    if left in right or right in left:
        return 0.85
    return SequenceMatcher(None, left, right).ratio()


def _track_confusions(observations: list[dict[str, Any]]) -> list[str]:
    values: list[str] = []
    for item in observations:
        values.extend((item.get("crop_verify") or {}).get("possible_confusions") or [])
    return list(dict.fromkeys(str(value) for value in values))
