"""Build deduplicated long-term memories from frame scene reasoning."""

from __future__ import annotations

from datetime import datetime
import hashlib
from typing import Any

from app.video.video_scene_reasoner import FrameSceneReasoningResult


_IMPORTANCE_RANK = {"low": 1, "medium": 2, "high": 3}


class VideoMemoryBuilder:
    def __init__(
        self,
        always_write: bool = True,
        min_importance: str = "low",
        max_entries: int = 200,
    ) -> None:
        self.always_write = always_write
        self.min_importance = min_importance
        self.max_entries = max_entries

    def build(
        self, results: list[FrameSceneReasoningResult]
    ) -> list[dict[str, Any]]:
        memories: list[dict[str, Any]] = []
        for result in results:
            update = result.memory_update
            if not (self.always_write or update.get("should_write")):
                continue
            importance = str(update.get("importance", "low"))
            if _IMPORTANCE_RANK.get(importance, 1) < _IMPORTANCE_RANK.get(
                self.min_importance, 1
            ):
                continue
            memory = self._from_result(result)
            if memories and _same_place_signature(memories[-1], memory):
                _merge_memory(memories[-1], memory)
            else:
                memories.append(memory)
            if len(memories) >= self.max_entries:
                break
        return memories

    def _from_result(
        self, result: FrameSceneReasoningResult
    ) -> dict[str, Any]:
        scene = result.scene_understanding
        target = result.target_evidence
        timestamp = datetime.now().astimezone().replace(microsecond=0).isoformat()
        seed = (
            f"{result.video_id}:{result.frame_id}:{result.timestamp_sec}:"
            f"{scene.get('room_type')}:{target.get('target')}"
        )
        memory_id = "mem_" + hashlib.sha1(seed.encode("utf-8")).hexdigest()[:16]
        landmark_names = [str(item.get("name")) for item in result.landmarks]
        return {
            "memory_id": memory_id,
            "source": "video",
            "video_id": result.video_id,
            "frame_id": result.frame_id,
            "frame_ids": [result.frame_id],
            "timestamp_sec": result.timestamp_sec,
            "time_range": [result.timestamp_sec, result.timestamp_sec],
            "image_path": result.image_path,
            "memory_kind": result.memory_update.get(
                "memory_kind", "environment_observation"
            ),
            "room_type": scene.get("room_type", "unknown"),
            "scene_summary": result.memory_update.get("summary")
            or scene.get("scene_summary", ""),
            "summary": result.memory_update.get("summary")
            or scene.get("scene_summary", ""),
            "place_signature": {
                "stable_landmarks": landmark_names,
                "layout": scene.get("scene_summary", ""),
                "egocentric_direction": "front",
            },
            "objects": [
                {
                    "name": item.get("label_zh") or item.get("label"),
                    "category": item.get("category"),
                    "position": item.get("image_position"),
                    "confidence": item.get("confidence"),
                }
                for item in result.objects
            ],
            "regions": result.regions,
            "target_context": {
                "target": target.get("target"),
                "found": bool(target.get("directly_found")),
                "candidate_found": bool(target.get("candidate_found")),
                "negative_evidence": list(result.negative_evidence),
                "negative_evidence_count": len(result.negative_evidence),
                "indirect_evidence": list(target.get("indirect_evidence", [])),
            },
            "psg_hypotheses": [dict(item) for item in result.psg_hypotheses],
            "importance": result.memory_update.get("importance", "low"),
            "tags": result.memory_update.get("tags", []),
            "created_at": timestamp,
        }


def _same_place_signature(
    left: dict[str, Any], right: dict[str, Any]
) -> bool:
    if left.get("room_type") != right.get("room_type"):
        return False
    if left.get("target_context", {}).get("found") != right.get(
        "target_context", {}
    ).get("found"):
        return False
    left_landmarks = set(left.get("place_signature", {}).get("stable_landmarks", []))
    right_landmarks = set(right.get("place_signature", {}).get("stable_landmarks", []))
    if not left_landmarks and not right_landmarks:
        return True
    union = left_landmarks | right_landmarks
    return bool(union) and len(left_landmarks & right_landmarks) / len(union) >= 0.6


def _merge_memory(target: dict[str, Any], incoming: dict[str, Any]) -> None:
    target["frame_ids"].extend(incoming["frame_ids"])
    target["time_range"][1] = incoming["time_range"][1]
    target["target_context"]["negative_evidence"].extend(
        incoming["target_context"].get("negative_evidence", [])
    )
    target["target_context"]["negative_evidence"] = list(
        dict.fromkeys(target["target_context"]["negative_evidence"])
    )
    target["target_context"]["negative_evidence_count"] = len(
        target["target_context"]["negative_evidence"]
    )
    target["psg_hypotheses"].extend(incoming.get("psg_hypotheses", []))
    target["scene_summary"] = (
        f"{target['scene_summary']} 时间范围扩展至 "
        f"{target['time_range'][0]:.2f}s-{target['time_range'][1]:.2f}s。"
    )
    target["summary"] = target["scene_summary"]
