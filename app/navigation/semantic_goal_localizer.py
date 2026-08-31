"""Convert visual target evidence into video-map goal candidates."""

from __future__ import annotations

import math
from typing import Any

from .models import Pose2D, VideoFramePose
from .priority import priority_to_confidence


TARGET_VISUAL_CONFIRMED = "target_visual_confirmed"
TARGET_CANDIDATE = "target_candidate"
TARGET_NOT_SEEN = "target_not_seen"
TARGET_LOST_AFTER_SEEN = "target_lost_after_seen"
TARGET_UNCONFIRMED_BUT_LIKELY_AREA_FOUND = "target_unconfirmed_but_likely_area_found"


def localize_semantic_goal(
    target_search_result: dict[str, Any],
    trajectory: list[VideoFramePose],
    navigation_map: dict[str, Any] | None = None,
) -> dict[str, Any]:
    del navigation_map
    status = _target_status(target_search_result)
    evidence = target_search_result.get("best_evidence") or {}
    if status in {TARGET_VISUAL_CONFIRMED, TARGET_LOST_AFTER_SEEN} and evidence:
        return _localize_evidence(target_search_result, trajectory, evidence, status, "target")
    if status in {TARGET_CANDIDATE, TARGET_UNCONFIRMED_BUT_LIKELY_AREA_FOUND}:
        candidate = _candidate_evidence(target_search_result)
        if candidate:
            return _localize_evidence(target_search_result, trajectory, candidate, status, "candidate")
    return {
        "target": target_search_result.get("target") or target_search_result.get("task", {}).get("target"),
        "target_status": status,
        "goal_type": "exploration",
        "target_pose": None,
        "source_frame_pose": trajectory[0].pose.to_dict() if trajectory else None,
        "confidence": 0.2,
        "provenance": {
            "source": "video_target_search",
            "reason": "no spatially localizable target evidence",
        },
    }


def _localize_evidence(
    target_search_result: dict[str, Any],
    trajectory: list[VideoFramePose],
    evidence: dict[str, Any],
    status: str,
    goal_type: str,
) -> dict[str, Any]:
    source_pose = _nearest_frame_pose(trajectory, evidence.get("frame_id"))
    if source_pose is None:
        source_pose = trajectory[0] if trajectory else None
    if source_pose is None:
        return {
            "target": target_search_result.get("target"),
            "target_status": status,
            "goal_type": goal_type,
            "target_pose": None,
            "source_frame_pose": None,
            "confidence": 0.0,
            "provenance": {"source": "video_target_search", "reason": "empty trajectory"},
        }
    lateral = _bbox_lateral_offset(evidence.get("bbox"))
    forward = 1.0 if source_pose.pose.scale_status == "metric" else 0.9
    target_pose = Pose2D(
        x=source_pose.pose.x + math.cos(source_pose.pose.yaw) * forward - math.sin(source_pose.pose.yaw) * lateral,
        y=source_pose.pose.y + math.sin(source_pose.pose.yaw) * forward + math.cos(source_pose.pose.yaw) * lateral,
        yaw=source_pose.pose.yaw,
        frame_id=source_pose.pose.frame_id,
        source="semantic_goal_localizer",
        scale_status=source_pose.pose.scale_status,
        provenance={
            "source": "video_target_search",
            "frame_id": evidence.get("frame_id"),
            "bbox": evidence.get("bbox"),
            "pose_source": source_pose.pose.source,
            "coordinate_frame": source_pose.pose.frame_id,
            "scale_verified": source_pose.pose.scale_status == "metric",
        },
    )
    return {
        "target": target_search_result.get("target") or target_search_result.get("task", {}).get("target"),
        "target_status": status,
        "goal_type": goal_type,
        "target_pose": target_pose.to_dict(),
        "source_frame_pose": source_pose.pose.to_dict(),
        "source_frame_id": source_pose.frame_id,
        "timestamp_sec": evidence.get("timestamp_sec"),
        "confidence": float(evidence.get("evidence_score", evidence.get("confidence", 0.55)) or 0.55),
        "provenance": target_pose.provenance,
    }


def _target_status(result: dict[str, Any]) -> str:
    status = result.get("target_status")
    if status:
        return str(status)
    if result.get("target_found") or result.get("target_confirmed"):
        return TARGET_VISUAL_CONFIRMED
    if result.get("candidate_regions"):
        return TARGET_CANDIDATE
    return TARGET_NOT_SEEN


def _candidate_evidence(result: dict[str, Any]) -> dict[str, Any] | None:
    if result.get("best_evidence"):
        return result["best_evidence"]
    regions = result.get("candidate_regions") or []
    if not regions:
        return None
    region = regions[0]
    return {
        "frame_id": region.get("frame_id"),
        "timestamp_sec": region.get("timestamp_sec"),
        "confidence": priority_to_confidence(region.get("priority"), 0.45),
        "bbox": region.get("bbox"),
    }


def _nearest_frame_pose(trajectory: list[VideoFramePose], frame_id: Any) -> VideoFramePose | None:
    if not trajectory:
        return None
    if frame_id is None:
        return trajectory[-1]
    target = int(frame_id)
    return min(trajectory, key=lambda item: abs(item.frame_id - target))


def _bbox_lateral_offset(bbox: Any) -> float:
    if not bbox or len(bbox) < 4:
        return 0.0
    x1, _, x2, _ = [float(v) for v in bbox[:4]]
    center = (x1 + x2) / 2.0
    if center > 1.0:
        center /= 1000.0
    return max(-0.45, min(0.45, (center - 0.5) * 0.9))
