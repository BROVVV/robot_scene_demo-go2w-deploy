"""Convert visual target evidence into video-map goal candidates.

计划书 §19：目标 3D 定位必须按 spatial quality 分级，不能再用固定 1m 假装
知道目标真实距离。

    METRIC_RGBD              -> map-frame target pose，可生成 approach viewpoints
    RELATIVE_RGBD/CAMERA_LOCAL -> 只保留 depth/bearing，标 local_only，不伪造 global pose
    RGB_ONLY/SEMANTIC_2D_ONLY  -> 只能 reobserve/candidate

固定 1m 启发式降级为 legacy fallback：默认关闭，只有离线视频链路
（``navigation_planning_pipeline``，那里根本没有深度）显式打开，并且带
``heuristic_only=True``，永远不允许用于 ``TARGET_REACHED`` 判定。
"""

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

APPROACH_MAP_VIEWPOINTS = "MAP_VIEWPOINTS"
APPROACH_LOCAL_VISUAL_SERVO = "LOCAL_VISUAL_SERVO"
APPROACH_REOBSERVE_ONLY = "REOBSERVE_ONLY"

_LOCAL_QUALITY = ("RELATIVE_RGBD", "CAMERA_LOCAL")


def localize_semantic_goal(
    target_search_result: dict[str, Any],
    trajectory: list[VideoFramePose],
    navigation_map: dict[str, Any] | None = None,
    *,
    allow_fixed_distance_fallback: bool = False,
) -> dict[str, Any]:
    del navigation_map
    status = _target_status(target_search_result)
    evidence = target_search_result.get("best_evidence") or {}
    if status in {TARGET_VISUAL_CONFIRMED, TARGET_LOST_AFTER_SEEN} and evidence:
        return _localize_evidence(
            target_search_result, trajectory, evidence, status, "target",
            allow_fixed_distance_fallback,
        )
    if status in {TARGET_CANDIDATE, TARGET_UNCONFIRMED_BUT_LIKELY_AREA_FOUND}:
        candidate = _candidate_evidence(target_search_result)
        if candidate:
            return _localize_evidence(
                target_search_result, trajectory, candidate, status, "candidate",
                allow_fixed_distance_fallback,
            )
    return {
        "target": target_search_result.get("target") or target_search_result.get("task", {}).get("target"),
        "target_status": status,
        "goal_type": "exploration",
        "target_pose": None,
        "source_frame_pose": trajectory[0].pose.to_dict() if trajectory else None,
        "confidence": 0.2,
        "spatial_quality": "RGB_ONLY",
        "local_only": False,
        "target_depth_m": None,
        "target_bearing_deg": None,
        "approach_mode": APPROACH_REOBSERVE_ONLY,
        "approach_allowed": False,
        "heuristic_only": False,
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
    allow_fixed_distance_fallback: bool,
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
            "spatial_quality": _spatial_quality(evidence),
            "local_only": False,
            "target_depth_m": None,
            "target_bearing_deg": None,
            "approach_mode": APPROACH_REOBSERVE_ONLY,
            "approach_allowed": False,
            "heuristic_only": False,
            "provenance": {"source": "video_target_search", "reason": "empty trajectory"},
        }

    quality = _spatial_quality(evidence)
    depth_m = _float_or_none(evidence.get("depth_m"))
    bearing_deg = _float_or_none(evidence.get("bearing_deg"))
    map_xyz = evidence.get("map_xyz")
    base = {
        "target": target_search_result.get("target") or target_search_result.get("task", {}).get("target"),
        "target_status": status,
        "goal_type": goal_type,
        "source_frame_pose": source_pose.pose.to_dict(),
        "source_frame_id": source_pose.frame_id,
        "timestamp_sec": evidence.get("timestamp_sec"),
        "confidence": float(evidence.get("evidence_score", evidence.get("confidence", 0.55)) or 0.55),
        "spatial_quality": quality,
        "target_depth_m": depth_m,
        "target_bearing_deg": bearing_deg,
    }
    provenance = {
        "source": "video_target_search",
        "frame_id": evidence.get("frame_id"),
        "bbox": evidence.get("bbox"),
        "pose_source": source_pose.pose.source,
        "coordinate_frame": source_pose.pose.frame_id,
        "spatial_quality": quality,
    }

    if quality == "METRIC_RGBD" and map_xyz is not None and len(map_xyz) >= 2:
        tx, ty = float(map_xyz[0]), float(map_xyz[1])
        yaw = math.atan2(ty - source_pose.pose.y, tx - source_pose.pose.x)
        target_pose = Pose2D(
            x=tx, y=ty, yaw=yaw,
            frame_id=source_pose.pose.frame_id,
            source="semantic_goal_localizer",
            scale_status="metric",
            provenance={**provenance, "map_xyz": [tx, ty], "scale_verified": True,
                        "heuristic_only": False},
        )
        return {
            **base,
            "target_pose": target_pose.to_dict(),
            "local_only": False,
            "approach_mode": APPROACH_MAP_VIEWPOINTS,
            "approach_allowed": True,
            "heuristic_only": False,
            "provenance": target_pose.provenance,
        }

    if quality in _LOCAL_QUALITY and depth_m is not None:
        # 有真实深度但没有可信 map 位姿：只做局部 stop-and-observe 接近（§20.2）。
        return {
            **base,
            "target_pose": None,
            "local_only": True,
            "approach_mode": APPROACH_LOCAL_VISUAL_SERVO,
            "approach_allowed": True,
            "heuristic_only": False,
            "provenance": {**provenance, "scale_verified": False,
                           "reason": "local depth/bearing only; no global target pose",
                           "heuristic_only": False},
        }

    if not allow_fixed_distance_fallback:
        return {
            **base,
            "target_pose": None,
            "local_only": False,
            "approach_mode": APPROACH_REOBSERVE_ONLY,
            "approach_allowed": False,
            "heuristic_only": False,
            "provenance": {**provenance, "scale_verified": False,
                           "reason": "RGB-only evidence cannot claim a target distance",
                           "heuristic_only": False},
        }

    # legacy：离线视频链路没有任何深度，只能沿视线方向猜一个观察点。
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
            **provenance,
            "scale_verified": source_pose.pose.scale_status == "metric",
            "heuristic_only": True,
            "reason": "fixed-distance heuristic; never valid for TARGET_REACHED",
        },
    )
    return {
        **base,
        "target_pose": target_pose.to_dict(),
        "local_only": False,
        "approach_mode": APPROACH_REOBSERVE_ONLY,
        "approach_allowed": False,
        "heuristic_only": True,
        "provenance": target_pose.provenance,
    }


def _spatial_quality(evidence: dict[str, Any]) -> str:
    return str(evidence.get("spatial_quality") or "RGB_ONLY")


def _float_or_none(value: Any) -> float | None:
    if value is None:
        return None
    return float(value)


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
