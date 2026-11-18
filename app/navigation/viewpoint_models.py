"""计划书 §10/§20：视点候选与目标接近上下文的数据契约。

候选不再是 "一个方向 + 一个距离"，而是 "一个可站住、看得见、能走到的位姿"，
并且把所有安全/视觉分量原样带出来，供打分、决策记录与 WebUI 复现。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

VIEWPOINT_EXPLORE_FRONTIER = "EXPLORE_FRONTIER"
VIEWPOINT_OBSERVE_CANDIDATE = "OBSERVE_CANDIDATE"
VIEWPOINT_APPROACH_TARGET = "APPROACH_TARGET"
VIEWPOINT_MODES = (
    VIEWPOINT_EXPLORE_FRONTIER,
    VIEWPOINT_OBSERVE_CANDIDATE,
    VIEWPOINT_APPROACH_TARGET,
)


@dataclass
class ViewpointCandidate:
    viewpoint_id: str = ""
    pose: tuple[float, float, float] = (0.0, 0.0, 0.0)
    mode: str = VIEWPOINT_EXPLORE_FRONTIER
    subject_id: str | None = None
    subject_distance_m: float | None = None
    line_of_sight: bool = False
    local_clearance_m: float | None = None
    path_min_clearance_m: float | None = None
    path_mean_clearance_m: float | None = None
    # 计划书 §7.5：路径离走廊中线的对称度，1 表示正中；没有栅格可量时为 None。
    center_score: float | None = None
    view_quality: float = 0.0
    information_gain: float = 0.0
    camera_occlusion_risk: float = 0.0
    path_cost: float = 0.0
    route_reachable: bool = False
    hard_safe: bool = False
    rejection_reason: str = ""
    score: float = 0.0
    components: dict[str, Any] = field(default_factory=dict)
    provenance: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "viewpoint_id": self.viewpoint_id,
            "pose": [round(value, 3) for value in self.pose],
            "mode": self.mode,
            "subject_id": self.subject_id,
            "subject_distance_m": _round(self.subject_distance_m),
            "line_of_sight": self.line_of_sight,
            "local_clearance_m": _round(self.local_clearance_m),
            "path_min_clearance_m": _round(self.path_min_clearance_m),
            "path_mean_clearance_m": _round(self.path_mean_clearance_m),
            "center_score": _round(self.center_score),
            "view_quality": round(self.view_quality, 3),
            "information_gain": round(self.information_gain, 3),
            "camera_occlusion_risk": round(self.camera_occlusion_risk, 3),
            "path_cost": round(self.path_cost, 3),
            "route_reachable": self.route_reachable,
            "hard_safe": self.hard_safe,
            "rejection_reason": self.rejection_reason,
            "score": round(self.score, 4),
            "components": dict(self.components),
            "provenance": dict(self.provenance),
        }


@dataclass
class TargetApproachContext:
    """目标接近的唯一真值容器；距离来自 RGBD，不允许写死 1m。"""

    object_id: str = ""
    label: str = ""
    depth_m: float | None = None
    bearing_deg: float | None = None
    camera_xyz: tuple[float, float, float] | None = None
    map_xyz: tuple[float, float, float] | None = None
    spatial_quality: str = "RGB_ONLY"
    confidence: float = 0.0
    desired_standoff_m: float = 0.90
    min_standoff_m: float = 0.70
    max_standoff_m: float = 1.10
    last_seen_timestamp: float = 0.0
    lost_count: int = 0

    def to_dict(self) -> dict[str, Any]:
        return {
            "object_id": self.object_id,
            "label": self.label,
            "depth_m": _round(self.depth_m),
            "bearing_deg": _round(self.bearing_deg),
            "camera_xyz": None if self.camera_xyz is None else [
                round(value, 3) for value in self.camera_xyz],
            "map_xyz": None if self.map_xyz is None else [
                round(value, 3) for value in self.map_xyz],
            "spatial_quality": self.spatial_quality,
            "confidence": round(self.confidence, 3),
            "desired_standoff_m": self.desired_standoff_m,
            "min_standoff_m": self.min_standoff_m,
            "max_standoff_m": self.max_standoff_m,
            "last_seen_timestamp": self.last_seen_timestamp,
            "lost_count": self.lost_count,
        }


@dataclass
class ViewpointRequest:
    mode: str = VIEWPOINT_EXPLORE_FRONTIER
    subject_id: str | None = None
    subject_position: tuple[float, float] | None = None
    subject_normal_deg: float | None = None
    desired_distance_min_m: float = 0.60
    desired_distance_max_m: float = 1.00
    preferred_distance_m: float = 0.80
    require_line_of_sight: bool = False
    prefer_high_clearance: bool = True
    prefer_information_gain: bool = True

    def to_dict(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "subject_id": self.subject_id,
            "subject_position": None if self.subject_position is None else [
                round(value, 3) for value in self.subject_position],
            "subject_normal_deg": _round(self.subject_normal_deg),
            "desired_distance_min_m": self.desired_distance_min_m,
            "desired_distance_max_m": self.desired_distance_max_m,
            "preferred_distance_m": self.preferred_distance_m,
            "require_line_of_sight": self.require_line_of_sight,
            "prefer_high_clearance": self.prefer_high_clearance,
            "prefer_information_gain": self.prefer_information_gain,
        }


def _round(value: float | None) -> float | None:
    return None if value is None else round(float(value), 3)
