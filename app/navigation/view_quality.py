"""计划书 §8：视角质量（view quality）。

"能站得住" 不等于 "看得见"。机器狗贴着 0.3m 的墙时几何上完全安全，但相机
整幅画面都是墙，语义搜索等于瞎了。本模块把 "这个位姿的相机能看到多少有用
东西" 量化成一个 0-1 分数，供候选打分与脱困转向共用。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Sequence

from app.navigation.clearance_map import ClearanceMap


@dataclass
class ViewQualityWeights:
    free_space: float = 0.30
    visible_unknown: float = 0.30
    frontier_visibility: float = 0.25
    near_obstacle: float = 0.40
    occlusion_risk: float = 0.35

    def to_dict(self) -> dict[str, Any]:
        return {
            "free_space": self.free_space,
            "visible_unknown": self.visible_unknown,
            "frontier_visibility": self.frontier_visibility,
            "near_obstacle": self.near_obstacle,
            "occlusion_risk": self.occlusion_risk,
        }


@dataclass
class ViewQualityConfig:
    fov_half_deg: float = 45.0
    useful_min_m: float = 0.30
    useful_max_m: float = 2.50
    near_obstacle_m: float = 0.50
    ray_count: int = 19
    obstacle_clearance_m: float = 0.05
    weights: ViewQualityWeights = field(default_factory=ViewQualityWeights)

    def to_dict(self) -> dict[str, Any]:
        return {
            "fov_half_deg": self.fov_half_deg,
            "useful_min_m": self.useful_min_m,
            "useful_max_m": self.useful_max_m,
            "near_obstacle_m": self.near_obstacle_m,
            "ray_count": self.ray_count,
            "obstacle_clearance_m": self.obstacle_clearance_m,
            "weights": self.weights.to_dict(),
        }


@dataclass
class ViewAssessment:
    view_quality: float = 0.0
    free_space_ratio: float = 0.0
    near_obstacle_ratio: float = 0.0
    visible_unknown_ratio: float = 0.0
    frontier_visibility: float = 0.0
    occlusion_risk: float = 0.0
    visible_frontier_ids: tuple[str, ...] = ()
    rays: int = 0

    def to_dict(self) -> dict[str, Any]:
        return {
            "view_quality": round(self.view_quality, 3),
            "free_space_ratio": round(self.free_space_ratio, 3),
            "near_obstacle_ratio": round(self.near_obstacle_ratio, 3),
            "visible_unknown_ratio": round(self.visible_unknown_ratio, 3),
            "frontier_visibility": round(self.frontier_visibility, 3),
            "occlusion_risk": round(self.occlusion_risk, 3),
            "visible_frontier_ids": list(self.visible_frontier_ids),
            "rays": self.rays,
        }


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def evaluate_view_quality(
    *,
    pose: Sequence[float],
    clearance_map: ClearanceMap,
    frontiers: Sequence[tuple[str, Sequence[float]]] = (),
    config: ViewQualityConfig | None = None,
    now: float | None = None,
) -> ViewAssessment:
    """在相机前方扇形里打射线，统计空旷 / 贴面 / 未知 / frontier 可见性。"""
    config = config or ViewQualityConfig()
    x0, y0 = float(pose[0]), float(pose[1])
    yaw_deg = math.degrees(float(pose[2])) if len(pose) > 2 else 0.0
    count = max(3, int(config.ray_count))
    reach_fractions: list[float] = []
    near_obstacle = 0
    unknown_rays = 0
    occluded = 0
    for index in range(count):
        share = -config.fov_half_deg + index * (2.0 * config.fov_half_deg) / (count - 1)
        reach, terminator = _cast_ray(
            x0, y0, math.radians(yaw_deg + share), clearance_map, config, now
        )
        reach_fractions.append(clamp01(reach / max(1e-6, config.useful_max_m)))
        if terminator == "unknown":
            unknown_rays += 1
        elif terminator == "obstacle" and reach < config.near_obstacle_m:
            near_obstacle += 1
        if reach < config.useful_min_m:
            occluded += 1
    free_space_ratio = sum(reach_fractions) / count
    near_obstacle_ratio = near_obstacle / count
    visible_unknown_ratio = unknown_rays / count
    occlusion_risk = occluded / count
    visible_ids: list[str] = []
    for frontier_id, position in frontiers:
        if _within_fov(x0, y0, yaw_deg, position, config.fov_half_deg) and \
                clearance_map.line_of_sight(
                    (x0, y0), position,
                    obstacle_clearance_m=config.obstacle_clearance_m, now=now):
            visible_ids.append(str(frontier_id))
    frontier_visibility = (len(visible_ids) / len(frontiers)) if frontiers else 0.0
    weights = config.weights
    quality = clamp01(
        weights.free_space * free_space_ratio
        + weights.visible_unknown * visible_unknown_ratio
        + weights.frontier_visibility * frontier_visibility
        - weights.near_obstacle * near_obstacle_ratio
        - weights.occlusion_risk * occlusion_risk
    )
    return ViewAssessment(
        view_quality=quality,
        free_space_ratio=free_space_ratio,
        near_obstacle_ratio=near_obstacle_ratio,
        visible_unknown_ratio=visible_unknown_ratio,
        frontier_visibility=frontier_visibility,
        occlusion_risk=occlusion_risk,
        visible_frontier_ids=tuple(visible_ids),
        rays=count,
    )


def _cast_ray(
    x0: float,
    y0: float,
    angle_rad: float,
    clearance_map: ClearanceMap,
    config: ViewQualityConfig,
    now: float | None,
) -> tuple[float, str]:
    step = max(1e-3, clearance_map.sample_step_m)
    distance = step
    cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
    while distance <= config.useful_max_m:
        clearance = clearance_map.clearance_at(
            x0 + distance * cos_a, y0 + distance * sin_a, now=now)
        if clearance is None:
            return distance - step, "unknown"
        if clearance <= config.obstacle_clearance_m:
            return distance - step, "obstacle"
        distance += step
    return config.useful_max_m, "free"


def _within_fov(
    x0: float, y0: float, yaw_deg: float, position: Sequence[float], fov_half_deg: float
) -> bool:
    bearing = math.degrees(math.atan2(float(position[1]) - y0, float(position[0]) - x0))
    delta = (bearing - yaw_deg + 180.0) % 360.0 - 180.0
    return abs(delta) <= fov_half_deg
