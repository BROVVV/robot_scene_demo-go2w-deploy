"""计划书 §10：视点候选生成与排序。

核心原则：不再直接把 frontier 质心或目标本体当目标点。真正下发的目标是
"离 frontier / 目标一段距离、站得住、看得见、路上净空足够" 的观察位姿。
硬安全先做门控（``hard_safe`` 为假直接拒绝），之后才允许打分。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

from app.navigation.clearance_map import ClearanceMap
from app.navigation.exploration_config import ScoringWeights
from app.navigation.view_quality import (
    ViewQualityConfig,
    clamp01,
    evaluate_view_quality,
)
from app.navigation.viewpoint_models import (
    VIEWPOINT_APPROACH_TARGET,
    VIEWPOINT_EXPLORE_FRONTIER,
    ViewpointCandidate,
    ViewpointRequest,
)


@dataclass
class ViewpointPlannerConfig:
    hard_clearance_m: float = 0.40
    preferred_clearance_m: float = 0.80
    footprint_radius_m: float = 0.25
    frontier_standoff_candidates_m: tuple[float, ...] = (0.60, 0.80, 1.00)
    frontier_lateral_offsets_m: tuple[float, ...] = (-0.30, 0.0, 0.30)
    target_ring_angles_deg: tuple[float, ...] = (-90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0)
    route_cost_scale_m: float = 3.0
    traversal_failure_scale: float = 3.0
    view_quality: ViewQualityConfig = field(default_factory=ViewQualityConfig)

    def to_dict(self) -> dict[str, Any]:
        return {
            "hard_clearance_m": self.hard_clearance_m,
            "preferred_clearance_m": self.preferred_clearance_m,
            "footprint_radius_m": self.footprint_radius_m,
            "frontier_standoff_candidates_m": list(self.frontier_standoff_candidates_m),
            "frontier_lateral_offsets_m": list(self.frontier_lateral_offsets_m),
            "target_ring_angles_deg": list(self.target_ring_angles_deg),
            "route_cost_scale_m": self.route_cost_scale_m,
            "traversal_failure_scale": self.traversal_failure_scale,
            "view_quality": self.view_quality.to_dict(),
        }


class ViewpointPlanner:
    def __init__(
        self,
        config: ViewpointPlannerConfig | None = None,
        weights: ScoringWeights | None = None,
    ) -> None:
        self.config = config or ViewpointPlannerConfig()
        self.weights = weights or ScoringWeights()

    # ---- generation ------------------------------------------------------

    def generate_and_rank(
        self,
        request: ViewpointRequest,
        *,
        robot_pose: Sequence[float],
        clearance_map: ClearanceMap,
        frontiers: Sequence[tuple[str, Sequence[float]]] = (),
        traversal_failures: Mapping[str, int] | None = None,
        now: float | None = None,
    ) -> list[ViewpointCandidate]:
        poses = self._candidate_poses(request, robot_pose)
        failures = int((traversal_failures or {}).get(request.subject_id or "", 0))
        candidates = [
            self._evaluate(
                request=request,
                viewpoint_id=f"vp_{index:02d}",
                pose=pose,
                robot_pose=robot_pose,
                clearance_map=clearance_map,
                frontiers=frontiers,
                traversal_failures=failures,
                provenance=provenance,
                now=now,
            )
            for index, (pose, provenance) in enumerate(poses)
        ]
        candidates.sort(key=lambda item: (item.hard_safe, item.score), reverse=True)
        return candidates

    def _candidate_poses(
        self, request: ViewpointRequest, robot_pose: Sequence[float]
    ) -> list[tuple[tuple[float, float, float], dict[str, Any]]]:
        subject = request.subject_position
        if subject is None:
            return []
        sx, sy = float(subject[0]), float(subject[1])
        rx, ry = float(robot_pose[0]), float(robot_pose[1])
        # 后退方向：默认从 subject 指回机器人，即 "从我来的那一侧观察"。
        if request.subject_normal_deg is not None:
            base_deg = float(request.subject_normal_deg)
        else:
            base_deg = math.degrees(math.atan2(ry - sy, rx - sx))
        poses: list[tuple[tuple[float, float, float], dict[str, Any]]] = []
        if request.mode == VIEWPOINT_EXPLORE_FRONTIER:
            for standoff in self.config.frontier_standoff_candidates_m:
                for lateral in self.config.frontier_lateral_offsets_m:
                    angle = math.radians(base_deg)
                    px = sx + standoff * math.cos(angle) - lateral * math.sin(angle)
                    py = sy + standoff * math.sin(angle) + lateral * math.cos(angle)
                    poses.append((
                        (px, py, math.atan2(sy - py, sx - px)),
                        {"standoff_m": standoff, "lateral_m": lateral,
                         "base_deg": round(base_deg, 1)},
                    ))
            return poses
        distance = min(
            max(request.preferred_distance_m, request.desired_distance_min_m),
            request.desired_distance_max_m,
        )
        for ring_deg in self.config.target_ring_angles_deg:
            angle = math.radians(base_deg + ring_deg)
            px = sx + distance * math.cos(angle)
            py = sy + distance * math.sin(angle)
            poses.append((
                (px, py, math.atan2(sy - py, sx - px)),
                {"ring_deg": ring_deg, "standoff_m": distance,
                 "base_deg": round(base_deg, 1)},
            ))
        return poses

    # ---- evaluation ------------------------------------------------------

    def _evaluate(
        self,
        *,
        request: ViewpointRequest,
        viewpoint_id: str,
        pose: tuple[float, float, float],
        robot_pose: Sequence[float],
        clearance_map: ClearanceMap,
        frontiers: Sequence[tuple[str, Sequence[float]]],
        traversal_failures: int,
        provenance: dict[str, Any],
        now: float | None,
    ) -> ViewpointCandidate:
        config = self.config
        local = clearance_map.pose_clearance(pose, config.footprint_radius_m, now=now)
        path = clearance_map.path_clearance(
            [(float(robot_pose[0]), float(robot_pose[1])), (pose[0], pose[1])],
            config.footprint_radius_m, now=now)
        subject = request.subject_position
        subject_distance = None if subject is None else math.hypot(
            pose[0] - float(subject[0]), pose[1] - float(subject[1]))
        line_of_sight = True if subject is None else clearance_map.line_of_sight(
            (pose[0], pose[1]), subject,
            obstacle_clearance_m=config.view_quality.obstacle_clearance_m, now=now)
        view = evaluate_view_quality(
            pose=pose, clearance_map=clearance_map, frontiers=frontiers,
            config=config.view_quality, now=now)
        unknown_ratio = (path.unknown_samples / path.samples) if path.samples else 1.0
        route_reachable = (
            path.min_clearance_m is not None
            and path.min_clearance_m >= config.hard_clearance_m
        )

        rejection = ""
        if local is None:
            rejection = "UNKNOWN_LOCAL_GEOMETRY: viewpoint clearance unobservable"
        elif local < config.hard_clearance_m:
            rejection = (
                f"LOCAL_CLEARANCE_TOO_LOW: {local:.2f}m < {config.hard_clearance_m:.2f}m")
        elif path.min_clearance_m is None:
            rejection = "UNKNOWN_PATH_GEOMETRY: path clearance unobservable"
        elif path.min_clearance_m < config.hard_clearance_m:
            rejection = (
                f"PATH_CLEARANCE_TOO_LOW: {path.min_clearance_m:.2f}m "
                f"< {config.hard_clearance_m:.2f}m")
        elif request.require_line_of_sight and not line_of_sight:
            rejection = "NO_LINE_OF_SIGHT: subject occluded from viewpoint"
        elif request.mode == VIEWPOINT_APPROACH_TARGET and subject_distance is not None and (
                subject_distance < request.desired_distance_min_m):
            rejection = (
                f"TOO_CLOSE_TO_SUBJECT: {subject_distance:.2f}m "
                f"< {request.desired_distance_min_m:.2f}m")

        candidate = ViewpointCandidate(
            viewpoint_id=viewpoint_id,
            pose=pose,
            mode=request.mode,
            subject_id=request.subject_id,
            subject_distance_m=subject_distance,
            line_of_sight=line_of_sight,
            local_clearance_m=local,
            path_min_clearance_m=path.min_clearance_m,
            path_mean_clearance_m=path.mean_clearance_m,
            view_quality=view.view_quality,
            information_gain=0.6 * view.visible_unknown_ratio
            + 0.4 * view.frontier_visibility,
            camera_occlusion_risk=view.occlusion_risk,
            path_cost=path.length_m,
            route_reachable=route_reachable,
            hard_safe=not rejection,
            rejection_reason=rejection,
            provenance={**provenance, "view": view.to_dict()},
        )
        if not candidate.hard_safe:
            # 计划书 §9：硬安全先过滤，被拒候选不参与打分。
            candidate.components = {"rejected": True}
            return candidate
        candidate.score, candidate.components = self._score(
            candidate, view.near_obstacle_ratio, unknown_ratio, traversal_failures)
        return candidate

    def _score(
        self,
        candidate: ViewpointCandidate,
        near_obstacle_ratio: float,
        unknown_ratio: float,
        traversal_failures: int,
    ) -> tuple[float, dict[str, Any]]:
        weights = self.weights
        preferred = max(1e-6, self.config.preferred_clearance_m)
        components = {
            "viewpoint_quality": weights.viewpoint_quality_weight * candidate.view_quality,
            "path_min_clearance": weights.path_min_clearance_weight
            * clamp01((candidate.path_min_clearance_m or 0.0) / preferred),
            "path_mean_clearance": weights.path_mean_clearance_weight
            * clamp01((candidate.path_mean_clearance_m or 0.0) / preferred),
            "route_reachability": weights.route_reachability_weight
            * (1.0 if candidate.route_reachable else 0.0),
            "information_gain": weights.information_gain * candidate.information_gain,
            "route_cost": -weights.route_cost_weight
            * clamp01(candidate.path_cost / max(1e-6, self.config.route_cost_scale_m)),
            "near_wall_risk": -weights.near_wall_risk_weight * clamp01(near_obstacle_ratio),
            "camera_occlusion_risk": -weights.camera_occlusion_risk_weight
            * clamp01(candidate.camera_occlusion_risk),
            "traversal_failure": -weights.traversal_failure_weight
            * clamp01(traversal_failures / max(1e-6, self.config.traversal_failure_scale)),
            "unknown_geometry_risk": -weights.unknown_geometry_risk_weight
            * clamp01(unknown_ratio),
        }
        return sum(components.values()), {
            key: round(value, 4) for key, value in components.items()
        }
