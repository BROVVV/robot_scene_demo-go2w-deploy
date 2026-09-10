"""计划书 §10/§33：目标点是观察位姿，不是 frontier 质心或目标本体。

两个关键场景：frontier 紧贴墙面（去质心=撞墙+相机糊墙），目标紧贴墙面
（绕到墙那一侧就什么都看不见）。规划器必须自己退到开放侧。
"""

from __future__ import annotations

import math

from app.navigation.clearance_map import clearance_map_from_points
from app.navigation.viewpoint_models import (
    VIEWPOINT_APPROACH_TARGET,
    VIEWPOINT_EXPLORE_FRONTIER,
    ViewpointRequest,
)
from app.navigation.viewpoint_planner import ViewpointPlanner, ViewpointPlannerConfig


def _wall(points, *, bounds=(-3.0, -3.0, 3.0, 3.0)):
    return clearance_map_from_points(points, known_bounds=bounds)


def _vertical_wall_at_x(x: float, *, span: float = 6.0, step: float = 0.02):
    count = int(span / step) + 1
    return [(x, -span / 2.0 + index * step) for index in range(count)]


def _horizontal_wall_at_y(y: float, *, span: float = 6.0, step: float = 0.02):
    count = int(span / step) + 1
    return [(-span / 2.0 + index * step, y) for index in range(count)]


def _planner() -> ViewpointPlanner:
    return ViewpointPlanner(ViewpointPlannerConfig(footprint_radius_m=0.20))


# 场景：frontier 质心贴在 x=2.0 的墙上 -> 必须选后退后的开放位姿。
def test_frontier_on_a_wall_is_observed_from_a_standoff() -> None:
    clearance_map = _wall(_vertical_wall_at_x(2.0))
    planner = _planner()
    # 质心本身站不住：这正是 "直接去 frontier centroid" 会撞墙的原因。
    assert (clearance_map.pose_clearance((1.9, 0.0), 0.20) or 0.0) < 0.40
    request = ViewpointRequest(
        mode=VIEWPOINT_EXPLORE_FRONTIER, subject_id="f_wall",
        subject_position=(1.9, 0.0))
    candidates = planner.generate_and_rank(
        request, robot_pose=(0.0, 0.0, 0.0), clearance_map=clearance_map,
        frontiers=[("f_wall", (1.9, 0.0))])
    best = candidates[0]
    assert best.hard_safe is True
    assert best.rejection_reason == ""
    assert best.subject_distance_m is not None and best.subject_distance_m >= 0.60
    assert best.local_clearance_m is not None and best.local_clearance_m >= 0.40
    assert best.path_min_clearance_m is not None and best.path_min_clearance_m >= 0.40
    assert best.route_reachable is True
    # 相机朝着 frontier。
    bearing = math.atan2(0.0 - best.pose[1], 1.9 - best.pose[0])
    assert abs(bearing - best.pose[2]) < 1e-6


# 场景：目标贴在 y=2.0 的墙上 -> 只能从机器人这一侧观察，不许绕到墙里。
def test_target_against_a_wall_is_approached_from_the_open_side() -> None:
    clearance_map = _wall(_horizontal_wall_at_y(2.0) + [(0.0, 1.9)])
    planner = _planner()
    request = ViewpointRequest(
        mode=VIEWPOINT_APPROACH_TARGET, subject_id="target_1",
        subject_position=(0.0, 1.9), desired_distance_min_m=0.70,
        desired_distance_max_m=1.10, preferred_distance_m=0.90,
        require_line_of_sight=True)
    candidates = planner.generate_and_rank(
        request, robot_pose=(0.0, 0.0, 0.0), clearance_map=clearance_map)
    best = candidates[0]
    assert best.hard_safe is True
    assert best.line_of_sight is True
    assert 0.70 <= (best.subject_distance_m or 0.0) <= 1.10
    # 站在目标与机器人之间的开放侧，而不是墙那一侧。
    assert best.pose[1] < 1.9
    assert abs(best.provenance["ring_deg"]) <= 30.0
    bearing = math.atan2(1.9 - best.pose[1], 0.0 - best.pose[0])
    assert abs(bearing - best.pose[2]) < 1e-6
    # 贴墙那一侧的 ring 候选必须被硬安全拒绝。
    rejected = [item for item in candidates if not item.hard_safe]
    assert rejected
    assert all(item.rejection_reason for item in rejected)
    assert any(abs(item.provenance["ring_deg"]) == 90.0 for item in rejected)


# 硬安全先过滤：被拒候选不参与打分，也不会排到安全候选前面。
def test_rejected_candidates_never_outrank_safe_ones() -> None:
    clearance_map = _wall(_vertical_wall_at_x(2.0))
    planner = _planner()
    request = ViewpointRequest(
        mode=VIEWPOINT_EXPLORE_FRONTIER, subject_id="f_wall",
        subject_position=(1.9, 0.0))
    candidates = planner.generate_and_rank(
        request, robot_pose=(0.0, 0.0, 0.0), clearance_map=clearance_map)
    safe_flags = [item.hard_safe for item in candidates]
    assert safe_flags == sorted(safe_flags, reverse=True)
    for item in candidates:
        if not item.hard_safe:
            assert item.components == {"rejected": True}
            assert item.score == 0.0


# 未观测区域里的候选必须被拒（不是 "还没看到所以应该没问题"）。
def test_unknown_geometry_candidate_is_rejected() -> None:
    clearance_map = clearance_map_from_points(
        _vertical_wall_at_x(2.0), known_bounds=(-0.5, -0.5, 0.5, 0.5))
    planner = _planner()
    request = ViewpointRequest(
        mode=VIEWPOINT_EXPLORE_FRONTIER, subject_id="f_far",
        subject_position=(1.9, 0.0))
    candidates = planner.generate_and_rank(
        request, robot_pose=(0.0, 0.0, 0.0), clearance_map=clearance_map)
    assert candidates
    assert all(not item.hard_safe for item in candidates)
    assert all("UNKNOWN" in item.rejection_reason for item in candidates)


# 净空更高的路线得分更高：走廊里应该偏向中线。
def test_higher_clearance_route_scores_higher() -> None:
    corridor = _wall(_horizontal_wall_at_y(1.2) + _horizontal_wall_at_y(-1.2))
    planner = _planner()
    request = ViewpointRequest(
        mode=VIEWPOINT_EXPLORE_FRONTIER, subject_id="f_end",
        subject_position=(2.0, 0.0))
    candidates = planner.generate_and_rank(
        request, robot_pose=(0.0, 0.0, 0.0), clearance_map=corridor,
        frontiers=[("f_end", (2.0, 0.0))])
    safe = [item for item in candidates if item.hard_safe]
    assert safe
    best = safe[0]
    assert abs(best.pose[1]) <= 0.05
    for other in safe[1:]:
        if abs(other.pose[1]) > 0.05:
            assert best.score >= other.score
