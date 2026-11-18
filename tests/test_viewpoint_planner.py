"""计划书 §10/§33：目标点是观察位姿，不是 frontier 质心或目标本体。

两个关键场景：frontier 紧贴墙面（去质心=撞墙+相机糊墙），目标紧贴墙面
（绕到墙那一侧就什么都看不见）。规划器必须自己退到开放侧。
"""

from __future__ import annotations

import math

import pytest

from app.navigation.clearance_map import clearance_map_from_points
from app.navigation.corridor_analyzer import CENTER_SCORE_WEIGHT, CorridorGrid
from app.navigation.view_quality import clamp01
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


# ---------------------------------------------------------------------------
# §7.5 对称净空分：算得出来才算，且只给已经安全的候选排序用
# ---------------------------------------------------------------------------

_GRID_RESOLUTION = 0.05
_GRID_WALL = 100


def _corridor_grid(half_width: float = 1.2) -> CorridorGrid:
    """上面那条走廊的占据栅格视图 —— §7.5 要量的就是这份数据。"""
    width = int(round(6.0 / _GRID_RESOLUTION))
    height = int(round(4.0 / _GRID_RESOLUTION))
    data = [0] * (width * height)
    for row in range(height):
        if abs(-2.0 + (row + 0.5) * _GRID_RESOLUTION) >= half_width:
            for column in range(width):
                data[row * width + column] = _GRID_WALL
    return CorridorGrid(
        resolution=_GRID_RESOLUTION, origin_x=-2.0, origin_y=-2.0,
        width=width, height=height, data=tuple(data))


def _corridor_request() -> ViewpointRequest:
    return ViewpointRequest(
        mode=VIEWPOINT_EXPLORE_FRONTIER, subject_id="f_end",
        subject_position=(2.0, 0.0))


def _corridor_candidates(*, occupancy_grid=None):
    corridor = _wall(_horizontal_wall_at_y(1.2) + _horizontal_wall_at_y(-1.2))
    return _planner().generate_and_rank(
        _corridor_request(), robot_pose=(0.0, 0.0, 0.0), clearance_map=corridor,
        frontiers=[("f_end", (2.0, 0.0))], occupancy_grid=occupancy_grid)


def test_center_score_term_is_zero_without_an_occupancy_grid() -> None:
    """没有栅格就没有这条证据，记 0 分 —— 不能当成「很居中」白送分。"""
    candidates = _corridor_candidates()
    safe = [item for item in candidates if item.hard_safe]
    assert safe
    assert all(item.center_score is None for item in safe)
    assert all(item.components["center_score"] == 0.0 for item in safe)


def test_center_score_term_is_the_weighted_measure_of_the_candidate_route() -> None:
    """打分项必须是**这条候选路径**量出来的 0.25 倍，不是别的什么东西。

    期望值在这里写死成计划书 §7.5 的 0.25，不从被测模块里取 —— 从被测模块取
    权重的话，把它改成 0 两边一起变，断言照样绿，等于什么都没测。
    """
    assert CENTER_SCORE_WEIGHT == 0.25  # 常量本身也是合同的一部分
    candidates = _corridor_candidates(occupancy_grid=_corridor_grid())
    safe = [item for item in candidates if item.hard_safe]
    assert safe
    measured = [item for item in safe if item.center_score is not None]
    assert measured, "走廊里的候选应该能量出对称净空分"
    for item in safe:
        expected = 0.25 * clamp01(item.center_score or 0.0)
        assert item.components["center_score"] == pytest.approx(expected, abs=1e-4)
    # 被拒候选不参与打分，这一项自然也不该出现。
    for item in candidates:
        if not item.hard_safe:
            assert item.components == {"rejected": True}


def test_centred_route_outscores_an_off_centre_one_on_this_term() -> None:
    """同一条走廊里，越靠中线的候选这一项越高，且差值正好是 0.25×Δcenter。

    这条走廊里其余各项本来就已经偏向中线，所以这里**不**声称这一项翻转了排序 ——
    它只是把已有的倾向按 §7.5 的固定权重记账。要断言的就只有这一项的量。
    """
    candidates = _corridor_candidates(occupancy_grid=_corridor_grid())
    safe = [item for item in candidates if item.hard_safe and item.center_score is not None]
    centre = min(safe, key=lambda item: abs(item.pose[1]))
    off_centre = max(safe, key=lambda item: abs(item.pose[1]))
    assert abs(off_centre.pose[1]) > abs(centre.pose[1])
    assert centre.center_score > off_centre.center_score
    assert centre.components["center_score"] > off_centre.components["center_score"]
    delta = centre.components["center_score"] - off_centre.components["center_score"]
    # 0.25 写死（同上一处）：差值必须正好是这个固定权重乘上两个 center 之差。
    assert delta == pytest.approx(
        0.25 * (centre.center_score - off_centre.center_score), abs=1e-4
    )


def _open_grid() -> CorridorGrid:
    """一片全空的栅格：任何方向都量到 2.0 m 上限，对称分恒为满分。

    专门用来验「被拒的候选拿不到这一项」——如果它拿到了，这里就会是白送的 0.25。
    """
    width = int(round(6.0 / _GRID_RESOLUTION))
    height = int(round(6.0 / _GRID_RESOLUTION))
    return CorridorGrid(
        resolution=_GRID_RESOLUTION, origin_x=-3.0, origin_y=-3.0,
        width=width, height=height, data=(0,) * (width * height))


def test_center_score_never_rescues_a_candidate_that_failed_hard_safety() -> None:
    """§7.5 明确这一项不允许覆盖「路径过窄」判定：被拒的仍是零分。

    用「目标贴墙」那个场景 —— 它本来就会拒掉绕到墙那一侧的 ring 候选；
    再喂一片全空栅格（对称分恒为满分），这样只要有人让被拒候选也参与打分，
    这里立刻能看出来。
    """
    candidates = _planner().generate_and_rank(
        ViewpointRequest(
            mode=VIEWPOINT_APPROACH_TARGET, subject_id="target_1",
            subject_position=(0.0, 1.9), desired_distance_min_m=0.70,
            desired_distance_max_m=1.10, preferred_distance_m=0.90,
            require_line_of_sight=True),
        robot_pose=(0.0, 0.0, 0.0),
        clearance_map=_wall(_horizontal_wall_at_y(2.0) + [(0.0, 1.9)]),
        occupancy_grid=_open_grid(),
    )
    safe = [item for item in candidates if item.hard_safe]
    rejected = [item for item in candidates if not item.hard_safe]
    assert rejected, "这个场景必须真的拒掉一些候选，否则这条测试什么也没测"
    assert safe, "也得留下安全候选，否则「被拒的没拿到分」没有对照"
    # 全空栅格下安全候选的对称分是满分 —— 所以下面那句断言是有分辨力的。
    assert any(item.center_score == pytest.approx(1.0) for item in safe)
    assert all(item.score == 0.0 for item in rejected)
    assert all(item.components == {"rejected": True} for item in rejected)
