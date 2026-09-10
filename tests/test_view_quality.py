"""计划书 §8/§33：站得住不等于看得见。

贴着 0.3m 的墙时几何上安全，但相机画面全是墙 —— 这里必须给出低分，否则
语义搜索会在墙角原地打转还以为自己在探索。
"""

from __future__ import annotations

from app.navigation.clearance_map import clearance_map_from_points
from app.navigation.view_quality import ViewQualityConfig, evaluate_view_quality


def _wall_at(distance_m: float, *, span_m: float = 4.0, bounds_m: float = 4.0):
    step = 0.02
    count = int(span_m / step) + 1
    points = [(distance_m, -span_m / 2.0 + index * step) for index in range(count)]
    return clearance_map_from_points(
        points, known_bounds=(-bounds_m, -bounds_m, bounds_m, bounds_m))


def _open_space(bounds_m: float = 4.0):
    return clearance_map_from_points(
        [], known_bounds=(-bounds_m, -bounds_m, bounds_m, bounds_m))


# 0.3m 处一面墙：视角质量必须接近 0，且贴面比例拉满。
def test_wall_at_030m_kills_view_quality() -> None:
    view = evaluate_view_quality(pose=(0.0, 0.0, 0.0), clearance_map=_wall_at(0.30))
    assert view.view_quality < 0.05
    assert view.near_obstacle_ratio > 0.9
    assert view.occlusion_risk > 0.5


# 开阔空间明显优于贴墙。
def test_open_space_beats_the_wall() -> None:
    open_view = evaluate_view_quality(pose=(0.0, 0.0, 0.0), clearance_map=_open_space())
    wall_view = evaluate_view_quality(pose=(0.0, 0.0, 0.0), clearance_map=_wall_at(0.30))
    assert open_view.free_space_ratio == 1.0
    assert open_view.near_obstacle_ratio == 0.0
    assert open_view.view_quality > wall_view.view_quality + 0.25


# 视野里有可见 frontier -> 加分；被墙挡住的同一个 frontier 不加分。
def test_visible_frontier_raises_quality() -> None:
    frontiers = [("f1", (2.0, 0.0))]
    open_view = evaluate_view_quality(
        pose=(0.0, 0.0, 0.0), clearance_map=_open_space(), frontiers=frontiers)
    blocked_view = evaluate_view_quality(
        pose=(0.0, 0.0, 0.0), clearance_map=_wall_at(1.0), frontiers=frontiers)
    assert open_view.frontier_visibility == 1.0
    assert open_view.visible_frontier_ids == ("f1",)
    assert blocked_view.frontier_visibility == 0.0
    assert open_view.view_quality > blocked_view.view_quality


# 背对墙面（frontier 在身后）不算可见：FOV 之外不得计入。
def test_frontier_outside_fov_is_not_visible() -> None:
    view = evaluate_view_quality(
        pose=(0.0, 0.0, 3.141592653589793), clearance_map=_open_space(),
        frontiers=[("f1", (2.0, 0.0))])
    assert view.frontier_visibility == 0.0


# 已知空旷 + 前方未观测：未知是可探索信息，得分应高于贴墙但低于全开阔。
def test_unknown_ahead_is_information_not_free_space() -> None:
    near_map = clearance_map_from_points([], known_bounds=(-1.0, -1.0, 1.0, 1.0))
    view = evaluate_view_quality(pose=(0.0, 0.0, 0.0), clearance_map=near_map)
    assert view.visible_unknown_ratio > 0.5
    assert view.near_obstacle_ratio == 0.0
    wall_view = evaluate_view_quality(pose=(0.0, 0.0, 0.0), clearance_map=_wall_at(0.30))
    assert view.view_quality > wall_view.view_quality


# 权重可配置：调高贴面惩罚会让贴墙分数更低。
def test_weights_are_configurable() -> None:
    config = ViewQualityConfig()
    config.weights.free_space = 1.0
    view = evaluate_view_quality(pose=(0.0, 0.0, 0.0), clearance_map=_open_space(),
                                 config=config)
    assert view.view_quality == 1.0
