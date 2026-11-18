"""计划书 §12.3：走廊分析测试。

全部用人工构造的占据栅格，不依赖 ROS、不依赖地图文件。
"""

from __future__ import annotations

import math

import pytest

from app.navigation.corridor_analyzer import (
    DECISION_OK,
    DECISION_ROTATION_BLOCKED,
    DECISION_TOO_NARROW,
    FOOTPRINT_PADDING_M,
    HALF_BODY_LENGTH_M,
    HALF_BODY_WIDTH_M,
    HARD_SIDE_GAP_M,
    ROTATION_REASON_BLOCKED,
    ROTATION_REASON_NOT_APPLICABLE,
    CorridorGrid,
    _footprint_cells,
    center_score,
    classify_path_clearance,
    classify_forward_padding_egress,
    classify_rotation_sweep,
    clear_current_physical_footprint_data,
    measure_path_clearance,
    path_center_score,
    speed_limit_from_clearance,
)

UNKNOWN = -1
FREE = 0
WALL = 100

RESOLUTION = 0.05
BODY_INSET = HALF_BODY_WIDTH_M + FOOTPRINT_PADDING_M
# 光线步进按栅格分辨率走，量到的距离天然会有 ±1 格（5 cm）的量化误差。
MEASURE_TOL = RESOLUTION + 1e-6


def _grid(x_min: float, x_max: float, y_min: float, y_max: float) -> CorridorGrid:
    width = int(round((x_max - x_min) / RESOLUTION))
    height = int(round((y_max - y_min) / RESOLUTION))
    return CorridorGrid(
        resolution=RESOLUTION,
        origin_x=x_min,
        origin_y=y_min,
        width=width,
        height=height,
        data=[FREE] * (width * height),
    )


def _paint(
    grid: CorridorGrid, x_min: float, x_max: float, y_min: float, y_max: float, value: int
) -> None:
    """把矩形区域涂成 value；两端的格子都算在内（含边界那格）。"""
    data = list(grid.data)
    col_lo = max(0, int(math.floor((x_min - grid.origin_x) / grid.resolution)))
    col_hi = min(
        grid.width, int(math.floor((x_max - grid.origin_x) / grid.resolution)) + 1
    )
    row_lo = max(0, int(math.floor((y_min - grid.origin_y) / grid.resolution)))
    row_hi = min(
        grid.height, int(math.floor((y_max - grid.origin_y) / grid.resolution)) + 1
    )
    for row in range(row_lo, row_hi):
        for column in range(col_lo, col_hi):
            data[row * grid.width + column] = value
    object.__setattr__(grid, "data", tuple(data))


def _corridor_along_x(half_width: float, *, x_max: float = 6.0) -> CorridorGrid:
    grid = _grid(-1.0, x_max, -3.0, 3.0)
    _paint(grid, -1.0, x_max, half_width, 3.0, WALL)
    _paint(grid, -1.0, x_max, -3.0, -half_width, WALL)
    return grid


def _straight_path(
    y: float, *, length: float = 3.0, x0: float = 0.0
) -> list[tuple[float, float]]:
    return [(x0, y), (x0 + length, y)]


def _body_gap(distance_m: float) -> float:
    return distance_m - BODY_INSET


def test_current_body_clear_removes_inner_stale_cells_but_preserves_padding_ring():
    grid = _grid(-1., 1., -1., 1.)
    _paint(grid, -.30, -.29, .15, .16, WALL)
    _paint(grid, -.30, -.29, .225, .235, WALL)
    result, evidence = clear_current_physical_footprint_data(
        grid.data, width=grid.width, height=grid.height,
        resolution=grid.resolution, origin_x=grid.origin_x, origin_y=grid.origin_y,
        robot_x=0., robot_y=0., robot_yaw=0.,
    )
    published = CorridorGrid(grid.resolution, grid.origin_x, grid.origin_y,
                             grid.width, grid.height, result)
    assert published.value_at(-.275, .175) == FREE
    assert published.value_at(-.275, .225) == WALL
    assert evidence['published_occupied_cells_cleared'] >= 1
    assert evidence['footprint_padding_masked'] is False


def test_forward_egress_allows_only_known_rear_padding_on_straight_short_path():
    grid = _grid(-1., 1., -1., 1.)
    _paint(grid, -.40, -.39, .17, .19, WALL)
    decision = classify_forward_padding_egress(
        grid, 0., 0., 0., [(-.02, .005), (.02, .003), (.24, 0.)]
    )
    assert decision.allowed is True
    assert decision.reason == 'known_rear_padding_blocker_and_monotonic_forward_path'
    assert decision.blocking_cells
    assert all(cell['local_x'] < -HALF_BODY_LENGTH_M for cell in decision.blocking_cells)


@pytest.mark.parametrize(
    ('box', 'value', 'reason'),
    [
        ((-.40, -.39, .17, .19), UNKNOWN, 'rear_padding_contains_unknown_or_off_map'),
        ((-.30, -.29, .225, .24), WALL, 'blocker_not_strictly_behind_body'),
        ((.36, .38, .0, .02), WALL, 'blocker_not_strictly_behind_body'),
    ],
)
def test_forward_egress_rejects_unknown_side_and_front_padding(box, value, reason):
    grid = _grid(-1., 1., -1., 1.)
    _paint(grid, *box, value)
    decision = classify_forward_padding_egress(
        grid, 0., 0., 0., [(0., 0.), (.24, 0.)]
    )
    assert decision.allowed is False
    assert decision.reason == reason


def test_forward_egress_rejects_lateral_or_nonmonotonic_paths():
    grid = _grid(-1., 1., -1., 1.)
    _paint(grid, -.40, -.39, .17, .19, WALL)
    lateral = classify_forward_padding_egress(
        grid, 0., 0., 0., [(0., 0.), (.12, .04), (.24, .04)]
    )
    reverse = classify_forward_padding_egress(
        grid, 0., 0., 0., [(0., 0.), (.12, 0.), (.08, 0.), (.24, 0.)]
    )
    assert lateral.allowed is False
    assert lateral.reason == 'path_not_straight'
    assert reverse.allowed is False
    assert reverse.reason == 'path_does_not_move_monotonically_forward'


def test_supervised_egress_can_leave_known_rear_corner_padding_only_straight():
    grid = _grid(-1., 1., -1., 1.)
    _paint(grid, -.38, -.36, -.24, -.22, WALL)
    default = classify_forward_padding_egress(
        grid, 0., 0., 0., [(0., 0.), (.50, 0.)],
        max_distance_m=.60,
    )
    allowed = classify_forward_padding_egress(
        grid, 0., 0., 0., [(0., 0.), (.50, 0.)],
        max_distance_m=.60, max_lateral_m=.05,
        allow_known_rear_corner_padding=True,
        max_segment_angle_rad=math.radians(5.),
    )
    curved = classify_forward_padding_egress(
        grid, 0., 0., 0., [(0., 0.), (.25, .04), (.50, .08)],
        max_distance_m=.60, max_lateral_m=.10,
        allow_known_rear_corner_padding=True,
        max_segment_angle_rad=math.radians(5.),
    )
    assert default.allowed is False
    assert allowed.allowed is True
    assert allowed.to_dict()['policy'] == (
        'known_rear_corner_padding_forward_egress_only'
    )
    assert curved.allowed is False
    assert curved.reason == 'path_segment_leaves_forward_cone'


def test_field_frontier_egress_admits_only_unknown_rear_contact_shell():
    grid = _grid(-1., 1., -1., 1.)
    # One-cell quantization makes this cell touch the rear-left body boundary.
    _paint(grid, -.40, -.39, .24, .25, UNKNOWN)
    decision = classify_forward_padding_egress(
        grid, 0., 0., 0., [(0., 0.), (.25, .04), (.52, .07)],
        max_distance_m=.60, max_lateral_m=.15, allow_unknown_rear=True,
    )
    assert decision.allowed is True
    assert decision.unknown_rear_allowed is True
    assert decision.to_dict()['policy'] == 'rear_contact_shell_forward_egress'

    front = _grid(-1., 1., -1., 1.)
    _paint(front, .36, .37, 0., .01, UNKNOWN)
    rejected = classify_forward_padding_egress(
        front, 0., 0., 0., [(0., 0.), (.52, .07)],
        max_distance_m=.60, max_lateral_m=.15, allow_unknown_rear=True,
    )
    assert rejected.allowed is False


# ---------------------------------------------------------------------------
# §12.3 宽直走廊
# ---------------------------------------------------------------------------


def test_wide_straight_corridor_measures_symmetric_and_runs_full_speed() -> None:
    grid = _corridor_along_x(1.25)
    samples = measure_path_clearance(_straight_path(0.0), grid)
    assert samples
    assert all(
        item.left_distance_m == pytest.approx(1.25, abs=MEASURE_TOL) for item in samples
    )
    assert all(
        item.right_distance_m == pytest.approx(1.25, abs=MEASURE_TOL) for item in samples
    )

    decision = classify_path_clearance(samples)
    assert decision.decision == DECISION_OK
    assert decision.reject_reason == ""
    assert decision.min_side_gap_m == pytest.approx(_body_gap(1.25), abs=MEASURE_TOL)
    assert decision.min_left_body_gap_m == pytest.approx(_body_gap(1.25), abs=MEASURE_TOL)
    assert decision.min_right_body_gap_m == pytest.approx(
        _body_gap(1.25), abs=MEASURE_TOL
    )
    assert decision.corridor_width_m == pytest.approx(2.50, abs=2 * MEASURE_TOL)
    assert decision.center_error_m == pytest.approx(0.0, abs=2 * MEASURE_TOL)
    assert decision.center_score > 0.95
    assert decision.speed_limit_mps == pytest.approx(0.10)


def test_path_is_sampled_every_twenty_centimetres_and_trimmed_at_both_ends() -> None:
    grid = _corridor_along_x(1.25)
    samples = measure_path_clearance(_straight_path(0.0, length=3.0), grid)
    assert samples[0].s_m == pytest.approx(0.20)
    assert samples[-1].s_m == pytest.approx(2.80)
    for previous, current in zip(samples, samples[1:]):
        assert current.s_m - previous.s_m == pytest.approx(0.20)


# ---------------------------------------------------------------------------
# §12.3 可通过的窄走廊
# ---------------------------------------------------------------------------


def test_passable_narrow_corridor_slows_down_within_band() -> None:
    grid = _corridor_along_x(0.90)
    decision = classify_path_clearance(measure_path_clearance(_straight_path(0.0), grid))
    assert decision.decision == DECISION_OK
    assert decision.min_side_gap_m == pytest.approx(_body_gap(0.90), abs=MEASURE_TOL)
    assert 0.05 <= decision.speed_limit_mps < 0.10


def test_speed_formula_matches_plan_section_7_4() -> None:
    assert speed_limit_from_clearance(HARD_SIDE_GAP_M) == pytest.approx(0.05)
    assert speed_limit_from_clearance(0.60) == pytest.approx(0.075)
    assert speed_limit_from_clearance(0.80) == pytest.approx(0.10)
    assert speed_limit_from_clearance(5.0) == pytest.approx(0.10)
    assert speed_limit_from_clearance(0.39) == 0.0
    assert speed_limit_from_clearance(None) == 0.0


# ---------------------------------------------------------------------------
# §12.3 机身单侧净空小于 0.40 m
# ---------------------------------------------------------------------------


def test_single_side_gap_below_hard_clearance_is_too_narrow() -> None:
    grid = _corridor_along_x(0.55)
    decision = classify_path_clearance(measure_path_clearance(_straight_path(0.0), grid))
    assert decision.min_side_gap_m == pytest.approx(_body_gap(0.55), abs=MEASURE_TOL)
    assert decision.min_side_gap_m < HARD_SIDE_GAP_M
    assert decision.decision == DECISION_TOO_NARROW
    assert decision.reject_reason == "TOO_NARROW"
    assert decision.speed_limit_mps == 0.0


def test_asymmetric_corridor_is_limited_by_the_near_wall() -> None:
    """只有一侧贴墙也必须拒绝，不能拿另一侧的宽敞平均掉。"""
    grid = _grid(-1.0, 6.0, -3.0, 3.0)
    _paint(grid, -1.0, 6.0, 0.60, 3.0, WALL)
    _paint(grid, -1.0, 6.0, -3.0, -1.90, WALL)
    decision = classify_path_clearance(measure_path_clearance(_straight_path(0.0), grid))
    assert decision.min_left_body_gap_m == pytest.approx(_body_gap(0.60), abs=MEASURE_TOL)
    assert decision.min_right_body_gap_m == pytest.approx(
        _body_gap(1.90), abs=MEASURE_TOL
    )
    assert decision.min_side_gap_m == pytest.approx(_body_gap(0.60), abs=MEASURE_TOL)
    assert decision.decision == DECISION_TOO_NARROW


def test_empty_path_is_fail_closed() -> None:
    grid = _corridor_along_x(1.25)
    decision = classify_path_clearance(measure_path_clearance([], grid))
    assert decision.decision == DECISION_TOO_NARROW
    assert decision.reject_reason == "NO_PATH_SAMPLES"
    assert decision.speed_limit_mps == 0.0
    assert not decision.passable


# ---------------------------------------------------------------------------
# §12.3 路径偏左 → center_score 低于中线候选
# ---------------------------------------------------------------------------


def test_off_centre_path_scores_lower_than_the_centre_line() -> None:
    grid = _corridor_along_x(1.20)
    centre = classify_path_clearance(measure_path_clearance(_straight_path(0.0), grid))
    off_centre = classify_path_clearance(measure_path_clearance(_straight_path(0.20), grid))
    assert centre.decision == DECISION_OK
    assert off_centre.decision == DECISION_OK
    assert centre.center_score > 0.9
    assert centre.center_error_m < off_centre.center_error_m
    # 路径偏离中线 0.20 m ⇒ 两侧机身净空相差 0.40 m，评分必须跟着降。
    assert off_centre.center_error_m == pytest.approx(0.40, abs=2 * MEASURE_TOL)
    assert 0.0 < off_centre.center_score < centre.center_score


def test_center_score_is_one_in_the_middle_and_stays_in_range() -> None:
    assert center_score(1.0, 1.0) == pytest.approx(1.0)
    assert center_score(2.0, 0.0) == pytest.approx(0.0)
    assert 0.0 <= center_score(0.05, 4.0) <= 1.0


def test_path_center_score_matches_the_binding_sample_of_classify() -> None:
    """§7.5 的候选打分必须和 §7.4 的判定看同一个约束点，不能各算各的。"""
    grid = _corridor_along_x(1.20)
    path = _straight_path(0.20)
    decision = classify_path_clearance(measure_path_clearance(path, grid))
    assert path_center_score(path, grid) == pytest.approx(decision.center_score)


def test_path_center_score_is_none_when_there_is_nothing_to_measure() -> None:
    grid = _corridor_along_x(1.20)
    # 整段落在地图外：量不到就是量不到，不能当成「居中」。
    assert path_center_score([(50.0, 50.0), (50.5, 50.0)], grid) is None
    assert path_center_score([], grid) is None


def test_path_center_score_refuses_to_score_a_path_that_does_not_fit() -> None:
    """被堵死的地方没有「居中」可言 —— 这里必须给 None 而不是满分。

    左右光线都在第一格撞墙时两侧距离相等，照对称公式算出来是 1.0；候选排序
    一旦拿到这个 1.0，就会把一条过不去的路当成「走得最正」的那条。
    """
    grid = _corridor_along_x(0.55)
    path = _straight_path(0.0)
    # §7.4 的判定照旧给一个 center_score（那是给人看的证据）。
    assert classify_path_clearance(measure_path_clearance(path, grid)).decision == (
        DECISION_TOO_NARROW
    )
    assert path_center_score(path, grid) is None


# ---------------------------------------------------------------------------
# §12.3 弯道使用局部法向
# ---------------------------------------------------------------------------


def test_corner_path_uses_local_normal_not_a_global_axis() -> None:
    """L 形走廊：第二段沿 +y 走，左右必须横跨 x，而不是继续用全局 ±y。"""
    grid = _grid(-2.0, 5.0, -2.0, 6.0)
    _paint(grid, -2.0, 5.0, -2.0, -1.40, WALL)  # 下墙
    _paint(grid, -2.0, -1.50, -2.0, 6.0, WALL)  # 横廊左端
    _paint(grid, 3.40, 5.0, -2.0, 6.0, WALL)  # 竖廊右墙
    _paint(grid, -2.0, 0.60, 1.40, 6.0, WALL)  # 横廊上墙 + 竖廊左墙
    _paint(grid, -2.0, 5.0, 5.00, 6.0, WALL)  # 竖廊顶端

    path = [(0.0, 0.0), (2.0, 0.0), (2.0, 4.0)]
    samples = measure_path_clearance(path, grid)
    # 只取横廊那一段（x > 0.6 就是拐角开口，上方本来就是通的）。
    first_leg = [item for item in samples if item.y == pytest.approx(0.0) and item.x < 0.5]
    second_leg = [item for item in samples if item.y > 1.5]
    assert first_leg and second_leg

    for item in first_leg:
        assert item.left_distance_m == pytest.approx(1.40, abs=MEASURE_TOL)
        assert item.right_distance_m == pytest.approx(1.40, abs=MEASURE_TOL)
    # 第二段如果沿用全局横轴，会量到竖廊顶端（2.0 m 上限），而不是两侧墙的 1.40 m。
    for item in second_leg:
        assert item.left_distance_m == pytest.approx(1.40, abs=MEASURE_TOL)
        assert item.right_distance_m == pytest.approx(1.40, abs=MEASURE_TOL)


def test_rotation_in_place_path_still_yields_one_sample() -> None:
    """原地转向的目标会产生零长度路径，不能因此判成不可通行。"""
    grid = _corridor_along_x(1.25)
    samples = measure_path_clearance([(1.0, 0.0)], grid)
    assert len(samples) == 1
    assert samples[0].s_m == 0.0
    decision = classify_path_clearance(samples)
    assert decision.decision == DECISION_OK
    assert decision.speed_limit_mps == pytest.approx(0.10)


# ---------------------------------------------------------------------------
# §12.3 未知栅格按障碍处理
# ---------------------------------------------------------------------------


def test_unknown_cells_count_as_obstacles() -> None:
    grid = _grid(-1.0, 6.0, -3.0, 3.0)
    _paint(grid, -1.0, 6.0, -3.0, -1.25, WALL)
    _paint(grid, -1.0, 6.0, 1.25, 3.0, UNKNOWN)
    decision = classify_path_clearance(measure_path_clearance(_straight_path(0.0), grid))
    assert decision.min_left_body_gap_m == pytest.approx(_body_gap(1.25), abs=MEASURE_TOL)
    assert decision.min_right_body_gap_m == pytest.approx(
        _body_gap(1.25), abs=MEASURE_TOL
    )
    assert decision.decision == DECISION_OK


def test_unknown_wall_is_as_blocking_as_a_known_wall() -> None:
    known = _corridor_along_x(1.00)
    unknown = _grid(-1.0, 6.0, -3.0, 3.0)
    _paint(unknown, -1.0, 6.0, -3.0, -1.00, WALL)
    _paint(unknown, -1.0, 6.0, 1.00, 3.0, UNKNOWN)
    a = classify_path_clearance(measure_path_clearance(_straight_path(0.0), known))
    b = classify_path_clearance(measure_path_clearance(_straight_path(0.0), unknown))
    assert a.min_side_gap_m == pytest.approx(b.min_side_gap_m, abs=MEASURE_TOL)
    assert a.speed_limit_mps == pytest.approx(b.speed_limit_mps)
    assert a.decision == b.decision


def test_off_map_coordinates_are_blocked() -> None:
    grid = _corridor_along_x(1.25, x_max=2.0)
    samples = measure_path_clearance([(0.0, 0.0), (10.0, 0.0)], grid)
    inside = [item for item in samples if item.x < 1.9]
    outside = [item for item in samples if item.x > 2.1]
    assert inside and outside
    assert all(item.min_side_gap_m > 0.0 for item in inside)
    assert all(item.min_side_gap_m < 0.0 for item in outside)
    assert classify_path_clearance(samples).decision == DECISION_TOO_NARROW


def test_grid_rejects_non_positive_resolution() -> None:
    grid = CorridorGrid(0.0, 0.0, 0.0, 4, 4, [0] * 16)
    with pytest.raises(ValueError):
        grid.value_at(0.0, 0.0)


# ---------------------------------------------------------------------------
# §7.2 原地转向的扫掠判据
# ---------------------------------------------------------------------------
#
# 现场（2026-10-15 只读搜索）：一个几乎零平移、目标 yaw≈30° 的 local_scan 目标
# 被*平移走廊*判成 TOO_NARROW（min_left/right_body_gap −0.055 / −0.105 m），而
# 那条路径长只有 0.008 m —— 左右射线打的是机器人自己身边的格，与这次转向无关。
#
# 下面这批用例盯两件事：扫掠几何本身对不对，以及它有没有分辨力。每条「通过」的
# 用例都配了一个只改一处就必然被拒的对照 —— 一组判据如果对什么都返 OK，用例也
# 会全绿，那等于没测。

BODY_HALF_LENGTH = HALF_BODY_LENGTH_M + FOOTPRINT_PADDING_M  # 0.39 m
BODY_HALF_WIDTH = HALF_BODY_WIDTH_M + FOOTPRINT_PADDING_M  # 0.255 m


def _open_grid(x_min: float = -3.0, x_max: float = 3.0) -> CorridorGrid:
    """整幅已知空闲的栅格（原点 −3, −3）；图外的坐标按障碍物拒。"""
    return _grid(x_min, x_max, -3.0, 3.0)


def test_open_grid_passes_turns_in_every_direction() -> None:
    """阴性对照：空场地上任何转向都必须通过，否则下面「被拒」的用例说明不了事。"""
    grid = _open_grid()
    for target_deg in (180.0, -180.0, 90.0, -90.0, 30.0, -30.0):
        decision = classify_rotation_sweep(grid, 0.0, 0.0, 0.0, math.radians(target_deg))
        assert decision.applicable, target_deg
        assert decision.decision == DECISION_OK, (target_deg, decision.to_dict())
        assert decision.reject_reason == ""
        assert decision.nearest_blocker_m is None
        assert decision.started_blocked is False


def test_left_front_wall_blocks_the_left_turn_but_not_the_matching_right_turn() -> None:
    """同一个位姿、同一张图、同样 20° 的转角，只有方向不同 —— 结果必须不同。

    墙在机身左前方 0.28~0.30 m（正前方偏左约 65~85°）。机身是 0.78×0.51 的矩形
    而不是圆，所以左转 20° 会把长轴扫过去，右转 20° 反而扫开 —— 现场那个
    +30° 的 local_scan 目标就是这一类。
    """
    grid = _open_grid()
    # Cell squares at y=.30..35 clear the initial padded body (top=.255).
    _paint(grid, 0.01, 0.09, 0.31, 0.34, WALL)

    left = classify_rotation_sweep(grid, 0.0, 0.0, 0.0, math.radians(20.0))
    right = classify_rotation_sweep(grid, 0.0, 0.0, 0.0, math.radians(-20.0))

    assert left.applicable and right.applicable
    assert left.decision == DECISION_ROTATION_BLOCKED, left.to_dict()
    assert left.reject_reason == ROTATION_REASON_BLOCKED
    assert right.decision == DECISION_OK, right.to_dict()

    # 起步那一刻（yaw=0）压不到墙：撞上是在转的过程中。断到 (0, 25°] 才说明测的
    # 是扫掠，而不是「起点就重叠」。
    assert left.started_blocked is False
    first = left.blocked_samples[0]
    assert 0.0 < first["yaw_deg"] <= 25.0
    assert first["cell_value"] == WALL


def test_pose_already_overlapping_the_map_is_flagged_separately() -> None:
    """``started_blocked`` 要把「现在机身就压在障碍上」单拎出来。

    这与「转过去才撞上」是两种现场：前者说明地图跟真实位姿对不齐（或机器人本来
    就贴太近），换个转向方向救不了。
    """
    grid = _open_grid()
    _paint(grid, 0.0, 0.05, 0.20, 0.25, WALL)  # 在 yaw=0 的机身矩形内
    decision = classify_rotation_sweep(grid, 0.0, 0.0, 0.0, math.radians(30.0))
    assert decision.decision == DECISION_ROTATION_BLOCKED
    assert decision.started_blocked is True
    assert decision.blocked_samples[0]["yaw_deg"] == 0.0


def test_unknown_is_blocked_exactly_like_occupancy() -> None:
    """fail-closed：未知区不是可转空间。"""
    free = _open_grid()
    unknown = _open_grid()
    _paint(unknown, -0.10, 0.10, 0.30, 0.60, UNKNOWN)

    assert (
        classify_rotation_sweep(free, 0.0, 0.0, 0.0, math.radians(90.0)).decision
        == DECISION_OK
    )
    as_unknown = classify_rotation_sweep(unknown, 0.0, 0.0, 0.0, math.radians(90.0))
    assert as_unknown.decision == DECISION_ROTATION_BLOCKED
    assert as_unknown.blocked_samples
    assert all(item["cell_value"] == UNKNOWN for item in as_unknown.blocked_samples)


def test_rotating_off_the_map_is_blocked() -> None:
    """地图外同样按障碍物算：同一张图放大一点，同一次转向就该从拒变成过。

    唯一变量是地图边界 —— 图里一格障碍都没有，所以拒的原因只能是「转出去」。
    """
    tight = _grid(-1.0, 1.0, -0.35, 0.35)
    roomy = _grid(-1.0, 1.0, -0.60, 0.60)

    out = classify_rotation_sweep(tight, 0.0, 0.0, 0.0, math.radians(20.0))
    assert out.decision == DECISION_ROTATION_BLOCKED, out.to_dict()
    assert out.started_blocked is False
    assert out.blocked_samples
    assert all(item["cell_value"] is None for item in out.blocked_samples)

    inside = classify_rotation_sweep(roomy, 0.0, 0.0, 0.0, math.radians(20.0))
    assert inside.decision == DECISION_OK, inside.to_dict()


def test_small_yaw_change_reports_not_applicable_rather_than_a_pass() -> None:
    """纯平移不该由扫掠判据下结论：报「不适用」，不是「通过」。"""
    grid = _open_grid()
    for target_deg in (0.0, 3.0, 9.99, -9.99):
        decision = classify_rotation_sweep(grid, 0.0, 0.0, 0.0, math.radians(target_deg))
        assert decision.applicable is False, target_deg
        assert decision.decision == ""
        assert decision.reject_reason == ROTATION_REASON_NOT_APPLICABLE
        assert decision.passable is False
        assert decision.samples == 0
        assert decision.to_dict()["rotation_decision"] == "NOT_APPLICABLE"

    at_threshold = classify_rotation_sweep(grid, 0.0, 0.0, 0.0, math.radians(10.0))
    assert at_threshold.applicable is True
    assert at_threshold.decision == DECISION_OK


def test_turn_direction_uses_the_shortest_signed_angle() -> None:
    """350° → 10° 是 +20°，不是 −340°：方向取反会把余量加到另一侧。"""
    grid = _open_grid()
    forward = classify_rotation_sweep(
        grid, 0.0, 0.0, math.radians(350.0), math.radians(10.0)
    )
    assert forward.applicable
    assert forward.sweep_rad == pytest.approx(math.radians(20.0))
    assert forward.checked_sweep_rad == pytest.approx(math.radians(25.0))
    assert forward.to_dict()["rotation_sweep_deg"] == pytest.approx(20.0, abs=1e-6)
    assert forward.samples <= 10  # 不该出现「绕 340°」的采样数

    backward = classify_rotation_sweep(
        grid, 0.0, 0.0, math.radians(10.0), math.radians(350.0)
    )
    assert backward.sweep_rad == pytest.approx(math.radians(-20.0))
    assert backward.checked_sweep_rad == pytest.approx(math.radians(-25.0))


def test_overshoot_margin_extends_past_the_target_in_the_turn_direction() -> None:
    """余量 5° 加在转向那一侧，而且要真的改变判定。"""
    grid = _open_grid()
    ccw = classify_rotation_sweep(grid, 0.0, 0.0, 0.0, math.radians(90.0))
    cw = classify_rotation_sweep(grid, 0.0, 0.0, 0.0, math.radians(-90.0))
    assert ccw.checked_sweep_rad == pytest.approx(math.radians(95.0))
    assert cw.checked_sweep_rad == pytest.approx(math.radians(-95.0))
    assert ccw.to_dict()["rotation_checked_sweep_deg"] == pytest.approx(95.0)
    assert cw.to_dict()["rotation_checked_sweep_deg"] == pytest.approx(-95.0)

    # 行为上的后果：左上角 0.43 m 处有一格，只有把 [90°, 95°] 这段余量算进去才
    # 扫得到。关掉余量就该判通过 —— 否则说明这格根本不是靠余量判出来的。
    margin_only = _open_grid()
    _paint(margin_only, -.329, -.321, .371, .379, WALL)
    with_margin = classify_rotation_sweep(margin_only, 0.0, 0.0, 0.0, math.radians(90.0))
    without = classify_rotation_sweep(
        margin_only, 0.0, 0.0, 0.0, math.radians(90.0), margin_rad=0.0
    )
    assert without.decision == DECISION_OK, without.to_dict()
    assert with_margin.decision == DECISION_ROTATION_BLOCKED, with_margin.to_dict()


def test_180_degree_uturn_is_sampled_along_the_way_not_only_at_the_ends() -> None:
    """掉头（现场真实用例）扫 185°，中间也要采样，不能只查两个端点。"""
    grid = _open_grid()
    decision = classify_rotation_sweep(grid, 0.0, 0.0, 0.0, math.radians(180.0))
    assert decision.decision == DECISION_OK
    assert decision.sweep_rad == pytest.approx(math.pi)
    assert decision.checked_sweep_rad == pytest.approx(math.radians(185.0))
    assert decision.samples == 63  # 185° / 3° ⇒ 62 段

    behind = _open_grid()
    _paint(behind, -0.10, 0.10, -0.60, -0.36, WALL)
    blocked = classify_rotation_sweep(behind, 0.0, 0.0, 0.0, math.radians(180.0))
    assert blocked.decision == DECISION_ROTATION_BLOCKED
    assert blocked.started_blocked is False  # 墙在 yaw=0 的机身之外
    yaws = [item["yaw_deg"] for item in blocked.blocked_samples]
    assert min(yaws) > 0.0  # 确实是转到中途才撞上


def test_sample_cap_only_bites_for_a_caller_supplied_fine_step() -> None:
    """``max_samples`` 是兜底，不是默认路径上的限制。

    最短有向角最多 180°、加余量 185°，默认 3° 步长下最多 63 个采样点，够不到
    121 的上限；这个上限是留给更细步长的。采样点被截，扫过的角度区间不能被截。
    """
    grid = _open_grid()
    assert classify_rotation_sweep(grid, 0.0, 0.0, 0.0, math.radians(180.0)).samples == 63

    fine = classify_rotation_sweep(
        grid, 0.0, 0.0, 0.0, math.radians(180.0), step_rad=math.radians(0.5)
    )
    assert fine.samples == 121
    assert fine.checked_sweep_rad == pytest.approx(math.radians(185.0))


def test_footprint_cells_follow_the_map_origin_not_the_cell_index() -> None:
    """格心 = origin + (index + 0.5) × resolution，不是 index × resolution。

    漏掉 origin 会让整片扫掠区域按格序号摆到 0 附近 —— 地图原点在 100 m 外时
    整整偏 100 m，而现场地图原点恰好不是 0，所以这条必须单独钉住。
    """
    half_cell = 0.025
    cells = list(_footprint_cells(0.0, 0.0, 0.0, 0.20, 0.10, 0.05, 100.0, 100.0))
    assert cells
    # 原点在 100 m 外，但格心必须还落在机器人身边的矩形里。
    assert all(abs(cx) <= 0.20 + half_cell + 1e-12 for cx, _ in cells)
    assert all(abs(cy) <= 0.10 + half_cell + 1e-12 for _, cy in cells)
    # 半长 0.20 / 半宽 0.10 在 0.05 的格上：格心落在 ±0.175 / ±0.075，各 8 列 4 行。
    # Include squares touching the boundary: 10 columns and 6 rows.
    assert len({round(cx, 3) for cx, _ in cells}) == 10
    assert len({round(cy, 3) for _, cy in cells}) == 6

    # 原点整体挪 0.02 m（不是格距的整数倍）：格心要跟着挪，不能钉在 0 上。
    base = {round(cx, 3) for cx, _ in cells}
    shifted = {
        round(cx, 3)
        for cx, _ in _footprint_cells(0.02, 0.0, 0.0, 0.20, 0.10, 0.05, 100.02, 100.0)
    }
    assert shifted and shifted != base


def test_rotation_decision_serialises_the_provenance_fields() -> None:
    """候选记录要能分辨「走廊判的」还是「扫掠判的」，字段名固定。"""
    grid = _open_grid()
    _paint(grid, -0.10, 0.10, 0.30, 0.60, WALL)
    decision = classify_rotation_sweep(grid, 0.0, 0.0, 0.0, math.radians(60.0))
    payload = decision.to_dict()
    for field in (
        "rotation_decision",
        "rotation_reject_reason",
        "rotation_sweep_deg",
        "rotation_checked_sweep_deg",
        "rotation_yaw_from_deg",
        "rotation_yaw_to_deg",
        "rotation_samples",
        "rotation_blocked_samples",
        "rotation_nearest_blocker_m",
        "rotation_nearest_blocker_bearing_deg",
        "rotation_started_blocked",
    ):
        assert field in payload, field
    assert payload["rotation_decision"] == DECISION_ROTATION_BLOCKED
    assert payload["rotation_reject_reason"] == ROTATION_REASON_BLOCKED
    assert payload["rotation_checked_sweep_deg"] == pytest.approx(65.0)
    # 方位角量的是障碍物相对机器人的朝向（墙在正前方偏左），不是当时转到的偏航角。
    assert payload["rotation_nearest_blocker_bearing_deg"] == pytest.approx(
        90.0, abs=30.0
    )


def test_occupied_square_intersecting_body_is_blocked_even_when_center_is_outside():
    grid = _open_grid()
    # Cell y=.25..30 intersects padded body y<=.255; center .275 is outside.
    _paint(grid, .021, .029, .271, .279, WALL)
    result = classify_rotation_sweep(grid, 0, 0, 0, math.radians(-20))
    assert result.decision == DECISION_ROTATION_BLOCKED
    assert result.started_blocked


def test_near_in_place_translation_envelope_cannot_be_omitted():
    grid = _open_grid()
    _paint(grid, .511, .539, .011, .039, WALL)
    pure = classify_rotation_sweep(grid, 0, 0, 0, math.radians(30))
    translated = classify_rotation_sweep(grid, 0, 0, 0, math.radians(30), translation_radius_m=.09)
    assert pure.passable
    assert not translated.passable
    assert not translated.started_blocked


def test_coarse_sample_cap_covers_obstacles_between_endpoint_footprints():
    grid = _open_grid()
    _paint(grid, .321, .329, .321, .329, WALL)
    # At 0 and 90 degrees the cell is outside; at 45 it is inside the body.
    result = classify_rotation_sweep(grid, 0, 0, 0, math.pi/2, margin_rad=0, max_samples=2)
    assert not result.passable
