"""计划书 §7：走廊宽度识别与适配。

只做三件事：沿 Nav2 返回的路径逐点测左右净空、按最小单侧机身净空判定这条
路径能不能过、以及算出该段路径允许的最高线速度。刻意不修改机器人
footprint、``inflation_radius`` 和 Collision Monitor 停止区 —— 那些是全局常量，
导航中途改它们只会让路径抖动。

UNKNOWN 一律按障碍物处理（fail-closed）：地图上没探到的地方不能当成空走廊。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Sequence

# 计划书 §7.3 / §8 固定的几何常量。
HALF_BODY_WIDTH_M = 0.215
# 站立外廓 0.70 m 长，绕 URDF base 原点对称 ⇒ 半长 0.35 m。与
# ``nav2_params_humble.yaml`` 的 ``footprint: [[0.35, 0.215], ...]`` 同源。
HALF_BODY_LENGTH_M = 0.35
FOOTPRINT_PADDING_M = 0.04
OCCUPIED_THRESHOLD = 65
MAX_LATERAL_SEARCH_M = 2.0
SAMPLE_STEP_M = 0.20
EDGE_TRIM_M = 0.20

# 计划书 §7.4 固定的安全规则。
HARD_SIDE_GAP_M = 0.40
PREFERRED_SIDE_GAP_M = 0.80
MIN_SPEED_MPS = 0.05
MAX_SPEED_MPS = 0.10

DECISION_OK = "OK"
DECISION_TOO_NARROW = "TOO_NARROW"

# A separately authorized collision diagnostic may start with a cable or other
# known return in the *rear* padding shell.  Field-frontier execution has a
# second, explicit scope for the one-cell UNKNOWN rear contact crescent caused
# by chassis occlusion.  Both require a short, nearly straight, monotonic
# forward path; every other unknown cell remains fail-closed.
FORWARD_EGRESS_MAX_M = 0.30
FORWARD_EGRESS_MAX_LATERAL_M = 0.03
FORWARD_EGRESS_START_BACKTRACK_M = 0.03
FORWARD_EGRESS_MAX_SEGMENT_ANGLE_RAD = math.radians(15.0)

# ---------------------------------------------------------------------------
# 原地转向的扫掠几何（计划书 §7.2「转向不能用平移走廊冒充」）
# ---------------------------------------------------------------------------
#
# 平移走廊量的是「沿切线左右各能走多远」，它**不是**转向判据：机器人原地转 30°
# 时左右两侧的 0.4 m 走廊是无关量，真正会撞的是前/后角点在旋转中扫过的那一圈。
# 2026-10-15 现场：一个几乎零平移、目标 yaw≈30° 的 local_scan 目标被平移走廊判成
# TOO_NARROW（min_left/right_body_gap −0.055 / −0.105 m），而那条路径长只有
# 0.008 m —— 左右两侧的射线打的是机器人自己身边的格，跟这次转向没有关系。
#
# 位移小于这个值就按「原地转向」处理：转向判据接管，平移走廊不再参与判定。
ROTATION_TRANSLATION_TOL_M = 0.10
# 偏航变化小于这个值不值得单开一遍扫掠检查（纯平移）。
ROTATION_MIN_SWEEP_RAD = math.radians(10.0)
# 扫掠角度的采样步长。3° 时外角点最多移动 0.41 m × 0.052 ≈ 0.021 m，小于半格。
ROTATION_SWEEP_STEP_RAD = math.radians(3.0)
# 控制器过冲与停车延迟的余量。现场实测回起点转向有 ±3° 量级的过冲，这里取 5°。
ROTATION_SWEEP_MARGIN_RAD = math.radians(5.0)
# 单次检查的采样上限；360° 全周在 3° 步长下是 121 个，正好卡住。
ROTATION_SWEEP_MAX_SAMPLES = 121

DECISION_ROTATION_BLOCKED = "ROTATION_SWEEP_BLOCKED"
ROTATION_REASON_BLOCKED = "ROTATION_SWEEP_BLOCKED"
ROTATION_REASON_NOT_APPLICABLE = "ROTATION_SWEEP_NOT_APPLICABLE"

# 候选评分的对称净空权重（计划书 §7.5，固定 0.25）。
CENTER_SCORE_WEIGHT = 0.25


@dataclass(frozen=True)
class CorridorGrid:
    """Nav2 OccupancyGrid 的最小只读视图。

    ``data`` 为行主序整数，``-1`` 表示未知；``value_at`` 返回 ``None`` 表示
    坐标落在地图之外（同样按障碍物处理）。
    """

    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int
    data: Sequence[int]

    def value_at(self, x: float, y: float) -> int | None:
        if self.resolution <= 0.0:
            raise ValueError("resolution 必须大于 0")
        column = int(math.floor((float(x) - self.origin_x) / self.resolution))
        row = int(math.floor((float(y) - self.origin_y) / self.resolution))
        if column < 0 or row < 0 or column >= self.width or row >= self.height:
            return None
        index = row * self.width + column
        if index >= len(self.data):
            return None
        return int(self.data[index])

    def blocked(self, x: float, y: float) -> bool:
        value = self.value_at(x, y)
        if value is None:
            return True
        if value < 0:
            return True
        return value >= OCCUPIED_THRESHOLD


@dataclass(frozen=True)
class CurrentFootprintObservedGrid:
    """Read-only grid view that reconciles stale cells under the robot body.

    A robot that has a valid current pose physically occupies its own outer
    body rectangle.  Occupied/unknown cells whose *centres* lie inside that
    rectangle are therefore stale self-observations or map drift, not a fixed
    obstacle the body can currently be intersecting.  Mask only that already
    occupied volume.  The footprint padding outside the body, every future
    swept cell, Nav2 costmaps, live scan gates and Collision Monitor remain
    unchanged.
    """

    source: CorridorGrid
    robot_x: float
    robot_y: float
    robot_yaw: float
    half_length_m: float = HALF_BODY_LENGTH_M
    half_width_m: float = HALF_BODY_WIDTH_M

    @property
    def resolution(self) -> float:
        return self.source.resolution

    @property
    def origin_x(self) -> float:
        return self.source.origin_x

    @property
    def origin_y(self) -> float:
        return self.source.origin_y

    @property
    def width(self) -> int:
        return self.source.width

    @property
    def height(self) -> int:
        return self.source.height

    def _inside_current_body(self, x: float, y: float) -> bool:
        dx = float(x) - self.robot_x
        dy = float(y) - self.robot_y
        cosine = math.cos(self.robot_yaw)
        sine = math.sin(self.robot_yaw)
        local_x = dx * cosine + dy * sine
        local_y = -dx * sine + dy * cosine
        return abs(local_x) <= self.half_length_m and abs(local_y) <= self.half_width_m

    def value_at(self, x: float, y: float) -> int | None:
        if self._inside_current_body(x, y):
            return 0
        return self.source.value_at(x, y)

    def blocked(self, x: float, y: float) -> bool:
        value = self.value_at(x, y)
        return value is None or value < 0 or value >= OCCUPIED_THRESHOLD


def current_footprint_observed_grid(
    grid: CorridorGrid, x: float, y: float, yaw_rad: float
) -> tuple[CurrentFootprintObservedGrid, dict[str, Any]]:
    """Return the bounded start-footprint overlay and auditable mask counts."""
    values = (float(x), float(y), float(yaw_rad))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("current footprint pose must be finite")
    overlay = CurrentFootprintObservedGrid(grid, *values)
    masked_occupied = 0
    masked_unknown = 0
    masked_cells = 0
    for cell_x, cell_y in _footprint_cells(
        values[0], values[1], values[2], HALF_BODY_LENGTH_M, HALF_BODY_WIDTH_M,
        grid.resolution, grid.origin_x, grid.origin_y,
    ):
        if not overlay._inside_current_body(cell_x, cell_y):
            continue
        value = grid.value_at(cell_x, cell_y)
        if value is None or value < 0:
            masked_unknown += 1
            masked_cells += 1
        elif value >= OCCUPIED_THRESHOLD:
            masked_occupied += 1
            masked_cells += 1
    return overlay, {
        "policy": "current_physical_body_only",
        "half_length_m": HALF_BODY_LENGTH_M,
        "half_width_m": HALF_BODY_WIDTH_M,
        "footprint_padding_masked": False,
        "masked_cells": masked_cells,
        "masked_occupied_cells": masked_occupied,
        "masked_unknown_cells": masked_unknown,
    }


def clear_current_physical_footprint_data(
    data: Sequence[int], *, width: int, height: int, resolution: float,
    origin_x: float, origin_y: float, robot_x: float, robot_y: float,
    robot_yaw: float,
) -> tuple[list[int], dict[str, Any]]:
    """Clear only grid-cell centres proven to lie under the current body.

    This is the publishable counterpart of :func:`current_footprint_observed_grid`.
    It deliberately leaves the footprint padding ring untouched.
    """
    grid = CorridorGrid(
        resolution=float(resolution), origin_x=float(origin_x), origin_y=float(origin_y),
        width=int(width), height=int(height), data=data,
    )
    if len(data) != grid.width * grid.height:
        raise ValueError("current footprint grid size mismatch")
    overlay, summary = current_footprint_observed_grid(
        grid, float(robot_x), float(robot_y), float(robot_yaw)
    )
    result = [int(value) for value in data]
    cleared_occupied = 0
    cleared_unknown = 0
    for cell_x, cell_y in _footprint_cells(
        overlay.robot_x, overlay.robot_y, overlay.robot_yaw,
        HALF_BODY_LENGTH_M, HALF_BODY_WIDTH_M,
        grid.resolution, grid.origin_x, grid.origin_y,
    ):
        if not overlay._inside_current_body(cell_x, cell_y):
            continue
        column = int(math.floor((cell_x - grid.origin_x) / grid.resolution))
        row = int(math.floor((cell_y - grid.origin_y) / grid.resolution))
        if not (0 <= column < grid.width and 0 <= row < grid.height):
            continue
        index = row * grid.width + column
        value = result[index]
        if value >= OCCUPIED_THRESHOLD:
            cleared_occupied += 1
        elif value < 0:
            cleared_unknown += 1
        else:
            continue
        result[index] = 0
    summary = dict(summary)
    summary.update(
        published_cells_cleared=cleared_occupied + cleared_unknown,
        published_occupied_cells_cleared=cleared_occupied,
        published_unknown_cells_cleared=cleared_unknown,
    )
    return result, summary


@dataclass(frozen=True)
class PathClearanceSample:
    s_m: float
    x: float
    y: float
    left_distance_m: float
    right_distance_m: float
    corridor_width_m: float
    left_body_gap_m: float
    right_body_gap_m: float
    min_side_gap_m: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "s_m": round(self.s_m, 3),
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "left_distance_m": round(self.left_distance_m, 3),
            "right_distance_m": round(self.right_distance_m, 3),
            "corridor_width_m": round(self.corridor_width_m, 3),
            "left_body_gap_m": round(self.left_body_gap_m, 3),
            "right_body_gap_m": round(self.right_body_gap_m, 3),
            "min_side_gap_m": round(self.min_side_gap_m, 3),
        }


@dataclass(frozen=True)
class CorridorDecision:
    decision: str
    reject_reason: str
    min_side_gap_m: float | None
    min_left_body_gap_m: float | None
    min_right_body_gap_m: float | None
    corridor_width_m: float | None
    center_error_m: float | None
    center_score: float
    speed_limit_mps: float
    samples: tuple[PathClearanceSample, ...] = ()

    @property
    def passable(self) -> bool:
        return self.decision == DECISION_OK

    def to_dict(self) -> dict[str, Any]:
        return {
            "corridor_decision": self.decision,
            "reject_reason": self.reject_reason,
            "min_side_gap_m": _round(self.min_side_gap_m),
            "min_left_body_gap_m": _round(self.min_left_body_gap_m),
            "min_right_body_gap_m": _round(self.min_right_body_gap_m),
            "corridor_width_m": _round(self.corridor_width_m),
            "center_error_m": _round(self.center_error_m),
            "center_score": round(self.center_score, 4),
            "speed_limit_mps": round(self.speed_limit_mps, 4),
            "samples": len(self.samples),
        }


def speed_limit_from_clearance(min_side_gap_m: float | None) -> float:
    """计划书 §7.4 的唯一速度公式。

    ``speed = clamp(0.05 + (gap - 0.40) / 0.40 * 0.05, 0.05, 0.10)``。
    低于硬净空返回 0（该路径本来就该被拒）。
    """
    if min_side_gap_m is None:
        return 0.0
    gap = float(min_side_gap_m)
    if gap < HARD_SIDE_GAP_M:
        return 0.0
    speed = MIN_SPEED_MPS + (gap - HARD_SIDE_GAP_M) / (
        PREFERRED_SIDE_GAP_M - HARD_SIDE_GAP_M
    ) * (MAX_SPEED_MPS - MIN_SPEED_MPS)
    return max(MIN_SPEED_MPS, min(MAX_SPEED_MPS, speed))


def center_score(left_body_gap_m: float, right_body_gap_m: float) -> float:
    """计划书 §7.5：路径离走廊中线有多近，1 表示正中。"""
    width = max(0.0, float(left_body_gap_m)) + max(0.0, float(right_body_gap_m))
    error = abs(float(left_body_gap_m) - float(right_body_gap_m))
    return max(0.0, 1.0 - error / max(width, 0.01))


def path_center_score(
    path: Sequence[Any], occupancy_grid: CorridorGrid
) -> float | None:
    """计划书 §7.5：候选路径的对称净空分，用来在多个**已经安全**的候选之间排序。

    取整条路径上最窄的那个采样点算对称性 —— 与 ``classify_path_clearance``
    取最小单侧净空是同一个约束点，两处不会各说各话。

    两种情况返回 ``None``，都按「没有这条证据」处理，绝不编造一个数：

    * 采不到点（路径退化，或整段落在地图外）；
    * 最窄处的单侧机身净空低于硬值 —— 过不去的地方没有「居中」可言。这条特别
      要命：完全被堵死时左右光线在第一格就被挡住，两侧"距离"相等，照公式算出
      来会是满分 1.0。§7.5 写明 center_score 只用于在多个**已经安全**的候选之间
      排序、不允许覆盖「路径过窄」判定，所以这里直接不给分。

    注意这**不是** ``classify_path_clearance`` 的行为：决策记录里对过窄路径照样
    记一个 center_score（那是给人看的证据），只有候选排序这一路才需要 None。
    """
    samples = measure_path_clearance(path, occupancy_grid)
    if not samples:
        return None
    binding = min(samples, key=lambda item: item.min_side_gap_m)
    if binding.min_side_gap_m < HARD_SIDE_GAP_M:
        return None
    return center_score(binding.left_body_gap_m, binding.right_body_gap_m)


def measure_path_clearance(
    path: Sequence[Any],
    occupancy_grid: CorridorGrid,
) -> list[PathClearanceSample]:
    """沿路径每 ``0.20 m`` 采样一次，量左右到障碍的距离。

    ``path`` 接受 ``(x, y)`` 序列，也接受带 ``x`` / ``y`` 属性的对象或字典
    （Nav2 的 ``PoseStamped`` 序列解包后即可）。
    """
    points = [_xy(item) for item in path]
    if not points:
        return []
    samples = _sample_polyline(points, SAMPLE_STEP_M)
    total_length = samples[-1][0] if samples else 0.0
    if total_length > 2.0 * EDGE_TRIM_M:
        trimmed = [
            item for item in samples if EDGE_TRIM_M <= item[0] <= total_length - EDGE_TRIM_M
        ]
        if trimmed:
            samples = trimmed
    result: list[PathClearanceSample] = []
    for s_m, x, y, heading in samples:
        left, right = _lateral_distances(x, y, heading, occupancy_grid)
        left_gap = left - HALF_BODY_WIDTH_M - FOOTPRINT_PADDING_M
        right_gap = right - HALF_BODY_WIDTH_M - FOOTPRINT_PADDING_M
        result.append(
            PathClearanceSample(
                s_m=s_m,
                x=x,
                y=y,
                left_distance_m=left,
                right_distance_m=right,
                corridor_width_m=left + right,
                left_body_gap_m=left_gap,
                right_body_gap_m=right_gap,
                min_side_gap_m=min(left_gap, right_gap),
            )
        )
    return result


def classify_path_clearance(samples: Sequence[PathClearanceSample]) -> CorridorDecision:
    """沿整条路径取最小单侧机身净空判定；找不到采样点时 fail-closed。"""
    if not samples:
        return CorridorDecision(
            decision=DECISION_TOO_NARROW,
            reject_reason="NO_PATH_SAMPLES",
            min_side_gap_m=None,
            min_left_body_gap_m=None,
            min_right_body_gap_m=None,
            corridor_width_m=None,
            center_error_m=None,
            center_score=0.0,
            speed_limit_mps=0.0,
            samples=(),
        )
    binding = min(samples, key=lambda item: item.min_side_gap_m)
    min_side_gap = binding.min_side_gap_m
    score = center_score(binding.left_body_gap_m, binding.right_body_gap_m)
    if min_side_gap < HARD_SIDE_GAP_M:
        return CorridorDecision(
            decision=DECISION_TOO_NARROW,
            reject_reason="TOO_NARROW",
            min_side_gap_m=min_side_gap,
            min_left_body_gap_m=binding.left_body_gap_m,
            min_right_body_gap_m=binding.right_body_gap_m,
            corridor_width_m=binding.corridor_width_m,
            center_error_m=abs(binding.left_body_gap_m - binding.right_body_gap_m),
            center_score=score,
            speed_limit_mps=0.0,
            samples=tuple(samples),
        )
    return CorridorDecision(
        decision=DECISION_OK,
        reject_reason="",
        min_side_gap_m=min_side_gap,
        min_left_body_gap_m=binding.left_body_gap_m,
        min_right_body_gap_m=binding.right_body_gap_m,
        corridor_width_m=binding.corridor_width_m,
        center_error_m=abs(binding.left_body_gap_m - binding.right_body_gap_m),
        center_score=score,
        speed_limit_mps=speed_limit_from_clearance(min_side_gap),
        samples=tuple(samples),
    )


@dataclass(frozen=True)
class RotationSweepDecision:
    """原地/近原地转向的扫掠检查结果（计划书 §7.2）。

    ``nearest_blocker_deg`` 是障碍物相对机器人的**方位角**（map 系，atan2），
    不是转到的偏航角；``blocked_samples`` 里每条同时带 ``yaw_deg``（扫到哪一步）
    与坐标，两者别混用。

    ``started_blocked`` 单独标出「当前偏航下机身就已经压在障碍/未知格上」——
    这与「转过去才撞上」是两种完全不同的现场：前者说明地图与真实位姿对不齐
    （或机器人本来就贴得太近），转哪个方向都救不了，不是选个方向就能绕开的。
    """

    decision: str
    reject_reason: str
    applicable: bool
    yaw_from_rad: float
    yaw_to_rad: float
    sweep_rad: float
    checked_sweep_rad: float
    samples: int
    blocked_samples: tuple[dict[str, Any], ...] = ()
    nearest_blocker_m: float | None = None
    nearest_blocker_deg: float | None = None
    started_blocked: bool = False

    @property
    def passable(self) -> bool:
        return self.applicable and self.decision == DECISION_OK

    def to_dict(self) -> dict[str, Any]:
        return {
            "rotation_decision": self.decision if self.applicable else "NOT_APPLICABLE",
            "rotation_reject_reason": self.reject_reason,
            "rotation_sweep_deg": round(math.degrees(self.sweep_rad), 2),
            "rotation_checked_sweep_deg": round(math.degrees(self.checked_sweep_rad), 2),
            "rotation_yaw_from_deg": round(math.degrees(self.yaw_from_rad), 2),
            "rotation_yaw_to_deg": round(math.degrees(self.yaw_to_rad), 2),
            "rotation_samples": self.samples,
            "rotation_blocked_samples": len(self.blocked_samples),
            "rotation_nearest_blocker_m": _round(self.nearest_blocker_m),
            "rotation_nearest_blocker_bearing_deg": _round(self.nearest_blocker_deg),
            "rotation_started_blocked": bool(self.started_blocked),
        }


@dataclass(frozen=True)
class ForwardPaddingEgressDecision:
    """Decision for leaving a known obstacle in the rear padding shell.

    The physical body is never masked here.  Every blocking cell must be a
    known occupied cell strictly behind the body's rear plane and within its
    width.  A separately selected field-frontier policy may also admit UNKNOWN
    cells only in the quantized rear contact shell.  The planned centre line
    must then move forward monotonically with tightly bounded lateral and
    angular deviation.
    """

    allowed: bool
    reason: str
    blocking_cells: tuple[dict[str, Any], ...] = ()
    forward_m: float = 0.0
    maximum_lateral_m: float = 0.0
    maximum_segment_angle_rad: float = 0.0
    unknown_rear_allowed: bool = False
    rear_corner_padding_allowed: bool = False

    def to_dict(self) -> dict[str, Any]:
        return {
            "allowed": bool(self.allowed),
            "reason": self.reason,
            "blocking_cells": list(self.blocking_cells),
            "forward_m": round(self.forward_m, 3),
            "maximum_lateral_m": round(self.maximum_lateral_m, 3),
            "maximum_segment_angle_deg": round(
                math.degrees(self.maximum_segment_angle_rad), 2
            ),
            "policy": (
                "rear_contact_shell_forward_egress"
                if self.unknown_rear_allowed
                else (
                    "known_rear_corner_padding_forward_egress_only"
                    if self.rear_corner_padding_allowed
                    else "known_rear_padding_forward_egress_only"
                )
            ),
        }


def classify_forward_padding_egress(
    grid: CorridorGrid,
    x: float,
    y: float,
    yaw_rad: float,
    path: Sequence[Any],
    *,
    max_distance_m: float = FORWARD_EGRESS_MAX_M,
    max_lateral_m: float = FORWARD_EGRESS_MAX_LATERAL_M,
    allow_unknown_rear: bool = False,
    allow_known_rear_corner_padding: bool = False,
    rear_corner_quantization_m: float = 0.0,
    max_segment_angle_rad: float = FORWARD_EGRESS_MAX_SEGMENT_ANGLE_RAD,
) -> ForwardPaddingEgressDecision:
    """Allow only a short straight path that moves away from rear padding.

    ``grid`` is expected to be the current-body overlay, so stale cells under
    the physical body have already been reconciled.  By default unknown and
    off-map cells reject.  ``allow_unknown_rear`` admits UNKNOWN only where a
    map cell touches the rear body boundary after accounting for one-cell
    quantization; off-map, known side/front obstacles, reverse motion and
    curved escape paths still reject.  The backend owns that policy scope.
    """
    values = (float(x), float(y), float(yaw_rad), float(max_distance_m),
              float(max_lateral_m), float(rear_corner_quantization_m),
              float(max_segment_angle_rad))
    if (not all(math.isfinite(value) for value in values)
            or max_distance_m <= 0 or max_lateral_m < 0
            or not 0 <= rear_corner_quantization_m <= grid.resolution + 1e-9
            or not 0 < max_segment_angle_rad <= math.pi / 2):
        raise ValueError("invalid forward padding egress geometry")
    points = [_xy(item) for item in path]
    if len(points) < 2 or not all(
        math.isfinite(value) for point in points for value in point
    ):
        return ForwardPaddingEgressDecision(False, "path_missing_or_nonfinite")

    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)

    def local(px: float, py: float) -> tuple[float, float]:
        dx, dy = px - x, py - y
        return dx * cosine + dy * sine, -dx * sine + dy * cosine

    local_path = [local(px, py) for px, py in points]
    forward = local_path[-1][0]
    maximum_lateral = max(abs(item[1]) for item in local_path)
    if not 0.10 < forward <= max_distance_m + 1e-9:
        return ForwardPaddingEgressDecision(
            False, "forward_distance_out_of_range", forward_m=forward,
            maximum_lateral_m=maximum_lateral,
        )
    if maximum_lateral > max_lateral_m + 1e-9:
        return ForwardPaddingEgressDecision(
            False, "path_not_straight", forward_m=forward,
            maximum_lateral_m=maximum_lateral,
        )
    previous = -FORWARD_EGRESS_START_BACKTRACK_M
    maximum_angle = 0.0
    for index, (path_forward, _path_lateral) in enumerate(local_path):
        if path_forward < -FORWARD_EGRESS_START_BACKTRACK_M - 1e-9:
            return ForwardPaddingEgressDecision(
                False, "path_starts_too_far_back", forward_m=forward,
                maximum_lateral_m=maximum_lateral,
            )
        if path_forward + 1e-9 < previous:
            return ForwardPaddingEgressDecision(
                False, "path_does_not_move_monotonically_forward", forward_m=forward,
                maximum_lateral_m=maximum_lateral,
            )
        previous = path_forward
        if index == 0:
            continue
        dx = local_path[index][0] - local_path[index - 1][0]
        dy = local_path[index][1] - local_path[index - 1][1]
        span = math.hypot(dx, dy)
        # Very short NavFn endpoint connectors are grid quantization, not a
        # commanded curve.  Longer segments must stay within the forward cone.
        if span >= grid.resolution * 0.75:
            angle = abs(math.atan2(dy, dx))
            maximum_angle = max(maximum_angle, angle)
            if dx <= 0 or angle > max_segment_angle_rad:
                return ForwardPaddingEgressDecision(
                    False, "path_segment_leaves_forward_cone", forward_m=forward,
                    maximum_lateral_m=maximum_lateral,
                    maximum_segment_angle_rad=maximum_angle,
                )

    blocked: list[dict[str, Any]] = []
    seen: set[tuple[float, float]] = set()
    padded_l = HALF_BODY_LENGTH_M + FOOTPRINT_PADDING_M
    padded_w = HALF_BODY_WIDTH_M + FOOTPRINT_PADDING_M
    for cell_x, cell_y in _footprint_cells(
        x, y, yaw_rad, padded_l, padded_w,
        grid.resolution, grid.origin_x, grid.origin_y,
    ):
        key = (cell_x, cell_y)
        if key in seen:
            continue
        seen.add(key)
        if not grid.blocked(cell_x, cell_y):
            continue
        value = grid.value_at(cell_x, cell_y)
        local_x, local_y = local(cell_x, cell_y)
        item = {
            "x": round(cell_x, 3), "y": round(cell_y, 3),
            "local_x": round(local_x, 3), "local_y": round(local_y, 3),
            "cell_value": None if value is None else int(value),
        }
        blocked.append(item)
        if value is None:
            return ForwardPaddingEgressDecision(
                False, "rear_padding_contains_off_map",
                tuple(blocked[:8]), forward, maximum_lateral, maximum_angle,
                bool(allow_unknown_rear),
            )
        if value < OCCUPIED_THRESHOLD:
            rear_contact = (
                local_x < -HALF_BODY_LENGTH_M + grid.resolution + 1e-9
                and abs(local_y) <= (
                    HALF_BODY_WIDTH_M + FOOTPRINT_PADDING_M + grid.resolution
                ) + 1e-9
            )
            if not (allow_unknown_rear and value == -1 and rear_contact):
                return ForwardPaddingEgressDecision(
                    False, "rear_padding_contains_unknown_or_off_map",
                    tuple(blocked[:8]), forward, maximum_lateral, maximum_angle,
                    bool(allow_unknown_rear),
                )
            continue
        rear_lateral_limit = HALF_BODY_WIDTH_M + (
            FOOTPRINT_PADDING_M if allow_known_rear_corner_padding else 0.0
        ) + rear_corner_quantization_m
        if local_x >= -HALF_BODY_LENGTH_M or abs(local_y) > rear_lateral_limit:
            return ForwardPaddingEgressDecision(
                False, "blocker_not_strictly_behind_body",
                tuple(blocked[:8]), forward, maximum_lateral, maximum_angle,
                bool(allow_unknown_rear),
            )
    if not blocked:
        return ForwardPaddingEgressDecision(
            False, "no_rear_padding_blocker", (), forward, maximum_lateral,
            maximum_angle, bool(allow_unknown_rear)
        )
    return ForwardPaddingEgressDecision(
        True, (
            "rear_contact_shell_and_monotonic_forward_path"
            if allow_unknown_rear
            else "known_rear_padding_blocker_and_monotonic_forward_path"
        ), tuple(blocked[:8]), forward, maximum_lateral, maximum_angle,
        bool(allow_unknown_rear), bool(allow_known_rear_corner_padding),
    )


def classify_rotation_sweep(
    grid: CorridorGrid,
    x: float,
    y: float,
    yaw_from_rad: float,
    yaw_to_rad: float,
    *,
    half_length_m: float = HALF_BODY_LENGTH_M,
    half_width_m: float = HALF_BODY_WIDTH_M,
    padding_m: float = FOOTPRINT_PADDING_M,
    margin_rad: float = ROTATION_SWEEP_MARGIN_RAD,
    step_rad: float = ROTATION_SWEEP_STEP_RAD,
    max_samples: int = ROTATION_SWEEP_MAX_SAMPLES,
    translation_radius_m: float = 0.0,
    minimum_sweep_rad: float = ROTATION_MIN_SWEEP_RAD,
) -> RotationSweepDecision:
    """检查机器人绕 ``(x, y)`` 从 ``yaw_from_rad`` 转到 ``yaw_to_rad`` 的扫掠区域。

    扫掠区域 = 带外扩的机身矩形在整个偏航区间上的并集。逐偏航采样把矩形栅格化，
    任何一格是占据、未知或地图外（``CorridorGrid.blocked`` 的 fail-closed 语义）
    就判不可转。**未知区不会被自动当成 free**，与平移走廊同一条红线。

    转向方向取最短有向角，所以 350°→10° 是 +20° 而不是 −340°。偏航变化小于
    ``ROTATION_MIN_SWEEP_RAD`` 时返回 ``applicable=False``（纯平移，不该由本函数
    下结论）；判定仍留给调用方，这里不替它说「通过」。
    """
    if (not all(math.isfinite(v) for v in (x, y, yaw_from_rad, yaw_to_rad,
            half_length_m, half_width_m, padding_m, margin_rad, step_rad, translation_radius_m, minimum_sweep_rad))
            or half_length_m <= 0 or half_width_m <= 0 or padding_m < 0
            or translation_radius_m < 0 or minimum_sweep_rad < 0 or step_rad <= 0 or margin_rad < 0 or max_samples < 2):
        raise ValueError('invalid rotation sweep geometry')
    delta = _shortest_angle_delta(yaw_from_rad, yaw_to_rad)
    if abs(delta) < minimum_sweep_rad:
        return RotationSweepDecision(
            decision="",
            reject_reason=ROTATION_REASON_NOT_APPLICABLE,
            applicable=False,
            yaw_from_rad=float(yaw_from_rad),
            yaw_to_rad=float(yaw_to_rad),
            sweep_rad=float(delta),
            checked_sweep_rad=0.0,
            samples=0,
        )

    # 过冲余量按转向方向延伸，不改变转向方向本身。
    checked = delta + math.copysign(margin_rad, delta)
    steps = max(1, int(math.ceil(abs(checked) / max(1e-6, step_rad))))
    if steps + 1 > max_samples:
        # Bound work; the angular displacement envelope below covers the larger gaps.
        steps = max(1, max_samples - 1)
    base_l = float(half_length_m) + float(padding_m)
    base_w = float(half_width_m) + float(padding_m)
    # Every intermediate orientation lies within half a sampling interval of
    # an endpoint. Inflate by its maximum corner displacement, so an obstacle
    # cannot fall between samples. The near-in-place path may also translate:
    # cover the full bounded center excursion rather than treating it as zero.
    angular_margin = 2 * math.hypot(base_l, base_w) * math.sin(min(math.pi, abs(checked)/steps/2)/2)
    half_l = base_l + translation_radius_m + angular_margin
    half_w = base_w + translation_radius_m + angular_margin

    blocked: list[dict[str, Any]] = []
    nearest: float | None = None
    nearest_deg: float | None = None
    started_blocked = any(grid.blocked(cx, cy) for cx, cy in _footprint_cells(
        x, y, yaw_from_rad, base_l, base_w, grid.resolution, grid.origin_x, grid.origin_y))
    for index in range(steps + 1):
        theta = float(yaw_from_rad) + checked * (index / steps)
        for cell_x, cell_y in _footprint_cells(
            x, y, theta, half_l, half_w, grid.resolution, grid.origin_x, grid.origin_y
        ):
            if not grid.blocked(cell_x, cell_y):
                continue
            distance = math.hypot(cell_x - x, cell_y - y)
            if nearest is None or distance < nearest:
                nearest = distance
                # 记的是**障碍物相对机器人的方位角**（map 系），不是当时转到的
                # 偏航角。两个数在正常机器上完全不同，混淆了会让人以为机器人
                # 转到 30° 才撞，其实障碍物一直在正前方。
                nearest_deg = math.degrees(math.atan2(cell_y - y, cell_x - x))
            if len(blocked) < 8:
                value = grid.value_at(cell_x, cell_y)
                blocked.append(
                    {
                        "yaw_deg": round(math.degrees(theta), 2),
                        "x": round(cell_x, 3),
                        "y": round(cell_y, 3),
                        "cell_value": None if value is None else int(value),
                        "distance_m": round(distance, 3),
                    }
                )
    if blocked:
        return RotationSweepDecision(
            decision=DECISION_ROTATION_BLOCKED,
            reject_reason=ROTATION_REASON_BLOCKED,
            applicable=True,
            yaw_from_rad=float(yaw_from_rad),
            yaw_to_rad=float(yaw_to_rad),
            sweep_rad=float(delta),
            checked_sweep_rad=float(checked),
            samples=steps + 1,
            blocked_samples=tuple(blocked),
            nearest_blocker_m=nearest,
            nearest_blocker_deg=nearest_deg,
            started_blocked=started_blocked,
        )
    return RotationSweepDecision(
        decision=DECISION_OK,
        reject_reason="",
        applicable=True,
        yaw_from_rad=float(yaw_from_rad),
        yaw_to_rad=float(yaw_to_rad),
        sweep_rad=float(delta),
        checked_sweep_rad=float(checked),
        samples=steps + 1,
        nearest_blocker_m=None,
        nearest_blocker_deg=None,
    )


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def _shortest_angle_delta(yaw_from_rad: float, yaw_to_rad: float) -> float:
    """把两个偏航角之差折到 (−π, π]，取最短有向角。"""
    return math.remainder(float(yaw_to_rad) - float(yaw_from_rad), 2.0 * math.pi)


def _footprint_cells(
    x: float,
    y: float,
    theta: float,
    half_l: float,
    half_w: float,
    resolution: float,
    origin_x: float,
    origin_y: float,
):
    """Yield centers of cells intersecting the rotated footprint, using SAT.

    Occupancy describes an entire square. Testing only its center misses a
    corner/edge collision. The four separating axes are world X/Y and body X/Y.

    格心必须按地图原点算：``CorridorGrid`` 的格心在
    ``origin + (index + 0.5) * resolution``，忘了 ``origin`` 会让整张扫掠区域整体
    平移，是最容易写错也最难看出来的一处。
    """
    step = float(resolution)
    if step <= 0.0:
        raise ValueError("resolution 必须大于 0")
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    extent_x = abs(half_l * cos_t) + abs(half_w * sin_t)
    extent_y = abs(half_l * sin_t) + abs(half_w * cos_t)
    cell_half = step / 2
    local_cell_extent = cell_half * (abs(cos_t) + abs(sin_t))
    col_lo = int(math.floor((x - extent_x - origin_x) / step)) - 1
    col_hi = int(math.ceil((x + extent_x - origin_x) / step))
    row_lo = int(math.floor((y - extent_y - origin_y) / step)) - 1
    row_hi = int(math.ceil((y + extent_y - origin_y) / step))
    for column in range(col_lo, col_hi + 1):
        cell_x = origin_x + (column + 0.5) * step
        dx = cell_x - x
        for row in range(row_lo, row_hi + 1):
            cell_y = origin_y + (row + 0.5) * step
            dy = cell_y - y
            if abs(dx) > extent_x + cell_half + 1e-12 or abs(dy) > extent_y + cell_half + 1e-12:
                continue
            local_x = dx * cos_t + dy * sin_t
            if abs(local_x) > half_l + local_cell_extent + 1e-12:
                continue
            local_y = -dx * sin_t + dy * cos_t
            if abs(local_y) > half_w + local_cell_extent + 1e-12:
                continue
            yield cell_x, cell_y


def _xy(item: Any) -> tuple[float, float]:
    if isinstance(item, dict):
        return (float(item["x"]), float(item["y"]))
    if hasattr(item, "x") and hasattr(item, "y"):
        return (float(item.x), float(item.y))
    return (float(item[0]), float(item[1]))


def _sample_polyline(
    points: Sequence[tuple[float, float]], step_m: float
) -> list[tuple[float, float, float, float]]:
    """等距采样；返回 ``(s_m, x, y, 切线朝向)``。

    切线用相邻路径点差分求得，所以弯道用的是局部法向而不是全局固定横轴。
    退化路径（单点或原地转向）没有可用的切线，取 0 弧度，左右即 ±y。
    """
    step = max(1e-3, float(step_m))
    if len(points) == 1:
        return [(0.0, points[0][0], points[0][1], 0.0)]
    samples: list[tuple[float, float, float, float]] = []
    cumulative = 0.0
    for index in range(len(points) - 1):
        ax, ay = points[index]
        bx, by = points[index + 1]
        span = math.hypot(bx - ax, by - ay)
        if span <= 1e-9:
            continue
        heading = math.atan2(by - ay, bx - ax)
        count = max(1, int(math.ceil(span / step)))
        for k in range(count):
            t = k / count
            samples.append(
                (cumulative + span * t, ax + (bx - ax) * t, ay + (by - ay) * t, heading)
            )
        cumulative += span
    if not samples:
        return [(0.0, points[0][0], points[0][1], 0.0)]
    last = points[-1]
    samples.append((cumulative, last[0], last[1], samples[-1][3]))
    return samples


def _lateral_distances(
    x: float, y: float, heading: float, grid: CorridorGrid
) -> tuple[float, float]:
    step = max(1e-3, float(grid.resolution))
    # 左法向 = 切线逆时针转 90°；右法向为反方向。
    left_x, left_y = -math.sin(heading), math.cos(heading)
    right_x, right_y = math.sin(heading), -math.cos(heading)
    return (
        _ray_distance(x, y, left_x, left_y, step, grid),
        _ray_distance(x, y, right_x, right_y, step, grid),
    )


def _ray_distance(
    x: float, y: float, dx: float, dy: float, step: float, grid: CorridorGrid
) -> float:
    distance = step
    while distance <= MAX_LATERAL_SEARCH_M:
        if grid.blocked(x + dx * distance, y + dy * distance):
            return distance
        distance += step
    return MAX_LATERAL_SEARCH_M


def _round(value: float | None) -> float | None:
    return None if value is None else round(float(value), 3)


__all__ = [
    "CENTER_SCORE_WEIGHT",
    "CorridorDecision",
    "CorridorGrid",
    "CurrentFootprintObservedGrid",
    "DECISION_OK",
    "DECISION_ROTATION_BLOCKED",
    "DECISION_TOO_NARROW",
    "FOOTPRINT_PADDING_M",
    "FORWARD_EGRESS_MAX_M",
    "ForwardPaddingEgressDecision",
    "HALF_BODY_LENGTH_M",
    "HALF_BODY_WIDTH_M",
    "HARD_SIDE_GAP_M",
    "MAX_LATERAL_SEARCH_M",
    "MAX_SPEED_MPS",
    "MIN_SPEED_MPS",
    "OCCUPIED_THRESHOLD",
    "PREFERRED_SIDE_GAP_M",
    "PathClearanceSample",
    "ROTATION_MIN_SWEEP_RAD",
    "ROTATION_SWEEP_MARGIN_RAD",
    "ROTATION_SWEEP_STEP_RAD",
    "ROTATION_TRANSLATION_TOL_M",
    "RotationSweepDecision",
    "SAMPLE_STEP_M",
    "center_score",
    "classify_path_clearance",
    "classify_forward_padding_egress",
    "classify_rotation_sweep",
    "clear_current_physical_footprint_data",
    "current_footprint_observed_grid",
    "measure_path_clearance",
    "path_center_score",
    "speed_limit_from_clearance",
]
