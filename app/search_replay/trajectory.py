"""Trajectory segmentation, distance accounting and world→pixel mapping.

轨迹只来自实测位姿（TF map→base_link）；**任何**规划路线、已发指令或
VLM 描述都不会进入这里。定位无效、明显跳变、坐标系/时钟周期变化都会
切断轨迹，绝不补一条直线跨过数据空洞。

累计行程取同段、有效、连续的轮式里程增量；定位跳变/重启导致的跳变不计为
移动。净位移单独保留，两者不混为一谈。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Iterable, Sequence

from app.search_replay.models import (
    OdomRow,
    PoseRow,
    finite_float,
)


@dataclass
class TrajectoryPoint:
    t: float
    x: float
    y: float
    yaw: float

    def to_dict(self) -> dict[str, float]:
        return {"t": self.t, "x": self.x, "y": self.y, "yaw": self.yaw}


@dataclass
class TrajectorySegment:
    segment_id: str
    reason: str
    points: list[TrajectoryPoint] = field(default_factory=list)
    invalid_spans: list[tuple[float, float]] = field(default_factory=list)
    distance_m: float = 0.0
    net_displacement_m: float = 0.0
    distance_source: str = "odom"
    gaps: int = 0

    @property
    def start_t(self) -> float:
        return self.points[0].t if self.points else 0.0

    @property
    def end_t(self) -> float:
        return self.points[-1].t if self.points else 0.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "segment_id": self.segment_id,
            "reason": self.reason,
            "start_t": self.start_t,
            "end_t": self.end_t,
            "points": len(self.points),
            "distance_m": round(self.distance_m, 3),
            "net_displacement_m": round(self.net_displacement_m, 3),
            "distance_source": self.distance_source,
            "gaps": self.gaps,
        }


@dataclass
class Trajectory:
    segments: list[TrajectorySegment]
    distance_m: float
    net_displacement_m: float
    distance_source: str
    distances_by_segment: dict[str, float]

    @property
    def points(self) -> list[TrajectoryPoint]:
        result: list[TrajectoryPoint] = []
        for segment in self.segments:
            result.extend(segment.points)
        return result

    def to_dict(self) -> dict[str, Any]:
        return {
            "segments": [segment.to_dict() for segment in self.segments],
            "distance_m": round(self.distance_m, 3),
            "net_displacement_m": round(self.net_displacement_m, 3),
            "distance_source": self.distance_source,
            "distances_by_segment": {
                key: round(value, 3) for key, value in self.distances_by_segment.items()
            },
        }


def _angle_delta(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def _finite_pose(row: PoseRow) -> bool:
    return all(
        isinstance(value, (int, float)) and math.isfinite(float(value))
        for value in (row.x, row.y, row.yaw, row.t)
    )


def build_trajectory(
    poses: Sequence[PoseRow],
    odom_rows: Sequence[OdomRow],
    *,
    segment_markers: Sequence[dict[str, Any]] = (),
    invalid_spans: Sequence[tuple[float, float]] = (),
    max_jump_m: float = 0.6,
    max_jump_window_s: float = 0.5,
    max_odom_speed_mps: float = 1.5,
) -> Trajectory:
    """Split pose rows into continuous, valid segments.

    ``segment_markers`` are the collector's explicit segment boundaries
    (localization loss, map/coordinate reset, collector restart, clock step).
    A marker starts a new segment; so does an implicit jump or an invalid
    interval overlapping the rows.
    """
    ordered_all = sorted((row for row in poses if _finite_pose(row)),
                         key=lambda item: item.t)
    spans = sorted(
        (max(0.0, float(start)), float(end))
        for start, end in invalid_spans
        if math.isfinite(float(start)) and math.isfinite(float(end))
    )
    markers = sorted(
        (float(marker.get("t") or 0.0), str(marker.get("reason") or "marker"))
        for marker in segment_markers
        if finite_float(marker.get("t"), None) is not None
    )

    # 失效定位不是"另一条轨迹"：它连点都不是。先把 valid=False 的行和落在
    # invalid_spans 内的行排除，再在有效性翻转/区间边界处切段，避免把一次
    # 中断画成跨洞直线，也避免失效位姿被拿来画箭头或计行程。
    def _inside_span(value: float) -> bool:
        return any(start <= value <= end for start, end in spans)

    validity_markers: list[tuple[float, str]] = []
    for previous, current in zip(ordered_all, ordered_all[1:]):
        if previous.valid != current.valid:
            validity_markers.append((
                current.t,
                "localization_resume" if current.valid else "localization_invalid",
            ))
    for start, _end in spans:
        validity_markers.append((start, "localization_invalid"))
    if validity_markers:
        markers = sorted(markers + validity_markers)
    ordered = [row for row in ordered_all
               if row.valid and not _inside_span(row.t)]

    segments: list[TrajectorySegment] = []
    current: TrajectorySegment | None = None
    marker_index = 0
    span_index = 0
    previous: PoseRow | None = None

    def start_segment(row: PoseRow, reason: str) -> TrajectorySegment:
        segment = TrajectorySegment(
            segment_id=f"seg{len(segments)}", reason=reason
        )
        segments.append(segment)
        return segment

    for row in ordered:
        reason: str | None = None
        if current is None:
            # 只有已经到时的标记才能解释"第一段为什么从这里开始"。收藏
            # 器通常把定位失效/重启标记写在开始后几百毫秒，若在这里无条件
            # 吃掉 markers[0]，该标记就不会再切段，整次中断被画成一条线。
            reason = "start"
            while marker_index < len(markers) and markers[marker_index][0] <= row.t:
                reason = markers[marker_index][1]
                marker_index += 1
        else:
            while marker_index < len(markers) and markers[marker_index][0] <= row.t:
                reason = f"marker:{markers[marker_index][1]}"
                marker_index += 1
            if reason is None:
                while span_index < len(spans) and spans[span_index][1] < row.t:
                    span_index += 1
                if span_index < len(spans) and spans[span_index][0] <= row.t <= spans[span_index][1]:
                    reason = "localization_invalid"
            if reason is None and previous is not None:
                gap = row.t - previous.t
                jump = math.hypot(row.x - previous.x, row.y - previous.y)
                if gap <= max_jump_window_s and jump > max_jump_m:
                    reason = "pose_jump"
                elif gap > 2.0:
                    reason = "pose_gap"
                elif _angle_delta(row.yaw, previous.yaw) > math.pi / 2 and gap < 0.05:
                    reason = "yaw_jump"
                elif row.frame_id != previous.frame_id:
                    reason = "frame_change"
            if reason is not None:
                current.invalid_spans.append((previous.t, row.t))
                current.gaps += 1
        if reason is not None or current is None:
            current = start_segment(row, reason or "start")
        current.points.append(
            TrajectoryPoint(row.t, float(row.x), float(row.y), float(row.yaw))
        )
        previous = row

    # 定位无效区间即使没有位姿空洞也要留在段上，渲染时会显式标注。
    for segment in segments:
        for start, end in spans:
            if end >= segment.start_t and start <= segment.end_t:
                segment.invalid_spans.append((start, end))
        segment.invalid_spans.sort()

    if not segments:
        return Trajectory([], 0.0, 0.0, "none", {})

    total_odom = 0.0
    total_pose = 0.0
    distances: dict[str, float] = {}
    any_odom = False
    for segment in segments:
        odom_distance, odom_used = _odom_distance(
            odom_rows, segment.start_t, segment.end_t,
            max_speed=max_odom_speed_mps,
        )
        pose_distance = _polyline_length(segment.points)
        # 里程不可用时退回到实测位姿路径长度，并在质量标记里写明来源。
        if odom_used:
            segment.distance_m = odom_distance
            segment.distance_source = "odom"
            any_odom = True
        else:
            segment.distance_m = pose_distance
            segment.distance_source = "pose_path"
        distances[segment.segment_id] = segment.distance_m
        total_odom += segment.distance_m
        total_pose += pose_distance
        first, last = segment.points[0], segment.points[-1]
        segment.net_displacement_m = math.hypot(last.x - first.x, last.y - first.y)

    first_segment = segments[0]
    last_segment = segments[-1]
    net = math.hypot(
        last_segment.points[-1].x - first_segment.points[0].x,
        last_segment.points[-1].y - first_segment.points[0].y,
    )
    source = "odom" if any_odom else "pose_path"
    if not odom_rows:
        source = "pose_path"
    return Trajectory(
        segments=segments,
        distance_m=total_odom,
        net_displacement_m=net,
        distance_source=source,
        distances_by_segment=distances,
    )


def _polyline_length(points: Sequence[TrajectoryPoint]) -> float:
    total = 0.0
    for previous, current in zip(points, points[1:]):
        total += math.hypot(current.x - previous.x, current.y - previous.y)
    return total


def _odom_distance(
    odom_rows: Sequence[OdomRow],
    start_t: float,
    end_t: float,
    *,
    max_speed: float,
) -> tuple[float, bool]:
    """Sum wheel-odometry increments inside one segment.

    Deltas are rejected when the sample gap is large (source dropout) or the
    implied speed is beyond the platform's physical ceiling (position jump).
    Returns ``(distance, used)``; ``used`` is False when no valid pair exists.
    """
    samples = sorted(
        (row for row in odom_rows
         if start_t <= row.t <= end_t
         and all(math.isfinite(float(value)) for value in (row.x, row.y, row.t))),
        key=lambda item: item.t,
    )
    total = 0.0
    used = False
    for previous, current in zip(samples, samples[1:]):
        dt = current.t - previous.t
        if dt <= 0 or dt > 0.5:
            continue
        step = math.hypot(current.x - previous.x, current.y - previous.y)
        if step / dt > max_speed:
            continue
        total += step
        used = True
    return total, used


# --------------------------------------------------------------------------- #
# Viewport / projection                                                       #
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Viewport:
    """World-space rectangle (metres) used for one segment's fixed view."""

    min_x: float
    min_y: float
    max_x: float
    max_y: float

    @property
    def width_m(self) -> float:
        return max(1e-6, self.max_x - self.min_x)

    @property
    def height_m(self) -> float:
        return max(1e-6, self.max_y - self.min_y)

    def project(
        self, x: float, y: float, *, panel_width: int, panel_height: int,
        padding: int = 0,
    ) -> tuple[float, float, float]:
        """Return ``(px, py, metres_per_pixel)``; y axis is screen-down."""
        usable_w = max(1, panel_width - 2 * padding)
        usable_h = max(1, panel_height - 2 * padding)
        scale = min(usable_w / self.width_m, usable_h / self.height_m)
        offset_x = padding + (usable_w - self.width_m * scale) / 2.0
        offset_y = padding + (usable_h - self.height_m * scale) / 2.0
        px = offset_x + (x - self.min_x) * scale
        # 世界 y 向上、屏幕 y 向下：这里只翻转一次，箭头方向由同一变换得到。
        py = offset_y + (self.max_y - y) * scale
        return px, py, scale

    def project_point(self, x: float, y: float, *, panel_width: int,
                      panel_height: int, padding: int = 0) -> tuple[int, int]:
        px, py, _ = self.project(
            x, y, panel_width=panel_width, panel_height=panel_height,
            padding=padding,
        )
        return int(round(px)), int(round(py))


def viewport_for(
    points: Iterable[tuple[float, float]],
    map_bounds: Iterable[tuple[float, float, float, float]] = (),
    *,
    margin_ratio: float = 0.08,
    min_span_m: float = 2.0,
) -> Viewport:
    xs: list[float] = []
    ys: list[float] = []
    for x, y in points:
        if math.isfinite(x) and math.isfinite(y):
            xs.append(float(x))
            ys.append(float(y))
    for min_x, min_y, max_x, max_y in map_bounds:
        if all(math.isfinite(value) for value in (min_x, min_y, max_x, max_y)):
            xs.extend((float(min_x), float(max_x)))
            ys.extend((float(min_y), float(max_y)))
    if not xs:
        return Viewport(-1.0, -1.0, 1.0, 1.0)
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    if max_x - min_x < min_span_m:
        center = (max_x + min_x) / 2.0
        min_x, max_x = center - min_span_m / 2, center + min_span_m / 2
    if max_y - min_y < min_span_m:
        center = (max_y + min_y) / 2.0
        min_y, max_y = center - min_span_m / 2, center + min_span_m / 2
    margin_x = (max_x - min_x) * margin_ratio
    margin_y = (max_y - min_y) * margin_ratio
    return Viewport(
        min_x - margin_x, min_y - margin_y, max_x + margin_x, max_y + margin_y
    )


def map_world_bounds(
    origin_x: float, origin_y: float, yaw: float,
    width: int, height: int, resolution: float,
) -> tuple[float, float, float, float]:
    """Axis-aligned world bounds of an (optionally rotated) occupancy grid."""
    corners = []
    for col, row in ((0, 0), (width, 0), (0, height), (width, height)):
        local_x, local_y = col * resolution, row * resolution
        world_x = origin_x + math.cos(yaw) * local_x - math.sin(yaw) * local_y
        world_y = origin_y + math.sin(yaw) * local_x + math.cos(yaw) * local_y
        corners.append((world_x, world_y))
    xs = [point[0] for point in corners]
    ys = [point[1] for point in corners]
    return min(xs), min(ys), max(xs), max(ys)


def first_motion_time(
    poses: Sequence[PoseRow], *, threshold_m: float = 0.05,
) -> float | None:
    previous: PoseRow | None = None
    for current in poses:
        if not current.valid:
            previous = None
            continue
        if previous is not None and \
                math.hypot(current.x - previous.x,
                           current.y - previous.y) >= threshold_m:
            return current.t
        previous = current
    return None
