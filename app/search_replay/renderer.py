"""Offline renderer: 1920x1080 地图展开 + 实测轨迹 + 相机 + 关键事件。

只做真实数据的可视化，不做任何"美化"：

* 未知区域浅灰、自由区域白、占用区域深灰；不膨胀/腐蚀障碍，不把未知当白。
* 地图按录制时间逐版出现；第一帧只画当时已有的地图。
* 轨迹来自实测位姿，跨定位中断/跳变不连线；规划路线从不参与。
* 缺图/缺相机/缺定位都有明确文字标注，不补造图片或位置。

编码由 :mod:`app.search_replay.encoder` 负责；本模块只产出 RGB 帧与
最终地图 PNG / 封面 JPEG。
"""

from __future__ import annotations

import math
from bisect import bisect_right
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import numpy as np

from app.search_replay.models import (
    EventRow,
    FrameRow,
    MapSnapshot,
    PoseRow,
    ReplayConfig,
)
from app.search_replay.recording import Recording
from app.search_replay.timeline import (
    KeyMoment,
    PlaybackTimemap,
    build_speed_plan,
    key_moments_from_events,
    split_chapters,
)
from app.search_replay.trajectory import (
    Trajectory,
    TrajectorySegment,
    Viewport,
    first_motion_time,
    map_world_bounds,
    viewport_for,
)

RENDERER_VERSION = "search_replay_renderer_v2"

# 事件 → 画面标签/颜色。疑似与确认严格分开；停止/失败绝不显示为成功。
EVENT_STYLE: dict[str, tuple[str, tuple[int, int, int]]] = {
    "SESSION_CREATED": ("会话创建", (120, 170, 235)),
    "SESSION_STARTED": ("开始搜索", (120, 170, 235)),
    "TARGET_CANDIDATE": ("疑似目标", (240, 160, 60)),
    "VERIFICATION_STARTED": ("目标核验中", (240, 190, 90)),
    "VERIFICATION_FINISHED": ("核验结束", (200, 180, 120)),
    "TARGET_CONFIRMED": ("确认目标", (226, 72, 72)),
    "OPERATOR_STOP": ("人工停止", (235, 140, 60)),
    "SEARCH_EXHAUSTED": ("搜索耗尽", (170, 150, 235)),
    "SEARCH_FINISHED": ("搜索结束", (200, 210, 225)),
    "ERROR": ("异常", (226, 72, 72)),
    "PAUSED": ("暂停", (200, 180, 120)),
    "RESUMED": ("继续", (120, 200, 150)),
}

# 只有这些事件会在地图上留下标记，避免图面被规划/运动事件塞满。
MARKER_EVENTS = {
    "TARGET_CANDIDATE", "TARGET_CONFIRMED", "OPERATOR_STOP",
    "SEARCH_EXHAUSTED", "SEARCH_FINISHED", "ERROR",
}

BANNER_EVENTS = MARKER_EVENTS | {
    "VERIFICATION_STARTED", "VERIFICATION_FINISHED", "PAUSED", "RESUMED",
}

RESULT_LABELS = {
    "TARGET_FOUND": "找到目标",
    "SEARCH_EXHAUSTED": "未找到（搜索耗尽）",
    "OPERATOR_STOP": "人工停止",
    "TIMEOUT": "超时结束",
    "FAILED": "失败",
    "FINISHED": "正常结束",
    "TASK_REJECTED": "任务被拒绝",
    "MAX_STEPS_REACHED": "达到步数上限",
    "MAX_PLANNING_CYCLES_REACHED": "达到规划轮数上限",
}

BG = (24, 28, 34)
PANEL = (33, 39, 47)
TEXT = (233, 237, 243)
MUTED = (150, 160, 173)
MAP_UNKNOWN = (232, 233, 235)
MAP_FREE = (255, 255, 255)
MAP_OCCUPIED = (52, 57, 64)
GREEN = (46, 194, 100)
GREEN_DIM = (28, 112, 62)
BLUE = (70, 152, 240)
AMBER = (232, 190, 92)
RED = (228, 76, 72)


class RenderError(RuntimeError):
    """Raised when a recording cannot produce a truthful video."""


@dataclass
class RenderMetadata:
    speed: int
    duration: float
    frames: int
    width: int
    height: int
    fps: int
    first_t: float
    last_t: float
    end_hold_seconds: float
    key_windows: list[dict[str, Any]] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    quality: str = "FULL"
    chapters: list[dict[str, Any]] = field(default_factory=list)
    distance_m: float = 0.0
    distance_source: str = "none"

    def to_dict(self) -> dict[str, Any]:
        return {
            "speed": self.speed,
            "duration": round(self.duration, 3),
            "frames": self.frames,
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "first_t": self.first_t,
            "last_t": self.last_t,
            "end_hold_seconds": self.end_hold_seconds,
            "key_windows": self.key_windows,
            "notes": self.notes,
            "quality": self.quality,
            "chapters": self.chapters,
            "distance_m": round(self.distance_m, 3),
            "distance_source": self.distance_source,
        }


@dataclass
class _Panel:
    x: int
    y: int
    width: int
    height: int

    @property
    def right(self) -> int:
        return self.x + self.width

    @property
    def bottom(self) -> int:
        return self.y + self.height


class _SegmentDraw:
    """Pre-computed per-segment draw data (numpy, no per-frame Python loops)."""

    def __init__(self, segment: TrajectorySegment) -> None:
        self.segment = segment
        self.times = np.asarray([point.t for point in segment.points],
                                dtype=np.float64)
        self.xs = np.asarray([point.x for point in segment.points],
                             dtype=np.float64)
        self.ys = np.asarray([point.y for point in segment.points],
                             dtype=np.float64)
        self.yaws = np.asarray([point.yaw for point in segment.points],
                               dtype=np.float64)
        # 断点：时间空洞 > 2 s，或相邻两点之间落在定位无效区间。
        breaks = np.zeros(len(self.times), dtype=bool)
        for index in range(1, len(self.times)):
            if self.times[index] - self.times[index - 1] > 2.0:
                breaks[index] = True
                continue
            for start, end in segment.invalid_spans:
                if start <= self.times[index] and end >= self.times[index - 1]:
                    breaks[index] = True
                    break
        self.breaks = breaks
        self.runs: list[tuple[int, int]] = []
        run_start = 0
        for index in range(1, len(self.times)):
            if breaks[index]:
                if index - run_start >= 2:
                    self.runs.append((run_start, index))
                run_start = index
        if len(self.times) - run_start >= 2:
            self.runs.append((run_start, len(self.times)))

    def index_at(self, source_t: float) -> int:
        if len(self.times) == 0:
            return -1
        return int(np.searchsorted(self.times, source_t, side="right")) - 1


def _load_font(path: str, size: int):
    from PIL import ImageFont

    for index in (2, 1, 0):
        try:
            return ImageFont.truetype(path, size, index=index)
        except (OSError, ValueError, TypeError):
            continue
    try:
        return ImageFont.load_default(size=size)
    except TypeError:  # Pillow < 10
        return ImageFont.load_default()


class ReplayRenderer:
    """Renders one recording at one playback speed."""

    def __init__(
        self,
        recording: Recording,
        config: ReplayConfig,
        *,
        speed: int | None = None,
        width: int | None = None,
        height: int | None = None,
        fps: int | None = None,
    ) -> None:
        self.recording = recording
        self.config = config
        self.speed = int(speed or config.default_speed)
        self.width = int(width or config.width)
        self.height = int(height or config.height)
        self.fps = max(5, int(fps or config.fps))
        self.metadata: RenderMetadata | None = None
        self.first_t = 0.0
        self.last_t = 0.0
        self._static_cache: dict[tuple[str, str], np.ndarray] = {}
        self._distance_tracks: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self._prepared = False
        self._end_frame_cache: np.ndarray | None = None
        self._fonts: dict[int, Any] = {}
        self._pending_text: list[tuple[int, int, str, int, tuple, str]] = []

    # ------------------------------------------------------------------ #
    # preparation                                                        #
    # ------------------------------------------------------------------ #
    def prepare(self) -> RenderMetadata:
        first = self.recording.first_t
        last = self.recording.last_t
        if first is None or last is None or last <= first:
            raise RenderError(
                "没有可回放的记录：录制包内没有地图、位姿、相机或事件时间"
            )
        self.first_t = float(first)
        # 末帧至少留出 0.2 s，避免极短会话出现零帧视频。
        self.last_t = max(float(last), self.first_t + 0.2)

        self.trajectory: Trajectory = self.recording.trajectory(
            max_jump_m=self.config.max_pose_jump_m,
            max_jump_window_s=self.config.max_pose_jump_window_s,
            max_odom_speed_mps=self.config.max_odom_speed_mps,
        )
        self.segment_draws = {
            segment.segment_id: _SegmentDraw(segment)
            for segment in self.trajectory.segments
        }
        events = [event.to_dict() for event in self.recording.events]
        self.key_moments: list[KeyMoment] = key_moments_from_events(
            events,
            first_motion_t=first_motion_time(self.recording.poses),
        )
        self.speed_plan = build_speed_plan(
            self.first_t,
            self.last_t,
            self.key_moments,
            default_speed=self.speed,
            key_speed=self.config.key_speed,
            pad_seconds=self.config.key_pad_seconds,
        )
        chapters = split_chapters(
            PlaybackTimemap(self.speed_plan),
            chapter_seconds=self.config.chapter_seconds,
        )
        self.timemap = PlaybackTimemap(
            self.speed_plan,
            end_hold_seconds=self.config.end_hold_seconds,
            chapters=chapters,
        )

        self._map_index = list(self.recording.maps)
        self._map_times = [item.t for item in self._map_index]
        self._map_msg_times = sorted(
            float(value) for value in getattr(self.recording, "map_message_times", [])
        )
        self._frame_times = [row.t for row in self.recording.frames]
        self._event_times = [event.t for event in self.recording.events]
        raw_poses = sorted(self.recording.poses, key=lambda row: row.t)
        self._raw_pose_times = [row.t for row in raw_poses]
        self._raw_pose_valid = [row.valid for row in raw_poses]

        self._compute_layout()
        self._compute_viewports()
        self._build_distance_tracks()
        self._markers = self._build_markers()

        notes = list(self.recording.coverage_notes())
        quality = "FULL"
        if not self.recording.maps or not self.trajectory.points \
                or not self.recording.frames or self.recording.invalid_spans \
                or any(not row.valid for row in self.recording.poses):
            quality = "DEGRADED"
        if len(self.trajectory.segments) > 1:
            notes.append(
                f"轨迹分 {len(self.trajectory.segments)} 段：定位中断/跳变处不连线"
            )
        if self.trajectory.distance_source != "odom":
            notes.append("轮式里程不可用，行程改用实测位姿路径估计")

        self.metadata = RenderMetadata(
            speed=self.speed,
            duration=self.timemap.duration,
            frames=int(math.ceil(self.timemap.duration * self.fps)),
            width=self.width,
            height=self.height,
            fps=self.fps,
            first_t=self.first_t,
            last_t=self.last_t,
            end_hold_seconds=self.config.end_hold_seconds,
            key_windows=self.timemap.key_windows(),
            notes=notes,
            quality=quality,
            chapters=chapters,
            distance_m=self.trajectory.distance_m,
            distance_source=self.trajectory.distance_source,
        )
        self._prepared = True
        return self.metadata

    def _compute_layout(self) -> None:
        scale = self.width / 1920.0
        self.margin = max(12, int(round(24 * scale)))
        header_h = max(52, int(round(76 * scale)))
        footer_h = max(110, int(round(184 * scale)))
        right_w = max(240, int(round(560 * scale)))
        gap = max(8, int(round(16 * scale)))

        top = self.margin + header_h
        bottom = self.height - self.margin - footer_h
        self.header = _Panel(self.margin, self.margin,
                             self.width - 2 * self.margin, header_h)
        map_w = self.width - 2 * self.margin - right_w - gap
        self.map_panel = _Panel(self.margin, top, map_w, bottom - top)
        self.right_panel = _Panel(self.map_panel.right + gap, top, right_w,
                                  bottom - top)
        self.footer = _Panel(self.margin, bottom + gap,
                             self.width - 2 * self.margin,
                             self.height - self.margin - (bottom + gap))
        camera_h = int(self.right_panel.width * 9 / 16)
        self.camera_panel = _Panel(self.right_panel.x, self.right_panel.y,
                                   self.right_panel.width, camera_h)
        self.event_panel = _Panel(
            self.right_panel.x, self.camera_panel.bottom + gap,
            self.right_panel.width,
            self.right_panel.bottom - self.camera_panel.bottom - gap,
        )
        if self.map_panel.width < 40 or self.map_panel.height < 40:
            raise RenderError(
                f"画布 {self.width}x{self.height} 太小：地图面板仅 "
                f"{self.map_panel.width}x{self.map_panel.height}，"
                "请把 width/height 调到至少 480x270"
            )

    def _compute_viewports(self) -> None:
        self.viewports: dict[str, Viewport] = {}
        segments = self.trajectory.segments
        for index, segment in enumerate(segments):
            points = [(point.x, point.y) for point in segment.points]
            segment_end = (
                segments[index + 1].start_t
                if index + 1 < len(segments)
                else self.last_t
            )
            bounds = [
                self._snapshot_bounds(snapshot)
                for snapshot in self._map_index
                if segment.start_t - 1e-6 <= snapshot.t <= segment_end + 1e-6
            ]
            if not bounds:
                # 段内没有地图版本时，至少用最近的一张，避免视窗退化成点。
                near = self._map_at(segment.start_t)
                if near is not None:
                    bounds.append(self._snapshot_bounds(near))
            self.viewports[segment.segment_id] = viewport_for(points, bounds)
        self.end_viewport = viewport_for(
            [(point.x, point.y) for point in self.trajectory.points],
            [self._snapshot_bounds(item) for item in self._map_index],
        )

    @staticmethod
    def _snapshot_bounds(snapshot: MapSnapshot) -> tuple[float, float, float, float]:
        yaw = _yaw_from_quaternion(snapshot.origin[3:7])
        return map_world_bounds(
            snapshot.origin[0], snapshot.origin[1], yaw,
            snapshot.width, snapshot.height, snapshot.resolution,
        )

    def _build_distance_tracks(self) -> None:
        """Cumulative distance per segment for the on-screen counter.

        Uses the same rule as the final card (wheel odometry with a pose-path
        fallback) so the live counter never contradicts the closing number.
        """
        for segment in self.trajectory.segments:
            times: list[float] = [segment.start_t]
            distances: list[float] = [0.0]
            rows = [row for row in self.recording.odom
                    if segment.start_t <= row.t <= segment.end_t]
            total = 0.0
            valid_pair = False
            for previous, current in zip(rows, rows[1:]):
                dt = current.t - previous.t
                step = math.hypot(current.x - previous.x, current.y - previous.y)
                if 0 < dt <= 0.5 and step / dt <= self.config.max_odom_speed_mps:
                    total += step
                    valid_pair = True
                times.append(current.t)
                distances.append(total)
            if not valid_pair:
                times = [point.t for point in segment.points] or [segment.start_t]
                distances = [0.0]
                total = 0.0
                for previous, current in zip(segment.points, segment.points[1:]):
                    total += math.hypot(current.x - previous.x, current.y - previous.y)
                    distances.append(total)
            if len(times) < 2:
                times.append(segment.end_t)
                distances.append(0.0)
            self._distance_tracks[segment.segment_id] = (
                np.asarray(times, dtype=np.float64),
                np.asarray(distances, dtype=np.float64),
            )

    def _distance_at(self, source_t: float) -> float:
        total = 0.0
        for segment in self.trajectory.segments:
            track = self._distance_tracks.get(segment.segment_id)
            if track is None:
                continue
            times, distances = track
            if source_t <= segment.start_t:
                break
            if source_t >= segment.end_t:
                total += float(distances[-1])
                continue
            index = int(np.searchsorted(times, source_t, side="right")) - 1
            index = max(0, min(index, len(times) - 1))
            total += float(distances[index])
            break
        return total

    def _build_markers(self) -> list[dict[str, Any]]:
        markers: list[dict[str, Any]] = []
        segments = self.trajectory.segments
        if not segments:
            return markers
        start = segments[0].points[0]
        markers.append({"t": start.t, "x": start.x, "y": start.y,
                        "label": "起点", "color": GREEN})
        last = segments[-1].points[-1]
        markers.append({"t": last.t, "x": last.x, "y": last.y,
                        "label": "终点", "color": (150, 200, 255)})
        for event in self.recording.events:
            if event.event_type not in MARKER_EVENTS:
                continue
            pose = self.pose_at(event.t)
            if pose is None:
                continue
            label, color = EVENT_STYLE[event.event_type]
            markers.append({"t": event.t, "x": pose.x, "y": pose.y,
                            "label": label, "color": color})
        markers.sort(key=lambda item: item["t"])
        return markers

    # ------------------------------------------------------------------ #
    # lookups                                                            #
    # ------------------------------------------------------------------ #
    def _map_at(self, source_t: float) -> MapSnapshot | None:
        """Latest map version at this time whose grid actually loads.

        A version can stay referenced while its NPZ is unreadable (legacy
        package, pruned file, corrupt write).  Showing an all-unknown panel
        there would claim "地图：已更新"; fall back to the previous readable
        version instead, and to ``None`` when none exists.
        """
        index = bisect_right(self._map_times, source_t) - 1
        while index >= 0:
            snapshot = self._map_index[index]
            if self.recording.map_grid(snapshot) is not None:
                return snapshot
            index -= 1
        return None

    def _map_stalled(self, source_t: float) -> float | None:
        """Seconds since the last /map message, when beyond 5 s."""
        if not self._map_msg_times:
            return None
        index = bisect_right(self._map_msg_times, source_t) - 1
        if index < 0:
            return None
        age = source_t - self._map_msg_times[index]
        return age if age > 5.0 else None

    def _frame_at(self, source_t: float) -> FrameRow | None:
        index = bisect_right(self._frame_times, source_t) - 1
        if index < 0:
            return None
        row = self.recording.frames[index]
        return row if source_t - row.t <= 1.0 else None

    def pose_at(self, source_t: float) -> PoseRow | None:
        # Retaining an older point for drawing a path does not make it a
        # valid current location during a known localization failure.
        raw_index = bisect_right(self._raw_pose_times, source_t) - 1
        if raw_index < 0 or not self._raw_pose_valid[raw_index]:
            return None
        if any(start <= source_t <= end
               for start, end in self.recording.invalid_spans):
            return None
        segment = self.segment_at(source_t)
        if segment is None:
            return None
        draw = self.segment_draws.get(segment.segment_id)
        if draw is None or len(draw.times) == 0:
            return None
        index = draw.index_at(source_t)
        if index < 0:
            return None
        return PoseRow(
            t=float(draw.times[index]), wall=0.0,
            x=float(draw.xs[index]), y=float(draw.ys[index]),
            yaw=float(draw.yaws[index]),
        )

    def segment_at(self, source_t: float) -> TrajectorySegment | None:
        segments = self.trajectory.segments
        if not segments:
            return None
        for segment in reversed(segments):
            if segment.start_t <= source_t:
                return segment
        return None

    # ------------------------------------------------------------------ #
    # drawing                                                            #
    # ------------------------------------------------------------------ #
    def render_frame(self, video_t: float) -> np.ndarray:
        if not self._prepared:
            self.prepare()
        if self.timemap.is_end_hold(video_t):
            return self._render_end_card()
        return self._render_live(self.timemap.video_to_source(video_t))

    def _base_canvas(self) -> np.ndarray:
        canvas = np.full((self.height, self.width, 3), BG, dtype=np.uint8)
        for panel in (self.header, self.map_panel, self.right_panel,
                      self.camera_panel, self.event_panel, self.footer):
            _fill_rect(canvas, panel, PANEL)
        return canvas

    def _render_live(self, source_t: float) -> np.ndarray:
        canvas = self._base_canvas()
        self._pending_text.clear()
        segment = self.segment_at(source_t)
        viewport = self.viewports.get(
            segment.segment_id if segment else "", self.end_viewport,
        )
        self._draw_map(canvas, self.map_panel, viewport, source_t)
        self._draw_trajectory(canvas, self.map_panel, viewport, source_t)
        self._draw_markers(canvas, self.map_panel, viewport, source_t)
        self._draw_robot(canvas, self.map_panel, viewport, source_t)
        self._draw_camera(canvas, source_t)
        self._draw_events(source_t)
        self._draw_header(source_t)
        self._draw_footer(canvas, source_t)
        self._compose_text_live(canvas, source_t)
        self._flush_text(canvas)
        return canvas

    def _render_end_card(self) -> np.ndarray:
        if self._end_frame_cache is not None:
            return self._end_frame_cache.copy()
        canvas = self._base_canvas()
        self._pending_text.clear()
        last_map = self._map_at(self.last_t)
        self._draw_map(canvas, self.map_panel, self.end_viewport, self.last_t,
                       force_map=last_map)
        self._draw_trajectory(canvas, self.map_panel, self.end_viewport,
                              self.last_t, show_all=True)
        self._draw_markers(canvas, self.map_panel, self.end_viewport,
                           self.last_t, show_all=True)
        self._draw_final_camera(canvas)
        self._draw_events(self.last_t, final=True)
        self._draw_header(self.last_t, final=True)
        self._draw_footer(canvas, self.last_t, final=True)
        self._compose_text_end(canvas)
        self._flush_text(canvas)
        self._end_frame_cache = canvas
        return canvas.copy()

    # -- map ------------------------------------------------------------ #
    def _draw_map(
        self, canvas: np.ndarray, panel: _Panel, viewport: Viewport,
        source_t: float, *, force_map: MapSnapshot | None = None,
    ) -> None:
        snapshot = force_map if force_map is not None else self._map_at(source_t)
        if snapshot is None:
            return
        key = (
            snapshot.content_hash,
            f"{viewport.min_x:.4f},{viewport.min_y:.4f},"
            f"{viewport.max_x:.4f},{viewport.max_y:.4f},"
            f"{panel.width}x{panel.height}",
        )
        layer = self._static_cache.get(key)
        if layer is None:
            layer = self._build_map_layer(snapshot, panel, viewport)
            if len(self._static_cache) > 12:
                self._static_cache.clear()
            self._static_cache[key] = layer
        canvas[panel.y:panel.y + panel.height,
               panel.x:panel.x + panel.width] = layer

    def _build_map_layer(
        self, snapshot: MapSnapshot, panel: _Panel, viewport: Viewport,
    ) -> np.ndarray:
        import cv2

        grid = self.recording.map_grid(snapshot)
        if grid is None or grid.size == 0:
            return np.full((panel.height, panel.width, 3), MAP_UNKNOWN,
                           dtype=np.uint8)
        height, width = grid.shape[:2]
        colors = np.empty((height, width, 3), dtype=np.uint8)
        colors[grid < 0] = MAP_UNKNOWN
        known = grid >= 0
        colors[known & (grid < 50)] = MAP_FREE
        colors[grid >= 50] = MAP_OCCUPIED

        yaw = _yaw_from_quaternion(snapshot.origin[3:7])
        origin_x, origin_y = float(snapshot.origin[0]), float(snapshot.origin[1])
        resolution = max(1e-4, float(snapshot.resolution))
        base_x, base_y, scale = viewport.project(
            viewport.min_x, viewport.max_y,
            panel_width=panel.width, panel_height=panel.height,
        )
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        # grid(col,row) → world → panel 的 2x3 仿射；屏幕 y 翻转一次。
        # 世界格心：x = ox + r*(cos*(col+.5) - sin*(row+.5))
        #           y = oy + r*(sin*(col+.5) + cos*(row+.5))
        # 展开后 col 的系数是 (r*cos, -r*sin)，row 的系数是 (-r*sin, -r*cos)
        # （屏幕 y 向下，所以世界 y 的系数再取负）。源像素 (col,row) 指的是
        # 格心，而不是格的左下角。
        half_x = 0.5 * resolution * (cos_y - sin_y)
        half_y = 0.5 * resolution * (sin_y + cos_y)
        a = scale * resolution * cos_y
        b = -scale * resolution * sin_y
        c = base_x + (origin_x + half_x - viewport.min_x) * scale
        d = -scale * resolution * sin_y
        e = -scale * resolution * cos_y
        f = base_y + (viewport.max_y - (origin_y + half_y)) * scale
        matrix = np.array([[a, b, c], [d, e, f]], dtype=np.float32)
        return cv2.warpAffine(
            colors, matrix, (panel.width, panel.height),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=MAP_UNKNOWN,
        )

    # -- trajectory ----------------------------------------------------- #
    def _draw_trajectory(
        self, canvas: np.ndarray, panel: _Panel, viewport: Viewport,
        source_t: float, *, show_all: bool = False,
    ) -> None:
        import cv2

        current = None if show_all else self.segment_at(source_t)
        thickness = max(2, int(round(5 * self.width / 1920)))
        for segment in self.trajectory.segments:
            draw = self.segment_draws.get(segment.segment_id)
            if draw is None or not draw.runs:
                continue
            limit = len(draw.times) if show_all else draw.index_at(source_t) + 1
            if limit <= 1:
                continue
            color = GREEN if segment is current else GREEN_DIM
            px, py, _ = viewport.project(
                draw.xs[:limit], draw.ys[:limit],
                panel_width=panel.width, panel_height=panel.height,
            )
            pixels = np.stack([np.asarray(px), np.asarray(py)], axis=1)
            pixels = np.round(pixels).astype(np.int32)
            pixels[:, 0] += panel.x
            pixels[:, 1] += panel.y
            for start, end in draw.runs:
                run_end = min(end, limit)
                if run_end - start < 2:
                    continue
                cv2.polylines(canvas, [pixels[start:run_end]], False, color,
                              thickness, cv2.LINE_AA)

    def _draw_markers(
        self, canvas: np.ndarray, panel: _Panel, viewport: Viewport,
        source_t: float, *, show_all: bool = False,
    ) -> None:
        import cv2

        radius = max(4, int(round(7 * self.width / 1920)))
        for marker in self._markers:
            if not show_all and marker["t"] > source_t:
                continue
            px, py = viewport.project_point(
                marker["x"], marker["y"],
                panel_width=panel.width, panel_height=panel.height,
            )
            if not (0 <= px < panel.width and 0 <= py < panel.height):
                continue
            center = (panel.x + px, panel.y + py)
            cv2.circle(canvas, center, radius, marker["color"], -1, cv2.LINE_AA)
            cv2.circle(canvas, center, radius + 2, (255, 255, 255), 1, cv2.LINE_AA)

    def _draw_robot(
        self, canvas: np.ndarray, panel: _Panel, viewport: Viewport,
        source_t: float,
    ) -> None:
        import cv2

        pose = self.pose_at(source_t)
        if pose is None:
            return
        px, py, scale = viewport.project(
            pose.x, pose.y, panel_width=panel.width, panel_height=panel.height,
        )
        stale = source_t - pose.t > 2.0
        size = max(9.0, min(22.0, 0.55 * scale))
        dx, dy = math.cos(pose.yaw), -math.sin(pose.yaw)
        nx, ny = -dy, dx
        tip = (panel.x + px + dx * size * 1.4, panel.y + py + dy * size * 1.4)
        left = (panel.x + px - dx * size * 0.7 + nx * size * 0.75,
                panel.y + py - dy * size * 0.7 + ny * size * 0.75)
        right = (panel.x + px - dx * size * 0.7 - nx * size * 0.75,
                 panel.y + py - dy * size * 0.7 - ny * size * 0.75)
        points = np.array([tip, left, right], dtype=np.int32)
        if stale:
            cv2.polylines(canvas, [points], True, MUTED, 2, cv2.LINE_AA)
        else:
            cv2.fillConvexPoly(canvas, points, BLUE, cv2.LINE_AA)
            cv2.polylines(canvas, [points], True, (240, 248, 255), 1, cv2.LINE_AA)

    # -- panels --------------------------------------------------------- #
    def _draw_camera(self, canvas: np.ndarray, source_t: float) -> None:
        self._paste_camera(canvas, self._frame_at(source_t), "图像中断")

    def _draw_final_camera(self, canvas: np.ndarray) -> None:
        row = self.recording.frames[-1] if self.recording.frames else None
        self._paste_camera(
            canvas, row, "无相机记录" if row is None else "只有较早的相机帧",
        )

    def _paste_camera(
        self, canvas: np.ndarray, row: FrameRow | None, empty_label: str,
    ) -> None:
        from PIL import Image

        panel = self.camera_panel
        canvas[panel.y:panel.y + panel.height,
               panel.x:panel.x + panel.width] = (18, 21, 26)
        if row is not None:
            path = self.recording.frame_path(row)
            if path is not None:
                try:
                    with Image.open(path) as image:
                        image = image.convert("RGB")
                        image.thumbnail((panel.width, panel.height))
                        array = np.asarray(image)
                    pixel_h, pixel_w = array.shape[:2]
                    x = panel.x + (panel.width - pixel_w) // 2
                    y = panel.y + (panel.height - pixel_h) // 2
                    canvas[y:y + pixel_h, x:x + pixel_w] = array
                    return
                except (OSError, ValueError):
                    pass
        # 没有可用帧：明确写"图像中断"，绝不拿未来帧或旧会话帧填充。
        self._text(panel.x + 16, panel.y + panel.height // 2 - 14,
                   empty_label, 22, MUTED)

    def _draw_events(self, source_t: float, *, final: bool = False) -> None:
        panel = self.event_panel
        if final:
            rows = self.recording.events[-8:]
        else:
            index = bisect_right(self._event_times, source_t)
            rows = self.recording.events[max(0, index - 8):index]
        shown = [event for event in rows if event.event_type in EVENT_STYLE]
        if not shown:
            # 空面板不留一块"像是坏了"的黑区：明确说没有关键事件。
            self._text(panel.x + 14, panel.y + 10, "关键事件：暂无", 17, MUTED)
            return
        line_height = self._fs(22)
        y = panel.bottom - 10 - line_height * len(shown)
        for event in shown:
            label, color = EVENT_STYLE[event.event_type]
            self._text(panel.x + 14, y, _fmt_clock(event.t - self.first_t),
                       17, MUTED)
            self._text(panel.x + 92, y, label, 17, color)
            detail = _event_detail(event)
            if detail:
                self._text(panel.x + 200, y, _elide(detail, 20), 17, TEXT)
            y += line_height

    def _draw_header(self, source_t: float, *, final: bool = False) -> None:
        panel = self.header
        target = _manifest_target(self.recording.manifest)
        self._text(panel.x + 4, panel.y + 10,
                   f"自主搜索回放 · {_elide(target, 30)}", 34, TEXT)
        when = _manifest_started(self.recording.manifest)
        sub = f"{self.speed}×  ·  {self.recording.session_id}"
        if when:
            sub = f"{when}  ·  " + sub
        self._text(panel.right - 4, panel.y + 18, sub, 18, MUTED,
                   align="right")

    def _draw_footer(self, canvas: np.ndarray, source_t: float, *,
                     final: bool = False) -> None:
        panel = self.footer
        elapsed = max(0.0, source_t - self.first_t)
        distance = self._distance_at(source_t)
        y = panel.y + 14
        if final:
            result = _manifest_result(self.recording.manifest)
            label = RESULT_LABELS.get(result, result or "结束")
            color = RED if result == "FAILED" else AMBER if result else TEXT
            self._text(panel.x + 6, y, f"结果：{label}", 30, color)
        else:
            self._text(panel.x + 6, y, f"已用时间 {_fmt_clock(elapsed)}",
                       30, TEXT)
        step = self._fs(370)
        self._text(panel.x + step, y, f"累计行程 {distance:.2f} m", 30, GREEN)
        self._text(panel.x + 2 * step, y, f"播放倍速 {self.speed}×", 30, BLUE)

        legend_y = y + self._fs(50)
        lx = panel.x + 6
        for color, label in ((MAP_UNKNOWN, "未知"), (MAP_FREE, "自由"),
                             (MAP_OCCUPIED, "占用"), (GREEN, "实测轨迹"),
                             (BLUE, "当前位置")):
            canvas[legend_y + 4:legend_y + 18, lx:lx + 16] = color
            self._text(lx + 24, legend_y, label, 18, TEXT)
            lx += self._fs(110)

        status: list[str] = []
        snapshot = self._map_at(source_t)
        if snapshot is None:
            status.append("地图：不可用")
        elif source_t - snapshot.t > 5.0:
            status.append(f"地图：沿用 {source_t - snapshot.t:.0f}s 前版本")
        else:
            status.append("地图：已更新")
        stalled = self._map_stalled(source_t)
        if stalled is not None:
            status.append(f"地图断流 {stalled:.0f}s")
        pose = self.pose_at(source_t)
        if pose is None:
            status.append("位姿：无")
        elif source_t - pose.t > 2.0:
            status.append(f"位姿中断 {source_t - pose.t:.0f}s")
        else:
            status.append("位姿：有效")
        status.append("相机：正常" if self._frame_at(source_t) is not None
                      else "相机：中断")
        if len(self.trajectory.segments) > 1:
            status.append(f"轨迹分段 {len(self.trajectory.segments)}")
        self._text(panel.x + 6, legend_y + self._fs(30),
                   " · ".join(status), 18, MUTED)

        notes = self.metadata.notes if self.metadata else []
        if notes:
            self._text(panel.right - 6, legend_y + self._fs(30),
                       _elide("；".join(notes), 56), 18, AMBER, align="right")

    # -- text ----------------------------------------------------------- #
    def _text_box(self, x: int, y: int, text: str, size: int,
                  *, align: str = "left") -> tuple[int, int, int, int]:
        from PIL import Image, ImageDraw

        font = self._font(self._fs(size))
        anchor = {"left": "la", "center": "ma", "right": "ra"}[align]
        box = ImageDraw.Draw(Image.new("L", (1, 1))).textbbox(
            (0, 0), text, font=font, anchor=anchor,
        )
        return (int(x + box[0]), int(y + box[1]),
                int(x + box[2]), int(y + box[3]))

    def _map_plate_box(self, canvas: np.ndarray,
                       left: int, top: int, right: int, bottom: int) -> None:
        """按底板色做 78% 混合：有对比又不盖住底下的地图。"""
        pad_x = max(5, self._fs(9))
        pad_y = max(3, self._fs(5))
        left = max(0, left - pad_x)
        top = max(0, top - pad_y)
        right = min(canvas.shape[1], right + pad_x)
        bottom = min(canvas.shape[0], bottom + pad_y)
        if right <= left or bottom <= top:
            return
        region = canvas[top:bottom, left:right]
        plate = np.array(BG, dtype=np.float32)
        canvas[top:bottom, left:right] = (
            region.astype(np.float32) * 0.22 + plate * 0.78
        ).astype(np.uint8)

    def _map_plate(self, canvas: np.ndarray, x: int, y: int, text: str,
                   size: int, *, align: str = "left") -> None:
        """给直接画在地图上的文字垫深色底板。

        地图底色常常是大片自由区（纯白），浅色文字直接画上去几乎看不见。
        """
        if not text:
            return
        self._map_plate_box(canvas, *self._text_box(x, y, text, size,
                                                    align=align))

    def _compose_text_live(self, canvas: np.ndarray, source_t: float) -> None:
        panel = self.map_panel
        index = bisect_right(self._event_times, source_t)
        banner: tuple[str, Any, EventRow] | None = None
        for event in reversed(self.recording.events[max(0, index - 6):index]):
            if source_t - event.t > 8.0:
                break
            if event.event_type not in BANNER_EVENTS:
                continue
            style = EVENT_STYLE[event.event_type]
            banner = (style[0], style[1], event)
            break
        if banner is not None:
            label, color, event = banner
            text = f"{_fmt_clock(event.t - self.first_t)}  {label}"
            detail = _event_detail(event)
            if detail:
                text += f" · {detail}"
            text = _elide(text, 48)
            self._map_plate(canvas, panel.x + 12, panel.y + 10, text, 22)
            self._text(panel.x + 12, panel.y + 10, text, 22, color)

        warnings: list[str] = []
        stalled = self._map_stalled(source_t)
        if self._map_at(source_t) is None:
            warnings.append("本段地图不可用")
        elif stalled is not None:
            warnings.append(f"地图源已 {stalled:.0f} 秒无更新")
        pose = self.pose_at(source_t)
        if pose is not None and source_t - pose.t > 2.0:
            warnings.append(f"位姿中断 {source_t - pose.t:.0f} 秒")
        if warnings:
            text = " · ".join(warnings)
            self._map_plate(canvas, panel.x + panel.width // 2,
                            panel.bottom - 46, text, 20, align="center")
            self._text(panel.x + panel.width // 2, panel.bottom - 46,
                       text, 20, AMBER, align="center")

    def _compose_text_end(self, canvas: np.ndarray) -> None:
        panel = self.map_panel
        manifest = self.recording.manifest
        result = _manifest_result(manifest)
        label = RESULT_LABELS.get(result, result or "结束")
        title = _elide(_manifest_target(manifest), 32)
        self._map_plate(canvas, panel.x + 16, panel.y + 14, title, 30)
        self._text(panel.x + 16, panel.y + 14, title, 30, TEXT)
        duration = self.last_t - self.first_t
        source_label = ("轮式里程" if self.trajectory.distance_source == "odom"
                        else "位姿路径估计")
        result_color = (GREEN if result == "TARGET_FOUND"
                        else RED if result == "FAILED"
                        else AMBER if result else TEXT)
        lines = [
            (f"结果：{label}", result_color),
            (f"记录时长：{_fmt_clock(duration)}", TEXT),
            (f"有效累计行程：{self.trajectory.distance_m:.2f} m"
             f"（{source_label}）", TEXT),
            (f"净位移：{self.trajectory.net_displacement_m:.2f} m", TEXT),
            (f"轨迹分段：{len(self.trajectory.segments)} 段", TEXT),
        ]
        line_height = self._fs(34)
        y = panel.bottom - 16 - line_height * len(lines)
        # 多行共用一块底板，避免行间出现横一道竖一道的接缝。
        boxes = [self._text_box(panel.x + 16, y + index * line_height,
                                line, 24)
                 for index, (line, _color) in enumerate(lines)]
        self._map_plate_box(
            canvas,
            min(box[0] for box in boxes), min(box[1] for box in boxes),
            max(box[2] for box in boxes), max(box[3] for box in boxes),
        )
        for line, color in lines:
            self._text(panel.x + 16, y, line, 24, color)
            y += line_height
        note = "历史定位轨迹，未经回环优化"
        self._map_plate(canvas, panel.right - 16, panel.bottom - 30, note, 17,
                        align="right")
        self._text(panel.right - 16, panel.bottom - 30, note, 17, MUTED,
                   align="right")

    def _fs(self, size: int) -> int:
        return max(10, int(round(size * self.width / 1920.0)))

    def _text(self, x: int, y: int, text: str, size: int, color,
              *, align: str = "left") -> None:
        self._pending_text.append(
            (int(x), int(y), str(text), self._fs(size), tuple(color), align)
        )

    def _flush_text(self, canvas: np.ndarray) -> None:
        if not self._pending_text:
            return
        from PIL import Image, ImageDraw

        image = Image.fromarray(canvas)
        draw = ImageDraw.Draw(image)
        for x, y, text, size, color, align in self._pending_text:
            if not text:
                continue
            if len(text) > 200:
                text = text[:199] + "…"
            font = self._font(size)
            anchor = {"left": "la", "center": "ma", "right": "ra"}[align]
            try:
                draw.text((x, y), text, font=font, fill=color, anchor=anchor)
            except (ValueError, OSError):
                draw.text((max(0, x), max(0, y)), text, font=font, fill=color)
        canvas[...] = np.asarray(image)
        self._pending_text.clear()

    def _font(self, size: int):
        size = max(10, int(size))
        font = self._fonts.get(size)
        if font is None:
            font = _load_font(self.config.font_path, size)
            self._fonts[size] = font
        return font

    # ------------------------------------------------------------------ #
    # artifacts                                                          #
    # ------------------------------------------------------------------ #
    def render(
        self,
        frame_sink: Callable[[np.ndarray], None],
        *,
        progress: Callable[[float], None] | None = None,
    ) -> RenderMetadata:
        metadata = self.prepare()
        total = metadata.frames
        interval = max(1, total // 50)
        for index in range(total):
            frame_sink(self.render_frame(index / self.fps))
            if progress is not None and (index % interval == 0 or index == total - 1):
                progress((index + 1) / max(1, total))
        return metadata

    def save_final_map(self, path: Path) -> bool:
        """Final map PNG: last map + all segments of the estimated trajectory."""
        import cv2

        if not self._prepared:
            self.prepare()
        width, height = 1800, 1200
        canvas = np.full((height, width, 3), MAP_UNKNOWN, dtype=np.uint8)
        panel = _Panel(0, 0, width, height)
        last_map = self._map_at(self.last_t)
        if last_map is not None:
            self._draw_map(canvas, panel, self.end_viewport, self.last_t,
                           force_map=last_map)
        self._draw_trajectory(canvas, panel, self.end_viewport, self.last_t,
                              show_all=True)
        self._draw_markers(canvas, panel, self.end_viewport, self.last_t,
                           show_all=True)
        # 成品图自己带上出处：谁看图都不用去翻 manifest。
        self._pending_text.clear()
        result = _manifest_result(self.recording.manifest)
        label = RESULT_LABELS.get(result, result or "结束")
        caption = (f"目标：{_manifest_target(self.recording.manifest)}"
                   f"  ·  结果：{label}"
                   f"  ·  有效累计行程 {self.trajectory.distance_m:.2f} m"
                   f"  ·  历史定位轨迹，未经回环优化")
        self._text(28, height - 52, caption, 26, MAP_OCCUPIED)
        self._text(28, 24, "最终二维地图 + 实测轨迹（绿色）", 30, MAP_OCCUPIED)
        self._flush_text(canvas)
        image = cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR)
        return bool(cv2.imwrite(str(path), image))

    def save_cover(self, path: Path) -> bool:
        """Cover JPEG from a real rendered frame (prefer the last key moment)."""
        if not self._prepared:
            self.prepare()
        target_source = self.first_t + 0.35 * (self.last_t - self.first_t)
        if self.key_moments:
            target_source = self.key_moments[-1].t
        video_t = self.timemap.source_to_video(target_source)
        frame = self.render_frame(video_t)
        from PIL import Image

        Image.fromarray(frame).save(path, "JPEG", quality=88)
        return True


# --------------------------------------------------------------------------- #
# helpers                                                                    #
# --------------------------------------------------------------------------- #
def _yaw_from_quaternion(quaternion) -> float:
    qx, qy, qz, qw = (float(value) for value in quaternion)
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def _fill_rect(canvas: np.ndarray, panel: _Panel, color) -> None:
    canvas[panel.y:panel.bottom, panel.x:panel.right] = color


def _event_detail(event: EventRow) -> str:
    payload = event.payload or {}
    for key in ("reason_zh", "phase", "message", "reason", "label", "target"):
        value = payload.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return ""


def _manifest_target(manifest: dict[str, Any]) -> str:
    for container in (manifest, manifest.get("params") or {},
                      manifest.get("search") or {}):
        if not isinstance(container, dict):
            continue
        value = container.get("target") or container.get("target_label")
        if isinstance(value, str) and value.strip():
            return value.strip()
    return "未命名目标"


def _manifest_result(manifest: dict[str, Any]) -> str:
    for container in (manifest.get("search") or {}, manifest,
                      manifest.get("result") or {}):
        if not isinstance(container, dict):
            continue
        value = container.get("result") or container.get("status")
        if isinstance(value, str) and value.strip():
            return value.strip()
    return ""


def _manifest_started(manifest: dict[str, Any]) -> str:
    for key in ("started_at_wall", "created_at_wall", "created_at"):
        value = manifest.get(key)
        if isinstance(value, (int, float)) and value > 0:
            import time

            try:
                return time.strftime("%Y-%m-%d %H:%M", time.localtime(value))
            except (OverflowError, OSError, ValueError):
                return ""
    return ""


def _elide(text: str, limit: int) -> str:
    text = " ".join(str(text).split())
    return text if len(text) <= limit else text[: max(1, limit - 1)] + "…"


def _fmt_clock(seconds: float) -> str:
    seconds = max(0.0, float(seconds))
    minutes, sec = divmod(int(seconds), 60)
    hours, minutes = divmod(minutes, 60)
    if hours:
        return f"{hours:02d}:{minutes:02d}:{sec:02d}"
    return f"{minutes:02d}:{sec:02d}"
