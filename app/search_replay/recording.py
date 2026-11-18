"""Load a persisted replay recording package for rendering / status queries.

读取端只认磁盘上的真实记录：manifest + JSONL + NPZ。这里不做任何"猜最近
一次会话"的动作 —— 调用方已经通过 session_id 解析出确切的会话目录。
"""

from __future__ import annotations

import math
from collections import OrderedDict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from app.search_replay.models import (
    EventRow,
    FrameRow,
    MapSnapshot,
    OdomRow,
    PoseRow,
    ReplayConfig,
    finite_float,
)
from app.search_replay import storage
from app.search_replay.trajectory import Trajectory, build_trajectory


@dataclass
class Recording:
    session_dir: Path
    replay_dir: Path
    manifest: dict[str, Any]
    poses: list[PoseRow] = field(default_factory=list)
    odom: list[OdomRow] = field(default_factory=list)
    frames: list[FrameRow] = field(default_factory=list)
    events: list[EventRow] = field(default_factory=list)
    maps: list[MapSnapshot] = field(default_factory=list)
    timeline: list[dict[str, Any]] = field(default_factory=list)
    # 每条 /map 消息的接收时间（内容没变的也记），用于区分"图没变"与
    # "地图源断流"。
    map_message_times: list[float] = field(default_factory=list)
    segment_markers: list[dict[str, Any]] = field(default_factory=list)
    invalid_spans: list[tuple[float, float]] = field(default_factory=list)
    bad_lines: dict[str, int] = field(default_factory=dict)
    # 地图版本引用存在、但对应 NPZ 不在了（旧包合并版本、磁盘被清理或
    # 采集器落盘失败）。这些版本不能进入播放索引，只能回退到上一有效版本。
    missing_maps: int = 0
    missing_map_hashes: list[str] = field(default_factory=list)
    # 地图网格按需加载并做小容量 LRU；渲染一帧只需要当前那张图。
    _map_cache: OrderedDict[str, np.ndarray] = field(default_factory=OrderedDict)
    _map_failures: set[str] = field(default_factory=set)

    # -- basics --------------------------------------------------------- #
    @property
    def session_id(self) -> str:
        return str(self.manifest.get("session_id") or self.session_dir.name)

    @property
    def recording_id(self) -> str:
        return str(self.manifest.get("recording_id") or "")

    @property
    def first_t(self) -> float | None:
        values = [row.t for row in self.poses]
        values += [row.t for row in self.odom]
        values += [row.t for row in self.frames]
        values += [row.t for row in self.events]
        values += [item.t for item in self.maps]
        values += list(self.map_message_times)
        values = [value for value in values if math.isfinite(value)]
        return min(values) if values else None

    @property
    def last_t(self) -> float | None:
        values = [row.t for row in self.poses]
        values += [row.t for row in self.odom]
        values += [row.t for row in self.frames]
        values += [row.t for row in self.events]
        values += [item.t for item in self.maps]
        values += list(self.map_message_times)
        values = [value for value in values if math.isfinite(value)]
        return max(values) if values else None

    # -- maps ----------------------------------------------------------- #
    def map_grid(self, snapshot: MapSnapshot) -> np.ndarray | None:
        key = snapshot.content_hash
        cached = self._map_cache.get(key)
        if cached is not None:
            self._map_cache.move_to_end(key)
            return cached
        if key in self._map_failures:
            return None
        try:
            with np.load(snapshot.path, allow_pickle=False) as archive:
                grid = np.array(archive["grid"], dtype=np.int8)
        except (OSError, ValueError, KeyError):
            self._map_failures.add(key)
            return None
        self._map_cache[key] = grid
        while len(self._map_cache) > 6:
            self._map_cache.popitem(last=False)
        return grid

    def map_at(self, source_t: float) -> MapSnapshot | None:
        found: MapSnapshot | None = None
        for item in self.maps:
            if item.t <= source_t + 1e-9:
                found = item
            else:
                break
        return found

    def map_near(self, source_t: float, window_s: float = 1.5) -> MapSnapshot | None:
        """Nearest map version within a window (used for event/final frames)."""
        if not self.maps:
            return None
        best = min(self.maps, key=lambda item: abs(item.t - source_t))
        if abs(best.t - source_t) > window_s:
            return None
        return best

    def first_map(self) -> MapSnapshot | None:
        return self.maps[0] if self.maps else None

    def last_map(self) -> MapSnapshot | None:
        return self.maps[-1] if self.maps else None

    # -- frames --------------------------------------------------------- #
    def frame_at(self, source_t: float, *, max_age_s: float = 1.0) -> FrameRow | None:
        found: FrameRow | None = None
        for row in self.frames:
            if row.t <= source_t + 1e-9:
                found = row
            else:
                break
        if found is None or source_t - found.t > max_age_s:
            return None
        return found

    def frame_path(self, row: FrameRow) -> Path | None:
        path = storage.resolve_artifact(
            self.replay_dir, "frames", Path(row.path).name
        )
        return path if path is not None and path.is_file() else None

    # -- trajectory ----------------------------------------------------- #
    def trajectory(self, *, max_jump_m: float = 0.6,
                   max_jump_window_s: float = 0.5,
                   max_odom_speed_mps: float = 1.5) -> Trajectory:
        return build_trajectory(
            self.poses,
            self.odom,
            segment_markers=self.segment_markers,
            invalid_spans=self.invalid_spans,
            max_jump_m=max_jump_m,
            max_jump_window_s=max_jump_window_s,
            max_odom_speed_mps=max_odom_speed_mps,
        )

    def coverage_notes(self) -> list[str]:
        notes: list[str] = []
        if not self.maps:
            notes.append("本段地图不可用：未记录到可读取的 /map 版本")
        if self.missing_maps:
            notes.append(
                f"{self.missing_maps} 个地图版本文件缺失，已回退到上一可用版本"
            )
        if not any(row.valid and not any(start <= row.t <= end
                                        for start, end in self.invalid_spans)
                   for row in self.poses):
            notes.append("没有有效位姿：无法绘制实测轨迹")
        elif any(not row.valid for row in self.poses) or self.invalid_spans:
            notes.append("定位记录存在失效区间，失效时不显示有效当前位置")
        if not self.frames:
            notes.append("没有相机帧：视频只显示文字状态")
        for name, count in self.bad_lines.items():
            if count:
                notes.append(f"{name} 有 {count} 行损坏记录（已跳过）")
        if self._map_failures:
            notes.append(f"{len(self._map_failures)} 个地图文件无法解码，已沿用可读版本")
        return notes


def load_recording(
    session_dir: str | Path,
    *,
    config: ReplayConfig | None = None,
) -> Recording:
    session_path = Path(session_dir)
    replay_path = storage.replay_dir(session_path)
    manifest = storage.read_json(replay_path / storage.MANIFEST) or {}
    recording = Recording(
        session_dir=session_path,
        replay_dir=replay_path,
        manifest=manifest,
    )

    pose_rows, bad = storage.iter_jsonl(replay_path / storage.POSES)
    if bad:
        recording.bad_lines["poses.jsonl"] = bad
    for row in pose_rows:
        kind = str(row.get("kind") or "pose")
        t = finite_float(row.get("t"), None)
        wall = finite_float(row.get("wall"), None)
        if kind == "segment":
            if t is not None:
                recording.segment_markers.append({
                    "t": t, "reason": str(row.get("reason") or "marker")
                })
            continue
        if t is None or wall is None:
            continue
        if kind == "odom":
            x, y, yaw = (finite_float(row.get(key), None)
                         for key in ("x", "y", "yaw"))
            if x is None or y is None or yaw is None:
                continue
            recording.odom.append(OdomRow(
                t=t, wall=wall, x=x, y=y, yaw=yaw,
                vx=finite_float(row.get("vx"), 0.0) or 0.0,
                stamp=finite_float(row.get("stamp"), None),
                frame_id=str(row.get("frame_id") or "odom_fused"),
            ))
            continue
        x, y, yaw = (finite_float(row.get(key), None)
                     for key in ("x", "y", "yaw"))
        if x is None or y is None or yaw is None:
            continue
        recording.poses.append(PoseRow(
            t=t, wall=wall, x=x, y=y, yaw=yaw,
            frame_id=str(row.get("frame_id") or "map"),
            segment_id=str(row.get("segment_id") or "seg0"),
            valid=bool(row.get("valid", True)),
            source=str(row.get("source") or "tf"),
            stamp=finite_float(row.get("stamp"), None),
            quality=str(row.get("quality") or "tf_lookup"),
        ))

    frame_rows, bad = storage.iter_jsonl(replay_path / storage.FRAMES)
    if bad:
        recording.bad_lines["frames.jsonl"] = bad
    for row in frame_rows:
        t = finite_float(row.get("t"), None)
        wall = finite_float(row.get("wall"), None)
        path = str(row.get("path") or "")
        if t is None or wall is None or not path:
            continue
        recording.frames.append(FrameRow(
            t=t, wall=wall, seq=int(finite_float(row.get("seq"), 0) or 0),
            path=path,
            width=int(finite_float(row.get("width"), 0) or 0) or None,
            height=int(finite_float(row.get("height"), 0) or 0) or None,
            quality=str(row.get("quality") or "http_receive_time"),
            gap_s=finite_float(row.get("gap_s"), None),
        ))
    recording.frames.sort(key=lambda item: item.t)

    event_rows, bad = storage.iter_jsonl(replay_path / storage.EVENTS)
    if bad:
        recording.bad_lines["events.jsonl"] = bad
    for row in event_rows:
        t = finite_float(row.get("t"), None)
        wall = finite_float(row.get("wall"), None)
        if t is None or wall is None:
            continue
        recording.events.append(EventRow(
            t=t, wall=wall,
            event_type=str(row.get("event_type") or ""),
            payload=dict(row.get("payload") or {}),
            cycle=int(finite_float(row.get("cycle"), 0) or 0) or None,
        ))
    recording.events.sort(key=lambda item: item.t)

    timeline_rows, bad = storage.iter_jsonl(replay_path / storage.TIMELINE)
    if bad:
        recording.bad_lines["timeline.jsonl"] = bad
    recording.timeline = timeline_rows

    invalid_start: float | None = None
    stored_markers: set[tuple[str, float]] = set()
    for row in timeline_rows:
        if str(row.get("kind") or "") != "map_stored":
            continue
        marker_t = finite_float(row.get("t"), None)
        marker_hash = str(row.get("hash") or "")
        if marker_t is not None and marker_hash:
            stored_markers.add((marker_hash, marker_t))
    for row in timeline_rows:
        kind = str(row.get("kind") or "")
        t = finite_float(row.get("t"), None)
        if t is None:
            continue
        if kind == "map":
            recording.map_message_times.append(t)
            if not bool(row.get("new_content", True)):
                continue
            digest = str(row.get("hash") or "")
            if not digest:
                continue
            # 采集器的 ``stored`` 才是"这是一张可播放版本"的凭据。被限频
            # 合并掉、从未落盘的中间版本写 ``stored: false``，不能进索引。
            # 旧包没有这个字段，只能退回按文件是否存在判断；等待限频时已
            # 提前刷出的行由 ``map_stored`` 标记补认。
            if row.get("stored") is False and (digest, t) not in stored_markers:
                continue
            path = storage.resolve_artifact(
                replay_path, "maps", f"{digest}.npz", must_exist=True
            )
            if path is None or not path.is_file():
                recording.missing_maps += 1
                if digest not in recording.missing_map_hashes:
                    recording.missing_map_hashes.append(digest)
                continue
            origin = row.get("origin") or [0.0] * 7
            try:
                origin_tuple = tuple(float(value) for value in origin[:7])
            except (TypeError, ValueError):
                origin_tuple = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
            while len(origin_tuple) < 7:
                origin_tuple += (0.0 if len(origin_tuple) < 6 else 1.0,)
            recording.maps.append(MapSnapshot(
                content_hash=digest,
                path=path,
                t=t,
                wall=finite_float(row.get("wall"), 0.0) or 0.0,
                stamp=finite_float(row.get("stamp"), None),
                frame_id=str(row.get("frame_id") or "map"),
                width=int(finite_float(row.get("width"), 0) or 0),
                height=int(finite_float(row.get("height"), 0) or 0),
                resolution=finite_float(row.get("resolution"), 0.05) or 0.05,
                origin=origin_tuple,  # type: ignore[arg-type]
            ))
        elif kind == "segment":
            recording.segment_markers.append({
                "t": t, "reason": str(row.get("reason") or "marker")
            })
        elif kind == "localization":
            valid = bool(row.get("valid", True))
            if not valid and invalid_start is None:
                invalid_start = t
            elif valid and invalid_start is not None:
                recording.invalid_spans.append((invalid_start, t))
                invalid_start = None
    if invalid_start is not None:
        last = recording.last_t
        if last is not None:
            recording.invalid_spans.append((invalid_start, last))

    recording.maps.sort(key=lambda item: item.t)
    recording.segment_markers.sort(key=lambda item: float(item.get("t") or 0.0))
    # 不按内容做全局去重：A→B→A 时必须保留 A 的第二次时间引用，否则末尾
    # 画面会停在 B。文件本身仍然只存一份（NPZ 同 hash 不重写）。
    _consume_coordinate_evidence(recording)
    _consume_source_clock(recording)
    return recording


def _consume_coordinate_evidence(recording: Recording) -> None:
    """Use map→odom_fused evidence rows to catch coordinate resets.

    The collector records the transform at 1 Hz as evidence.  A large jump in
    that transform is a coordinate/localization reset even when the source
    stamps keep advancing; the trajectory must not be drawn straight across
    it.  Older packages have the evidence rows but no explicit marker.
    """
    samples: list[dict[str, Any]] = []
    for row in recording.timeline:
        if str(row.get("kind") or "") != "evidence":
            continue
        t = finite_float(row.get("t"), None)
        x = finite_float(row.get("x"), None)
        y = finite_float(row.get("y"), None)
        if t is None or x is None or y is None:
            continue
        samples.append({"t": t, "x": x, "y": y,
                        "yaw": finite_float(row.get("yaw"), 0.0) or 0.0})
    samples.sort(key=lambda item: item["t"])
    for previous, current in zip(samples, samples[1:]):
        gap = current["t"] - previous["t"]
        jump = math.hypot(current["x"] - previous["x"],
                          current["y"] - previous["y"])
        yaw_jump = abs(math.atan2(
            math.sin(current["yaw"] - previous["yaw"]),
            math.cos(current["yaw"] - previous["yaw"]),
        ))
        if gap <= 3.0 and (jump > 1.0 or yaw_jump > math.pi / 4):
            recording.segment_markers.append({
                "t": current["t"], "reason": "map_odom_discontinuity",
            })


def _consume_source_clock(recording: Recording) -> None:
    """Detect source-stamp rollback and non-advancing (cached) TF rows.

    ``t`` is host monotonic, so it stays ordered even when the robot clock
    steps backwards.  The *source* stamp is the only evidence of that step.
    Rows whose source stamp did not advance are diagnostic only: they must not
    count as valid localization or as trajectory points.
    """
    previous: float | None = None
    for row in sorted(recording.poses, key=lambda item: item.t):
        if row.stamp is None or not math.isfinite(row.stamp):
            continue
        if previous is not None:
            delta = row.stamp - previous
            if delta < -0.5:
                recording.segment_markers.append({
                    "t": row.t, "reason": "source_clock_step",
                })
            elif delta <= 0.0:
                row.valid = False
                row.quality = "stale_source_stamp"
        previous = row.stamp
    recording.segment_markers.sort(key=lambda item: float(item.get("t") or 0.0))


def recording_summary(recording: Recording) -> dict[str, Any]:
    first, last = recording.first_t, recording.last_t
    return {
        "session_id": recording.session_id,
        "recording_id": recording.recording_id,
        "source": recording.manifest.get("source") or "unknown",
        "first_t": first,
        "last_t": last,
        "span_seconds": (last - first) if first is not None and last is not None else 0.0,
        "poses": len(recording.poses),
        "odom": len(recording.odom),
        "frames": len(recording.frames),
        "events": len(recording.events),
        "map_versions": len(recording.maps),
        "bad_lines": dict(recording.bad_lines),
        "notes": recording.coverage_notes(),
    }
