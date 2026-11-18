"""Deterministic synthetic replay packages for tests and offline acceptance.

The generator writes exactly the same on-disk shapes the real read-only
collector produces (manifest + capture_status + JSONL + NPZ + JPEG), so the
renderer/encoder/manager code paths under test are the production ones.  Every
row carries a ``source`` marker so an offline demo can never be mistaken for a
real robot run.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np

FIXTURE_SOURCE = "fixture_synthetic"
DEFAULT_SESSION_ID = "search_20261021_120000_fixture"
GRID_SIZE = 240
GRID_RES = 0.05
ORIGIN = (-6.0, -6.0, 0.0, 0.0, 0.0, 0.0, 1.0)


def _write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    # 唯一临时名：归档/会话服务的写手也在写同目录的同名文件，共用一个
    # ``name.tmp`` 会让先完成的一方 replace 掉临时文件、另一方 ENOENT。
    temporary = path.with_name(
        f"{path.name}.tmp-{os.getpid()}-{threading.get_ident()}"
    )
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=1),
                         encoding="utf-8")
    os.replace(temporary, path)


def _append(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False) + "\n")


def _grid(reveal_seconds: float, seconds: float, *, stalled: bool) -> np.ndarray:
    """Occupancy grid slowly revealed around a rectangular path."""
    grid = np.full((GRID_SIZE, GRID_SIZE), -1, dtype=np.int8)
    free_until = 0.0 if stalled else max(0.0, min(1.0, reveal_seconds / seconds))
    radius = 1.2 + 3.2 * free_until
    ys, xs = np.mgrid[0:GRID_SIZE, 0:GRID_SIZE]
    wx = ORIGIN[0] + (xs + 0.5) * GRID_RES
    wy = ORIGIN[1] + (ys + 0.5) * GRID_RES
    inside = (np.abs(wx - 1.0) <= radius) & (np.abs(wy - 1.0) <= radius)
    grid[inside] = 0
    # 墙：一条竖墙与一条横墙，真实占据值 100。
    wall_v = (np.abs(wx - (1.0 + radius - 0.4)) <= 0.05) & (wy <= 1.0 + radius)
    wall_h = (np.abs(wy - (1.0 - radius + 0.4)) <= 0.05) & (wx <= 1.0 + radius)
    grid[wall_v | wall_h] = 100
    return grid


def _trajectory(seconds: float) -> tuple[float, float, float]:
    """A rounded rectangular sweep, in map metres / yaw rad."""
    phase = (seconds % 8.0) / 8.0 * 2.0 * math.pi
    x = 1.0 + 1.5 * math.sin(phase)
    y = 1.0 + 1.0 * math.sin(2.0 * phase)
    dx = 1.5 * math.cos(phase)
    dy = 2.0 * math.cos(2.0 * phase)
    return x, y, math.atan2(dy, dx)


def _frame_jpeg(path: Path, index: int, source_t: float,
                *, width: int = 640, height: int = 480) -> None:
    from PIL import Image, ImageDraw

    image = Image.new("RGB", (width, height), (26, 30, 38))
    draw = ImageDraw.Draw(image)
    # 渐变背景 + 移动方块，保证抽帧校验的亮度方差不为 0。
    for row in range(0, height, 8):
        shade = 20 + int(60 * row / height)
        draw.rectangle([0, row, width, row + 8], fill=(shade, shade + 6, shade + 18))
    x, y, _yaw = _trajectory(source_t)
    box_x = int((x + 3.0) / 6.0 * (width - 90)) + 20
    box_y = int((y + 2.0) / 4.0 * (height - 90)) + 20
    draw.rectangle([box_x, box_y, box_x + 70, box_y + 70], fill=(70, 200, 120))
    draw.text((16, 12), f"fixture frame {index:03d}  t={source_t:6.2f}s",
              fill=(240, 240, 240))
    draw.text((16, height - 26), "SYNTHETIC / NOT ROBOT DATA",
              fill=(250, 120, 120))
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path, "JPEG", quality=82)


def make_recording(
    root: str | Path,
    *,
    session_id: str = DEFAULT_SESSION_ID,
    seconds: float = 12.0,
    pose_hz: float = 10.0,
    odom_hz: float = 20.0,
    camera_fps: float = 2.0,
    map_hz: float = 1.0,
    with_events: bool = True,
    with_invalid_span: bool = False,
    invalid_span: tuple[float, float] = (4.0, 6.0),
    map_stall_after: float | None = None,
    camera_gap_after: float | None = None,
    bad_final_line: bool = False,
    start_gap_s: float = 0.0,
    recording_id: str = "rec_fixture01",
) -> Path:
    """Write a complete replay package and return the session directory."""
    session_dir = Path(root) / session_id
    replay = session_dir / "replay"
    for sub in ("maps", "frames", "artifacts"):
        (replay / sub).mkdir(parents=True, exist_ok=True)
    # 采集器由 --recording-id 决定；测试里可以让"假采集器"沿用管理器
    # 已经写进 manifest 的那个 id，免得包内出现两个录音身份。
    # 与真实采集器同一时基：t 必须是本机 monotonic，事件钩子（走
    # ``ReplayManager.record_event``）记的也是 monotonic。用合成原点会让
    # 事件落在记录窗外几千秒处，把回放时长撑成"几小时"。
    wall0 = time.time() - seconds
    t0 = time.monotonic() - seconds

    poses: list[dict[str, Any]] = []
    odom: list[dict[str, Any]] = []
    timeline: list[dict[str, Any]] = []
    frames: list[dict[str, Any]] = []
    events: list[dict[str, Any]] = []

    step = 1.0 / pose_hz
    count = int(seconds * pose_hz) + 1
    previous: tuple[float, float] | None = None
    invalid_open = False
    for index in range(count):
        offset = index * step
        source_t = t0 + offset
        wall = wall0 + offset
        x, y, yaw = _trajectory(offset)
        if with_invalid_span and invalid_span[0] <= offset < invalid_span[1]:
            timeline.append({
                "kind": "localization", "t": source_t, "wall": wall,
                "valid": False, "blocking": False,
                "reason": "fixture_pose_gap",
            })
            if not invalid_open:
                timeline.append({
                    "kind": "segment", "t": source_t, "wall": wall,
                    "reason": "localization_invalid",
                })
                invalid_open = True
            previous = None
            continue
        if invalid_open:
            # 定位恢复：必须写一条 valid=True 把无效区间关掉。否则区间一直
            # 开到录制结尾，恢复后的正常位姿会被逐点误判成新段（等于把
            # 一次中断画成一地散点）。
            timeline.append({
                "kind": "localization", "t": source_t, "wall": wall,
                "valid": True, "blocking": False,
                "reason": "fixture_pose_recovered",
            })
            invalid_open = False
        if previous is not None and math.hypot(x - previous[0], y - previous[1]) > 0.6:
            timeline.append({
                "kind": "segment", "t": source_t, "wall": wall,
                "reason": "pose_jump",
            })
        previous = (x, y)
        poses.append({
            "kind": "pose", "session_id": session_id,
            "recording_id": recording_id, "segment_id": "seg0",
            "seq": index + 1, "t": source_t, "wall": wall,
            "stamp": source_t - 0.05, "frame_id": "map",
            "x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 5),
            "valid": True, "source": "fixture_tf", "quality": "tf_lookup",
        })

    odom_step = 1.0 / odom_hz
    odom_count = int(seconds * odom_hz) + 1
    for index in range(odom_count):
        offset = index * odom_step
        source_t = t0 + offset
        x, y, yaw = _trajectory(offset)
        odom.append({
            "kind": "odom", "session_id": session_id,
            "recording_id": recording_id, "segment_id": "seg0",
            "seq": index + 1, "t": source_t, "wall": wall0 + offset,
            "stamp": source_t - 0.06, "frame_id": "odom_fused",
            "x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 5),
            "vx": round(0.6, 3),
        })

    map_step = 1.0 / max(0.1, map_hz)
    map_count = int(seconds * max(0.1, map_hz)) + 1
    last_hash = ""
    for index in range(map_count):
        offset = index * map_step
        source_t = t0 + offset
        stalled = map_stall_after is not None and offset >= map_stall_after
        if stalled:
            # 地图源断流：不再有任何 /map 消息（连"内容没变"也不记）。
            continue
        grid = _grid(offset, seconds, stalled=False)
        digest = f"fixture{index:04d}"
        path = replay / "maps" / f"{digest}.npz"
        np.savez_compressed(path, grid=grid)
        timeline.append({
            "kind": "map", "t": source_t, "wall": wall0 + offset,
            "stamp": source_t - 0.04, "frame_id": "map",
            "width": GRID_SIZE, "height": GRID_SIZE,
            "resolution": GRID_RES, "origin": list(ORIGIN),
            "hash": digest, "new_content": digest != last_hash,
            "size_bytes": path.stat().st_size,
        })
        last_hash = digest

    frame_step = 1.0 / max(0.2, camera_fps)
    frame_count = int(seconds * max(0.2, camera_fps)) + 1
    for index in range(frame_count):
        offset = index * frame_step
        if camera_gap_after is not None and offset >= camera_gap_after:
            continue
        source_t = t0 + offset
        name = f"{index + 1:06d}.jpg"
        _frame_jpeg(replay / "frames" / name, index, offset)
        frames.append({
            "kind": "frame", "session_id": session_id,
            "recording_id": recording_id, "segment_id": "seg0",
            "seq": index + 1, "t": source_t, "wall": wall0 + offset,
            "path": name, "width": 640, "height": 480,
            "quality": "http_receive_time", "gap_s": round(frame_step, 3),
        })

    if with_events:
        samples = [
            (2.0, "TARGET_CANDIDATE", {"target": "垃圾桶", "confidence": 0.41}),
            (5.5, "VERIFICATION_STARTED", {"target": "垃圾桶"}),
            (8.0, "TARGET_CONFIRMED", {"target": "垃圾桶", "confidence": 0.83}),
            (seconds - 0.2, "SEARCH_FINISHED",
             {"result": "TARGET_FOUND", "finish_reason": "target_confirmed"}),
        ]
        for offset, event_type, payload in samples:
            events.append({
                "kind": "search_event", "session_id": session_id,
                "t": t0 + offset, "wall": wall0 + offset,
                "event_type": event_type, "payload": payload,
                "cycle": 1, "event_id": f"fixture-{event_type}",
            })

    timeline.insert(0, {
        "kind": "start", "t": t0, "wall": wall0, "session_id": session_id,
        "recording_id": recording_id, "read_only": True,
        "source": FIXTURE_SOURCE,
    })
    timeline.append({"kind": "stop", "t": t0 + seconds,
                     "wall": wall0 + seconds, "source": FIXTURE_SOURCE})

    _append(replay / "poses.jsonl", poses + odom)
    if bad_final_line:
        with (replay / "poses.jsonl").open("a", encoding="utf-8") as handle:
            handle.write('{"kind":"pose","t":')
    _append(replay / "timeline.jsonl", timeline)
    _append(replay / "frames.jsonl", frames)
    _append(replay / "events.jsonl", events)

    manifest = {
        "schema_version": "go2w_search_replay_v1",
        "manager": "fixture",
        "session_id": session_id,
        "recording_id": recording_id,
        "source": FIXTURE_SOURCE,
        "created_at": t0,
        "created_at_wall": wall0,
        "target": "垃圾桶",
        "task_text": "搜索垃圾桶",
        "backend": "nav2",
        "params": {"target": "垃圾桶"},
        "search": {
            "target": "垃圾桶",
            "task_text": "搜索垃圾桶",
            "backend": "nav2",
            "status": "TARGET_FOUND",
            "result": "TARGET_FOUND",
        },
        "capture": {
            "state": "FINALIZED", "ready": True,
            "start_gap_s": start_gap_s,
        },
        "recording": {
            "collector": "fixture",
            "read_only": True,
            "quality": "FULL",
            "notes": [],
            "counts": {
                "poses": len(poses), "odom": len(odom),
                "frames": len(frames), "events": len(events),
                "maps_stored": map_count,
            },
        },
        "render": {},
    }
    _write_json(replay / "manifest.json", manifest)
    _write_json(replay / "capture_status.json", {
        "schema_version": "go2w_search_replay_v1",
        "session_id": session_id,
        "recording_id": recording_id,
        "status": "FINALIZED",
        "ready": True,
        "read_only": True,
        "source": FIXTURE_SOURCE,
        "counts": {
            "poses": len(poses), "odom": len(odom), "frames": len(frames),
            "events": len(events), "map_messages": map_count,
            "maps_stored": map_count, "maps_merged": 0,
        },
        "camera": {"state": "fixture", "frames": len(frames)},
        "degraded": [],
        "notes": [],
        "updated_t": t0 + seconds,
    })
    # 会话状态文件归 WebUI 自己写；只有在离线单独生成录制包（这些文件还
    # 不存在）时才補一份，免得假采集器把正在跑的搜索状态覆盖成终态。
    if not (session_dir / "webui_session.json").exists():
        _write_json(session_dir / "webui_session.json", {
            "schema_version": "webui_search_archive_v1",
            "session_id": session_id,
            "status": "TARGET_FOUND",
            "result": "TARGET_FOUND",
            "target": "垃圾桶",
            "task_text": "搜索垃圾桶",
            "backend": "nav2",
            "created_at": wall0,
            "updated_at": wall0 + seconds,
            "finished_at": wall0 + seconds,
        })
    if not (session_dir / "webui_state.json").exists():
        _write_json(session_dir / "webui_state.json", {
            "session_id": session_id, "status": "TARGET_FOUND",
            "result": "TARGET_FOUND", "target": "垃圾桶",
            "started_at": wall0, "finished_at": wall0 + seconds,
            "cycle": 1, "elapsed_seconds": seconds,
        })
    return session_dir


def _main() -> int:
    import sys

    if len(sys.argv) < 2:
        print("usage: search_replay_fixture.py <output_root> [session_id]")
        return 2
    session_id = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_SESSION_ID
    path = make_recording(sys.argv[1], session_id=session_id)
    print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
