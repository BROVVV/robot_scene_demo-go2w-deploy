"""Regression tests for the 12 findings in the 2026-10-21 independent review.

Each test is the inverse of a probe in
``artifacts/go2w_search_replay_review_20261021/reproduce.py``: the review
asserted the defect, these assert the fixed behaviour.  Faults are injected
through fixtures / fake handles / monkeypatch, never by editing production
files in place.
"""

from __future__ import annotations

import asyncio
import json
import math
import time
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from app.manual_web_demo.search_routes import create_search_router
from app.search_replay import storage
from app.search_replay.manager import ReplayManager
from app.search_replay.models import MapSnapshot, ReplayConfig
from app.search_replay.recording import Recording, load_recording
from app.search_replay.renderer import MAP_OCCUPIED, ReplayRenderer
from app.search_replay.trajectory import Viewport
from tests.search_replay_fixture import FIXTURE_SOURCE, make_recording

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _config(root: Path, **overrides) -> ReplayConfig:
    values = dict(
        width=480, height=270, fps=8, end_hold_seconds=0.5,
        key_pad_seconds=0.5, default_speed=10, speeds=(1, 5, 10),
        encoder_preset="ultrafast", encoder_crf=30, root=str(root),
        capture_ready_timeout=1.0, capture_stop_timeout=3.0,
    )
    values.update(overrides)
    return ReplayConfig(**values)


class FakeService:
    def __init__(self, manager: ReplayManager | None) -> None:
        self.replay_manager = manager

    def subscribe_events(self, callback):
        return lambda: None


def _app(manager: ReplayManager) -> FastAPI:
    app = FastAPI()
    router, _hub, _ws = create_search_router(FakeService(manager))
    app.include_router(router)
    return app


def _rewrite_timeline(session_dir: Path, mutate) -> None:
    path = session_dir / "replay" / storage.TIMELINE
    rows, bad = storage.iter_jsonl(path)
    assert bad == 0
    rows = mutate(rows)
    path.write_text(
        "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in rows),
        encoding="utf-8",
    )


def _rewrite_poses(session_dir: Path, mutate) -> None:
    path = session_dir / "replay" / storage.POSES
    rows, bad = storage.iter_jsonl(path)
    assert bad == 0
    rows = mutate(rows)
    path.write_text(
        "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in rows),
        encoding="utf-8",
    )


# --------------------------------------------------------------------------- #
# R01 / R11 — map versions                                                  #
# --------------------------------------------------------------------------- #
def test_flushed_map_row_is_playable_only_through_its_stored_marker(
    tmp_path: Path,
) -> None:
    """``stored:false`` 的行只有在出现对应 ``map_stored`` 标记后才进索引。

    采集器会先刷时间线、再按 1 Hz 落盘；落盘成功的版本用 ``map_stored``
    补认。没有标记的 ``stored:false`` 行是被限频合并掉的中间版本。
    """
    sid = "search_20261021_150000_stored"
    session = make_recording(tmp_path, session_id=sid, seconds=5.0,
                             camera_fps=0.5, map_hz=1.0)
    stored_hash = "fixture0002"

    def mutate(rows):
        marker_t = None
        for row in rows:
            if row.get("kind") == "map":
                row["stored"] = False
                if row.get("hash") == stored_hash:
                    marker_t = row["t"]
        rows.append({
            "kind": "map_stored", "t": marker_t, "wall": marker_t,
            "hash": stored_hash, "stored": True,
        })
        return rows

    _rewrite_timeline(session, mutate)
    recording = load_recording(session)
    assert [item.content_hash for item in recording.maps] == [stored_hash]
    assert recording.missing_maps == 0
    # 标记行不是 /map 消息，不能进"地图源是否断流"的时间轴。
    map_rows = [row for row in recording.timeline
                if row.get("kind") == "map"]
    assert len(recording.map_message_times) == len(map_rows)


def test_missing_map_file_falls_back_and_is_counted(tmp_path: Path) -> None:
    sid = "search_20261021_150100_missing"
    session = make_recording(tmp_path, session_id=sid, seconds=5.0,
                             camera_fps=0.5, map_hz=1.0)
    replay = session / "replay"
    (replay / "maps" / "fixture0003.npz").unlink()

    recording = load_recording(session)
    assert "fixture0003" in recording.missing_map_hashes
    assert recording.missing_maps == 1
    assert all(item.content_hash != "fixture0003" for item in recording.maps)
    assert any("地图版本文件缺失" in note for note in recording.coverage_notes())

    # 渲染端必须回退到上一张能读的图，而不是选一张空的/坏的。
    renderer = object.__new__(ReplayRenderer)
    renderer.recording = recording
    renderer._map_times = [item.t for item in recording.maps]
    renderer._map_index = list(recording.maps)
    missing_t = next(
        float(row["t"]) for row in recording.timeline
        if row.get("kind") == "map" and row.get("hash") == "fixture0003"
    )
    chosen = renderer._map_at(missing_t + 0.01)
    assert chosen is not None
    assert chosen.content_hash != "fixture0003"
    assert recording.map_grid(chosen) is not None


def test_map_content_revisit_keeps_the_last_time_reference(tmp_path: Path) -> None:
    """A→B→A：末尾必须还是 A 的那次引用，而不是被去重成 B。"""
    sid = "search_20261021_150200_revisit"
    session = make_recording(tmp_path, session_id=sid, seconds=3.0,
                             camera_fps=0.5, map_hz=1.0)
    replay = session / "replay"
    # 三个不同内容文件，按 A→B→A 的顺序引用。
    for name in ("A", "B"):
        source = replay / "maps" / ("fixture0000.npz" if name == "A"
                                    else "fixture0001.npz")
        (replay / "maps" / f"{name}.npz").write_bytes(source.read_bytes())

    def mutate(rows):
        maps = [row for row in rows if row.get("kind") == "map"]
        for row, digest in zip(maps, ("A", "B", "A")):
            row["hash"] = digest
            row["new_content"] = True
        for row in maps[3:]:
            row["new_content"] = False
        return rows

    _rewrite_timeline(session, mutate)
    recording = load_recording(session)
    assert [item.content_hash for item in recording.maps] == ["A", "B", "A"]
    last_t = recording.last_t
    assert recording.map_at(last_t).content_hash == "A"


# --------------------------------------------------------------------------- #
# R02 — rotated origin                                                       #
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("yaw", [0.0, math.pi / 2, -math.pi / 2, 0.35])
def test_rotated_grid_cell_center_lands_on_its_world_position(
    tmp_path: Path, yaw: float,
) -> None:
    grid = np.full((6, 4), -1, dtype=np.int8)
    col, row = 2, 1
    grid[row, col] = 100
    path = tmp_path / f"rotated_{yaw:.3f}.npz"
    np.savez_compressed(path, grid=grid)
    recording = Recording(tmp_path, tmp_path, {})
    renderer = object.__new__(ReplayRenderer)
    renderer.recording = recording
    snapshot = MapSnapshot(
        "rotated", path, 0.0, 0.0, 0.0, "map", 4, 6, 1.0,
        (0.0, 0.0, 0.0, 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)),
    )
    viewport = Viewport(-7.0, -7.0, 7.0, 7.0)
    panel = NS(width=420, height=420)
    layer = renderer._build_map_layer(snapshot, panel, viewport)
    ys, xs = np.nonzero(np.all(layer == np.array(MAP_OCCUPIED), axis=-1))
    assert len(xs) > 0, "占用格没有被画出来"

    world_x = math.cos(yaw) * (col + 0.5) - math.sin(yaw) * (row + 0.5)
    world_y = math.sin(yaw) * (col + 0.5) + math.cos(yaw) * (row + 0.5)
    expected_x, expected_y, _ = viewport.project(
        world_x, world_y, panel_width=panel.width, panel_height=panel.height,
    )
    assert math.hypot(xs.mean() - expected_x, ys.mean() - expected_y) <= 1.5


# --------------------------------------------------------------------------- #
# R03 / R04 — source clock and invalid localization                          #
# --------------------------------------------------------------------------- #
def test_source_clock_rollback_splits_and_stale_stamp_is_not_valid(
    tmp_path: Path,
) -> None:
    sid = "search_20261021_150300_clock"
    session = make_recording(tmp_path, session_id=sid, seconds=5.0,
                             camera_fps=0.5, map_hz=0.5)

    def mutate(rows):
        t0 = min(float(row["t"]) for row in rows if row.get("kind") == "pose")
        for row in rows:
            if row.get("kind") != "pose":
                continue
            offset = float(row["t"]) - t0
            # 2 s 之后源时钟往回退了 10 s（仿真重启后的计数）。
            row["stamp"] = offset - 10.0 if offset >= 2.0 else offset
        return rows

    _rewrite_poses(session, mutate)
    recording = load_recording(session)
    assert any(marker["reason"] == "source_clock_step"
               for marker in recording.segment_markers)
    trajectory = recording.trajectory()
    assert len(trajectory.segments) >= 2
    for first, second in zip(trajectory.segments, trajectory.segments[1:]):
        assert first.end_t < second.start_t

    # 同一源时间戳重复出现 = 缓存 TF，不能当新鲜定位用。
    sid_same = "search_20261021_150310_stale"
    session_same = make_recording(tmp_path, session_id=sid_same, seconds=4.0,
                                  camera_fps=0.5, map_hz=0.5)

    def freeze(rows):
        poses = [row for row in rows if row.get("kind") == "pose"]
        for row in poses[15:20]:
            row["stamp"] = poses[14]["stamp"]
        return rows

    _rewrite_poses(session_same, freeze)
    recording_same = load_recording(session_same)
    stale = [row for row in recording_same.poses if not row.valid]
    assert len(stale) >= 4
    assert all(row.quality == "stale_source_stamp" for row in stale)
    stale_times = {row.t for row in stale}
    assert not [point for point in recording_same.trajectory().points
                if point.t in stale_times]


def test_invalid_pose_rows_are_excluded_from_trajectory(tmp_path: Path) -> None:
    sid = "search_20261021_150400_invalid"
    session = make_recording(tmp_path, session_id=sid, seconds=7.0,
                             camera_fps=0.5, map_hz=0.5)

    def mutate(rows):
        t0 = min(float(row["t"]) for row in rows if row.get("kind") == "pose")
        for row in rows:
            if row.get("kind") != "pose":
                continue
            offset = float(row["t"]) - t0
            if 3.0 <= offset < 4.0:
                row["valid"] = False
                row["quality"] = "localization_invalid"
        return rows

    _rewrite_poses(session, mutate)
    recording = load_recording(session)
    trajectory = recording.trajectory()
    t0 = min(row.t for row in recording.poses)
    blocked = [point for point in trajectory.points
               if 3.0 <= point.t - t0 < 4.0]
    assert blocked == [], "失效位姿不能进入轨迹点（箭头也不能画在上面）"
    assert len(trajectory.segments) >= 2


# --------------------------------------------------------------------------- #
# R05 — file routes must not block the event loop                            #
# --------------------------------------------------------------------------- #
def test_video_route_verification_runs_off_the_event_loop(tmp_path: Path) -> None:
    import httpx

    sid = "search_20261021_150500_video"
    replay = storage.ensure_replay_layout(tmp_path / sid)
    storage.atomic_write_json(replay / storage.MANIFEST, {
        "session_id": sid, "recording_id": "rec_review",
    })
    storage.atomic_write_json(replay / storage.JOB, {
        "status": "READY", "speed": 10,
    })
    (replay / storage.ARTIFACTS_DIR / "replay.mp4").write_bytes(b"stream-bytes")
    manager = ReplayManager(_config(tmp_path), sessions_root=tmp_path,
                            project_root=PROJECT_ROOT)
    app = _app(manager)

    def slow_verify(*args, **kwargs):
        time.sleep(0.3)
        return True, "", {}

    async def scenario():
        transport = httpx.ASGITransport(app=app)
        async with httpx.AsyncClient(transport=transport,
                                     base_url="http://test") as client:
            async def tick():
                start = time.monotonic()
                await asyncio.sleep(0.01)
                return time.monotonic() - start

            with patch("app.search_replay.manager.verify_video",
                       side_effect=slow_verify):
                timer = asyncio.create_task(tick())
                request = asyncio.create_task(client.get(
                    f"/api/search/history/{sid}/replay/video"
                ))
                delay, response = await asyncio.gather(timer, request)
        return delay, response

    try:
        delay, response = asyncio.run(scenario())
    finally:
        manager.shutdown()
    assert response.status_code == 200
    assert response.content == b"stream-bytes"
    # 修复前这里会被 0.3 s 的同步校验拖成 >=0.3 s。
    assert delay < 0.15, f"event loop blocked for {delay:.3f}s"


# --------------------------------------------------------------------------- #
# R06 / R07 / R08 — lifecycle and scheduling                                 #
# --------------------------------------------------------------------------- #
class _FakeHandle:
    def __init__(self, info) -> None:
        self.info = info
        self.running = False
        self.started = 0
        self.stopped = __import__("threading").Event()

    def start(self) -> None:
        self.running = True
        self.started += 1
        self._status("RECORDING", ready=True)

    def _status(self, state: str, *, ready: bool) -> None:
        storage.atomic_write_json(
            storage.replay_dir(self.info.session_dir) / storage.CAPTURE_STATUS,
            {"status": state, "ready": ready, "updated_t": time.monotonic()},
        )

    def wait_ready(self, timeout: float) -> bool:
        return True

    def alive(self) -> bool:
        return self.running

    def send(self, payload: dict) -> bool:
        return True

    def stop(self, timeout: float) -> int:
        self.running = False
        self._status("FINALIZED", ready=False)
        self.stopped.set()
        return 0

    def log_tail(self, limit: int = 600) -> str:
        return ""


def test_finish_during_launch_never_starts_an_orphan_collector(
    tmp_path: Path,
) -> None:
    import threading

    sid = "search_20261021_150600_race"
    (tmp_path / sid).mkdir(parents=True)
    entered = threading.Event()
    release = threading.Event()
    finished_launch = threading.Event()
    handles: list[_FakeHandle] = []

    def launcher(info, config):
        entered.set()
        assert release.wait(5)
        handle = _FakeHandle(info)
        handles.append(handle)
        return handle

    manager = ReplayManager(_config(tmp_path), sessions_root=tmp_path,
                            project_root=PROJECT_ROOT,
                            capture_launcher=launcher)
    manager.start = lambda: None  # 只测启动/收尾两条线程的竞态
    original = manager._launch_capture

    def launch(info):
        try:
            original(info)
        finally:
            finished_launch.set()

    manager._launch_capture = launch
    try:
        manager.bind_session(sid, tmp_path / sid)
        assert entered.wait(3)
        manager.finish_session(sid, {"status": "FAILED"})
        manager._handle_finish(sid, {"status": "FAILED"})
        release.set()
        assert finished_launch.wait(3)
        assert not handles or all(not handle.running for handle in handles), \
            "会话结束后不应再启动采集器"
        assert sid not in manager._collectors
        capture = storage.read_json(
            storage.replay_dir(tmp_path / sid) / storage.CAPTURE_STATUS
        ) or {}
        assert capture.get("status") == "INTERRUPTED"
    finally:
        release.set()


def test_finish_hook_does_not_fsync_on_the_search_thread(tmp_path: Path) -> None:
    sid = "search_20261021_150700_fsync"
    (tmp_path / sid).mkdir(parents=True)
    replay = storage.ensure_replay_layout(tmp_path / sid)
    storage.atomic_write_json(replay / storage.MANIFEST, {
        "session_id": sid, "recording_id": "rec_review",
    })
    manager = ReplayManager(_config(tmp_path), sessions_root=tmp_path,
                            project_root=PROJECT_ROOT)
    manager.start = lambda: None
    try:
        with patch("app.search_replay.storage.os.fsync",
                   side_effect=lambda fd: time.sleep(0.3)):
            start = time.monotonic()
            manager.finish_session(sid, {"status": "FAILED"})
            elapsed = time.monotonic() - start
            assert elapsed < 0.05, f"终态钩子仍同步写盘 {elapsed:.3f}s"
            assert manager.wait_idle(0.2) is False, \
                "收尾还在队列里时 wait_idle 不能报空闲"
            # 收尾线程才是真正写盘的地方。
            assert manager._finalize.get(timeout=1.0)[0] == "finish"
    finally:
        manager._finalize.put(None)


def test_slow_render_no_longer_delays_capture_stop(tmp_path: Path) -> None:
    import threading

    session_a = "search_20261021_150800_render_a"
    session_b = "search_20261021_150810_render_b"
    entered = threading.Event()
    release = threading.Event()
    handles: list[_FakeHandle] = []

    def render(session_id, speed, force):
        if session_id == session_a:
            entered.set()
            assert release.wait(10)
        return {"ok": False}

    def launcher(info, config):
        handle = _FakeHandle(info)
        handles.append(handle)
        return handle

    manager = ReplayManager(_config(tmp_path), sessions_root=tmp_path,
                            project_root=PROJECT_ROOT,
                            capture_launcher=launcher, render_impl=render)
    try:
        manager.start()
        for sid in (session_a, session_b):
            storage.ensure_replay_layout(tmp_path / sid)
        storage.atomic_write_json(
            tmp_path / session_a / "replay" / storage.MANIFEST,
            {"session_id": session_a, "recording_id": "rec_a"},
        )
        manager._enqueue_render(session_a, tmp_path / session_a, 10, force=False)
        assert entered.wait(3)
        manager.bind_session(session_b, tmp_path / session_b)
        deadline = time.monotonic() + 3.0
        while (not handles or not handles[0].running) and time.monotonic() < deadline:
            time.sleep(0.01)
        assert handles and handles[0].running
        manager.finish_session(session_b, {"status": "FAILED"})
        assert handles[0].stopped.wait(2.0), \
            "编码还在跑时，另一个会话的采集器仍必须按时停"
    finally:
        release.set()
        manager.wait_idle(5.0)
        manager.shutdown()


# --------------------------------------------------------------------------- #
# R09 — interrupted capture can never be reported READY                      #
# --------------------------------------------------------------------------- #
def test_interrupted_capture_is_partial_with_a_reason(tmp_path: Path) -> None:
    sid = "search_20261021_150900_interrupted"
    session = make_recording(tmp_path, session_id=sid, seconds=2.0,
                             camera_fps=0.5, map_hz=0.5)
    manifest = storage.read_json(session / "replay" / storage.MANIFEST) or {}
    manifest.pop("recording", None)
    manifest["capture"] = {"state": "INTERRUPTED", "error": "capture lost"}
    storage.atomic_write_json(session / "replay" / storage.MANIFEST, manifest)
    storage.atomic_write_json(session / "replay" / storage.CAPTURE_STATUS, {
        "status": "INTERRUPTED", "ready": False, "error": "capture lost",
    })
    manager = ReplayManager(_config(tmp_path), sessions_root=tmp_path,
                            project_root=PROJECT_ROOT)
    try:
        result = manager.render_offline(sid, speed=10)
        assert result.get("ok"), result
        status = manager.status(sid)
    finally:
        manager.shutdown()
    assert status["status"] == "PARTIAL", status
    assert status["playable"] is True
    assert status["reasons"], "降级必须带原因"
    assert any("INTERRUPTED" in reason for reason in status["reasons"]), status
    assert any("recording" in reason for reason in status["reasons"]), status


# --------------------------------------------------------------------------- #
# R10 — artifact ownership                                                   #
# --------------------------------------------------------------------------- #
def test_symlinked_replay_directory_cannot_serve_outside_files(
    tmp_path: Path,
) -> None:
    root = tmp_path / "sessions"
    root.mkdir()
    sid = "search_20261021_151000_symlink"
    session = root / sid
    session.mkdir()
    outside = tmp_path / "outside"
    (outside / storage.ARTIFACTS_DIR).mkdir(parents=True)
    (session / storage.REPLAY_DIRNAME).symlink_to(outside,
                                                 target_is_directory=True)
    storage.atomic_write_json(outside / storage.MANIFEST, {
        "session_id": sid, "recording_id": "rec_review",
    })
    (outside / storage.ARTIFACTS_DIR / "final_map.png").write_bytes(
        b"EXTERNAL_DIRECTORY_CONTENT"
    )
    manager = ReplayManager(_config(root), sessions_root=root,
                            project_root=PROJECT_ROOT)
    try:
        client = TestClient(_app(manager))
        response = client.get(f"/api/search/history/{sid}/replay/final-map")
        assert response.status_code == 404
        assert b"EXTERNAL" not in response.content
        assert storage.resolve_artifact(session / storage.REPLAY_DIRNAME,
                                        storage.ARTIFACTS_DIR,
                                        "final_map.png") is None
    finally:
        manager.shutdown()


# --------------------------------------------------------------------------- #
# R12 — player switching (runs the production JS with node)                  #
# --------------------------------------------------------------------------- #
def test_player_reloads_src_after_switching_away_and_back() -> None:
    import shutil
    import subprocess

    node = shutil.which("node")
    if node is None:
        pytest.skip("node 不可用，跳过播放器切换回归")
    script = Path(__file__).with_name("test_search_replay_player_switch.js")
    result = subprocess.run([node, str(script)], capture_output=True,
                            text=True, timeout=60)
    assert result.returncode == 0, result.stderr
