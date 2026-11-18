"""ReplayManager lifecycle: bind → record → finish → render → serve.

Uses a fake collector so the test never touches ROS.  The recording package
itself is the deterministic fixture, so the render path is the production one.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from app.search_replay import storage
from app.search_replay.manager import ReplayManager
from app.search_replay.models import ReplayConfig
from tests.search_replay_fixture import FIXTURE_SOURCE, make_recording

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _config(root: Path, **overrides) -> ReplayConfig:
    values = dict(
        width=480, height=270, fps=8, end_hold_seconds=0.5,
        key_pad_seconds=0.5, default_speed=10, speeds=(1, 5, 10),
        encoder_preset="ultrafast", encoder_crf=30, root=str(root),
        capture_ready_timeout=1.0,
    )
    values.update(overrides)
    return ReplayConfig(**values)


class FakeHandle:
    """Stands in for the ROS collector subprocess."""

    def __init__(self, info, config) -> None:
        self.info = info
        self.replay = storage.replay_dir(info.session_dir)
        self.commands: list[dict] = []
        self._alive = True

    def start(self) -> None:
        self._write("RECORDING", ready=True)

    def _write(self, status: str, *, ready: bool, error: str = "") -> None:
        storage.atomic_write_json(self.replay / storage.CAPTURE_STATUS, {
            "schema_version": "go2w_search_replay_v1",
            "session_id": self.info.session_id,
            "recording_id": self.info.recording_id,
            "status": status, "ready": ready, "read_only": True,
            "error": error, "updated_t": time.monotonic(),
            "updated_wall": time.time(),
        })

    def wait_ready(self, timeout: float) -> bool:
        return True

    def alive(self) -> bool:
        return self._alive

    def send(self, payload: dict) -> bool:
        self.commands.append(payload)
        if payload.get("cmd") == "stop":
            self._write("FINALIZED", ready=False)
            self._alive = False
        return True

    def stop(self, timeout: float) -> int:
        self.send({"cmd": "stop"})
        # 真实采集器在收尾时会把 recording 段写进 manifest。成品完整性判据
        # 要读它（缺失即 PARTIAL），所以假件必须照做，否则测试验的不是
        # 生产里的那个正常路径。
        manifest = storage.read_json(self.replay / storage.MANIFEST) or {}
        manifest["recording"] = {
            "collector": "fake", "read_only": True,
            "quality": "FULL", "notes": [], "counts": {},
        }
        storage.atomic_write_json(self.replay / storage.MANIFEST, manifest)
        return 0

    def log_tail(self, limit: int = 600) -> str:
        return ""


def _manager(root: Path) -> ReplayManager:
    return ReplayManager(
        _config(root), sessions_root=root, project_root=PROJECT_ROOT,
        source=FIXTURE_SOURCE,
        capture_launcher=lambda info, config: FakeHandle(info, config),
    )


def _wait_capture_state(root: Path, session_id: str, state: str,
                        timeout: float = 10.0) -> None:
    path = root / session_id / "replay" / storage.CAPTURE_STATUS
    deadline = time.monotonic() + timeout
    last = ""
    while time.monotonic() < deadline:
        payload = storage.read_json(path) or {}
        last = str(payload.get("status") or "")
        if last == state:
            return
        time.sleep(0.05)
    raise AssertionError(f"capture never reached {state}, last={last}")


def _event(event_type: str, cycle: int = 1) -> SimpleNamespace:
    return SimpleNamespace(event_type=event_type, payload={"cycle": cycle},
                           cycle=cycle, event_id=f"e{cycle}")


def test_bind_finish_renders_and_reuses_existing_speed(tmp_path: Path) -> None:
    root = tmp_path / "sessions"
    sid = "search_20261021_130000_lifecycle"
    make_recording(root, session_id=sid, seconds=3.0, camera_fps=1.0)
    manager = _manager(root)
    try:
        bound = manager.bind_session(
            sid, root / sid, target="蓝色垃圾桶", task_text="找到蓝色垃圾桶",
            backend="mock",
        )
        assert bound and bound["enabled"]
        replay = root / sid / "replay"
        manifest = storage.read_json(replay / storage.MANIFEST)
        assert manifest["recording_id"] == bound["recording_id"]
        assert manifest["source"] == FIXTURE_SOURCE
        _wait_capture_state(root, sid, "RECORDING")

        for name in ("SESSION_CREATED", "TASK_UNDERSTANDING", "OBSERVATION"):
            manager.record_event(sid, _event(name))
        assert manager.session_protected(sid) is True

        manager.finish_session(sid, {
            "status": "TARGET_FOUND", "result": "found",
            "finished_at": time.time(),
        })
        assert manager.wait_idle(180), "render did not finish"
        _wait_capture_state(root, sid, "FINALIZED")

        status = manager.status(sid)
        assert status["status"] == "READY", status
        assert status["playable"] is True
        assert status["speed"] == 10
        assert status["can_render"] is True
        assert manager.session_protected(sid) is False

        rows, bad = storage.iter_jsonl(replay / storage.EVENTS)
        assert bad == 0
        assert {"SESSION_CREATED", "TASK_UNDERSTANDING",
                "OBSERVATION"} <= {row["event_type"] for row in rows}

        again = manager.request_render(sid, speed=10)
        assert again["ok"] and again["queued"] is False, again

        other = manager.request_render(sid, speed=5)
        assert other["ok"] and other["queued"] is True, other
        assert manager.wait_idle(180)
        status = manager.status(sid)
        assert status["status"] == "READY" and status["speed"] == 5
        versions = list((replay / "artifacts" / "versions").glob("*.mp4"))
        assert versions, "再生新倍速前应归档旧成品"
    finally:
        manager.shutdown()


def test_abandoned_session_never_renders_and_says_why(tmp_path: Path) -> None:
    root = tmp_path / "sessions"
    sid = "search_20261021_130100_abandon"
    make_recording(root, session_id=sid, seconds=3.0, camera_fps=1.0)
    manager = _manager(root)
    try:
        manager.bind_session(sid, root / sid, target="x", task_text="x",
                             backend="mock")
        _wait_capture_state(root, sid, "RECORDING")
        manager.abandon_session(sid, "任务不可执行：目标过于笼统")
        assert manager.wait_idle(30)
        _wait_capture_state(root, sid, "ABANDONED")

        replay = root / sid / "replay"
        assert not (replay / storage.JOB).exists()
        status = manager.status(sid)
        assert status["status"] == "FAILED"
        assert status["can_render"] is False
        assert any("任务不可执行" in item for item in status["reasons"]), status

        refused = manager.request_render(sid, speed=10)
        assert refused["ok"] is False
        assert "拒绝" in refused["error"]
    finally:
        manager.shutdown()


def test_finish_is_idempotent(tmp_path: Path) -> None:
    root = tmp_path / "sessions"
    sid = "search_20261021_130200_finish_twice"
    make_recording(root, session_id=sid, seconds=2.0, camera_fps=1.0)
    manager = _manager(root)
    try:
        manager.bind_session(sid, root / sid)
        _wait_capture_state(root, sid, "RECORDING")
        manager.finish_session(sid, {"status": "TARGET_FOUND"})
        manager.finish_session(sid, {"status": "TARGET_FOUND"})
        assert manager.wait_idle(180)
        job = storage.read_json(root / sid / "replay" / storage.JOB)
        assert job["attempts"] == 1, job
    finally:
        manager.shutdown()


def test_recovery_marks_orphaned_recording_interrupted(tmp_path: Path) -> None:
    root = tmp_path / "sessions"
    sid = "search_20261021_130300_orphan"
    make_recording(root, session_id=sid, seconds=2.0, camera_fps=1.0)
    replay = root / sid / "replay"
    payload = storage.read_json(replay / storage.CAPTURE_STATUS) or {}
    payload.update({"status": "RECORDING", "ready": True,
                    "updated_t": time.monotonic() - 60.0})
    storage.atomic_write_json(replay / storage.CAPTURE_STATUS, payload)

    manager = _manager(root)
    try:
        manager._housekeeping_once()
        capture = storage.read_json(replay / storage.CAPTURE_STATUS)
        assert capture["status"] == "INTERRUPTED"
        assert "已退出" in capture["error"]
        # A terminal session without a job must be rendered by recovery.
        assert manager.wait_idle(180)
        assert manager.status(sid)["status"] in {"READY", "PARTIAL"}
    finally:
        manager.shutdown()


def test_budget_pruning_keeps_artifacts_and_active_session(tmp_path: Path) -> None:
    root = tmp_path / "sessions"
    old = "search_20261021_130400_old"
    new = "search_20261021_130500_new"
    make_recording(root, session_id=old, seconds=2.0, camera_fps=1.0)
    make_recording(root, session_id=new, seconds=2.0, camera_fps=1.0)
    for sid in (old, new):
        artifacts = root / sid / "replay" / "artifacts"
        artifacts.mkdir(parents=True, exist_ok=True)
        (artifacts / "replay.mp4").write_bytes(b"old-bytes")
    # Make `old` older than `new` for the budget order.
    manifest_path = root / old / "replay" / storage.MANIFEST
    manifest = storage.read_json(manifest_path)
    manifest["created_at_wall"] = 1.0
    storage.atomic_write_json(manifest_path, manifest)

    config = _config(root, global_soft_bytes=1)
    manager = ReplayManager(config, sessions_root=root,
                            project_root=PROJECT_ROOT, source=FIXTURE_SOURCE)
    try:
        with manager._lock:
            manager._active_session = new
        manager._enforce_budgets()
        old_manifest = storage.read_json(
            root / old / "replay" / storage.MANIFEST
        )
        new_manifest = storage.read_json(
            root / new / "replay" / storage.MANIFEST
        )
        assert old_manifest["raw_material"]["pruned"] is True
        assert (root / old / "replay" / "artifacts" / "replay.mp4").is_file()
        assert list((root / old / "replay" / "frames").iterdir()) == []
        assert not (new_manifest.get("raw_material") or {}).get("pruned")
        status = manager.status(old)
        assert status["raw_material"]["pruned"] is True
        assert status["can_render"] is False
    finally:
        manager.shutdown()


def test_housekeeping_leaves_in_flight_render_alone(tmp_path: Path) -> None:
    """编码中（会话锁被持有）时补排默认倍速会把刚成片的 5× 覆盖掉。

    真机验收时实测到：请求 5× 后，后台清理每 20 s 跑一次，看到 RENDERING
    且锁被占用会掉进"没有任务"的兜底分支，按默认倍速再排一次；5× 刚 READY
    就被 10× 重渲染覆盖。这里直接把那一刻的状态摆出来。
    """
    root = tmp_path / "sessions"
    sid = "search_20261021_130600_rendering_lock"
    make_recording(root, session_id=sid, seconds=2.0, camera_fps=1.0)
    replay = root / sid / "replay"
    manifest = storage.read_json(replay / storage.MANIFEST) or {}
    manager = _manager(root)
    fingerprint = manager._fingerprint(str(manifest["recording_id"]), 5)
    storage.atomic_write_json(replay / storage.JOB, {
        "schema_version": "go2w_search_replay_v1",
        "session_id": sid,
        "recording_id": manifest["recording_id"],
        "status": "RENDERING",
        "speed": 5,
        "fingerprint": fingerprint,
        "attempts": 2,
        "progress": 0.4,
        "updated_t": time.monotonic(),
    })
    lock = storage.FileLock(replay / storage.LOCK)
    assert lock.acquire(shared=False, timeout=1.0)
    try:
        manager._recover_session(sid, root / sid)
    finally:
        lock.release()
        manager.shutdown()

    job = storage.read_json(replay / storage.JOB) or {}
    assert job["status"] == "RENDERING", job
    assert job["speed"] == 5, job
    assert job["attempts"] == 2, "编码中不该补排新任务"


def test_offline_render_needs_no_web_session_state(tmp_path: Path) -> None:
    """独立 CLI 路径：只有持久录制包，没有 Web 会话状态文件也能成片。"""
    root = tmp_path / "sessions"
    sid = "search_20261021_130700_offline"
    make_recording(root, session_id=sid, seconds=2.0, camera_fps=1.0)
    for name in ("webui_session.json", "webui_state.json"):
        (root / sid / name).unlink()
    # 录制未结束时必须拒绝：离线路径不能把"还在录"当成"已结束"。
    capture = root / sid / "replay" / storage.CAPTURE_STATUS
    payload = storage.read_json(capture) or {}
    payload.update({"status": "RECORDING", "ready": True,
                    "updated_t": time.monotonic()})
    storage.atomic_write_json(capture, payload)
    manager = _manager(root)
    try:
        refused = manager.render_offline(sid, speed=10)
        assert refused["ok"] is False and "尚未结束" in refused["error"], refused

        payload.update({"status": "FINALIZED", "ready": False,
                        "updated_t": time.monotonic()})
        storage.atomic_write_json(capture, payload)
        result = manager.render_offline(sid, speed=10)
        assert result["ok"] is True and result["status"] == "READY", result
        status = manager.status(sid)
        assert status["playable"] is True and status["speed"] == 10
        assert (root / sid / "replay" / "artifacts" / "replay.mp4").is_file()
    finally:
        manager.shutdown()


def test_record_event_never_blocks_and_counts_drops(tmp_path: Path) -> None:
    """搜索线程不能被回放拖死：队列满时立即丢弃并计数。"""
    root = tmp_path / "sessions"
    sid = "search_20261021_131500_backpressure"
    make_recording(root, session_id=sid, seconds=2.0, camera_fps=1.0)
    config = _config(root, event_queue_capacity=64)
    manager = ReplayManager(
        config, sessions_root=root, project_root=PROJECT_ROOT,
        source=FIXTURE_SOURCE,
        capture_launcher=lambda info, cfg: FakeHandle(info, cfg),
    )
    try:
        # 故意不 start()：没有消费者线程，队列会真的被填满。
        assert manager.bind_session(sid, root / sid)
        capacity = manager._events.maxsize
        assert capacity >= 64
        started = time.monotonic()
        for index in range(capacity + 50):
            manager.record_event(sid, _event("OBSERVATION", index))
        elapsed = time.monotonic() - started
        assert manager._events.full()
        assert manager._dropped_events == 50
        # put_nowait-only 路径：填满队列 + 50 次丢弃也必须毫秒级返回。
        assert elapsed < 1.0, elapsed
    finally:
        manager.shutdown()
