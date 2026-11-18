"""归档滚动清理不能删掉回放仍在用的会话（计划 §18 保护钩子）。"""

from __future__ import annotations

import json
import time
from pathlib import Path

from app.manual_web_demo.search_session_archive import SearchSessionArchive
from app.search_replay import storage
from app.search_replay.manager import ReplayManager
from app.search_replay.models import ReplayConfig
from tests.search_replay_fixture import FIXTURE_SOURCE, make_recording

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _manager(root: Path) -> ReplayManager:
    config = ReplayConfig(
        width=480, height=270, fps=8, default_speed=10, speeds=(10,),
        root=str(root), capture_ready_timeout=1.0,
    )
    # 不 start()：这里只问"这个会话是否受保护"，不需要后台线程。
    return ReplayManager(config, sessions_root=root,
                         project_root=PROJECT_ROOT, source=FIXTURE_SOURCE)


def _archive(root: Path, manager: ReplayManager, *, limit: int) -> SearchSessionArchive:
    return SearchSessionArchive(
        root, max_sessions=limit,
        protect=lambda sid: manager.session_protected(sid),
    )


def _archive_session(archive: SearchSessionArchive, session_id: str,
                     status: str, updated_at: float) -> None:
    archive.begin(session_id, {"target": "垃圾桶"})
    archive.record({"session_id": session_id, "status": status,
                    "finished_at": time.time()})
    marker_path = archive.root / session_id / "webui_session.json"
    marker = json.loads(marker_path.read_text(encoding="utf-8"))
    marker["updated_at"] = updated_at
    marker_path.write_text(json.dumps(marker, ensure_ascii=False), encoding="utf-8")


def _set_capture(root: Path, session_id: str, status: str) -> None:
    path = root / session_id / "replay" / storage.CAPTURE_STATUS
    payload = storage.read_json(path) or {}
    payload.update({"status": status, "ready": status == "RECORDING",
                    "updated_t": time.monotonic()})
    storage.atomic_write_json(path, payload)


def test_archive_prune_keeps_recording_session(tmp_path: Path) -> None:
    root = tmp_path / "live_runs"
    active = "search_20261021_142000_active"
    make_recording(root, session_id=active, seconds=2.0, camera_fps=1.0)
    _set_capture(root, active, "RECORDING")

    manager = _manager(root)
    archive = _archive(root, manager, limit=2)
    now = time.time()
    _archive_session(archive, "search_20261021_142001_old", "TARGET_FOUND", now - 300)
    _archive_session(archive, "search_20261021_142002_mid", "TARGET_FOUND", now - 200)
    # 第三个 begin 触发清理：records 降序 = [mid, old, active]，
    # records[2:] 里的 old 被删，active 因回放保护而保留。
    _archive_session(archive, "search_20261021_142003_new", "TARGET_FOUND", now - 100)

    assert not (root / "search_20261021_142001_old").exists()
    assert (root / active).is_dir()
    assert (root / active / "replay" / storage.MANIFEST).is_file()
    assert (root / "search_20261021_142003_new").is_dir()


def test_protection_lifts_once_artifact_is_ready(tmp_path: Path) -> None:
    root = tmp_path / "live_runs"
    sid = "search_20261021_142100_finished"
    make_recording(root, session_id=sid, seconds=2.0, camera_fps=1.0)
    _set_capture(root, sid, "FINALIZED")
    storage.atomic_write_json(
        root / sid / "replay" / storage.JOB,
        {"schema_version": "go2w_search_replay_v1", "status": "QUEUED",
         "speed": 10, "attempts": 1, "updated_t": time.monotonic()},
    )
    manager = _manager(root)
    try:
        assert manager.session_protected(sid) is True
        job = storage.read_json(root / sid / "replay" / storage.JOB) or {}
        job.update({"status": "READY"})
        storage.atomic_write_json(root / sid / "replay" / storage.JOB, job)
        assert manager.session_protected(sid) is False

        archive = _archive(root, manager, limit=1)
        _archive_session(archive, "search_20261021_142101_newer", "TARGET_FOUND",
                         time.time())
        assert not (root / sid).exists()
    finally:
        manager.shutdown()


def test_rendering_session_and_held_lock_are_protected(tmp_path: Path) -> None:
    root = tmp_path / "live_runs"
    sid = "search_20261021_142200_rendering"
    make_recording(root, session_id=sid, seconds=2.0, camera_fps=1.0)
    _set_capture(root, sid, "FINALIZED")
    storage.atomic_write_json(
        root / sid / "replay" / storage.JOB,
        {"schema_version": "go2w_search_replay_v1", "status": "RENDERING",
         "speed": 10, "attempts": 1, "updated_t": time.monotonic()},
    )
    manager = _manager(root)
    try:
        assert manager.session_protected(sid) is True
        job = storage.read_json(root / sid / "replay" / storage.JOB) or {}
        job.update({"status": "READY"})
        storage.atomic_write_json(root / sid / "replay" / storage.JOB, job)

        lock = storage.FileLock(root / sid / "replay" / storage.LOCK)
        assert lock.acquire(shared=False, timeout=1.0)
        try:
            assert manager.session_protected(sid) is True
        finally:
            lock.release()
        assert manager.session_protected(sid) is False
    finally:
        manager.shutdown()
