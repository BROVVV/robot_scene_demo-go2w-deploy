"""端到端（离线）：真实 SearchSessionService 驱动一次搜索，回放自动成片。

链路与真机一致：start_search → bind_session（注入的假采集器）→ store 事件
钩子写 events.jsonl → 终态 finish_session → 后台渲染 → 历史页可播放/下载。
唯一被替换的是 ROS 采集子进程；渲染、编码、归档、保护钩子都是生产代码。
"""

from __future__ import annotations

import time
from pathlib import Path
from types import SimpleNamespace
import threading

import pytest

from app.manual_web_demo.control_ownership import ControlOwner
from app.manual_web_demo.search_models import SearchStartRequest
from app.manual_web_demo.search_session_service import (
    SearchSessionService, make_mock_executor_factory,
)
from app.search_replay import storage
from app.search_replay.manager import ReplayManager
from app.search_replay.models import ReplayConfig
from tests.search_replay_fixture import FIXTURE_SOURCE, make_recording

PROJECT_ROOT = Path(__file__).resolve().parents[1]


class FixtureCapture:
    """假采集器：按真实布局往会话目录写一段合成只读记录。

    真正的 ``record_search_replay_ros.py`` 也是这个形状 —— 管理器先把
    manifest/capture_status 建好，采集器再往同一个 replay 目录追加数据。
    """

    def __init__(self, info, config, *, seconds: float) -> None:
        self.info = info
        self.replay = storage.replay_dir(info.session_dir)
        self.seconds = seconds
        self.commands: list[dict] = []
        self._alive = True
        self._started = threading.Event()
        self._stopped = False

    def start(self) -> None:
        try:
            make_recording(
                self.info.session_dir.parent,
                session_id=self.info.session_id,
                recording_id=self.info.recording_id,
                seconds=self.seconds,
                camera_fps=1.0,
            )
        finally:
            self._started.set()
        # 停止可能先到（短搜索里 finish 比假采集器落盘还早）：不要再把
        # FINALIZED 覆盖回 RECORDING，否则管理器会以为还在录。
        if not self._stopped:
            self._status("RECORDING", ready=True)

    def _status(self, status: str, *, ready: bool, error: str = "") -> None:
        storage.atomic_write_json(self.replay / storage.CAPTURE_STATUS, {
            "schema_version": "go2w_search_replay_v1",
            "session_id": self.info.session_id,
            "recording_id": self.info.recording_id,
            "status": status, "ready": ready, "read_only": True,
            "error": error, "updated_t": time.monotonic(),
            "updated_wall": time.time(),
        })

    def wait_ready(self, timeout: float) -> bool:
        return self._started.wait(timeout)

    def alive(self) -> bool:
        return self._alive

    def send(self, payload: dict) -> bool:
        self.commands.append(payload)
        if payload.get("cmd") == "stop":
            self._stopped = True
            self._status("FINALIZED", ready=False)
            self._alive = False
        return True

    def stop(self, timeout: float) -> int:
        self.send({"cmd": "stop"})
        return 0

    def log_tail(self, limit: int = 600) -> str:
        return ""


def _wait_status(service: SearchSessionService, statuses: set[str],
                 timeout: float = 90.0) -> str:
    deadline = time.monotonic() + timeout
    last = ""
    while time.monotonic() < deadline:
        session = service.current_session()
        last = session.status if session else "IDLE"
        if last in statuses:
            return last
        time.sleep(0.05)
    raise AssertionError(f"timed out waiting for {statuses}, last={last}")


def test_search_lifecycle_produces_playable_replay(tmp_path: Path) -> None:
    sessions_root = tmp_path / "live_runs"
    config = ReplayConfig(
        width=480, height=270, fps=8, end_hold_seconds=0.5, key_pad_seconds=0.5,
        default_speed=10, speeds=(1, 5, 10), encoder_preset="ultrafast",
        encoder_crf=30, root=str(sessions_root),
    )
    manager = ReplayManager(
        config, sessions_root=sessions_root, project_root=PROJECT_ROOT,
        source=FIXTURE_SOURCE,
        capture_launcher=lambda info, cfg: FixtureCapture(info, cfg, seconds=3.0),
    )
    service = SearchSessionService(
        owner=ControlOwner(),
        executor_factory=make_mock_executor_factory(
            scene_steps=[{
                "objects": ["blue trash bin"], "target_present": True,
                "target_score": 0.95, "bundle_id": "obs_first",
            }],
            confirm_after_seen=1,
        ),
        session_dir=str(sessions_root),
        replay_manager=manager,
    )
    try:
        started = service.start_search(SearchStartRequest.from_dict({
            "target": "蓝色垃圾桶", "backend": "mock",
            "finish_on_visual_confirmation": True,
        }))
        assert started["ok"] is True, started
        session_id = started["session_id"]
        _wait_status(service, {"TARGET_FOUND", "FINISHED", "FAILED",
                               "SEARCH_EXHAUSTED", "OPERATOR_STOP"})
        assert manager.wait_idle(180), "回放渲染未在预算内完成"

        replay = sessions_root / session_id / "replay"
        status = manager.status(session_id)
        assert status["status"] == "READY", status
        assert status["playable"] is True
        assert (replay / "artifacts" / "replay.mp4").is_file()
        assert (replay / "artifacts" / "final_map.png").is_file()
        assert (replay / "artifacts" / "cover.jpg").is_file()
        # 3 秒短片没有章节（chapter_seconds 默认 1800 s），不应留下空文件。
        assert not (replay / "artifacts" / "chapters.json").exists()

        rows, bad = storage.iter_jsonl(replay / storage.EVENTS)
        assert bad == 0
        kinds = {row["event_type"] for row in rows}
        assert {"SESSION_CREATED", "TASK_UNDERSTANDING"} <= kinds
        assert any(row["event_type"] in {"TARGET_CONFIRMED", "SEARCH_FINISHED"}
                   for row in rows)
        # 事件时间必须落在记录窗内：否则回放会被撑成几小时（本次修的回归）。
        first = min(row["t"] for row in rows)
        last = max(row["t"] for row in rows)
        assert last - first < 120.0

        manifest = storage.read_json(replay / storage.MANIFEST) or {}
        assert manifest["source"] == FIXTURE_SOURCE
        assert manifest["session_id"] == session_id

        # 终态后归档不再为回放保留它（历史滚动策略接管）。
        assert manager.session_protected(session_id) is False

        history = service.history()
        ids = [item.get("session_id") for item in history]
        assert session_id in ids
    finally:
        service.shutdown()
        manager.shutdown()


def test_bind_happens_before_first_event(tmp_path: Path) -> None:
    """录制必须在任何运动事件之前绑定，否则前几秒会丢。"""
    sessions_root = tmp_path / "live_runs"
    config = ReplayConfig(
        width=480, height=270, fps=8, default_speed=10, speeds=(10,),
        encoder_preset="ultrafast", encoder_crf=30, root=str(sessions_root),
    )
    seen: list[str] = []

    def launcher(info, cfg):
        handle = FixtureCapture(info, cfg, seconds=2.0)
        original_start = handle.start

        def start() -> None:
            seen.append("collector_started")
            original_start()

        handle.start = start  # type: ignore[method-assign]
        return handle

    manager = ReplayManager(
        config, sessions_root=sessions_root, project_root=PROJECT_ROOT,
        source=FIXTURE_SOURCE, capture_launcher=launcher,
    )
    service = SearchSessionService(
        owner=ControlOwner(),
        executor_factory=make_mock_executor_factory(
            scenario="no_target", mock_target_after=999,
        ),
        session_dir=str(sessions_root),
        replay_manager=manager,
    )
    try:
        started = service.start_search(SearchStartRequest.from_dict({
            "target": "蓝色垃圾桶", "backend": "mock",
        }))
        assert started["ok"] is True, started
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and not seen:
            time.sleep(0.02)
        assert seen == ["collector_started"], "采集器没有在搜索启动时拉起"
        service.stop_search()
        _wait_status(service, {"OPERATOR_STOP", "FINISHED", "FAILED",
                               "TARGET_FOUND", "SEARCH_EXHAUSTED"}, timeout=60)
        manager.wait_idle(120)
    finally:
        service.shutdown()
        manager.shutdown()
