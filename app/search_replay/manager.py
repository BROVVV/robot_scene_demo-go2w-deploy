"""Replay lifecycle manager: bind → record → finalize → render → serve.

Wiring rules that keep search safety intact:

* the search callback path only does bounded, non-blocking work (queue puts and
  small atomic JSON writes); it never reads maps, encodes video or waits on a
  child process;
* the collector is a separate ``/usr/bin/python3`` ROS process started before
  the executor works, waiting at most ``capture_ready_timeout`` seconds for its
  readiness flag;
* at most one encoder runs globally (``<root>/encoder.lock``); rendering is
  off the request path and can pause while a new search is active;
* any replay failure only changes replay status — search results, navigation
  readiness, e-stop and motion authorization are never touched.
"""

from __future__ import annotations

import json
import math
import os
import queue
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

from app.search_replay import storage
from app.search_replay.encoder import verify_video
from app.search_replay.models import (
    ACTIVE_REPLAY_STATUSES,
    DEFAULT_SPEEDS,
    DISABLED,
    FAILED,
    FINALIZING,
    PARTIAL,
    PLAYABLE_REPLAY_STATUSES,
    QUEUED,
    READY,
    RECORDING,
    SCHEMA_VERSION,
    STARTING,
    TERMINAL_REPLAY_STATUSES,
    ReplayConfig,
    json_safe,
    load_replay_config,
)
from app.search_replay.timeline import PlaybackTimemap  # noqa: F401  (typing aid)

COLLECTOR_SCRIPT = "scripts/go2w/record_search_replay_ros.py"
TERMINAL_SESSION_STATUSES = frozenset({
    "TARGET_FOUND", "SEARCH_EXHAUSTED", "FAILED", "OPERATOR_STOP",
    "FINISHED", "TASK_REJECTED", "MAX_STEPS_REACHED",
    "MAX_PLANNING_CYCLES_REACHED", "TIMEOUT", "STOPPED",
})
MANAGER_VERSION = "go2w_search_replay_manager_v1"


def _now() -> tuple[float, float]:
    return time.monotonic(), time.time()


def _log(message: str) -> None:
    print(f"[search-replay] {message}", flush=True)


@dataclass
class CaptureInfo:
    session_id: str
    session_dir: Path
    recording_id: str
    started_t: float
    handle: Any | None = None
    ready_t: float | None = None
    start_gap_s: float | None = None
    # 启动线程与收尾线程是两条独立线程：collector 的 handle 在启动线程里
    # 才被赋值。没有这把锁，收尾看到一个 handle=None 就当作"没在采集"直接
    # 返回，随后启动线程再把采集器拉起来 —— 会话已经结束了，录制却刚开始。
    lock: threading.Lock = field(default_factory=threading.Lock)
    cancelled: bool = False


class RosCaptureHandle:
    """A real read-only collector subprocess with a stdin control channel."""

    def __init__(self, command: list[str], *, cwd: Path, log_path: Path,
                 replay_dir: Path) -> None:
        self.command = command
        self.cwd = cwd
        self.log_path = log_path
        self.replay_dir = replay_dir
        self.process: subprocess.Popen | None = None
        self._log_handle = None

    def start(self) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log_handle = self.log_path.open("ab")
        self.process = subprocess.Popen(
            self.command,
            stdin=subprocess.PIPE,
            stdout=self._log_handle,
            stderr=subprocess.STDOUT,
            cwd=str(self.cwd),
            env=os.environ.copy(),
            start_new_session=True,
        )

    @property
    def pid(self) -> int | None:
        return self.process.pid if self.process is not None else None

    def alive(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def wait_ready(self, timeout: float) -> bool:
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            if self.process is not None and self.process.poll() is not None:
                return False
            status = storage.read_json(
                self.replay_dir / storage.CAPTURE_STATUS
            ) or {}
            if status.get("ready"):
                return True
            time.sleep(0.1)
        status = storage.read_json(self.replay_dir / storage.CAPTURE_STATUS) or {}
        return bool(status.get("ready"))

    def send(self, payload: dict[str, Any]) -> bool:
        if self.process is None or self.process.stdin is None:
            return False
        try:
            self.process.stdin.write(
                (json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8")
            )
            self.process.stdin.flush()
            return True
        except (BrokenPipeError, OSError, ValueError):
            return False

    def stop(self, timeout: float) -> int | None:
        if self.process is None:
            return None
        self.send({"cmd": "stop", "reason": "search_finished"})
        try:
            if self.process.stdin is not None:
                self.process.stdin.close()
        except (BrokenPipeError, OSError):
            pass
        try:
            code = self.process.wait(timeout=max(0.5, timeout))
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                code = self.process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                try:
                    code = self.process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    code = -9
        finally:
            if self._log_handle is not None:
                try:
                    self._log_handle.close()
                except OSError:
                    pass
                self._log_handle = None
        return code

    def log_tail(self, limit: int = 600) -> str:
        try:
            data = self.log_path.read_bytes()
        except OSError:
            return ""
        return data[-limit:].decode("utf-8", "replace").strip()


class ReplayManager:
    """Owns replay state for every search session under ``sessions_root``."""

    def __init__(
        self,
        config: ReplayConfig | None = None,
        *,
        sessions_root: str | Path,
        project_root: str | Path | None = None,
        enabled: bool | None = None,
        source: str = "real",
        capture_launcher: Callable[[CaptureInfo, ReplayConfig], Any] | None = None,
        render_impl: Callable[[str, int, bool], dict[str, Any]] | None = None,
        config_path: str | Path | None = None,
        python: str | None = None,
        logger: Callable[[str], None] | None = None,
    ) -> None:
        self.project_root = Path(project_root or Path(__file__).resolve().parents[2])
        self.sessions_root = Path(sessions_root)
        if not self.sessions_root.is_absolute():
            self.sessions_root = (self.project_root / self.sessions_root)
        self.config = config or load_replay_config(self.project_root, config_path)
        self.config_path = Path(config_path) if config_path else None
        self.source = source
        self.enabled = bool(self.config.enabled if enabled is None else enabled)
        self._capture_launcher = capture_launcher
        self._render_impl = render_impl
        self._python = python or os.getenv("GO2W_SEARCH_REPLAY_PYTHON") \
            or "/usr/bin/python3"
        self._logger = logger or _log

        self.root = Path(self.config.root)
        if not self.root.is_absolute():
            self.root = self.project_root / self.root
        self.root.mkdir(parents=True, exist_ok=True)

        self._lock = threading.RLock()
        self._collectors: dict[str, CaptureInfo] = {}
        self._terminal: dict[str, dict[str, Any]] = {}
        self._finished: set[str] = set()
        self._finishing: set[str] = set()
        self._finalize_retry_at: dict[str, float] = {}
        self._abandoned: set[str] = set()
        self._pending_render: set[tuple[str, str]] = set()
        self._rendering: tuple[str, str] | None = None
        self._working = False
        self._active_session: str | None = None
        self._dropped_events = 0
        self._probe_cache: dict[tuple[Any, ...], bool] = {}
        self._started = False
        self._stopping = threading.Event()
        self._pause_render = threading.Event()
        self._events: queue.Queue[dict[str, Any] | None] = queue.Queue(
            maxsize=max(64, int(self.config.event_queue_capacity))
        )
        self._jobs: queue.Queue[tuple[Any, ...] | None] = queue.Queue(
            maxsize=max(4, int(self.config.render_queue_capacity))
        )
        # 收尾（停采集器 + 写终态）与渲染分成两条线程。"停采集器"是秒级、
        # 必须及时的操作，而渲染一次可能好几分钟；串在一条线程上会让会话
        # 结束后采集器继续录、wait_idle 也一直不空闲。
        self._finalize: queue.Queue[tuple[Any, ...] | None] = queue.Queue(
            maxsize=max(4, int(self.config.render_queue_capacity))
        )
        self._event_thread: threading.Thread | None = None
        self._worker_thread: threading.Thread | None = None
        self._finalize_thread: threading.Thread | None = None
        self._housekeeping_thread: threading.Thread | None = None
        self._finalizing = False

    # ------------------------------------------------------------------ #
    # lifecycle                                                          #
    # ------------------------------------------------------------------ #
    def start(self) -> None:
        with self._lock:
            if self._started or not self.enabled:
                self._started = True
                return
            self._started = True
            self._event_thread = threading.Thread(
                target=self._event_loop, name="replay-events", daemon=True,
            )
            self._event_thread.start()
            self._worker_thread = threading.Thread(
                target=self._worker_loop, name="replay-render", daemon=True,
            )
            self._worker_thread.start()
            self._finalize_thread = threading.Thread(
                target=self._finalize_loop, name="replay-finalize", daemon=True,
            )
            self._finalize_thread.start()
            self._housekeeping_thread = threading.Thread(
                target=self._housekeeping_loop, name="replay-recovery",
                daemon=True,
            )
            self._housekeeping_thread.start()

    def shutdown(self, *, timeout: float | None = None) -> None:
        if not self.enabled:
            return
        limit = self.config.capture_stop_timeout if timeout is None else timeout
        self._stopping.set()
        self._pause_render.clear()
        with self._lock:
            handles = list(self._collectors.values())
        for info in handles:
            self._stop_capture(info.session_id, timeout=limit)
        self._flush_events()
        for target in (self._finalize, self._jobs, self._events):
            try:
                target.put_nowait(None)
            except queue.Full:
                pass
        for thread in (self._finalize_thread, self._worker_thread,
                       self._event_thread, self._housekeeping_thread):
            if thread is not None and thread.is_alive():
                thread.join(timeout=3.0)

    # ------------------------------------------------------------------ #
    # search lifecycle hooks                                             #
    # ------------------------------------------------------------------ #
    def bind_session(
        self,
        session_id: str,
        session_dir: str | Path,
        *,
        target: str = "",
        task_text: str = "",
        backend: str = "",
        params: dict[str, Any] | None = None,
    ) -> dict[str, Any] | None:
        """Create the replay package and start the collector (non-blocking)."""
        if not self.enabled:
            return None
        if not storage.valid_session_id(session_id):
            self._logger(f"非法 session_id，跳过回放绑定: {session_id!r}")
            return None
        directory = Path(session_dir)
        try:
            replay = storage.ensure_replay_layout(directory)
        except OSError as exc:
            self._logger(f"无法创建回放目录 {directory}: {exc}")
            return None
        recording_id = f"rec_{os.urandom(6).hex()}"
        stamp, wall = _now()
        manifest = {
            "schema_version": SCHEMA_VERSION,
            "manager": MANAGER_VERSION,
            "session_id": session_id,
            "recording_id": recording_id,
            "source": self.source,
            "created_at": stamp,
            "created_at_wall": wall,
            "target": target,
            "task_text": task_text,
            "backend": backend,
            "params": json_safe(dict(params or {})),
            "search": {
                "target": target, "task_text": task_text,
                "backend": backend, "status": "STARTING", "result": "",
            },
            "capture": {
                "state": STARTING,
                "ready": False,
                "backend": "ros_subprocess",
                "collector": COLLECTOR_SCRIPT,
            },
            "render": {},
        }
        storage.atomic_write_json(replay / storage.MANIFEST, manifest)
        storage.atomic_write_json(replay / storage.CAPTURE_STATUS, {
            "schema_version": SCHEMA_VERSION,
            "session_id": session_id,
            "recording_id": recording_id,
            "status": STARTING,
            "ready": False,
            "read_only": True,
            "managed_by": "replay_manager",
            "updated_t": stamp,
            "updated_wall": wall,
        })
        info = CaptureInfo(
            session_id=session_id, session_dir=directory,
            recording_id=recording_id, started_t=stamp,
        )
        with self._lock:
            self._collectors[session_id] = info
            self._active_session = session_id
            self._pause_render.set()
            self._finished.discard(session_id)
            self._terminal.pop(session_id, None)
        self.start()
        thread = threading.Thread(
            target=self._launch_capture, args=(info,),
            name=f"replay-capture-{session_id}", daemon=True,
        )
        thread.start()
        return {
            "enabled": True,
            "recording_id": recording_id,
            "replay_dir": str(replay),
            "source": self.source,
        }

    def _launch_capture(self, info: CaptureInfo) -> None:
        replay = storage.replay_dir(info.session_dir)
        try:
            handle = (
                self._capture_launcher(info, self.config)
                if self._capture_launcher is not None
                else self._build_ros_capture(info, replay)
            )
            # 赋值与 start 必须与 cancel 互斥：收尾线程要么在这里之前把
            # cancelled 置上（于是根本不启动），要么看得到 handle 并去停它。
            with info.lock:
                if info.cancelled:
                    self._logger(f"采集器已被取消，不再启动 {info.session_id}")
                    return
                info.handle = handle
                handle.start()
        except Exception as exc:  # noqa: BLE001 - replay must never break search
            self._logger(f"采集器启动失败 {info.session_id}: {exc}")
            self._mark_capture_failed(info, f"采集器启动失败: {exc}")
            return
        ready = False
        try:
            ready = bool(handle.wait_ready(self.config.capture_ready_timeout))
        except Exception as exc:  # noqa: BLE001
            self._logger(f"等待采集器就绪异常 {info.session_id}: {exc}")
        with info.lock:
            if info.cancelled:
                # 收尾线程已经在停了（或已停完）。这里再写 RECORDING 会把
                # 已经结束的会话标成"还在录"。
                return
            stamp, _wall = _now()
            if ready:
                info.ready_t = stamp
            gap = stamp - info.started_t
            info.start_gap_s = gap if not ready else max(0.0, gap)
        self._update_manifest(info.session_dir, {
            "capture": {
                "state": RECORDING,
                "ready": ready,
                "ready_t": info.ready_t,
                "start_gap_s": round(gap, 3),
                "start_gap_note": (
                    "" if ready else
                    f"采集器未在 {self.config.capture_ready_timeout:.1f}s 内就绪，"
                    "起始画面可能缺失"
                ),
            },
        })
        if not ready and not handle.alive():
            tail = handle.log_tail() if hasattr(handle, "log_tail") else ""
            self._mark_capture_failed(
                info,
                "采集器启动后立即退出" + (f"：{tail}" if tail else ""),
            )

    def _build_ros_capture(self, info: CaptureInfo, replay: Path) -> Any:
        command = [
            self._python,
            str(self.project_root / COLLECTOR_SCRIPT),
            "--session-dir", str(info.session_dir),
            "--session-id", info.session_id,
            "--recording-id", info.recording_id,
            "--camera-url", self.config.camera_url or "-",
            "--camera-fps", str(self.config.camera_fps),
            "--stdin-control",
        ]
        if self.config_path is not None:
            command.extend(["--config", str(self.config_path)])
        return RosCaptureHandle(
            command, cwd=self.project_root,
            log_path=replay / "collector.log", replay_dir=replay,
        )

    def _mark_capture_failed(self, info: CaptureInfo, reason: str) -> None:
        with info.lock:
            if info.cancelled:
                # 会话已经结束，这次"启动失败"只是被收尾掐掉的副作用。
                return
        replay = storage.replay_dir(info.session_dir)
        status = storage.read_json(replay / storage.CAPTURE_STATUS) or {}
        status.update({
            "session_id": info.session_id,
            "recording_id": info.recording_id,
            "status": FAILED,
            "ready": False,
            "error": reason,
            "updated_t": _now()[0],
            "notes": [reason],
        })
        storage.atomic_write_json(replay / storage.CAPTURE_STATUS,
                                  json_safe(status))
        self._update_manifest(info.session_dir, {
            "capture": {"state": FAILED, "error": reason}
        })
        with self._lock:
            if self._collectors.get(info.session_id) is info:
                self._collectors.pop(info.session_id, None)

    def record_event(self, session_id: str, event: Any) -> None:
        """Bounded enqueue; called from the search store callback."""
        if not self.enabled or not session_id:
            return
        if session_id not in self._collectors and session_id not in self._terminal:
            return
        stamp, wall = _monotonic_for(event)
        payload = getattr(event, "payload", None)
        row = {
            "kind": "search_event",
            "session_id": session_id,
            "t": stamp,
            "wall": wall,
            "event_type": str(getattr(event, "event_type", "") or ""),
            "payload": json_safe(dict(payload) if isinstance(payload, dict) else {}),
            "cycle": getattr(event, "cycle", None),
            "event_id": getattr(event, "event_id", None),
        }
        try:
            self._events.put_nowait(row)
        except queue.Full:
            self._dropped_events += 1

    def finish_session(self, session_id: str, snapshot: dict[str, Any]) -> None:
        """Idempotent terminal hook; never blocks and never renders inline.

        这里只做两件不碰磁盘的事：去重、把收尾任务排进收尾线程。清单写盘
        （fsync）在高负载下可能被拖住几百毫秒，原来放在搜索回调线程上，
        直接拖慢了"搜索结束"这一步。
        """
        if not self.enabled or not session_id:
            return
        queued = False
        with self._lock:
            if session_id in self._finished:
                return
            if session_id in self._abandoned:
                self._finished.add(session_id)
                return
            if self._terminal.get(session_id) is None:
                self._terminal[session_id] = {
                    "status": str(snapshot.get("status") or ""),
                    "result": str(snapshot.get("result") or ""),
                    "finished_at": snapshot.get("finished_at"),
                    "finish_reason": snapshot.get("finish_reason"),
                }
            self._finished.add(session_id)
            # 收尾还在队列里/正在跑时 wait_idle 不能报空闲。
            self._finishing.add(session_id)
        try:
            self._finalize.put_nowait(
                ("finish", session_id, dict(snapshot or {}))
            )
            queued = True
        except queue.Full:
            self._logger("收尾队列已满，收尾推迟到下一次检查")
        finally:
            if not queued:
                with self._lock:
                    self._finished.discard(session_id)
                    self._finishing.discard(session_id)

    def abandon_session(self, session_id: str, reason: str) -> None:
        """Stop recording a session that was rejected before any movement."""
        if not self.enabled or not session_id:
            return
        with self._lock:
            self._abandoned.add(session_id)
            self._finished.add(session_id)
        session_dir = self._session_dir(session_id)
        if session_dir is not None:
            self._update_manifest(session_dir, {
                "search": {"status": "TASK_REJECTED", "result": "",
                           "finish_reason": reason},
                "capture": {"state": "ABANDONED", "note": reason},
            }, only_if_exists=True)
        try:
            self._finalize.put_nowait(("abandon", session_id, reason))
        except queue.Full:
            self._stop_capture(session_id, timeout=2.0)

    def _handle_abandon(self, session_id: str, reason: str) -> None:
        self._stop_capture(session_id, timeout=3.0)
        self._flush_events()
        session_dir = self._session_dir(session_id)
        if session_dir is None:
            return
        replay = storage.replay_dir(session_dir)
        status = storage.read_json(replay / storage.CAPTURE_STATUS) or {}
        status.update({
            "status": "ABANDONED", "ready": False, "error": reason,
            "updated_t": _now()[0],
        })
        try:
            storage.atomic_write_json(replay / storage.CAPTURE_STATUS,
                                      json_safe(status))
        except OSError:
            pass
        self._update_manifest(session_dir, {
            "capture": {"state": "ABANDONED", "note": reason},
        })

    def _stop_capture(self, session_id: str, *, timeout: float) -> None:
        with self._lock:
            info = self._collectors.pop(session_id, None)
        if info is None:
            return
        with info.lock:
            info.cancelled = True
            handle = info.handle
        code = None
        if handle is not None:
            try:
                code = handle.stop(timeout)
            except Exception as exc:  # noqa: BLE001
                self._logger(f"停止采集器异常 {session_id}: {exc}")
        replay = storage.replay_dir(info.session_dir)
        status = storage.read_json(replay / storage.CAPTURE_STATUS) or {}
        if str(status.get("status") or "") not in {"FINALIZED", "FAILED"}:
            if handle is None:
                note = "采集器尚未启动即随会话结束"
            else:
                note = ("采集器未能及时收尾" if code not in (0, None)
                        else "采集器收尾状态缺失")
            status.update({
                "status": "INTERRUPTED",
                "ready": False,
                "error": note,
                "exit_code": code,
                "updated_t": _now()[0],
            })
            try:
                storage.atomic_write_json(replay / storage.CAPTURE_STATUS,
                                          json_safe(status))
            except OSError:
                pass
            self._update_manifest(info.session_dir, {
                "capture": {"state": "INTERRUPTED", "exit_code": code,
                            "error": note},
            })
        if self._dropped_events:
            self._update_manifest(info.session_dir, {
                "events_dropped": self._dropped_events,
            })

    # ------------------------------------------------------------------ #
    # render requests                                                    #
    # ------------------------------------------------------------------ #
    def request_render(self, session_id: str, *, speed: int | None = None,
                       force: bool = False) -> dict[str, Any]:
        session_dir = self._session_dir(session_id)
        if session_dir is None:
            return {"ok": False, "error": "会话不存在"}
        if not self.enabled:
            return {"ok": False, "error": "回放功能已禁用"}
        self.start()
        if session_id in self._abandoned:
            return {"ok": False,
                    "error": "该任务在开始前已被拒绝，没有可回放的记录"}
        if session_id in self._collectors:
            return {"ok": False, "error": "会话仍在录制，结束后才能生成视频"}
        manifest = self._manifest(session_dir)
        content = manifest.get("raw_material") or {}
        if content.get("pruned") and not force:
            return {"ok": False,
                    "error": "原始素材已按预算清理，只能播放现有成品"}
        selected = self._coerce_speed(speed)
        if not self._session_terminal(session_id, session_dir):
            return {"ok": False, "error": "会话尚未结束"}
        manifest = self._manifest(session_dir)
        recording_id = str(manifest.get("recording_id") or "")
        fingerprint = self._fingerprint(recording_id, selected) \
            if recording_id else ""
        with self._lock:
            pending_same = bool(fingerprint) and \
                (session_id, fingerprint) in self._pending_render
            pending_other = any(
                key[0] == session_id for key in self._pending_render
            ) and not pending_same
            rendering = self._rendering
        if pending_same:
            return {"ok": True, "queued": True, "status": QUEUED,
                    "speed": selected, "message": "该倍速已在队列中"}
        if pending_other or (rendering is not None and rendering[0] == session_id):
            return {"ok": False,
                    "error": "该会话正在生成其他倍速的成品，请稍后再试"}
        job = self._existing_job(session_dir, selected)
        if (not force and job.get("status") in PLAYABLE_REPLAY_STATUSES
                and self._coerce_speed(job.get("speed")) == selected
                and job.get("fingerprint") == fingerprint
                and self._video_ok(session_dir)):
            return {"ok": True, "queued": False, "status": job.get("status"),
                    "message": "已存在同倍速成品"}
        if self._enqueue_render(session_id, session_dir, selected, force=force):
            return {"ok": True, "queued": True, "status": QUEUED,
                    "speed": selected}
        return {"ok": False, "error": "已有相同的生成任务在进行"}

    def render_offline(self, session_id: str, *, speed: int | None = None,
                       force: bool = False) -> dict[str, Any]:
        """Synchronously rebuild one video from its persistent package.

        This is the standalone-CLI path (``scripts/go2w/render_search_replay.py``).
        Unlike :meth:`request_render` it does not require the Web session's
        state files: a persistent recording that says it finished is enough,
        so a package copied off the robot host can be rendered anywhere. It
        still takes the global encoder lock and the per-session lock, so it is
        safe to run next to a live Web server.
        """
        session_dir = self._session_dir(session_id)
        if session_dir is None:
            return {"ok": False, "error": "会话不存在"}
        if not self.enabled:
            return {"ok": False, "error": "回放功能已禁用"}
        if session_id in self._collectors:
            return {"ok": False, "error": "会话仍在录制，结束后才能生成视频"}
        capture = storage.read_json(
            storage.replay_dir(session_dir) / storage.CAPTURE_STATUS
        ) or {}
        capture_status = str(capture.get("status") or "").upper()
        if (capture_status not in {"FINALIZED", "FAILED", "INTERRUPTED"}
                and not self._session_terminal(session_id, session_dir)):
            return {"ok": False, "error": "录制尚未结束"}
        manifest = self._manifest(session_dir)
        content = manifest.get("raw_material") or {}
        if content.get("pruned") and not force:
            return {"ok": False, "error": "原始素材已按预算清理，只能播放现有成品"}
        if not str(manifest.get("recording_id") or ""):
            return {"ok": False, "error": "录制包缺少 recording_id，无法生成"}
        selected = self._coerce_speed(speed)
        outcome = self._render_with_encoder(session_id, session_dir,
                                            selected, force)
        return {"ok": True, "speed": selected, **outcome}

    def _enqueue_render(self, session_id: str, session_dir: Path,
                        speed: int, *, force: bool) -> bool:
        if session_id in self._abandoned and not force:
            return False
        manifest = self._manifest(session_dir)
        recording_id = str(manifest.get("recording_id") or "")
        if not recording_id:
            self._write_job(session_dir, {
                "schema_version": SCHEMA_VERSION,
                "session_id": session_id,
                "status": FAILED,
                "speed": speed,
                "error": "录制包缺少 recording_id，无法生成",
                "updated_t": _now()[0],
            })
            return False
        fingerprint = self._fingerprint(recording_id, speed)
        key = (session_id, fingerprint)
        with self._lock:
            if key in self._pending_render:
                return False
            self._pending_render.add(key)
        stamp, wall = _now()
        job = self._existing_job(session_dir, speed)
        job.update({
            "schema_version": SCHEMA_VERSION,
            "session_id": session_id,
            "recording_id": recording_id,
            "status": QUEUED,
            "speed": speed,
            "speeds": list(self._speeds()),
            "fingerprint": fingerprint,
            "queued_t": stamp,
            "queued_wall": wall,
            "updated_t": stamp,
            "progress": 0.0,
            "error": "",
            "reasons": [],
            "attempts": int(job.get("attempts") or 0) + 1,
        })
        try:
            self._write_job(session_dir, job)
            self._jobs.put_nowait(("render", session_id, speed, force))
        except (OSError, queue.Full) as exc:
            with self._lock:
                self._pending_render.discard(key)
            self._logger(f"无法排队回放生成 {session_id}: {exc}")
            return False
        return True

    def _worker_loop(self) -> None:
        while not self._stopping.is_set():
            try:
                item = self._jobs.get(timeout=0.25)
            except queue.Empty:
                continue
            if item is None:
                break
            kind = item[0]
            with self._lock:
                self._working = True
            try:
                if kind == "render":
                    self._handle_render(str(item[1]), int(item[2]),
                                        bool(item[3]))
            except Exception as exc:  # noqa: BLE001 - worker must survive
                self._logger(f"回放任务异常 {kind}: {exc}")
                self._record_job_failure(item, exc)
            finally:
                with self._lock:
                    self._working = False

    def _finalize_loop(self) -> None:
        while not self._stopping.is_set():
            self._retry_finalizations()
            try:
                item = self._finalize.get(timeout=0.25)
            except queue.Empty:
                continue
            if item is None:
                break
            kind = item[0]
            session_id = str(item[1])
            with self._lock:
                self._finalizing = True
            try:
                if kind == "finish":
                    snapshot = item[2] if len(item) > 2 else {}
                    self._handle_finish(session_id, snapshot)
                elif kind == "abandon":
                    self._handle_abandon(session_id, str(item[2]))
            except Exception as exc:  # noqa: BLE001 - worker must survive
                self._logger(f"回放收尾异常 {kind}: {exc}")
                if kind == "finish":
                    # Enqueue success is not durable finalization success.
                    # Retain the terminal snapshot and retry a transient write
                    # failure just as we retry a temporarily full queue.
                    with self._lock:
                        self._finished.discard(session_id)
                        self._finalize_retry_at[session_id] = time.monotonic()+1.0
            finally:
                with self._lock:
                    self._finalizing = False
                    self._finishing.discard(session_id)

    def _retry_finalizations(self) -> None:
        """A full queue defers a terminal hook; it must never discard it."""
        with self._lock:
            pending = [
                (session_id, dict(snapshot))
                for session_id, snapshot in self._terminal.items()
                if session_id not in self._finished
                and session_id not in self._abandoned
                and time.monotonic() >= self._finalize_retry_at.get(session_id, 0.0)
            ]
        for session_id, snapshot in pending:
            if self._stopping.is_set() or self._finalize.full():
                break
            self.finish_session(session_id, snapshot)

    def _handle_finish(self, session_id: str,
                       snapshot: dict[str, Any] | None = None) -> None:
        session_dir = self._session_dir(session_id)
        if session_dir is None:
            return
        snapshot = snapshot or {}
        self._update_manifest(
            session_dir,
            {
                "search": {
                    "status": str(snapshot.get("status") or ""),
                    "result": str(snapshot.get("result") or ""),
                    "finished_at": snapshot.get("finished_at"),
                    "finish_reason": snapshot.get("finish_reason"),
                }
            },
            only_if_exists=True,
        )
        if session_id in self._collectors:
            self._stop_capture(session_id,
                               timeout=self.config.capture_stop_timeout)
        self._flush_events()
        capture = storage.read_json(
            storage.replay_dir(session_dir) / storage.CAPTURE_STATUS
        ) or {}
        current = str(capture.get("status") or "")
        if current in {"FINALIZED", "FAILED", "INTERRUPTED"}:
            self._update_manifest(session_dir, {
                "capture": {"state": current,
                            "stopped_t": _now()[0]},
            })
        else:
            capture.update({
                "status": "FINALIZED", "ready": False,
                "updated_t": _now()[0],
            })
            try:
                storage.atomic_write_json(
                    storage.replay_dir(session_dir) / storage.CAPTURE_STATUS,
                    json_safe(capture),
                )
            except OSError:
                pass
            self._update_manifest(session_dir, {
                "capture": {"state": "FINALIZED",
                            "stopped_t": _now()[0]},
            })
        self._enqueue_render(session_id, session_dir,
                             self._coerce_speed(None), force=False)
        with self._lock:
            if self._active_session == session_id:
                self._active_session = None
                self._pause_render.clear()

    def _handle_render(self, session_id: str, speed: int, force: bool) -> None:
        session_dir = self._session_dir(session_id)
        if session_dir is None:
            return
        manifest = self._manifest(session_dir)
        fingerprint = self._fingerprint(
            str(manifest.get("recording_id") or ""), speed,
        )
        key = (session_id, fingerprint)
        with self._lock:
            self._rendering = key
        try:
            if self._render_impl is not None:
                outcome = self._render_impl(session_id, speed, force)
            else:
                outcome = self._render_with_encoder(session_id, session_dir,
                                                    speed, force)
        finally:
            with self._lock:
                self._pending_render.discard(key)
                self._rendering = None
        self._housekeeping_once()

    # ------------------------------------------------------------------ #
    # rendering implementation                                           #
    # ------------------------------------------------------------------ #
    def _render_with_encoder(self, session_id: str, session_dir: Path,
                             speed: int, force: bool) -> dict[str, Any]:
        from app.search_replay.encoder import (ENCODER_VERSION, VideoEncoder,
                                               fingerprint as make_fingerprint)
        from app.search_replay.recording import load_recording
        from app.search_replay.renderer import (RENDERER_VERSION, RenderError,
                                                ReplayRenderer)

        replay = storage.replay_dir(session_dir)
        artifacts = replay / storage.ARTIFACTS_DIR
        artifacts.mkdir(parents=True, exist_ok=True)
        manifest = self._manifest(session_dir)
        recording_id = str(manifest.get("recording_id") or "")
        fingerprint = make_fingerprint(
            recording_id, speed=speed, config=self.config,
            width=self.config.width, height=self.config.height,
            fps=self.config.fps,
        )
        job = self._existing_job(session_dir, speed)
        output = artifacts / "replay.mp4"

        # 全局只允许一个编码任务。
        global_lock = storage.FileLock(self.root / "encoder.lock")
        if not global_lock.acquire(timeout=300.0):
            return self._fail_job(session_id, session_dir, job, fingerprint,
                                  "另一个编码任务长期占用，稍后重试")
        session_lock = storage.FileLock(replay / storage.LOCK)
        try:
            if not session_lock.acquire(timeout=30.0):
                return self._fail_job(
                    session_id, session_dir, job, fingerprint,
                    "会话记录正在被占用（可能是播放中的下载），稍后重试",
                )
            try:
                os.nice(10)
            except OSError:
                pass
            stamp, wall = _now()
            job.update({
                "status": "RENDERING", "speed": speed,
                "fingerprint": fingerprint, "started_t": stamp,
                "started_wall": wall, "updated_t": stamp,
                "progress": 0.0, "error": "",
            })
            self._write_job(session_dir, job)
            try:
                recording = load_recording(session_dir, config=self.config)
                renderer = ReplayRenderer(
                    recording, self.config, speed=speed,
                    width=self.config.width, height=self.config.height,
                    fps=self.config.fps,
                )
                metadata = renderer.prepare()
            except RenderError as exc:
                return self._fail_job(session_id, session_dir, job,
                                      fingerprint, str(exc), reusable=True)
            except Exception as exc:  # noqa: BLE001
                return self._fail_job(session_id, session_dir, job,
                                      fingerprint,
                                      f"读取录制包失败: {type(exc).__name__}: {exc}")

            history = list(job.get("render_history") or [])
            previous = ((manifest.get("render") or {}).get("fingerprint")
                        or job.get("fingerprint") or "")
            if output.exists() and (force or previous != fingerprint):
                archived = self._archive_previous(
                    output, artifacts, previous, history,
                )
                if archived:
                    history.append(archived)

            encoder = VideoEncoder(self.config,
                                   render_log=replay / storage.RENDER_LOG)
            last_progress = [0.0]

            def progress(fraction: float) -> None:
                stamp_now = time.monotonic()
                if stamp_now - last_progress[0] < 2.0 and fraction < 0.999:
                    return
                last_progress[0] = stamp_now
                current = self._existing_job(session_dir, speed)
                current.update({
                    "status": "RENDERING", "speed": speed,
                    "fingerprint": fingerprint,
                    "progress": round(min(0.99, max(0.0, fraction)), 4),
                    "updated_t": stamp_now,
                })
                try:
                    self._write_job(session_dir, current)
                except OSError:
                    pass
                self._wait_if_paused()

            result = encoder.encode(renderer, output, progress=progress)
            if not result.ok:
                reusable = output.is_file() and self._probe_ok(output)
                return self._fail_job(
                    session_id, session_dir, job, fingerprint,
                    result.error or "编码失败", reusable=reusable,
                )

            reasons: list[str] = []
            recording_block = manifest.get("recording") or {}
            capture_status = str((storage.read_json(
                replay / storage.CAPTURE_STATUS) or {}).get("status") or "")
            # 采集状态是成品完整性的第一依据：中断/失败/从未收尾的素材可以
            # 播，但绝不能报成 READY。
            if capture_status in {"INTERRUPTED", "FAILED", "ABANDONED"}:
                reasons.append(
                    f"采集状态为 {capture_status}，记录在结束前中断"
                )
            elif capture_status != "FINALIZED":
                reasons.append(
                    f"采集状态为 {capture_status or '缺失'}，未正常收尾"
                )
            if not recording_block:
                reasons.append("录制包缺少 recording 段，无法核对采样完整性")
            elif str(recording_block.get("quality") or "FULL") != "FULL":
                reasons.append("录制质量降级：" + "；".join(
                    recording_block.get("notes") or ["采集数据不完整"]
                ))
            bad_lines = {name: count for name, count in recording.bad_lines.items()
                         if count}
            if bad_lines:
                reasons.append("录制文件有损坏行：" + "，".join(
                    f"{name} {count} 行" for name, count in bad_lines.items()
                ))
            if recording.missing_maps:
                reasons.append(
                    f"{recording.missing_maps} 个地图版本文件缺失，"
                    "画面已回退到上一可用版本"
                )
            if recording._map_failures:
                reasons.append(
                    f"{len(recording._map_failures)} 个地图文件无法解码，"
                    "画面已沿用可读版本"
                )
            if not renderer.trajectory.points:
                reasons.append("没有有效定位轨迹，无法核对实际移动路径")
            if metadata.quality != "FULL":
                reasons.extend(metadata.notes)
            gap = float((manifest.get("capture") or {}).get("start_gap_s") or 0.0)
            if gap > self.config.capture_ready_timeout:
                reasons.append(f"采集器就绪晚了 {gap:.1f}s，起始区间未录到")
            dropped = int((manifest.get("events_dropped") or 0))
            if dropped:
                reasons.append(f"事件队列溢出丢弃 {dropped} 条事件")
            reasons = _dedupe(reasons)

            artifact_info: dict[str, Any] = {}
            artifact_info["replay.mp4"] = {
                "size_bytes": result.size_bytes,
                "sha256": storage.sha256_file(output),
                "fingerprint": fingerprint,
            }
            final_map = artifacts / "final_map.png"
            cover = artifacts / "cover.jpg"
            map_ok = self._save_image(
                lambda path: renderer.save_final_map(path), final_map,
            )
            cover_ok = self._save_image(
                lambda path: renderer.save_cover(path), cover,
            )
            if not map_ok:
                reasons.append("最终地图 PNG 生成失败")
            else:
                artifact_info["final_map.png"] = {
                    "size_bytes": final_map.stat().st_size,
                    "sha256": storage.sha256_file(final_map),
                }
            if not cover_ok:
                reasons.append("封面图生成失败")
            else:
                artifact_info["cover.jpg"] = {
                    "size_bytes": cover.stat().st_size,
                    "sha256": storage.sha256_file(cover),
                }
            chapters_path = artifacts / "chapters.json"
            if metadata.chapters:
                storage.atomic_write_json(chapters_path, {
                    "schema_version": SCHEMA_VERSION,
                    "session_id": session_id,
                    "speed": speed,
                    "chapters": metadata.chapters,
                    "timemap": renderer.timemap.to_dict(),
                })
                artifact_info["chapters.json"] = {
                    "size_bytes": chapters_path.stat().st_size,
                }
            elif chapters_path.exists():
                try:
                    chapters_path.unlink()
                except OSError:
                    pass

            status = PARTIAL if reasons else READY
            finish_stamp, finish_wall = _now()
            render_block = {
                "fingerprint": fingerprint,
                "speed": speed,
                "status": status,
                "generated_t": finish_stamp,
                "generated_wall": finish_wall,
                "renderer": RENDERER_VERSION,
                "encoder": ENCODER_VERSION,
                "width": result.width,
                "height": result.height,
                "fps": round(result.fps, 3),
                "duration": round(result.duration, 3),
                "frames": result.frames,
                "size_bytes": result.size_bytes,
                "distance_m": round(metadata.distance_m, 3),
                "distance_source": metadata.distance_source,
                "segments": len(renderer.trajectory.segments),
                "quality": status,
                "reasons": reasons,
                "key_windows": metadata.key_windows,
                "chapters": metadata.chapters,
                "sample_checks": result.sample_checks,
                "probe": result.probe,
                "artifacts": artifact_info,
                "first_t": metadata.first_t,
                "last_t": metadata.last_t,
            }
            job.update({
                "status": status,
                "speed": speed,
                "fingerprint": fingerprint,
                "finished_t": finish_stamp,
                "finished_wall": finish_wall,
                "updated_t": finish_stamp,
                "progress": 1.0,
                "error": "",
                "reasons": reasons,
                "duration": round(result.duration, 3),
                "width": result.width,
                "height": result.height,
                "fps": round(result.fps, 3),
                "frames": result.frames,
                "size_bytes": result.size_bytes,
                "artifacts": artifact_info,
                "render_history": history[-10:],
                "stale_artifact": False,
            })
            self._write_job(session_dir, job)
            self._update_manifest(session_dir, {
                "render": render_block,
                "raw_material": {"pruned": False},
            })
            self._log_line(replay, f"render {status} speed={speed} "
                                   f"duration={result.duration:.1f}s "
                                   f"size={result.size_bytes}B")
            return {"ok": True, "status": status, "reasons": reasons}
        finally:
            session_lock.release()
            global_lock.release()

    def _fail_job(self, session_id: str, session_dir: Path,
                  job: dict[str, Any], fingerprint: str, error: str,
                  *, reusable: bool = False) -> dict[str, Any]:
        stamp, _wall = _now()
        status = FAILED
        reasons: list[str] = [error]
        if reusable:
            status = PARTIAL
            reasons.append("重新生成失败，仍可播放上一次成品")
        job.update({
            "status": status,
            "error": error,
            "reasons": reasons,
            "fingerprint": fingerprint,
            "finished_t": stamp,
            "updated_t": stamp,
            "stale_artifact": reusable,
        })
        try:
            self._write_job(session_dir, job)
        except OSError:
            pass
        self._update_manifest(session_dir, {
            "render": {
                "status": status, "fingerprint": fingerprint,
                "error": error, "reasons": reasons,
                "generated_t": stamp,
            }
        })
        self._log_line(storage.replay_dir(session_dir),
                       f"render {status}: {error}")
        return {"ok": False, "status": status, "error": error}

    def _archive_previous(self, output: Path, artifacts: Path,
                          previous_fingerprint: str,
                          history: list[dict[str, Any]]) -> dict[str, Any] | None:
        versions = artifacts / "versions"
        try:
            versions.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            target = versions / f"replay_{stamp}_{len(history) + 1:02d}.mp4"
            size = output.stat().st_size
            # Keep the published inode available until the new MP4 passes
            # validation. A failed re-encode must not remove a playable file.
            try:
                os.link(output, target)
            except OSError:
                shutil.copy2(output, target)
        except OSError as exc:
            self._logger(f"归档旧成品失败: {exc}")
            return None
        return {
            "path": f"versions/{target.name}",
            "size_bytes": size,
            "replaced_fingerprint": previous_fingerprint,
            "archived_wall": time.time(),
        }

    def _save_image(self, writer: Callable[[Path], bool], target: Path) -> bool:
        # 保留扩展名：OpenCV/PIL 都按后缀选编码器，"x.png.partial" 会失败。
        temporary = target.with_name(
            f"{target.stem}.partial{target.suffix}"
        )
        try:
            if not writer(temporary):
                return False
            if not temporary.is_file() or temporary.stat().st_size <= 0:
                return False
            os.replace(temporary, target)
            return True
        except Exception:  # noqa: BLE001 - cv2.error 不是 OSError
            try:
                temporary.unlink()
            except OSError:
                pass
            return False

    # ------------------------------------------------------------------ #
    # status                                                             #
    # ------------------------------------------------------------------ #
    def status(self, session_id: str) -> dict[str, Any] | None:
        session_dir = self._session_dir(session_id)
        if session_dir is None:
            return None
        replay = storage.replay_dir(session_dir)
        manifest = self._manifest(session_dir)
        if not manifest:
            return {
                "session_id": session_id,
                "status": DISABLED if not self.enabled else DISABLED,
                "available": False,
                "reasons": ["本次会话没有回放记录（回放功能未启用或绑定失败）"],
                "urls": self._urls(session_id),
            }
        status = self._effective_status(session_dir, manifest)
        job = storage.read_json(replay / storage.JOB) or {}
        capture = storage.read_json(replay / storage.CAPTURE_STATUS) or {}
        recording = manifest.get("recording") or {}
        render = manifest.get("render") or {}
        artifacts = replay / storage.ARTIFACTS_DIR
        video = artifacts / "replay.mp4"
        try:
            video_exists = video.stat().st_size > 0
        except OSError:
            video_exists = False
        has_video = video_exists and self._video_ok(session_dir)
        reasons: list[str] = []
        if status == FAILED:
            reason = str(job.get("error") or render.get("error") or "")
            if not reason and str(capture.get("status") or "") == "ABANDONED":
                reason = str(capture.get("error") or "任务被拒绝，未生成回放")
            reasons.append(reason or "回放生成失败")
        reasons.extend(job.get("reasons") or [])
        reasons.extend(render.get("reasons") or [])
        if not has_video and status in PLAYABLE_REPLAY_STATUSES:
            reasons.append("成品文件缺失" if not video_exists
                           else "成品文件无法解码，需要重新生成")
        notes = list(recording.get("notes") or [])
        notes.extend(capture.get("notes") or [])
        notes.extend(capture.get("degraded") or [])
        return {
            "session_id": session_id,
            "available": True,
            "status": status,
            "playable": has_video and status in PLAYABLE_REPLAY_STATUSES,
            "speed": int(job.get("speed") or render.get("speed")
                         or self.config.default_speed),
            "speeds": list(self._speeds()),
            "progress": float(job.get("progress") or 0.0),
            "duration": job.get("duration") or render.get("duration"),
            "width": job.get("width") or render.get("width"),
            "height": job.get("height") or render.get("height"),
            "fps": job.get("fps") or render.get("fps"),
            "size_bytes": (job.get("size_bytes")
                           or render.get("size_bytes")
                           or (video.stat().st_size if has_video else 0)),
            "reasons": _dedupe([str(item) for item in reasons if item]),
            "notes": _dedupe(notes),
            "recording_id": manifest.get("recording_id"),
            "source": manifest.get("source") or self.source,
            "quality": render.get("quality") or recording.get("quality"),
            "can_render": bool(
                self.enabled
                and session_id not in self._collectors
                and session_id not in self._abandoned
                and str(capture.get("status") or "") != "ABANDONED"
                and self._session_terminal(session_id, session_dir)
                and not (manifest.get("raw_material") or {}).get("pruned")
            ),
            "raw_material": manifest.get("raw_material") or {},
            "counts": {
                "poses": (recording.get("counts") or {}).get("poses"),
                "frames": (recording.get("counts") or {}).get("frames"),
                "map_versions": (recording.get("counts") or {}).get(
                    "maps_stored"
                ),
            },
            "capture": {
                "state": capture.get("status") or (manifest.get("capture") or {}).get("state"),
                "ready": capture.get("ready"),
                "degraded": capture.get("degraded") or [],
                "counts": capture.get("counts") or {},
                "camera": capture.get("camera") or {},
            },
            "artifacts": render.get("artifacts") or {},
            "urls": self._urls(session_id),
            "updated_wall": job.get("updated_wall") or render.get("generated_wall"),
        }

    def _effective_status(self, session_dir: Path,
                          manifest: dict[str, Any]) -> str:
        replay = storage.replay_dir(session_dir)
        capture = storage.read_json(replay / storage.CAPTURE_STATUS) or {}
        job = storage.read_json(replay / storage.JOB) or {}
        capture_status = str(capture.get("status") or "")
        job_status = str(job.get("status") or "")
        if capture_status in {STARTING, RECORDING, FINALIZING}:
            updated = capture.get("updated_t")
            stale = (isinstance(updated, (int, float))
                     and time.monotonic() - float(updated) > 15.0
                     and not storage.lock_is_held(replay / storage.LOCK))
            if not stale:
                return capture_status
        if job_status in {QUEUED, "RENDERING", READY, PARTIAL, FAILED}:
            return job_status
        if capture_status == FAILED:
            return FAILED
        if capture_status == "ABANDONED":
            # TASK_REJECTED: recording was stopped before any work started, so
            # there is nothing to render and nothing will ever be queued.
            return FAILED
        if capture_status in {"FINALIZED", "INTERRUPTED"}:
            return QUEUED
        if manifest:
            return QUEUED
        return DISABLED

    def _urls(self, session_id: str) -> dict[str, str]:
        base = f"/api/search/history/{session_id}/replay"
        return {
            "status": base,
            "video": f"{base}/video",
            "download": f"{base}/download",
            "final_map": f"{base}/final-map",
            "cover": f"{base}/cover",
            "render": f"{base}/render",
        }

    def artifact_path(self, session_id: str, kind: str) -> Path | None:
        session_dir = self._session_dir(session_id)
        if session_dir is None:
            return None
        names = {
            "video": ("artifacts", "replay.mp4"),
            "final_map": ("artifacts", "final_map.png"),
            "cover": ("artifacts", "cover.jpg"),
            "chapters": ("artifacts", "chapters.json"),
        }
        parts = names.get(kind)
        if parts is None:
            return None
        path = storage.resolve_artifact(
            storage.replay_dir(session_dir), *parts, must_exist=False,
        )
        if path is None or not path.is_file():
            return None
        return path

    def stream_lock(self, session_id: str) -> storage.FileLock:
        session_dir = self._session_dir(session_id)
        if session_dir is None:
            return storage.FileLock(self.root / "missing.lock")
        return storage.FileLock(storage.replay_dir(session_dir) / storage.LOCK)

    # ------------------------------------------------------------------ #
    # protection / recovery / budgets                                    #
    # ------------------------------------------------------------------ #
    def session_protected(self, session_id: str) -> bool:
        with self._lock:
            if session_id in self._collectors:
                return True
            if self._rendering is not None and self._rendering[0] == session_id:
                return True
        session_dir = self._session_dir(session_id)
        if session_dir is None:
            return False
        replay = storage.replay_dir(session_dir)
        if storage.lock_is_held(replay / storage.LOCK):
            return True
        job = storage.read_json(replay / storage.JOB) or {}
        if str(job.get("status") or "") in ACTIVE_REPLAY_STATUSES:
            return True
        capture = storage.read_json(replay / storage.CAPTURE_STATUS) or {}
        return str(capture.get("status") or "") in ACTIVE_REPLAY_STATUSES

    def _housekeeping_loop(self) -> None:
        while not self._stopping.is_set():
            try:
                self._housekeeping_once()
            except Exception as exc:  # noqa: BLE001
                self._logger(f"回放恢复检查异常: {exc}")
            if self._stopping.wait(20.0):
                break

    def _housekeeping_once(self) -> None:
        if not self.enabled:
            return
        # Recovery enqueues render jobs; make sure the worker exists even when
        # this is called directly (tests, or a future caller before start()).
        self.start()
        for session_dir in self._iter_replay_sessions():
            session_id = session_dir.name
            try:
                self._recover_session(session_id, session_dir)
            except Exception as exc:  # noqa: BLE001
                self._logger(f"恢复会话 {session_id} 失败: {exc}")
        self._enforce_budgets()

    def _recover_session(self, session_id: str, session_dir: Path) -> None:
        # A collector can still be alive after the finish queue filled up.
        # Its presence must not override a terminal result already received.
        self._retry_finalizations()
        replay = storage.replay_dir(session_dir)
        manifest = storage.read_json(replay / storage.MANIFEST) or {}
        if not manifest:
            return
        capture = storage.read_json(replay / storage.CAPTURE_STATUS) or {}
        job = storage.read_json(replay / storage.JOB) or {}
        capture_status = str(capture.get("status") or "")
        job_status = str(job.get("status") or "")
        lock_held = storage.lock_is_held(replay / storage.LOCK)
        terminal = self._session_terminal(session_id, session_dir)

        if capture_status in {STARTING, RECORDING, FINALIZING} and not lock_held:
            updated = capture.get("updated_t")
            # The collector heartbeats ``updated_t``; only a capture that has
            # gone quiet is an orphan.  Interrupting a *fresh* STARTING row
            # would kill a recording that is still launching (bind happens
            # right before the first housekeeping tick).
            stale_after = max(15.0,
                              float(self.config.capture_ready_timeout) * 3 + 5)
            stale = (isinstance(updated, (int, float))
                     and time.monotonic() - float(updated) > stale_after)
            if not stale:
                return
            capture.update({
                "status": "INTERRUPTED",
                "ready": False,
                "error": capture.get("error") or "采集进程已退出，记录中断",
                "updated_t": _now()[0],
            })
            try:
                storage.atomic_write_json(replay / storage.CAPTURE_STATUS,
                                          json_safe(capture))
            except OSError:
                pass
            self._update_manifest(session_dir, {
                "capture": {"state": "INTERRUPTED",
                            "error": "采集进程已退出，记录中断"}
            })
            capture_status = "INTERRUPTED"

        if not terminal:
            return
        if job_status in {QUEUED, "RENDERING"}:
            if lock_held:
                # 编码器就在本机跑着（``_render_with_encoder`` 整段持有会话
                # 锁），或有人正在下载。这里什么都不做：放行会给同一会话
                # 补排一个默认倍速，等 5× 刚成片就被 10× 覆盖掉。锁随进程
                # 消失，崩溃留下的 RENDERING 下次自然落到下面重新排队。
                return
            if job_status == "RENDERING":
                # 上一次 Web 进程在编码中途退出：成品只可能是 partial。
                self._write_job(session_dir, {
                    **job,
                    "status": QUEUED,
                    "error": "",
                    "progress": 0.0,
                    "updated_t": _now()[0],
                })
            with self._lock:
                key = (session_id, str(job.get("fingerprint") or ""))
                already = key in self._pending_render
            if not already:
                self._enqueue_render(session_id, session_dir,
                                     self._coerce_speed(job.get("speed")),
                                     force=False)
            return
        if job_status in {READY, PARTIAL}:
            expected = self._fingerprint(
                str(manifest.get("recording_id") or ""),
                self._coerce_speed(job.get("speed")),
            )
            outdated = (job.get("fingerprint") != expected
                        and not (manifest.get("raw_material") or {}).get("pruned"))
            if not self._video_ok(session_dir) or outdated:
                self._enqueue_render(session_id, session_dir,
                                     self._coerce_speed(job.get("speed")),
                                     force=not outdated)
            return
        if job_status == FAILED:
            return
        if capture_status in {"FINALIZED", "INTERRUPTED", FAILED} or lock_held:
            self._enqueue_render(session_id, session_dir,
                                 self._coerce_speed(None), force=False)

    def _enforce_budgets(self) -> None:
        sessions: list[tuple[float, Path, int]] = []
        total = 0
        for session_dir in self._iter_replay_sessions():
            replay = storage.replay_dir(session_dir)
            size = storage.dir_size_bytes(replay / storage.FRAMES_DIR) \
                + storage.dir_size_bytes(replay / storage.MAPS_DIR)
            total += size
            manifest = self._manifest(session_dir)
            created = manifest.get("created_at_wall") or 0.0
            sessions.append((float(created), session_dir, size))
        if total <= self.config.global_soft_bytes:
            return
        sessions.sort(key=lambda item: item[0])
        for _created, session_dir, size in sessions:
            if total <= self.config.global_soft_bytes:
                break
            session_id = session_dir.name
            if session_id == self._active_session or self.session_protected(session_id):
                continue
            manifest = self._manifest(session_dir)
            if (manifest.get("raw_material") or {}).get("pruned"):
                continue
            removed = 0
            for name in (storage.FRAMES_DIR, storage.MAPS_DIR):
                directory = storage.replay_dir(session_dir) / name
                if directory.is_dir():
                    removed += storage.dir_size_bytes(directory)
                    shutil.rmtree(directory, ignore_errors=True)
                    directory.mkdir(parents=True, exist_ok=True)
            total -= removed
            self._update_manifest(session_dir, {
                "raw_material": {
                    "pruned": True,
                    "pruned_wall": time.time(),
                    "reason": "global_budget",
                    "removed_bytes": removed,
                    "note": "原始地图/相机素材已按预算清理：成品仍可播放，不能重新生成",
                }
            })
            self._log_line(storage.replay_dir(session_dir),
                           f"pruned raw material {removed}B (global budget)")

    # ------------------------------------------------------------------ #
    # event writer                                                       #
    # ------------------------------------------------------------------ #
    def _event_loop(self) -> None:
        while True:
            batch: list[dict[str, Any]] = []
            deadline = time.monotonic() + 0.3
            stop = False
            while len(batch) < 100:
                timeout = max(0.0, deadline - time.monotonic())
                try:
                    item = self._events.get(timeout=timeout)
                except queue.Empty:
                    break
                if item is None:
                    stop = True
                    break
                batch.append(item)
            if batch:
                self._write_events(batch)
            if stop or (self._stopping.is_set() and self._events.empty()):
                break

    def _write_events(self, batch: list[dict[str, Any]]) -> None:
        grouped: dict[str, list[dict[str, Any]]] = {}
        for row in batch:
            grouped.setdefault(str(row.get("session_id") or ""), []).append(row)
        for session_id, rows in grouped.items():
            session_dir = self._session_dir(session_id)
            if session_dir is None:
                continue
            replay = storage.replay_dir(session_dir)
            try:
                storage.append_jsonl(replay / storage.EVENTS, rows)
            except OSError as exc:
                self._logger(f"写回放事件失败 {session_id}: {exc}")

    def _flush_events(self) -> None:
        batch: list[dict[str, Any]] = []
        while True:
            try:
                item = self._events.get_nowait()
            except queue.Empty:
                break
            if item is not None:
                batch.append(item)
        if batch:
            self._write_events(batch)

    # ------------------------------------------------------------------ #
    # helpers                                                            #
    # ------------------------------------------------------------------ #
    def _session_dir(self, session_id: str) -> Path | None:
        return storage.resolve_session_dir(self.sessions_root, session_id)

    def _iter_replay_sessions(self) -> list[Path]:
        result: list[Path] = []
        try:
            entries = list(self.sessions_root.iterdir())
        except OSError:
            return result
        for entry in entries:
            if not entry.is_dir() or not entry.name.startswith("search_"):
                continue
            if not storage.valid_session_id(entry.name):
                continue
            if (entry / storage.REPLAY_DIRNAME / storage.MANIFEST).is_file():
                result.append(entry)
        return result

    def _manifest(self, session_dir: Path) -> dict[str, Any]:
        return storage.read_json(
            storage.replay_dir(session_dir) / storage.MANIFEST
        ) or {}

    def _session_marker(self, session_dir: Path) -> dict[str, Any]:
        return storage.read_json(session_dir / "webui_session.json") or {}

    def _session_terminal(self, session_id: str, session_dir: Path) -> bool:
        if session_id in self._collectors:
            return False
        marker = self._session_marker(session_dir)
        status = str(marker.get("status") or "").upper()
        if status in TERMINAL_SESSION_STATUSES:
            return True
        state = storage.read_json(session_dir / "webui_state.json") or {}
        state_status = str(state.get("status") or "").upper()
        return state_status in TERMINAL_SESSION_STATUSES

    def _existing_job(self, session_dir: Path, speed: int) -> dict[str, Any]:
        return storage.read_json(
            storage.replay_dir(session_dir) / storage.JOB
        ) or {}

    def _write_job(self, session_dir: Path, job: dict[str, Any]) -> None:
        job.setdefault("schema_version", SCHEMA_VERSION)
        job["updated_wall"] = time.time()
        storage.atomic_write_json(
            storage.replay_dir(session_dir) / storage.JOB, json_safe(job),
        )

    def _update_manifest(self, session_dir: Path, patch: dict[str, Any], *,
                         only_if_exists: bool = False) -> None:
        path = storage.replay_dir(session_dir) / storage.MANIFEST
        if only_if_exists and not path.is_file():
            return
        manifest = storage.read_json(path) or {}
        for key, value in patch.items():
            if isinstance(value, dict) and isinstance(manifest.get(key), dict):
                merged = dict(manifest[key])
                merged.update(value)
                manifest[key] = merged
            else:
                manifest[key] = value
        try:
            storage.atomic_write_json(path, json_safe(manifest))
        except OSError as exc:
            self._logger(f"写 manifest 失败 {session_dir}: {exc}")

    def _video_ok(self, session_dir: Path) -> bool:
        """Published artifacts must decode, not merely exist.

        Recovery and ``status()`` both use this: a truncated ``replay.mp4``
        keeps reporting ``READY`` for as long as the check is only
        ``size > 0``.  Results are cached by (path, size, mtime) because a
        full decode is too expensive for the 2 s UI poll.
        """
        video = storage.replay_dir(session_dir) / storage.ARTIFACTS_DIR / "replay.mp4"
        try:
            stat = video.stat()
        except OSError:
            return False
        if stat.st_size <= 0:
            return False
        key = (str(video), stat.st_size, stat.st_mtime_ns)
        with self._lock:
            cached = self._probe_cache.get(key)
        if cached is not None:
            return cached
        job = self._existing_job(session_dir, 0)
        render = self._manifest(session_dir).get("render") or {}
        ok, _reason, _probe = verify_video(
            video,
            width=job.get("width") or render.get("width"),
            height=job.get("height") or render.get("height"),
            fps=job.get("fps") or render.get("fps"),
            duration=job.get("duration") or render.get("duration"),
        )
        with self._lock:
            if len(self._probe_cache) > 64:
                self._probe_cache.clear()
            self._probe_cache[key] = ok
        return ok

    def _probe_ok(self, path: Path) -> bool:
        from app.search_replay.encoder import probe_video

        return bool(probe_video(path).get("ok"))

    def _speeds(self) -> tuple[int, ...]:
        speeds = tuple(int(item) for item in self.config.speeds if int(item) >= 1)
        return speeds or DEFAULT_SPEEDS

    def _coerce_speed(self, speed: Any) -> int:
        try:
            value = int(speed)
        except (TypeError, ValueError):
            value = self.config.default_speed
        return value if value in self._speeds() else self.config.default_speed

    def _fingerprint(self, recording_id: str, speed: int) -> str:
        from app.search_replay.encoder import fingerprint

        return fingerprint(
            recording_id, speed=speed, config=self.config,
            width=self.config.width, height=self.config.height,
            fps=self.config.fps,
        )

    def _wait_if_paused(self) -> None:
        deadline = time.monotonic() + 60.0
        while (self._pause_render.is_set() and not self._stopping.is_set()
               and time.monotonic() < deadline):
            time.sleep(0.2)

    def _record_job_failure(self, item: tuple[Any, ...], exc: Exception) -> None:
        try:
            session_id = str(item[1])
        except (IndexError, TypeError):
            return
        session_dir = self._session_dir(session_id)
        if session_dir is None:
            return
        job = self._existing_job(session_dir, self._coerce_speed(None))
        self._fail_job(session_id, session_dir, job,
                       str(job.get("fingerprint") or ""),
                       f"回放任务异常: {type(exc).__name__}: {exc}")

    def _log_line(self, replay: Path, message: str) -> None:
        try:
            replay.mkdir(parents=True, exist_ok=True)
            with (replay / storage.RENDER_LOG).open("a", encoding="utf-8") as handle:
                handle.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {message}\n")
        except OSError:
            pass

    # ------------------------------------------------------------------ #
    # test/debug surface                                                 #
    # ------------------------------------------------------------------ #
    def wait_idle(self, timeout: float = 30.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                busy = (bool(self._pending_render) or self._rendering is not None
                        or self._working or self._finalizing
                        or bool(self._finishing))
            if (not busy and self._jobs.empty()
                    and self._finalize.empty()):
                return True
            time.sleep(0.1)
        return False


def _monotonic_for(event: Any) -> tuple[float, float]:
    """Map an event's wall timestamp onto host monotonic time.

    Events are enqueued immediately after they were created, so the offset is
    milliseconds.  A timestamp outside a sane window falls back to "now" and is
    still recorded with its wall value.
    """
    stamp = time.monotonic()
    wall = time.time()
    timestamp = getattr(event, "timestamp", None)
    if isinstance(timestamp, (int, float)) and math.isfinite(float(timestamp)):
        delta = wall - float(timestamp)
        if -5.0 <= delta <= 30.0:
            return stamp - max(0.0, delta), float(timestamp)
        return stamp, float(timestamp)
    return stamp, wall


def _dedupe(values: list[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []
    for value in values:
        text = " ".join(str(value).split())
        if not text or text in seen:
            continue
        seen.add(text)
        result.append(text)
    return result
