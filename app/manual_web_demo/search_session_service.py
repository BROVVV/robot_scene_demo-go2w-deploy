"""SearchSessionService: session lifecycle owner for autonomous semantic
search (plan book §37-§41, §56, §80).

One service instance per web process.  It owns at most one active search
session, connects the SearchExecutor (worker subprocess or in-process mock)
to the SearchEventBus / SearchStateStore / ExplorerSearchAdapter, and exposes
the REST-friendly operations used by ``search_routes``.

Control ownership is enforced through the shared ``ControlOwner`` so manual
and autonomous motion never conflict.
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Any, Callable

from app.live_robot.explorer_search_adapter import ExplorerSearchAdapter
from app.live_robot.search_event import (
    ERROR,
    OPERATOR_STOP,
    SEARCH_FINISHED,
    SEARCH_STATE_CHANGED,
    SESSION_CREATED,
    TASK_REJECTED,
    TASK_UNDERSTANDING,
    SearchEvent,
    make_event,
)
from app.live_robot.search_event_bus import SearchEventBus
from app.live_robot.search_state_store import (
    STATUS_FAILED,
    STATUS_FINISHED,
    STATUS_IDLE,
    STATUS_OPERATOR_STOP,
    STATUS_SEARCH_EXHAUSTED,
    STATUS_RUNNING,
    STATUS_TARGET_FOUND,
    SearchStateStore,
    normalize_search_snapshot,
)
from app.manual_web_demo.control_ownership import ControlOwner, OwnerState
from app.manual_web_demo.search_executor import SearchExecutor
from app.manual_web_demo.search_errors import search_error
from app.manual_web_demo.search_models import (
    SearchSessionInfo,
    SearchStartRequest,
    new_session_id,
)
from app.manual_web_demo.search_session_archive import (
    TERMINAL_STATUSES,
    SearchSessionArchive,
)
from app.task_understanding.search_task_context import SearchTaskContext

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_DEFAULT_SESSION_DIR = "outputs/live_runs"

# Lifecycle constants.  A STARTING transition is synchronous from the web API
# but the worker (ROS subprocess / LLM task understanding / RGB-D preflight)
# can take a while, fail silently, or get killed.  Without a recovery path the
# session permanently appears "already active in state STARTING" and the WebUI
# cannot start a new search until the web process restarts.
STATUS_STARTING = "STARTING"
STATUS_STOPPING = "STOPPING"
# 超过该时限仍未离开 STARTING/STOPPING 且 worker 不再存活 -> 视为僵尸会话回收
STARTING_TIMEOUT_SEC = 120.0


class SearchSessionService:
    """Lifecycle + state hub for one web process."""

    def __init__(
        self,
        *,
        owner: ControlOwner,
        executor_factory: Callable[[], SearchExecutor] | None = None,
        session_dir: str = _DEFAULT_SESSION_DIR,
        event_buffer: int = 500,
        task_understanding_runner: Callable[[str], Any] | None = None,
        allow_mock_task_fallback: bool = True,
        history_limit: int = 10,
    ) -> None:
        self.owner = owner
        self._executor_factory = executor_factory or _default_executor_factory
        self._executor_factory_is_mock = bool(
            getattr(self._executor_factory, "is_mock_factory", False)
        )
        self._session_dir = session_dir
        self._bus = SearchEventBus(max_recent=int(event_buffer))
        self._archive = SearchSessionArchive(session_dir, max_sessions=history_limit)
        self._store = SearchStateStore(on_change=self._on_store_change)
        self._adapter: ExplorerSearchAdapter | None = None
        self._executor: SearchExecutor | None = None
        self._lock = threading.Lock()
        self._session_id: str | None = None
        self._info: SearchSessionInfo | None = None
        self._started_at: float | None = None
        self._status = STATUS_IDLE
        self._task_context: SearchTaskContext | None = None
        self._executor_id: str | None = None
        self._worker_generation = 0
        self._task_understanding_runner = task_understanding_runner
        self._allow_mock_task_fallback = bool(allow_mock_task_fallback)
        self._restore_latest_session()

    # ------------------------------------------------------------------ #
    # queries                                                            #
    # ------------------------------------------------------------------ #
    def current_session(self) -> SearchSessionInfo | None:
        with self._lock:
            if self._info is None:
                return None
            # The store is the authoritative status source once a session is
            # active (PAUSED / RESUMED / TARGET_FOUND / ... all flow through
            # SearchEvents); the service-level status only tracks IDLE vs
            # active for the lifecycle gate.
            store_status = self._store.snapshot().get("status") or self._status
            store_result = self._store.snapshot().get("result") or ""
            info = SearchSessionInfo(
                session_id=self._info.session_id,
                target=self._info.target,
                status=store_status,
                task_text=self._info.task_text,
                task_context=dict(self._info.task_context),
                result=store_result or self._info.result,
                started_at=self._info.started_at,
                finished_at=self._info.finished_at,
                backend=self._info.backend,
                reasoner=self._info.reasoner,
            )
            return info

    def state_snapshot(self) -> dict[str, Any]:
        self.reap_stale_session()
        snapshot = self._store.snapshot()
        session = self.current_session()
        if session is not None:
            snapshot["session_id"] = session.session_id
            snapshot["status"] = session.status
            snapshot["target"] = session.target
            snapshot["task"] = dict(session.task_context)
            snapshot["backend"] = session.backend
            snapshot["reasoner"] = session.reasoner
            snapshot["result"] = session.result
            snapshot["elapsed_seconds"] = self._elapsed(session)
            try:
                from app.config import get_settings

                settings = get_settings()
                snapshot["vision_model"] = settings.vision_model
                snapshot["reasoning_model"] = settings.reasoning_model
            except Exception:
                pass
        return snapshot

    def map_snapshot(self) -> dict[str, Any]:
        return self._store.map_snapshot()

    def spatial_snapshot(self) -> dict[str, Any]:
        return self._store.spatial_snapshot()

    def objects_snapshot(self) -> dict[str, Any]:
        return self._store.objects_snapshot()

    def decisions_snapshot(self) -> list[dict[str, Any]]:
        return self._store.decisions_snapshot()

    def recent_events(self, limit: int | None = None) -> list[dict[str, Any]]:
        # The process-wide bus may have restored the previous session before a
        # new service/test session id is installed.  Never mix two sessions in
        # one WebSocket/event response; historical sessions have their own API.
        events = self._bus.recent_events(None)
        if self._session_id:
            events = [item for item in events if item.get("session_id") == self._session_id]
        if limit is not None:
            events = events[-max(0, int(limit)):]
        return events

    def subscribe_events(
        self, callback: Callable[[SearchEvent], None]
    ) -> Callable[[], None]:
        self._bus.subscribe(callback)
        return lambda: self._bus.unsubscribe(callback)

    def executor_state(self) -> dict[str, Any]:
        self.reap_stale_session()
        if self._executor is None:
            return {"state": "stopped", "session_id": None}
        status = dict(self._executor.status() or {})
        status["alive"] = bool(self._executor.alive())
        return status

    # ------------------------------------------------------------------ #
    # session commands                                                    #
    # ------------------------------------------------------------------ #
    # ------------------------------------------------------------------ #
    # stale-session recovery                                           #
    # ------------------------------------------------------------------ #
    def _reset_session_locked(self) -> None:
        """Clear only the active slot when there is genuinely no session."""
        self._status = STATUS_IDLE
        self._info = None
        self._session_id = None
        self._task_context = None
        self._started_at = None
        self._executor = None
        self._adapter = None
        self._store = SearchStateStore(on_change=self._on_store_change)
        try:
            self.owner.release(OwnerState.AUTONOMOUS)
        except Exception:  # noqa: BLE001 - release is best effort/idempotent
            pass

    def _fail_interrupted_session_locked(self, message: str) -> None:
        """Finalize a dead worker without erasing its last useful snapshot."""
        if self._session_id is None:
            self._reset_session_locked()
            return
        detail = search_error(
            message,
            code="WORKER_INTERRUPTED",
            source="search_session_service",
            stage=str(self._store.snapshot().get("phase") or "RUNNING"),
        )
        error_event = make_event(
            allocator=self._bus.allocator,
            session_id=self._session_id,
            event_type=ERROR,
            payload={"error_type": detail["code"], **detail},
        )
        self._bus.publish(error_event)
        self._store.apply(error_event)
        finish = make_event(
            allocator=self._bus.allocator,
            session_id=self._session_id,
            event_type=SEARCH_FINISHED,
            payload={
                "result": "FAILED",
                "finish_reason": "WORKER_INTERRUPTED",
                "error": detail["message"],
                "error_detail": detail,
            },
        )
        self._bus.publish(finish)
        self._store.apply(finish)
        self._status = STATUS_FAILED
        if self._info is not None:
            self._info.status = STATUS_FAILED
            self._info.result = "FAILED"
            self._info.finished_at = time.time()
        self._executor = None
        self._adapter = None
        self.owner.release(OwnerState.AUTONOMOUS)

    def _force_stop_locked(self) -> None:
        """Forcefully terminate a lingering STOPPING session.

        A STOPPING state means the old worker is still shutting down.  A new
        start request should not wait for it forever; terminate the executor
        (best-effort) and drop the session back to IDLE so the user can start
        immediately.
        Caller must hold ``self._lock``.
        """
        executor = self._executor
        if executor is not None:
            try:
                executor.stop()
            except Exception:  # noqa: BLE001
                pass
            try:
                executor.shutdown()
            except Exception:  # noqa: BLE001 - cleanup is best effort
                pass
        self._fail_interrupted_session_locked(
            "旧任务在 STOPPING 阶段未能正常结束，已强制终止 worker 并保留其完整记录。"
        )

    def _reap_stale_locked(self) -> bool:
        """Return True and reset when a RUNNING/STARTING/STOPPING session's
        worker is gone.

        The WebUI survives page refreshes, but the in-memory search session is
        only cleaned up by the worker's terminal event.  If the worker has
        exited (crash / kill / page-reload orphan) without sending that event,
        the backend would keep showing RUNNING/STARTING and the frontend would
        keep the Start button disabled until the user manually hits Stop.  We
        therefore auto-reap:

          * STARTING/STOPPING: worker dead OR stuck for STARTING_TIMEOUT_SEC
          * RUNNING: worker dead only (a long-running search must not be
            killed just because it exceeded a timeout).

        Caller must hold ``self._lock``.
        """
        if self._status not in (STATUS_STARTING, STATUS_STOPPING, STATUS_RUNNING):
            return False
        executor_alive = bool(self._executor is not None and self._executor.alive())
        if self._status == STATUS_RUNNING:
            if not executor_alive:
                self._fail_interrupted_session_locked(
                    "搜索 worker 已退出，未收到正常结束事件；已保留退出前的全部搜索状态。"
                )
                return True
            # 正常搜索在跑：不因超时回收（搜索有自己的 max_seconds 预算）
            return False
        timed_out = False
        if self._started_at is not None:
            timed_out = (time.time() - self._started_at) > STARTING_TIMEOUT_SEC
        if timed_out or not executor_alive:
            if timed_out and executor_alive and self._executor is not None:
                try:
                    self._executor.stop()
                    self._executor.shutdown()
                except Exception:  # noqa: BLE001 - terminal cleanup is best effort
                    pass
            reason = (
                "搜索启动超过等待时限，worker 未进入运行状态；已保留启动诊断。"
                if timed_out else
                "搜索 worker 在启动或停止阶段退出；已保留退出前的全部搜索状态。"
            )
            self._fail_interrupted_session_locked(reason)
            return True
        return False

    def reap_stale_session(self) -> bool:
        """Public recovery: reset a zombie STARTING/STOPPING session."""
        with self._lock:
            return self._reap_stale_locked()

    def _active_status(self) -> str:
        """Status gate source: the store once a session exists (it tracks
        PAUSED / RUNNING / terminal states from SearchEvents), otherwise the
        service-level IDLE."""
        if self._info is not None:
            return self._store.snapshot().get("status") or self._status
        return self._status

    def start_search(self, request: SearchStartRequest) -> dict[str, Any]:
        error = request.validate()
        if error:
            detail = search_error(error, code="TASK_INVALID", source="search_api", stage="VALIDATE")
            return {"ok": False, "error": error, "error_detail": detail}
        task_text = request.task_text or request.target
        task_context = self._understand_task(task_text, backend=request.backend)
        with self._lock:
            # A session stuck in STARTING (worker died, timed out) must not
            # block a fresh search; recover it automatically first.
            # A STOPPING session is "the old one is shutting down" - terminate
            # it now so the new search can start immediately.
            self._reap_stale_locked()
            if self._status == STATUS_STOPPING:
                self._force_stop_locked()
            if self._status != STATUS_IDLE and self._status not in TERMINAL_STATUSES:
                detail = search_error(
                    f"search already active in state {self._status}",
                    code="SEARCH_ALREADY_ACTIVE", source="search_service", stage="START",
                )
                return {
                    "ok": False,
                    "error": detail["message"],
                    "error_detail": detail,
                    "conflict": True,
                }
            ok, reason = self.owner.try_autonomous(detail="autonomous_search")
            if not ok:
                detail = search_error(reason, source="control_owner", stage="START")
                return {"ok": False, "error": reason, "error_detail": detail, "conflict": True}
            session_id = new_session_id()
            self._worker_generation += 1
            generation = self._worker_generation
            self._executor_id = f"executor_{session_id}"
            self._session_id = session_id
            self._task_context = task_context
            self._status = "STARTING"
            self._started_at = time.time()
            self._info = SearchSessionInfo(
                session_id=session_id,
                target=task_context.canonical_target,
                status="STARTING",
                task_text=task_context.raw_text,
                task_context=task_context.to_dict(),
                backend=request.backend,
                reasoner=request.reasoner,
                started_at=self._started_at,
            )

            self._archive.begin(session_id, {
                "target": task_context.canonical_target,
                "task_text": task_context.raw_text,
                "task": task_context.to_dict(),
                "backend": request.backend,
                "reasoner": request.reasoner,
                "motion_enabled": request.enable_autonomous_motion,
            })

            # One bus lives for the whole web process (the /ws/search hub
            # subscribes once); a new session only clears its event history.
            self._bus.clear()
            self._store = SearchStateStore(on_change=self._on_store_change)
            self._store.reset(
                session_id=session_id,
                target=task_context.canonical_target,
                reasoner=request.reasoner,
                backend=request.backend,
                task_context=task_context.to_dict(),
            )
            self._adapter = ExplorerSearchAdapter(
                self._bus, self._store, session_id=session_id,
            )
            # A direct ``backend=mock`` request is an offline contract.  The
            # production default factory is a ROS subprocess, which is both
            # unnecessary for mock runs and may use a different Python
            # environment.  Keep explicit test factories authoritative, but
            # make the WebUI mock path deterministic and dependency-free.
            if request.backend in {"mock", "mock_metric"} and not self._executor_factory_is_mock:
                from app.manual_web_demo.search_executor import InProcessMockExecutor

                executor = InProcessMockExecutor()
            else:
                executor = self._executor_factory()
            executor.set_on_message(self._on_executor_message)
            self._executor = executor
            session_dir_path = Path(self._session_dir)
            session_dir_path.mkdir(parents=True, exist_ok=True)
            run_dir = session_dir_path / session_id
            run_dir.mkdir(parents=True, exist_ok=True)
            params = {
                "target": task_context.canonical_target,
                "task_text": task_context.raw_text,
                "task_context": task_context.to_dict(),
                "task_id": task_context.task_id,
                "session_id": session_id,
                "executor_id": self._executor_id,
                "worker_generation": generation,
                "reasoner": request.reasoner,
                "backend": request.backend,
                "finish_on_visual_confirmation": request.finish_on_visual_confirmation,
                "turn_only": request.turn_only,
                "enable_autonomous_motion": request.enable_autonomous_motion,
                "operator_supervised_experiment": request.operator_supervised_experiment,
                "dry_run_motion": request.dry_run_motion,
                "allow_degraded": request.allow_degraded,
                "rgbd_source": request.rgbd_source,
                "rgbd_base_url": request.rgbd_base_url,
                "spatial_v2": request.spatial_v2,
                "spatial_provider": request.spatial_provider,
                "rtabmap": request.rtabmap,
                "session_dir": str(session_dir_path),
                "output": str(run_dir / "events.jsonl"),
            }
            for key in (
                "max_seconds", "max_planning_cycles", "max_motion_steps",
                "llm_model", "verify_min_confidence",
            ):
                value = getattr(request, key)
                if value is not None:
                    params[key] = value
            # Emit SESSION_CREATED from the web side so a page that connects
            # before the worker reports anything still sees a session.
            created = make_event(
                allocator=self._bus.allocator,
                session_id=session_id,
                event_type=SESSION_CREATED,
                payload={
                    "target": task_context.canonical_target,
                    "task_text": task_context.raw_text,
                    "task": task_context.to_dict(),
                    "reasoner": request.reasoner,
                    "backend": request.backend,
                    "phase": "STARTING",
                },
            )
            self._bus.publish(created)
            self._store.apply(created)
            understood = make_event(
                allocator=self._bus.allocator,
                session_id=session_id,
                event_type=TASK_UNDERSTANDING,
                payload={"task": task_context.to_dict()},
            )
            self._bus.publish(understood)
            self._store.apply(understood)
            if not task_context.executable:
                rejected = make_event(
                    allocator=self._bus.allocator,
                    session_id=session_id,
                    event_type=TASK_REJECTED,
                    payload={
                        "task": task_context.to_dict(),
                        "reason": task_context.rejection_reason or "任务不可执行",
                        "error_detail": search_error(
                            task_context.rejection_reason or "任务不可执行",
                            code="TASK_REJECTED", source="task_understanding",
                            stage="TASK_UNDERSTANDING",
                        ),
                    },
                )
                self._bus.publish(rejected)
                self._store.apply(rejected)
                self._status = "TASK_REJECTED"
                if self._info is not None:
                    self._info.status = "TASK_REJECTED"
                self.owner.release(OwnerState.AUTONOMOUS)
                return {
                    "ok": False,
                    "task_rejected": True,
                    "session_id": session_id,
                    "task": task_context.to_dict(),
                    "error": task_context.rejection_reason or "任务不可执行",
                    "error_detail": self._store.snapshot().get("error") or
                                    rejected.payload.get("error_detail"),
                }
            try:
                executor.start(params)
            except Exception as exc:  # noqa: BLE001
                detail = search_error(
                    f"executor start failed: {type(exc).__name__}: {exc}",
                    source="search_executor", stage="SPAWN_WORKER",
                )
                error_event = make_event(
                    allocator=self._bus.allocator, session_id=session_id,
                    event_type=ERROR, payload={"error_type": detail["code"], **detail},
                )
                self._bus.publish(error_event)
                self._store.apply(error_event)
                finish = make_event(
                    allocator=self._bus.allocator, session_id=session_id,
                    event_type=SEARCH_FINISHED,
                    payload={"result": "FAILED", "finish_reason": detail["code"],
                             "error": detail["message"], "error_detail": detail},
                )
                self._bus.publish(finish)
                self._store.apply(finish)
                self._status = STATUS_FAILED
                self._info.status = STATUS_FAILED
                self._info.result = "FAILED"
                self._info.finished_at = time.time()
                self.owner.release(OwnerState.AUTONOMOUS)
                return {"ok": False, "error": detail["message"], "error_detail": detail}
            return {
                "ok": True,
                "session_id": session_id,
                "status": "STARTING",
                "task": task_context.to_dict(),
            }

    def pause_search(self) -> dict[str, Any]:
        with self._lock:
            if self._active_status() != STATUS_RUNNING:
                message = f"not running (state={self._active_status()})"
                return {"ok": False, "error": message,
                        "error_detail": search_error(
                            message, code="SEARCH_NOT_RUNNING",
                            source="search_service", stage="PAUSE")}
        if self._executor is not None:
            self._executor.pause()
        return {"ok": True, "status": "PAUSED"}

    def resume_search(self) -> dict[str, Any]:
        with self._lock:
            if self._active_status() != "PAUSED":
                message = f"not paused (state={self._active_status()})"
                return {"ok": False, "error": message,
                        "error_detail": search_error(
                            message, code="SEARCH_NOT_PAUSED",
                            source="search_service", stage="RESUME")}
        if self._executor is not None:
            self._executor.resume()
        return {"ok": True, "status": "RUNNING"}

    def stop_search(self, *, reason: str = "operator_stop") -> dict[str, Any]:
        with self._lock:
            self._reap_stale_locked()
            if self._active_status() == STATUS_IDLE:
                return {"ok": True, "status": "IDLE", "note": "no active session"}
            alive = bool(self._executor is not None and self._executor.alive())
            if not alive:
                # Worker is gone / never came up (e.g. stuck STARTING after a
                # crash): clean up immediately so the operator can start again.
                if self._active_status() not in TERMINAL_STATUSES:
                    self._fail_interrupted_session_locked(
                        "停止任务时发现搜索 worker 已退出；已保留停止前的完整记录。"
                    )
                return {"ok": True, "status": self._active_status(),
                        "note": "worker already stopped; state preserved"}
            self._status = STATUS_STOPPING
            if self._info is not None:
                self._info.status = STATUS_STOPPING
        if self._executor is not None:
            self._executor.stop()
        return {"ok": True, "status": "STOPPING"}

    def estop_search(self) -> dict[str, Any]:
        """Estop overrides ownership and halts the search (plan book §42)."""
        self.owner.estop()
        with self._lock:
            active = self._status not in (STATUS_IDLE,)
        if self._executor is not None and active:
            self._executor.estop()
        return {"ok": True, "status": "ESTOP"}

    def shutdown(self) -> None:
        """Stop the owned search + worker and release ownership."""
        with self._lock:
            active = self._status not in (STATUS_IDLE, "FINISHED")
        if self._executor is not None and active:
            try:
                self._executor.stop()
            except Exception:  # noqa: BLE001
                pass
        if self._executor is not None:
            try:
                self._executor.shutdown()
            except Exception:  # noqa: BLE001
                pass
        self.owner.release(OwnerState.AUTONOMOUS)

    # ------------------------------------------------------------------ #
    # history                                                            #
    # ------------------------------------------------------------------ #
    def history(self, limit: int = 10) -> list[dict[str, Any]]:
        return self._archive.list(min(10, max(1, int(limit))))

    def history_session(self, session_id: str) -> dict[str, Any] | None:
        return self._archive.load(session_id)

    # ------------------------------------------------------------------ #
    # executor message routing                                            #
    # ------------------------------------------------------------------ #
    def _on_executor_message(self, message: dict[str, Any]) -> None:
        if not self._message_matches_current_worker(message):
            return
        msg_type = str(message.get("type") or "")
        if msg_type == "event":
            raw = message.get("event")
            if isinstance(raw, dict) and self._adapter is not None:
                # Release the motion lease at the raw terminal boundary before
                # durable snapshot I/O.  This prevents persistence latency
                # from extending autonomous control ownership after STOP or a
                # confirmed target.
                raw_name = str(raw.get("event") or "")
                if raw_name in {"session_finish", "target_found", "search_exhausted"}:
                    self.owner.release(OwnerState.AUTONOMOUS)
                self._adapter.on_explorer_event(raw)
                # The adapter can expose a terminal explorer event before the
                # worker emits its final session_result.  Release ownership at
                # the observable terminal boundary so a stop is not held by a
                # tiny IPC cleanup race.
                store_status = self._store.snapshot().get("status")
                if store_status in {
                    STATUS_TARGET_FOUND,
                    STATUS_SEARCH_EXHAUSTED,
                    STATUS_OPERATOR_STOP,
                    STATUS_FINISHED,
                    STATUS_FAILED,
                }:
                    self.owner.release(OwnerState.AUTONOMOUS)
        elif msg_type == "session_result":
            result = message.get("result") or {}
            if isinstance(result, dict) and result.get("session_id") not in {None, self._session_id}:
                return
            self._apply_session_result(result)
        elif msg_type == "worker_status":
            status = message.get("status") or {}
            if str(status.get("state")) == "running":
                self._mark_status(STATUS_RUNNING)
            # Surface concrete startup stage progression to the store so the
            # WebUI can render "正在等待 RGB-D" instead of an undifferentiated
            # STARTING phase, and can detect a stalled startup.
            startup_payload = dict(status or {})
            startup_payload["worker_state"] = startup_payload.get("state")
            startup_payload["stage_started_at"] = startup_payload.get("stage_started_at")
            startup_payload["last_progress_at"] = startup_payload.get("last_progress_at")
            startup_payload["worker_alive"] = True
            startup_payload["last_worker_message_at"] = time.time()
            startup_payload["last_error"] = startup_payload.get("last_error")
            current_snapshot = self._store.snapshot()
            worker_phase = status.get("phase") or current_snapshot.get("phase")
            if worker_phase in {None, "", "IDLE"}:
                worker_phase = (
                    "STARTING"
                    if current_snapshot.get("status") == STATUS_STARTING
                    else "RUNNING"
                )
            state_payload = {
                "startup": startup_payload,
                "worker": startup_payload,
                "phase": worker_phase,
            }
            if "phase_detail" in status:
                state_payload["phase_detail"] = status.get("phase_detail") or ""
            if status.get("phase_started_at") is not None:
                state_payload["phase_started_at"] = status.get("phase_started_at")
            self._store.apply(
                make_event(
                    allocator=self._bus.allocator,
                    session_id=self._session_id or "",
                    event_type=SEARCH_STATE_CHANGED,
                    payload=state_payload,
                )
            )
            if self._adapter is not None:
                self._adapter.on_explorer_event(
                    {"event": "search_state_changed", **state_payload}
                )
        elif msg_type == "error":
            self._publish_error(message.get("message") or "search worker error")

    def _message_matches_current_worker(self, message: dict[str, Any]) -> bool:
        """Reject delayed messages from a retired session before state access."""
        expected_session = self._session_id
        expected_task = self._task_context.task_id if self._task_context else None
        expected_executor = self._executor_id
        expected_generation = self._worker_generation
        if not expected_session:
            return False
        event = message.get("event") if isinstance(message.get("event"), dict) else {}
        payload = event.get("payload") if isinstance(event, dict) and isinstance(event.get("payload"), dict) else {}
        values = {
            "session_id": message.get("session_id") or event.get("session_id"),
            "task_id": message.get("task_id") or payload.get("task_id"),
            "executor_id": message.get("executor_id") or payload.get("executor_id"),
            "worker_generation": message.get("worker_generation") or payload.get("worker_generation"),
        }
        if values["session_id"] and values["session_id"] != expected_session:
            return False
        if values["task_id"] and values["task_id"] != expected_task:
            return False
        if values["executor_id"] and values["executor_id"] != expected_executor:
            return False
        if values["worker_generation"] is not None:
            try:
                if int(values["worker_generation"]) != expected_generation:
                    return False
            except (TypeError, ValueError):
                return False
        return True

    def _understand_task(self, task_text: str, *, backend: str) -> SearchTaskContext:
        runner = self._task_understanding_runner
        # Mock sessions are deterministic/offline by contract.  They must
        # not unexpectedly spend a network timeout on the production parser.
        if runner is None and (
            backend in {"mock", "mock_metric"} or self._executor_factory_is_mock
        ):
            return SearchTaskContext.mock_fallback(task_text)
        if runner is None:
            from app.task_understanding.task_pipeline import run_task_understanding_pipeline

            runner = lambda text: run_task_understanding_pipeline(
                text, enable_verifier=False
            )
        try:
            context = runner(task_text)
            if isinstance(context, SearchTaskContext):
                return context
            structured = SearchTaskContext.from_pipeline_result(
                context, raw_text=task_text
            )
            parser_source = str(
                getattr(getattr(context, "parsed_task", None), "parser_source", "")
            )
            if (
                not structured.executable
                and backend in {"mock", "mock_metric"}
                and self._allow_mock_task_fallback
                and parser_source in {"llm_unavailable", "llm_verification_failed"}
            ):
                return SearchTaskContext.mock_fallback(task_text)
            return structured
        except Exception as exc:
            if backend in {"mock", "mock_metric"} and self._allow_mock_task_fallback:
                return SearchTaskContext.mock_fallback(task_text)
            return SearchTaskContext(
                task_id=f"task_error_{int(time.time() * 1000)}",
                raw_text=task_text,
                intent="unknown",
                canonical_target=task_text,
                executable=False,
                rejection_reason=f"任务理解失败：{type(exc).__name__}: {exc}",
            )

    def _apply_session_result(self, result: dict[str, Any]) -> None:
        if not isinstance(result, dict):
            return
        session_id = self._session_id or str(result.get("session_id") or "")
        finish_reason = str(result.get("finish_reason") or result.get("result") or "")
        finish = make_event(
            allocator=self._bus.allocator,
            session_id=session_id,
            event_type=SEARCH_FINISHED,
            payload={
                "result": finish_reason,
                "finish_reason": finish_reason,
                **result,
            },
        )
        self._bus.publish(finish)
        self._store.apply(finish)
        with self._lock:
            if self._info is not None:
                self._info.result = finish_reason
                self._info.finished_at = time.time()
            if finish_reason == "TARGET_FOUND":
                self._status = "TARGET_FOUND"
            elif finish_reason == "OPERATOR_STOP":
                self._status = "OPERATOR_STOP"
            elif finish_reason == "SEARCH_EXHAUSTED":
                self._status = "SEARCH_EXHAUSTED"
            else:
                try:
                    failed_exit = int(result.get("exit_code") or 0) != 0
                except (TypeError, ValueError):
                    failed_exit = False
                if finish_reason == "FAILED" or failed_exit:
                    self._status = "FAILED"
                else:
                    self._status = "FINISHED"
            self.owner.release(OwnerState.AUTONOMOUS)
        # A subprocess worker is intentionally long-lived while a session is
        # active, but it must not survive a terminal session.  Retire it off
        # the executor callback thread: SubprocessSearchExecutor receives the
        # result from its stdout reader, while the in-process test executor
        # reports the result from its own worker thread.
        executor = self._executor
        if executor is not None and executor.alive():
            threading.Thread(
                target=self._retire_executor,
                args=(executor,),
                daemon=True,
                name="search-executor-retire",
            ).start()

    @staticmethod
    def _retire_executor(executor: SearchExecutor) -> None:
        try:
            executor.shutdown()
        except Exception:  # noqa: BLE001 - terminal cleanup is best effort
            pass

    def _publish_error(self, message: str) -> None:
        session_id = self._session_id or ""
        snapshot = self._store.snapshot()
        detail = search_error(
            message,
            source="autonomous_search_worker",
            stage=str(snapshot.get("phase") or
                      (snapshot.get("startup") or {}).get("stage") or "RUNNING"),
        )
        error_event = make_event(
            allocator=self._bus.allocator,
            session_id=session_id,
            event_type=ERROR,
            payload={"error_type": detail["code"], **detail},
        )
        self._bus.publish(error_event)
        self._store.apply(error_event)
        self._mark_status(STATUS_FAILED)

    # ------------------------------------------------------------------ #
    # durable state                                                      #
    # ------------------------------------------------------------------ #
    def _on_store_change(
        self, snapshot: dict[str, Any], event: SearchEvent | None,
    ) -> None:
        """Persist every materialized state transition atomically.

        This callback runs after SearchStateStore releases its lock, so slow
        storage can never expose a half-mutated in-memory snapshot.
        """
        try:
            self._archive.record(snapshot, event)
        except (OSError, ValueError):
            # Search safety must not depend on storage health.  A later state
            # transition will retry the atomic snapshot write.
            pass

    def _restore_latest_session(self) -> None:
        """Restore the newest WebUI session when the web process starts."""
        try:
            record = self._archive.latest()
        except (OSError, ValueError):
            return
        if not record:
            return
        marker = dict(record.get("session") or {})
        state = normalize_search_snapshot(dict(record.get("state") or {}))
        session_id = str(marker.get("session_id") or state.get("session_id") or "")
        if not session_id:
            return
        self._session_id = session_id
        self._status = str(state.get("status") or marker.get("status") or STATUS_IDLE)
        self._started_at = state.get("started_at")
        task = dict(state.get("task") or marker.get("task") or {})
        self._task_context = None
        self._info = SearchSessionInfo(
            session_id=session_id,
            target=str(state.get("target") or marker.get("target") or ""),
            task_text=str(marker.get("task_text") or task.get("raw_text") or ""),
            task_context=task,
            status=self._status,
            result=str(state.get("result") or marker.get("result") or ""),
            started_at=state.get("started_at"),
            finished_at=state.get("finished_at"),
            backend=str(state.get("backend") or marker.get("backend") or ""),
            reasoner=str(state.get("reasoner") or marker.get("reasoner") or "semantic"),
        )
        self._store.restore(state)
        # Persist the compatibility migration so subsequent page loads and
        # history metadata no longer resurrect the obsolete FAILED label.
        try:
            self._archive.record(state)
        except (OSError, ValueError):
            pass
        self._bus.restore_recent(list(record.get("events") or []))
        if self._status not in TERMINAL_STATUSES and self._status != STATUS_IDLE:
            # A new web process cannot safely reclaim the old worker's stdin,
            # control lease or ROS handles.  Preserve the snapshot and make
            # that interruption explicit rather than pretending it is live.
            self._fail_interrupted_session_locked(
                "WebUI 服务重启后无法安全接管原搜索 worker；原任务已标记为中断，重启前状态已完整保留。"
            )

    def _mark_status(self, status: str) -> None:
        with self._lock:
            self._status = status
            if self._info is not None:
                self._info.status = status

    def _elapsed(self, session: SearchSessionInfo) -> float:
        if session.finished_at is not None and session.started_at is not None:
            return round(max(0.0, session.finished_at - session.started_at), 2)
        if session.started_at is not None:
            return round(max(0.0, time.time() - session.started_at), 2)
        return 0.0


def _default_executor_factory() -> SearchExecutor:
    """Real deployment: subprocess worker under the ROS2 system Python."""
    from app.manual_web_demo.search_executor import SubprocessSearchExecutor

    return SubprocessSearchExecutor(
        log_path=_PROJECT_ROOT
        / "outputs"
        / "autonomous_search"
        / "logs"
        / "search_worker.log",
    )


def make_mock_executor_factory(
    *, scenario: str = "anchor_then_target", mock_target_after: int = 3,
    confirm_after_seen: int = 1, outcome_sequence: list[str] | None = None,
    backend_latency_sec: float = 0.0,
    scene_steps: list[dict[str, Any]] | None = None,
) -> Callable[[], SearchExecutor]:
    """Factory for tests / offline frontend dev (in-process mock)."""

    def factory() -> SearchExecutor:
        from app.manual_web_demo.search_executor import InProcessMockExecutor

        return InProcessMockExecutor(
            scenario=scenario,
            mock_target_after=mock_target_after,
            confirm_after_seen=confirm_after_seen,
            outcome_sequence=list(outcome_sequence or []),
            backend_latency_sec=backend_latency_sec,
            scene_steps=list(scene_steps or []),
        )

    factory.is_mock_factory = True  # type: ignore[attr-defined]
    return factory
