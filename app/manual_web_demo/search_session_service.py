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
import os
import time
from pathlib import Path
from typing import Any, Callable, Mapping

from app.live_robot.explorer_search_adapter import ExplorerSearchAdapter
from app.live_robot.search_event import (
    ERROR,
    OPERATOR_STOP,
    SEARCH_FINISHED,
    SEARCH_STATE_CHANGED,
    SESSION_CREATED,
    STOP_REQUESTED,
    TASK_REJECTED,
    TASK_UNDERSTANDING,
    SearchEvent,
    make_event,
)


def _touch_slam_reset_marker() -> None:
    """计划书 §10.3：新搜索 session 触发 plain_slam WebUI 桥清空旧累积点云。"""
    if os.environ.get("GO2W_WEB_POINTCLOUD_BRIDGE_ENABLED", "1").lower() in {"0", "false", "no"}:
        return
    marker = os.environ.get("GO2W_SLAM_RESET_MARKER", "")
    if not marker:
        return
    try:
        path = Path(marker)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
    except OSError:
        pass
from app.live_robot.search_event_bus import SearchEventBus
from app.live_robot.search_state_store import (
    FAILURE_COMPLETION_RESULTS,
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
    SEARCH_REASONER,
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


def _navigation_trial_start_error(request: SearchStartRequest) -> str | None:
    """Revalidate a bounded trial after potentially slow task parsing.

    The HTTP route checks readiness before calling ``start_search``, but task
    understanding can take tens of seconds.  A direct acceptance tool may
    reserve or exhaust the shared ledger during that interval.  Re-reading it
    here closes the observed check/use gap before this service claims control
    or launches a worker.  Unlimited field mode has no ledger and is unchanged.
    """
    if request.backend != "nav2" or request.dry_run_motion:
        return None
    trial_path = os.getenv("GO2W_NAV2_TRIAL_LEDGER", "").strip()
    if not trial_path:
        return None
    try:
        trial = json.loads(Path(trial_path).read_text(encoding="utf-8"))
        maximum_steps = int(trial["maximum_steps"])
        steps = trial["steps"]
        if (maximum_steps <= 0 or not isinstance(steps, list)
                or len(steps) > maximum_steps):
            raise ValueError("invalid bounded trial counters")
    except (OSError, ValueError, KeyError, TypeError):
        return "navigation trial ledger became invalid during task understanding"
    if trial.get("active_token") is not None:
        return "navigation trial became active during task understanding"
    if len(steps) >= maximum_steps:
        return "navigation trial was exhausted during task understanding"
    return None


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
        map_paths: Mapping[str, str] | None = None,
        replay_manager: Any | None = None,
    ) -> None:
        self.owner = owner
        self._executor_factory = executor_factory or _default_executor_factory
        self._executor_factory_is_mock = bool(
            getattr(self._executor_factory, "is_mock_factory", False)
        )
        self._session_dir = session_dir
        # 回放管理器由 Web 进程注入（禁用的实例或 None 都表示不录制）。
        # 它只能做有界的入队/小文件写入，搜索安全不依赖它是否健康。
        self._replay_manager = replay_manager
        self._bus = SearchEventBus(max_recent=int(event_buffer))
        self._archive = SearchSessionArchive(
            session_dir, max_sessions=history_limit,
            protect=self._session_protected,
        )
        self._store = SearchStateStore(on_change=self._on_store_change)
        self._adapter: ExplorerSearchAdapter | None = None
        self._executor: SearchExecutor | None = None
        self._lock = threading.Lock()
        self._session_id: str | None = None
        self._info: SearchSessionInfo | None = None
        self._started_at: float | None = None
        self._stopping_at: float | None = None
        self._status = STATUS_IDLE
        self._task_context: SearchTaskContext | None = None
        self._executor_id: str | None = None
        self._worker_generation = 0
        self._task_understanding_runner = task_understanding_runner
        self._allow_mock_task_fallback = bool(allow_mock_task_fallback)
        # §2.3：三维对象上屏的两个路径来自 Web 侧配置（显式路径，不 glob、不按
        # mtime 猜）。它们必须与地图桥/读取端用的是同一份配置 —— 否则生产者的
        # 写点和读取端会静默错开，页面上只是"没有物体"，看不出是路径没对上。
        self._map_paths = dict(map_paths or {})
        self._restore_latest_session()

    # ------------------------------------------------------------------ #
    # queries                                                            #
    # ------------------------------------------------------------------ #
    @property
    def replay_manager(self) -> Any | None:
        """Replay lifecycle owner injected by the web runtime (may be None)."""
        return self._replay_manager

    def current_session(self) -> SearchSessionInfo | None:
        with self._lock:
            if self._info is None:
                return None
            # The store is the authoritative status source once a session is
            # active (PAUSED / RESUMED / TARGET_FOUND / ... all flow through
            # SearchEvents); the service-level status only tracks IDLE vs
            # active for the lifecycle gate.
            store_snapshot = self._store.lifecycle_snapshot()
            store_status = self._effective_status(store_snapshot)
            store_result = store_snapshot.get("result") or ""
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
                # Compatibility with restored/pre-detector session records
                # and lightweight test doubles created before this field was
                # introduced.
                detector=getattr(self._info, "detector", ""),
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
            # Once the worker reports its health, that effective value is
            # authoritative.  Before then the requested value is a clearly
            # temporary startup fallback for backends without a health event.
            snapshot["detector_backend"] = str(
                snapshot.get("detector_backend") or session.detector
            )
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
        alive = bool(self._executor.alive())
        # A terminal worker is retired asynchronously after its final result.
        # Do not expose its last RUNNING payload through /api/status while the
        # session is already archived and no worker can accept commands.
        if not alive:
            return {"state": "stopped", "session_id": None, "alive": False}
        status["alive"] = True
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
            stage=str(self._store.lifecycle_snapshot().get("phase") or "RUNNING"),
        )
        error_event = make_event(
            allocator=self._bus.allocator,
            session_id=self._session_id,
            event_type=ERROR,
            payload={"error_type": detail["code"], **detail},
        )
        self._store.apply(error_event)
        self._bus.publish(error_event)
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
        self._store.apply(finish)
        self._bus.publish(finish)
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
        if self._status == STATUS_STOPPING:
            # A search may run longer than the startup timeout before Stop.
            # Give cooperative shutdown its own window, starting at Stop.
            if self._stopping_at is not None:
                timed_out = (time.monotonic() - self._stopping_at) > STARTING_TIMEOUT_SEC
        elif self._started_at is not None:
            timed_out = (time.time() - self._started_at) > STARTING_TIMEOUT_SEC
        if timed_out or not executor_alive:
            if timed_out and executor_alive and self._executor is not None:
                try:
                    self._executor.stop()
                    self._executor.shutdown()
                except Exception:  # noqa: BLE001 - terminal cleanup is best effort
                    pass
            if timed_out:
                reason = (
                    "搜索停止超过等待时限，worker 未正常结束；已保留停止诊断。"
                    if self._status == STATUS_STOPPING else
                    "搜索启动超过等待时限，worker 未进入运行状态；已保留启动诊断。"
                )
            else:
                reason = "搜索 worker 在启动或停止阶段退出；已保留退出前的全部搜索状态。"
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
            return self._effective_status(self._store.lifecycle_snapshot())
        return self._status

    def _effective_status(self, snapshot: dict[str, Any]) -> str:
        status = snapshot.get("status") or self._status
        # The control message is sent before durable event I/O. Cover that
        # short interval without hiding a synchronous terminal response.
        if self._status == STATUS_STOPPING and status not in TERMINAL_STATUSES:
            return STATUS_STOPPING
        return status

    def start_search(self, request: SearchStartRequest) -> dict[str, Any]:
        # 计划书 §6.6-2：mock 只能由测试台注入，不能从生产请求参数进来。
        # 生产跑的是 ROS 子进程工厂（search_executor_factory 为 None ⇒ 默认工厂），
        # 这个标记恒 False；只有传了 make_mock_executor_factory 的测试实例才为真。
        legacy_trial = os.environ.get("GO2W_ALLOW_SUPERVISED_LEGACY_SEARCH", "").lower() in {
            "1", "true", "yes", "on",
        }
        launcher_armed = os.environ.get("AUTONOMOUS_SEARCH_ENABLE_AUTONOMOUS_MOTION", "").lower() in {
            "1", "true", "yes", "on",
        }
        error = request.validate(
            allow_mock=self._executor_factory_is_mock,
            allow_legacy_supervised=legacy_trial,
            legacy_motion_armed=launcher_armed,
        )
        if error:
            detail = search_error(error, code="TASK_INVALID", source="search_api", stage="VALIDATE")
            return {"ok": False, "error": error, "error_detail": detail}
        if (request.backend == "nav2" and not request.dry_run_motion
                and not self._executor_factory_is_mock
                and os.getenv("GO2W_NAV2_EXECUTION_AUTHORIZED") != "1"):
            error = "Nav2 运动尚未由启动流程授权；当前网页只允许只读验证。"
            return {"ok": False, "error": error, "forbidden": True}
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
            trial_error = _navigation_trial_start_error(request)
            if trial_error:
                detail = search_error(
                    trial_error,
                    code="NAVIGATION_TRIAL_UNAVAILABLE",
                    source="search_service",
                    stage="START",
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
            self._stopping_at = None
            self._started_at = time.time()
            self._info = SearchSessionInfo(
                session_id=session_id,
                target=task_context.canonical_target,
                status="STARTING",
                task_text=task_context.raw_text,
                task_context=task_context.to_dict(),
                backend=request.backend,
                reasoner=SEARCH_REASONER,
                detector=request.detector,
                started_at=self._started_at,
            )

            self._archive.begin(session_id, {
                "target": task_context.canonical_target,
                "task_text": task_context.raw_text,
                "task": task_context.to_dict(),
                "backend": request.backend,
                "reasoner": SEARCH_REASONER,
                # 会话归档里记明物体识别走的是哪条路：两条路的产物不同
                # （grounded_sam 有 3D box，llm 没有），事后取证要用。
                "detector": request.detector,
                "motion_enabled": request.enable_autonomous_motion,
            })

            # One bus lives for the whole web process (the /ws/search hub
            # subscribes once); a new session only clears its event history.
            self._bus.clear()
            self._store = SearchStateStore(on_change=self._on_store_change)
            self._store.reset(
                session_id=session_id,
                target=task_context.canonical_target,
                reasoner=SEARCH_REASONER,
                backend=request.backend,
                task_context=task_context.to_dict(),
            )
            self._adapter = ExplorerSearchAdapter(
                self._bus, self._store, session_id=session_id,
            )
            # 工厂一律用注入的那个。这里原本还有一条「生产收到 backend=mock
            # 就用裸的 InProcessMockExecutor」的分支，§6.6-2 把 mock 挡在
            # validate() 之后它已经不可达（能走到这儿的 mock 请求必然
            # _executor_factory_is_mock，也就是测试台自己注入的工厂，
            # 而那个工厂产出的同样是 InProcessMockExecutor，只是参数是测试给的）。
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
                "backend": request.backend,
                "detector": request.detector,
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
                "session_dir": str(session_dir_path),
                "output": str(run_dir / "events.jsonl"),
                **self._map_paths,
            }
            for key in (
                "max_seconds", "max_planning_cycles", "max_motion_steps",
                "llm_model", "verify_min_confidence",
            ):
                value = getattr(request, key)
                if value is not None:
                    params[key] = value
            # 回放录制在第一条事件、也是任何运动之前绑定：采集器是独立只读
            # ROS 进程，最多等 capture_ready_timeout 秒就绪，绝不阻塞搜索。
            self._bind_replay_session(
                session_id, run_dir, task_context=task_context,
                request=request, params=params,
            )
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
                    "backend": request.backend,
                    "phase": "STARTING",
                },
            )
            self._store.apply(created)
            self._bus.publish(created)
            understood = make_event(
                allocator=self._bus.allocator,
                session_id=session_id,
                event_type=TASK_UNDERSTANDING,
                payload={"task": task_context.to_dict()},
            )
            self._store.apply(understood)
            self._bus.publish(understood)
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
                self._abandon_replay_session(
                    session_id, task_context.rejection_reason or "任务不可执行",
                )
                self._store.apply(rejected)
                self._bus.publish(rejected)
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
            # 计划书 §10.3：新 session 清空 WebUI 旧 3D 累积点云。
            _touch_slam_reset_marker()
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
                self._store.apply(error_event)
                self._bus.publish(error_event)
                finish = make_event(
                    allocator=self._bus.allocator, session_id=session_id,
                    event_type=SEARCH_FINISHED,
                    payload={"result": "FAILED", "finish_reason": detail["code"],
                             "error": detail["message"], "error_detail": detail},
                )
                self._store.apply(finish)
                self._bus.publish(finish)
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
            if self._status != STATUS_STOPPING:
                self._stopping_at = time.monotonic()
            self._status = STATUS_STOPPING
            if self._info is not None:
                self._info.status = STATUS_STOPPING
            executor = self._executor
            session_id = self._session_id
        if executor is not None:
            executor.stop()
        # Never delay the stop command behind archive/replay/WebSocket work.
        # A concurrent restart must not receive the old session's intent.
        with self._lock:
            if self._session_id != session_id or self._executor is not executor:
                return {"ok": True, "status": STATUS_STOPPING}
            requested = make_event(
                allocator=self._bus.allocator, session_id=session_id or "",
                event_type=STOP_REQUESTED, payload={"reason": reason},
            )
            self._store.apply(requested)
            status = self._active_status()
        self._bus.publish(requested)
        return {"ok": True, "status": status}

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
                store_status = self._store.lifecycle_snapshot().get("status")
                if store_status in {
                    STATUS_TARGET_FOUND,
                    STATUS_SEARCH_EXHAUSTED,
                    STATUS_OPERATOR_STOP,
                    STATUS_FINISHED,
                    STATUS_FAILED,
                }:
                    self.owner.release(OwnerState.AUTONOMOUS)
        elif msg_type == "graph_snapshot":
            graph = message.get("graph")
            if isinstance(graph, dict) and self._adapter is not None:
                # 归档先落盘，再做 WebSocket 扇出：推送失败不能丢事实。
                try:
                    self._archive.record_graph(self._session_id or "", graph)
                except (OSError, ValueError):
                    # 搜索安全不依赖存储健康；下一个 revision 会重试。
                    pass
                self._adapter.on_graph_snapshot(graph)
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
            current_snapshot = self._store.lifecycle_snapshot()
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
                if (
                    backend == "nav2"
                    and not context.executable
                    and str((context.parsed_task or {}).get("parser_source") or "")
                    == "llm_unavailable"
                ):
                    return SearchTaskContext.safe_navigation_fallback(
                        task_text,
                        failure_reason=context.rejection_reason or "llm_unavailable",
                    )
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
            if (
                not structured.executable
                and backend == "nav2"
                and parser_source == "llm_unavailable"
            ):
                return SearchTaskContext.safe_navigation_fallback(
                    task_text,
                    failure_reason=structured.rejection_reason or parser_source,
                )
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
        self._store.apply(finish)
        self._bus.publish(finish)
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
                if finish_reason in FAILURE_COMPLETION_RESULTS or failed_exit:
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
        self._store.apply(error_event)
        self._bus.publish(error_event)
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
        self._record_replay_change(snapshot, event)

    def _record_replay_change(
        self, snapshot: dict[str, Any], event: SearchEvent | None,
    ) -> None:
        """Bounded replay hooks: enqueue events and finalize on terminal state.

        Both calls only enqueue work for the replay worker thread; they never
        read maps, encode video or wait for the collector.
        """
        manager = self._replay_manager
        if manager is None:
            return
        session_id = str(snapshot.get("session_id") or "")
        if not session_id:
            return
        try:
            if event is not None:
                manager.record_event(session_id, event)
            status = str(snapshot.get("status") or "")
            if status in TERMINAL_STATUSES:
                manager.finish_session(session_id, snapshot)
        except Exception:  # noqa: BLE001 - replay must never break the search
            pass

    def _bind_replay_session(
        self, session_id: str, run_dir: Path, *,
        task_context: SearchTaskContext, request: SearchStartRequest,
        params: Mapping[str, Any],
    ) -> None:
        manager = self._replay_manager
        if manager is None:
            return
        try:
            manager.bind_session(
                session_id, run_dir,
                target=task_context.canonical_target,
                task_text=task_context.raw_text,
                backend=request.backend,
                params=dict(params),
            )
        except Exception:  # noqa: BLE001 - 录制失败不能挡住搜索启动
            pass

    def _abandon_replay_session(self, session_id: str, reason: str) -> None:
        manager = self._replay_manager
        if manager is None:
            return
        try:
            manager.abandon_session(session_id, reason)
        except Exception:  # noqa: BLE001
            pass

    def _session_protected(self, session_id: str) -> bool:
        """Archive prune guard: never delete a session the replay still owns."""
        if not session_id:
            return False
        if (session_id == self._session_id
                and self._status not in TERMINAL_STATUSES
                and self._status != STATUS_IDLE):
            return True
        manager = self._replay_manager
        if manager is None:
            return False
        try:
            return bool(manager.session_protected(session_id))
        except Exception:  # noqa: BLE001
            return False

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
        restored_status = str(state.get("status") or marker.get("status") or STATUS_IDLE)
        # History is durable evidence, not a live worker lease.  A fresh web
        # process must start with an empty IDLE slot after a terminal task;
        # the archived session remains available through the history API.
        if restored_status in TERMINAL_STATUSES or restored_status == STATUS_IDLE:
            return
        self._session_id = session_id
        self._status = restored_status
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
            reasoner=str(state.get("reasoner") or marker.get("reasoner") or SEARCH_REASONER),
            detector=str(
                state.get("detector_backend") or marker.get("detector") or ""
            ),
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
            # The interruption record above is archived.  Do not expose it as
            # the current live session or reuse its map/target evidence.
            self._reset_session_locked()
            self._bus.clear()

    def _mark_status(self, status: str) -> None:
        with self._lock:
            if self._status == STATUS_STOPPING and status == STATUS_RUNNING:
                return
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
