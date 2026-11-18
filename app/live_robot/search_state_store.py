"""SearchStateStore: authoritative latest-snapshot of the current search
session (plan book §20-§21, §96).

The store is updated exclusively from SearchEvents so the WebUI (page load,
F5, WebSocket reconnect) can always recover the full state without replaying
the whole stream.  All reads take a lock and return deep copies; the store
never half-updates a snapshot.

计划书 §2.2 的状态形状：``snapshot()`` 返回**小状态加一份图**。

* ``graph`` / ``graph_revision`` —— 唯一图，由 :meth:`set_graph` 安装，只认
  ``revision`` 递增的那一份。它**不放在** ``self._snapshot`` 里，因此每个普通
  事件在锁内的 ``deepcopy`` 不带上整张图。
* ``spatial`` 只保留真正独立的空间数据（RGB-D、位姿、路线、地图健康、PSG 先验、
  长期/局部目标、关联调试）。``semantic_graph``/``spatial_map``/``place_graph``/
  ``semantic_objects``/``frontiers`` 都是旧图副本，已删除 —— 对象、Place、
  Frontier、拓扑一律从 ``graph`` 读。
* 旧 ``map`` 字段删除。
"""

from __future__ import annotations

import copy
import threading
import time
from collections import Counter, deque
from typing import Any, Callable

from app.live_robot.search_event import (
    ACTION_FINISHED,
    ACTION_STARTED,
    CANDIDATES_GENERATED,
    DECISION_RECORDED,
    ERROR,
    FRONTIERS_UPDATED,
    GOAL_SELECTED,
    LOCAL_GOAL_PROGRESS,
    LONG_TERM_GOAL_SELECTED,
    MAP_UPDATED,
    OBJECTS_UPDATED,
    OBSERVATION_UPDATED,
    OPERATOR_STOP,
    PAUSED,
    PLACE_CREATED,
    PLACE_UPDATED,
    PSG_PRIOR_UPDATED,
    REPLAN,
    RESUMED,
    RGBD_FRAME_UPDATED,
    SEARCH_EXHAUSTED,
    SEARCH_FINISHED,
    SEARCH_STATE_CHANGED,
    STOP_REQUESTED,
    SEMANTIC_OBJECT_LOCALIZED,
    SEMANTIC_REGION_CREATED,
    SESSION_CREATED,
    TASK_REJECTED,
    TASK_UNDERSTANDING,
    SESSION_STARTED,
    SPATIAL_MAP_UPDATED,
    SPATIAL_POSE_UPDATED,
    TARGET_CONFIRMED,
    TARGET_MATCH_UPDATED,
    VERIFICATION_FINISHED,
    SearchEvent,
)

TIMELINE_LIMIT = 500
FAILURE_COMPLETION_RESULTS = frozenset({
    'FAILED', 'TIMEOUT', 'BACKEND_FAILURE', 'PERCEPTION_FAILURE',
    'PLANNING_ERROR', 'MAP_UNAVAILABLE',
})

# 计划书 §2.2：这些事件仍然进时间线、WebSocket 和归档事件日志（原始证据不能
# 丢），但**不再**往 store 里投影第二份图数据 —— 对象、Place、Frontier 一律从
# 唯一的 ``graph`` 读。它们必须在这里显式列出，否则会被当成未知事件整条丢掉。
_RECORD_ONLY_EVENTS = frozenset({
    MAP_UPDATED,
    SPATIAL_MAP_UPDATED,
    FRONTIERS_UPDATED,
    PLACE_CREATED,
    PLACE_UPDATED,
    SEMANTIC_OBJECT_LOCALIZED,
})
# Search-session status values (plan book §38).
STATUS_IDLE = "IDLE"
STATUS_STARTING = "STARTING"
STATUS_RUNNING = "RUNNING"
STATUS_PAUSED = "PAUSED"
STATUS_STOPPING = "STOPPING"
STATUS_TARGET_FOUND = "TARGET_FOUND"
STATUS_SEARCH_EXHAUSTED = "SEARCH_EXHAUSTED"
STATUS_FAILED = "FAILED"
STATUS_OPERATOR_STOP = "OPERATOR_STOP"
STATUS_FINISHED = "FINISHED"

# Reaching an operator-configured exploration budget is a normal terminal
# condition, not a runtime fault.  Keep the precise result for history while
# rendering the session as completed and without an error card.
BUDGET_COMPLETION_RESULTS = frozenset({
    "MAX_STEPS_REACHED",
    "MAX_PLANNING_CYCLES_REACHED",
    "BUDGET_EXHAUSTED",
})


def normalize_search_snapshot(snapshot: dict[str, Any]) -> dict[str, Any]:
    """Migrate historical budget-limited snapshots away from FAILED."""
    value = copy.deepcopy(snapshot or {})
    result = str(value.get("result") or value.get("finish_reason") or "")
    if result in BUDGET_COMPLETION_RESULTS:
        value["status"] = STATUS_FINISHED
        value["phase"] = STATUS_FINISHED
        value["error"] = None
        value["last_error"] = ""
    error = value.get("error") if isinstance(value.get("error"), dict) else {}
    if (
        str(value.get("status") or "") == STATUS_FAILED
        and not str(value.get("finish_reason") or "")
        and str(error.get("source") or "") in {"observer_retry", "perception_retry"}
    ):
        # Older builds incorrectly persisted a retry notification as a
        # terminal failure.  If no terminal finish reason ever arrived, keep
        # the diagnostic as a warning and mark the interrupted attempt closed.
        value["status"] = STATUS_FINISHED
        value["phase"] = STATUS_FINISHED
        value["result"] = "INTERRUPTED_DURING_RETRY"
        value["finish_reason"] = "WEBUI_RESTART_DURING_RETRY"
        value["last_warning"] = copy.deepcopy(error)
        value["error"] = None
    return value


def _empty_snapshot() -> dict[str, Any]:
    return {
        "session_id": "",
        "status": STATUS_IDLE,
        "result": "",
        "target": "",
        "reasoner": "semantic",
        "task": {},
        "backend": "",
        # Effective perception backend reported by the worker.  The Web
        # service may show the requested value only until this arrives.
        "detector_backend": "",
        "phase": "",
        "phase_detail": "",
        "phase_started_at": None,
        "cycle": 0,
        "replans": 0,
        "navigation_failures": 0,
        "elapsed_seconds": 0.0,
        "started_at": None,
        "finished_at": None,
        "finish_reason": "",
        "observation": {
            "bundle_id": None,
            "timestamp": None,
            "objects": [],
            "detections": [],
            "target_present": False,
            "target_state": "ABSENT",
            "heading_sector": None,
            "pose": None,
            "image_ref": None,
            "depth_ref": None,
            "rgbd_frame_id": None,
            "intrinsics": None,
            "depth_scale": None,
            "spatial_quality": "RGB_ONLY",
            "camera_xyz": None,
            "sensor_health": {},
        },
        "objects": {
            "current": [],
            "session_seen": [],
            "target_evidence": {},
        },
        "target_match": {
            "level": "none",
            "target_state": "ABSENT",
            "target_confirmed": False,
            "explicit_anchor_found": False,
            "anchor_labels": [],
            "target_score": 0.0,
            "directive": None,
            "graph_match": None,
        },
        "goal_graph": None,
        "selected_goal": None,
        "next_motion_command": None,
        "last_decision": None,
        "decisions": [],
        "candidates": [],
        "robot": {
            "motion_status": "IDLE",
            "pose_quality": "unavailable",
            "pose": None,
            "last_motion_result": None,
        },
        "spatial": {
            "rgbd_frame": None,
            "spatial_pose": None,
            "route_plan": None,
            "psg_prior": None,
            "long_term_goal": None,
            "local_goal_progress": None,
        },
        "startup": {
            "stage": "IDLE",
            "stage_started_at": None,
            "last_progress_at": None,
            "worker_alive": False,
            "worker_state": "idle",
            "last_worker_message_at": None,
            "last_error": None,
        },
        # 计划书 §29/§30：净空 / 感知质量 / 卡住 / 视点 / 脱困 / 目标接近的
        # 可观测摘要。这些状态决定"能不能动"，刷新页面后必须还在。
        "embodiment": {
            "local_safety": None,
            "perception_quality": None,
            "stuck_state": None,
            "viewpoint_candidates": [],
            "selected_viewpoint": None,
            "safety_rejections": [],
            "recovery": None,
            "target_approach": None,
        },
        "health": {},
        # 任务 Agent 面板的状态（计划书 §5.4）。复用 TARGET_MATCH_UPDATED 里
        # 已经携带的 directive，不新增 AGENT_UPDATED 事件。
        "agent": _empty_agent(),
        "last_warning": None,
        "timeline": [],
        "error": None,
    }


def _empty_agent() -> dict[str, Any]:
    return {
        "trace": None,
        "updated_at": None,
        "decision_id": None,
        "downstream_status": "pending",
        "downstream_reason": None,
    }


def _agent_from_directive(directive: Any, *, updated_at: Any) -> dict[str, Any] | None:
    """从 directive 里取出任务 Agent 的 trace。

    取不到就返回 `None`，让调用方保留上一份 trace —— 不能用一个空壳把
    已有面板内容清掉。
    """
    if not isinstance(directive, dict):
        return None
    trace = directive.get("decision_trace")
    if not isinstance(trace, dict):
        return None
    return {
        "trace": trace,
        "updated_at": updated_at,
        "decision_id": None,
        "downstream_status": "pending",
        "downstream_reason": None,
    }


def _downstream_status(trace: dict[str, Any] | None, record: dict[str, Any]) -> str:
    """Agent 的建议有没有成为最终决定（计划书 §5.3）。

    只在**可核对**时下结论：最终候选带 `directive_id` 且等于本轮 trace_id ⇒
    采纳；最终候选的来源明确不是语义指令 ⇒ 未采纳（"最终由 local_scan 获胜"
    这类结论就是这个事实）。其余一律 `pending` —— 绝不猜"被拒绝"。

    具体拒绝原因只取下游真的给了的 `replan_reason`；没有就是 None，
    由前端显示"下游未提供拒绝原因"。
    """
    if not trace:
        return "pending"
    ranking = record.get("candidate_ranking") or []
    if not ranking:
        return "pending"
    goal = (ranking[0] or {}).get("goal") or {}
    provenance = goal.get("provenance") or {}
    source = str(provenance.get("source") or "")
    if str(provenance.get("directive_id") or "") == str(trace.get("trace_id") or ""):
        return "accepted"
    if source and source != "semantic_navigation_directive":
        return "rejected"
    return "pending"


class SearchStateStore:
    """Thread-safe latest-snapshot store for one search session."""

    def __init__(self, *, max_timeline: int = TIMELINE_LIMIT,
                 on_change: Callable[[dict[str, Any], SearchEvent | None], None] | None = None) -> None:
        self._lock = threading.Lock()
        self._snapshot: dict[str, Any] = _empty_snapshot()
        # 唯一图与它的版本号，刻意放在 _snapshot 之外（见模块 docstring）。
        # -1 表示本会话还没有任何图，因此 revision 0 也会被接受。
        self._graph: dict[str, Any] | None = None
        self._graph_revision = -1
        self._session_seen: Counter[str] = Counter()
        self._current_objects: list[dict[str, Any]] = []
        self._timeline: deque[dict[str, Any]] = deque(maxlen=max(1, int(max_timeline)))
        self._last_goal: dict[str, Any] | None = None
        self._on_change = on_change

    # ------------------------------------------------------------------ #
    # lifecycle                                                          #
    # ------------------------------------------------------------------ #
    def reset(
        self,
        *,
        session_id: str,
        target: str,
        reasoner: str = "semantic",
        backend: str = "mock",
        task_context: dict[str, Any] | None = None,
    ) -> None:
        with self._lock:
            self._snapshot = _empty_snapshot()
            self._snapshot["session_id"] = session_id
            self._snapshot["target"] = target
            self._snapshot["reasoner"] = reasoner
            self._snapshot["backend"] = backend
            self._snapshot["task"] = copy.deepcopy(task_context or {})
            self._snapshot["status"] = STATUS_STARTING
            self._snapshot["started_at"] = time.time()
            self._graph = None
            self._graph_revision = -1
            self._session_seen = Counter()
            self._current_objects = []
            self._timeline = deque(maxlen=self._timeline.maxlen)
            self._last_goal = None

    # ------------------------------------------------------------------ #
    # event application                                                  #
    # ------------------------------------------------------------------ #
    def apply(self, event: SearchEvent) -> None:
        handler = {
            SESSION_CREATED: self._on_session_created,
            TASK_UNDERSTANDING: self._on_task_understanding,
            TASK_REJECTED: self._on_task_rejected,
            SESSION_STARTED: self._on_session_started,
            SEARCH_STATE_CHANGED: self._on_search_state_changed,
            OBSERVATION_UPDATED: self._on_observation,
            OBJECTS_UPDATED: self._on_objects,
            TARGET_MATCH_UPDATED: self._on_target_match,
            VERIFICATION_FINISHED: self._on_verification,
            TARGET_CONFIRMED: self._on_target_confirmed,
            CANDIDATES_GENERATED: self._on_candidates,
            GOAL_SELECTED: self._on_goal_selected,
            ACTION_STARTED: self._on_action_started,
            ACTION_FINISHED: self._on_action_finished,
            REPLAN: self._on_replan,
            RGBD_FRAME_UPDATED: self._on_rgbd_frame,
            SPATIAL_POSE_UPDATED: self._on_spatial_pose,
            PSG_PRIOR_UPDATED: self._on_psg_prior,
            SEMANTIC_REGION_CREATED: self._on_semantic_region_created,
            LONG_TERM_GOAL_SELECTED: self._on_long_term_goal_selected,
            LOCAL_GOAL_PROGRESS: self._on_local_goal_progress,
            DECISION_RECORDED: self._on_decision_recorded,
            PAUSED: self._on_paused,
            RESUMED: self._on_resumed,
            STOP_REQUESTED: self._on_stop_requested,
            SEARCH_EXHAUSTED: self._on_search_exhausted,
            OPERATOR_STOP: self._on_operator_stop,
            ERROR: self._on_error,
            SEARCH_FINISHED: self._on_search_finished,
        }.get(event.event_type)
        if handler is None and event.event_type not in _RECORD_ONLY_EVENTS:
            return
        changed: dict[str, Any] | None = None
        with self._lock:
            # Keep late telemetry and target evidence, but do not let queued
            # worker progress undo a durable stop request. Only an actual
            # terminal event can acknowledge completion. reset() starts anew.
            lifecycle_keys = (
                "status", "phase", "phase_detail", "phase_started_at",
                "result", "finish_reason", "finished_at", "summary",
            )
            stopped_lifecycle = None
            if (self._snapshot.get("stop_requested_at") is not None or
                    self._snapshot.get("status") == STATUS_STOPPING):
                if event.event_type not in {
                    OPERATOR_STOP, SEARCH_FINISHED, SEARCH_EXHAUSTED,
                    ERROR, TASK_REJECTED,
                }:
                    stopped_lifecycle = {
                        key: copy.deepcopy(self._snapshot[key])
                        for key in lifecycle_keys if key in self._snapshot
                    }
            # ``_RECORD_ONLY_EVENTS`` 没有 handler —— 它们只进时间线。
            if handler is not None:
                handler(event)
            if stopped_lifecycle is not None:
                for key in lifecycle_keys:
                    if key in stopped_lifecycle:
                        self._snapshot[key] = stopped_lifecycle[key]
                    else:
                        self._snapshot.pop(key, None)
            elif (self._snapshot.get("stop_requested_at") is not None and
                    event.event_type in {
                        OPERATOR_STOP, SEARCH_FINISHED, SEARCH_EXHAUSTED,
                        ERROR, TASK_REJECTED,
                    }):
                self._snapshot["phase"] = self._snapshot["status"]
                self._snapshot["phase_detail"] = ""
                self._snapshot["phase_started_at"] = event.timestamp
            self._append_timeline_locked(event)
            changed = copy.deepcopy(self._snapshot)
        if self._on_change is not None and changed is not None:
            self._on_change(changed, event)

    def restore(self, snapshot: dict[str, Any]) -> None:
        """Restore an already materialized snapshot without replaying events.

        The snapshot carries the session's single graph (small state + graph),
        so restoring reads it back into the graph slot rather than the state.
        """
        with self._lock:
            base = _empty_snapshot()
            base.update(normalize_search_snapshot(snapshot))
            graph = base.pop("graph", None)
            base.pop("graph_revision", None)
            self._snapshot = base
            self._graph = copy.deepcopy(graph) if isinstance(graph, dict) else None
            self._graph_revision = (
                int(self._graph.get("revision") or 0) if self._graph else -1
            )
            self._timeline = deque(list(base.get("timeline") or [])[-self._timeline.maxlen:],
                                   maxlen=self._timeline.maxlen)
            self._current_objects = list((base.get("objects") or {}).get("current") or [])
            self._session_seen = Counter({
                str(item.get("label") or ""): int(item.get("observations") or 0)
                for item in (base.get("objects") or {}).get("session_seen") or []
                if item.get("label")
            })
            self._last_goal = copy.deepcopy(base.get("selected_goal"))

    # ------------------------------------------------------------------ #
    # snapshots                                                          #
    # ------------------------------------------------------------------ #
    def lifecycle_snapshot(self) -> dict[str, Any]:
        """Read lifecycle fields atomically without copying frames or graph.

        Status polling and command gates do not consume the full scene. Keep
        their lock hold short even when masks and the semantic graph grow.
        """
        with self._lock:
            return copy.deepcopy({key: self._snapshot.get(key) for key in
                                  ("status", "result", "phase")})

    def snapshot(self) -> dict[str, Any]:
        """Small state + the session's single ``graph`` and ``graph_revision``."""
        with self._lock:
            value = copy.deepcopy(self._snapshot)
            graph = copy.deepcopy(self._graph)
            revision = self._graph_revision
        value["graph"] = graph
        value["graph_revision"] = revision if graph is not None else 0
        return value

    def set_graph(self, graph: dict[str, Any]) -> bool:
        """Install the session's single Web graph, newest revision wins.

        Only ``revision`` decides; a delayed older graph is dropped rather than
        regressing the display.  Returns whether the stored graph changed.
        """
        if not isinstance(graph, dict):
            return False
        revision = int(graph.get("revision") or 0)
        value = copy.deepcopy(graph)
        with self._lock:
            if revision <= self._graph_revision:
                return False
            self._graph = value
            self._graph_revision = revision
        return True

    # ------------------------------------------------------------------ #
    # handlers (callers hold the lock)                                   #
    # ------------------------------------------------------------------ #
    def _on_session_created(self, event: SearchEvent) -> None:
        payload = event.payload
        if not self._snapshot["session_id"]:
            self._snapshot["session_id"] = event.session_id
        self._snapshot["target"] = payload.get("target") or self._snapshot["target"]
        self._snapshot["phase"] = payload.get("phase") or "STARTING"
        if not self._snapshot["started_at"]:
            self._snapshot["started_at"] = event.timestamp
        self._snapshot["health"] = dict(payload.get("health") or self._snapshot["health"])

    def _on_task_understanding(self, event: SearchEvent) -> None:
        task = dict(event.payload.get("task") or {})
        self._snapshot["task"] = task
        if task.get("canonical_target"):
            self._snapshot["target"] = task["canonical_target"]
        self._snapshot["phase"] = "TASK_UNDERSTANDING"

    def _on_task_rejected(self, event: SearchEvent) -> None:
        self._snapshot["status"] = "TASK_REJECTED"
        self._snapshot["phase"] = "TASK_REJECTED"
        detail = copy.deepcopy(event.payload.get("error_detail") or {})
        detail.setdefault("error_type", "TASK_REJECTED")
        detail.setdefault("code", "TASK_REJECTED")
        detail.setdefault("message", event.payload.get("reason") or "任务不可执行")
        self._snapshot["error"] = detail
        self._snapshot["result"] = "TASK_REJECTED"
        self._snapshot["finish_reason"] = "TASK_REJECTED"
        self._snapshot["finished_at"] = event.timestamp

    def _on_session_started(self, event: SearchEvent) -> None:
        self._snapshot["status"] = STATUS_RUNNING
        self._snapshot["phase"] = event.payload.get("phase") or "OBSERVE"
        if not self._snapshot["started_at"]:
            self._snapshot["started_at"] = event.timestamp

    def _on_search_state_changed(self, event: SearchEvent) -> None:
        payload = event.payload
        next_phase = payload.get("phase") or self._snapshot["phase"]
        if next_phase != self._snapshot["phase"]:
            self._snapshot["phase_started_at"] = (
                payload.get("phase_started_at") or event.timestamp
            )
        self._snapshot["phase"] = next_phase
        if "phase_detail" in payload:
            self._snapshot["phase_detail"] = payload.get("phase_detail") or ""
        if payload.get("health"):
            health = dict(self._snapshot["health"])
            health.update(payload["health"])
            self._snapshot["health"] = health
        if payload.get("detector_backend"):
            self._snapshot["detector_backend"] = str(payload["detector_backend"])
        if payload.get("robot"):
            self._snapshot["robot"].update(payload["robot"])
        if payload.get("startup"):
            self._snapshot["startup"] = dict(payload["startup"])
        if payload.get("worker"):
            worker = dict(self._snapshot["startup"])
            worker.update(payload["worker"])
            worker["last_worker_message_at"] = event.timestamp
            worker["worker_alive"] = True
            self._snapshot["startup"] = worker
        if payload.get("warning"):
            self._snapshot["last_warning"] = copy.deepcopy(payload["warning"])
        if payload.get("embodiment"):
            # 目标接近是跨多个事件累积的（确认 -> 定位 -> 逐步 -> 最终确认），
            # 所以同名 dict 做合并而不是整体覆盖。
            embodiment = dict(self._snapshot["embodiment"])
            for key, value in dict(payload["embodiment"]).items():
                if isinstance(value, dict) and isinstance(embodiment.get(key), dict):
                    merged = dict(embodiment[key])
                    merged.update(value)
                    embodiment[key] = merged
                else:
                    embodiment[key] = copy.deepcopy(value)
            self._snapshot["embodiment"] = embodiment

    def _on_observation(self, event: SearchEvent) -> None:
        payload = event.payload
        observation = self._snapshot["observation"]
        observation["bundle_id"] = payload.get("bundle_id") or observation["bundle_id"]
        observation["timestamp"] = payload.get("timestamp") or event.timestamp
        observation["objects"] = list(payload.get("scene_objects") or payload.get("objects") or [])
        observation["detections"] = list(payload.get("detections") or [])
        observation["target_present"] = bool(payload.get("target_present", False))
        observation["target_state"] = str(
            payload.get("target_state")
            or ("PRESENT" if observation["target_present"] else "ABSENT")
        )
        observation["heading_sector"] = payload.get("heading_sector")
        observation["navigation_heading_sector"] = payload.get("navigation_heading_sector")
        observation["semantic_status"] = payload.get("semantic_status")
        observation["semantic_quality"] = payload.get("semantic_quality")
        observation["semantic_source_frame_id"] = payload.get("semantic_source_frame_id")
        observation["semantic_age_ms"] = payload.get("semantic_age_ms")
        observation["pose"] = payload.get("pose")
        observation["image_ref"] = payload.get("image_ref")
        observation["depth_ref"] = payload.get("depth_ref")
        observation["rgbd_frame_id"] = payload.get("rgbd_frame_id")
        observation["intrinsics"] = payload.get("intrinsics")
        observation["depth_scale"] = payload.get("depth_scale")
        observation["spatial_quality"] = payload.get("spatial_quality")
        observation["camera_xyz"] = payload.get("camera_xyz")
        observation["sensor_health"] = dict(payload.get("sensor_health") or {})
        if event.cycle is not None:
            self._snapshot["cycle"] = max(self._snapshot["cycle"], event.cycle)
        self._snapshot["phase"] = payload.get("phase") or self._snapshot["phase"]
        if payload.get("pose"):
            self._snapshot["robot"]["pose"] = payload["pose"]
            self._snapshot["robot"]["pose_quality"] = (
                payload.get("pose_quality") or payload["pose"].get("quality") or "relative"
            )

    def _on_objects(self, event: SearchEvent) -> None:
        payload = event.payload
        current = list(payload.get("current") or [])
        self._current_objects = current
        for item in current:
            label = str(item.get("label_zh") or item.get("label") or item.get("name") or "")
            if label:
                self._session_seen[label] += 1
        seen = [
            {"label": label, "observations": count}
            for label, count in sorted(
                self._session_seen.items(), key=lambda kv: (-kv[1], kv[0])
            )
        ]
        self._snapshot["objects"] = {
            "current": current,
            "session_seen": seen,
            "target_evidence": dict(payload.get("target_evidence") or {}),
        }

    def _on_target_match(self, event: SearchEvent) -> None:
        payload = event.payload
        match = self._snapshot["target_match"]
        match["level"] = payload.get("target_match_level") or match["level"]
        match["target_state"] = str(payload.get("target_state") or match["target_state"])
        match["target_score"] = float(payload.get("target_score", match["target_score"]))
        match["explicit_anchor_found"] = bool(
            payload.get("explicit_anchor_found", match["explicit_anchor_found"])
        )
        anchors = list(payload.get("anchor_labels") or [])
        if anchors:
            match["anchor_labels"] = anchors
        if payload.get("directive") is not None:
            match["directive"] = payload["directive"]
            agent = _agent_from_directive(
                payload["directive"], updated_at=getattr(event, "timestamp", None)
            )
            if agent is not None:
                self._snapshot["agent"] = agent
        if payload.get("graph_match") is not None:
            match["graph_match"] = payload["graph_match"]
        if payload.get("goal_graph") is not None:
            self._snapshot["goal_graph"] = payload["goal_graph"]
        self._snapshot["phase"] = payload.get("phase") or self._snapshot["phase"]
        evidence = self._snapshot["objects"]["target_evidence"]
        evidence.update(
            {
                "target_match_level": match["level"],
                "target_score": match["target_score"],
                "anchor_labels": match["anchor_labels"],
                "explicit_anchor_found": match["explicit_anchor_found"],
            }
        )

    def _on_verification(self, event: SearchEvent) -> None:
        payload = event.payload
        if payload.get("confirmed"):
            self._snapshot["target_match"]["target_confirmed"] = True
        self._snapshot["phase"] = "VERIFY"

    def _on_target_confirmed(self, event: SearchEvent) -> None:
        self._snapshot["status"] = STATUS_TARGET_FOUND
        self._snapshot["phase"] = "TARGET_FOUND"
        self._snapshot["result"] = "TARGET_FOUND"
        self._snapshot["finish_reason"] = "TARGET_FOUND"
        self._snapshot["target_match"]["target_confirmed"] = True
        self._snapshot["target_match"]["level"] = "confirmed"
        self._snapshot["objects"]["target_evidence"]["target_confirmed"] = True
        # TARGET_CONFIRMED is already an observable terminal boundary.  Keep a
        # minimal summary immediately so refresh/history views never depend on
        # the worker's following SESSION_FINISH IPC message winning a race.
        self._snapshot.setdefault("summary", {
            "result": "TARGET_FOUND",
            "finish_reason": "TARGET_FOUND",
            "planning_cycles": max(0, int(self._snapshot.get("cycle") or 0) - 1),
            "replans": int(self._snapshot.get("replans") or 0),
            "navigation_failures": int(
                self._snapshot.get("navigation_failures") or 0
            ),
        })

    def _on_candidates(self, event: SearchEvent) -> None:
        candidates = list(event.payload.get("candidates") or [])
        if candidates:
            self._snapshot["candidates"] = candidates
        if event.payload.get("selected_goal_id"):
            for candidate in candidates:
                if candidate.get("goal", {}).get("goal_id") == event.payload["selected_goal_id"]:
                    candidate["selected"] = True
        self._snapshot["phase"] = event.payload.get("phase") or self._snapshot["phase"]

    def _on_goal_selected(self, event: SearchEvent) -> None:
        payload = event.payload
        goal = dict(payload.get("goal") or {})
        selected = {
            "goal": goal,
            "score": payload.get("score"),
            "components": dict(payload.get("components") or {}),
            "reasons": list(payload.get("reasons") or []),
            "planning_cycles": payload.get("planning_cycles"),
        }
        self._last_goal = selected
        self._snapshot["selected_goal"] = selected
        if payload.get("planning_cycles") is not None:
            self._snapshot["cycle"] = max(
                self._snapshot["cycle"], int(payload["planning_cycles"])
            )
        self._snapshot["phase"] = "PLAN"

    def _on_decision_recorded(self, event: SearchEvent) -> None:
        record = dict(event.payload.get("decision") or event.payload)
        if not record:
            return
        decision_id = record.get("decision_id")
        # 把最近的真实决定与最新 trace 关联（计划书 §5.4 第 4 条）。
        # 只更新 agent 里下游那三项，不动 trace 本身 —— trace 是决策当时的快照，
        # 事后改写它就不再是"当时的依据"了。
        agent = self._snapshot.setdefault("agent", _empty_agent())
        trace = agent.get("trace") or {}
        agent["decision_id"] = decision_id
        agent["downstream_status"] = _downstream_status(trace, record)
        agent["downstream_reason"] = record.get("replan_reason") or None
        # §5.4 历史：把这一轮的依据随决定一起留在**已有的** decisions 列表里，
        # 不新开一份缓冲区，也不改冻结的 DecisionRecord 模型。这里的 record 是
        # payload 的浅副本，写进去影响不到生产者手里那个对象。
        if trace:
            preference = trace.get("selected_preference") or {}
            record["agent"] = {
                "trace_id": trace.get("trace_id"),
                "cycle": event.cycle,
                "active_subtask": (trace.get("active_subtask") or {}).get("title"),
                "directive_kind": preference.get("directive_kind"),
                "relative_heading_deg": preference.get("relative_heading_deg"),
                "reason_code": preference.get("reason_code"),
                "evidence_refs": list(preference.get("evidence_refs") or []),
                "downstream_status": agent["downstream_status"],
            }
        decisions = [
            item for item in self._snapshot["decisions"]
            if item.get("decision_id") != decision_id
        ]
        decisions.append(record)
        self._snapshot["decisions"] = decisions
        self._snapshot["last_decision"] = record
        self._snapshot["next_motion_command"] = record.get("next_motion_command")

    def _on_action_started(self, event: SearchEvent) -> None:
        self._snapshot["robot"]["motion_status"] = "EXECUTING"
        # A result belongs to the goal that just finished.  Keeping it while
        # the next goal is executing makes the WebUI combine the current
        # EXECUTING state with the previous goal's reject reason and corridor
        # decision.  Clear it at the action boundary; ACTION_FINISHED will
        # publish the result for this goal.
        self._snapshot["robot"]["last_motion_result"] = None
        self._snapshot["phase"] = "EXECUTE"
        self._snapshot["phase_detail"] = (
            event.payload.get("phase_detail")
            or "正在等待动作服务器执行并回传结果"
        )
        command = event.payload.get("next_motion_command")
        if command:
            self._snapshot["next_motion_command"] = command

    def _on_action_finished(self, event: SearchEvent) -> None:
        payload = event.payload
        status = str(payload.get("status") or "")
        planned_only = (payload.get('dry_run') is True and payload.get('path_checked') is True
            and (payload.get('candidate_decision') or {}).get('accepted') is True)
        self._snapshot["robot"]["motion_status"] = (
            "PLANNED" if planned_only else ("SUCCEEDED" if status == "succeeded" else "FAILED")
        )
        self._snapshot["robot"]["last_motion_result"] = {
            "status": status,
            "dry_run": payload.get('dry_run') is True,
            "path_checked": payload.get('path_checked') is True,
            "message": payload.get("message") or "",
            "elapsed_sec": payload.get("elapsed_sec"),
            "requested_motion": copy.deepcopy(payload.get("requested_motion") or {}),
            "observed_motion": copy.deepcopy(payload.get("observed_motion") or {}),
            # 计划书 §14：候选决策的结构化记录要能在一个地方读到——走廊判定、
            # 左右最小侧向净空、速度上限、拒因。现场验收就是照这条查的。
            "candidate_decision": copy.deepcopy(payload.get("candidate_decision")),
        }
        self._snapshot["phase"] = "WAIT_RESULT"
        self._snapshot["phase_detail"] = payload.get("phase_detail") or (
            payload.get("message") or "动作已结束"
        )

    def _on_replan(self, event: SearchEvent) -> None:
        self._snapshot["phase"] = "RECOVER"
        self._snapshot["replans"] = int(self._snapshot.get("replans") or 0) + 1
        value = event.payload.get("navigation_failures")
        if value is not None:
            self._snapshot["navigation_failures"] = max(
                int(self._snapshot.get("navigation_failures") or 0), int(value)
            )

    def _on_rgbd_frame(self, event: SearchEvent) -> None:
        payload = event.payload
        self._snapshot["spatial"]["rgbd_frame"] = {
            "frame_id": payload.get("frame_id"),
            "depth_ref": payload.get("depth_ref"),
            "intrinsics": payload.get("intrinsics"),
            "depth_scale": payload.get("depth_scale"),
            "spatial_quality": payload.get("spatial_quality"),
        }

    def _on_spatial_pose(self, event: SearchEvent) -> None:
        payload = event.payload
        self._snapshot["spatial"]["spatial_pose"] = payload.get("pose")
        self._snapshot["robot"]["pose_quality"] = payload.get("quality") or self._snapshot["robot"]["pose_quality"]

    def _on_psg_prior(self, event: SearchEvent) -> None:
        self._snapshot["spatial"]["psg_prior"] = event.payload.get("prior")

    def _on_semantic_region_created(self, event: SearchEvent) -> None:
        prior = dict(self._snapshot["spatial"]["psg_prior"] or {})
        regions = list(prior.get("region_hypotheses") or [])
        regions.append(event.payload.get("region") or {})
        prior["region_hypotheses"] = regions
        self._snapshot["spatial"]["psg_prior"] = prior

    def _on_long_term_goal_selected(self, event: SearchEvent) -> None:
        self._snapshot["spatial"]["long_term_goal"] = event.payload.get("intent")
        if event.payload.get("route_plan"):
            self._snapshot["spatial"]["route_plan"] = event.payload.get("route_plan")

    def _on_local_goal_progress(self, event: SearchEvent) -> None:
        self._snapshot["spatial"]["local_goal_progress"] = event.payload.get("progress")

    def _on_paused(self, event: SearchEvent) -> None:
        self._snapshot["status"] = STATUS_PAUSED
        self._snapshot["phase"] = "PAUSED"

    def _on_resumed(self, event: SearchEvent) -> None:
        self._snapshot["status"] = STATUS_RUNNING
        self._snapshot["phase"] = "OBSERVE"

    def _on_stop_requested(self, event: SearchEvent) -> None:
        if self._snapshot["status"] not in {
            STATUS_STARTING, STATUS_RUNNING, STATUS_PAUSED, STATUS_STOPPING,
        }:
            return  # A synchronous terminal acknowledgement already won.
        if event.session_id != self._snapshot["session_id"]:
            return
        if self._snapshot.get("stop_requested_at") is None:
            self._snapshot["stop_requested_at"] = event.timestamp
            self._snapshot["phase_started_at"] = event.timestamp
        self._snapshot["status"] = STATUS_STOPPING
        self._snapshot["phase"] = STATUS_STOPPING
        self._snapshot["phase_detail"] = "正在停止任务，等待执行端确认"

    def _on_search_exhausted(self, event: SearchEvent) -> None:
        self._snapshot["status"] = STATUS_SEARCH_EXHAUSTED
        self._snapshot["phase"] = "SEARCH_EXHAUSTED"
        self._snapshot["result"] = "SEARCH_EXHAUSTED"
        self._snapshot["finish_reason"] = "SEARCH_EXHAUSTED"

    def _on_operator_stop(self, event: SearchEvent) -> None:
        self._snapshot["status"] = STATUS_OPERATOR_STOP
        self._snapshot["phase"] = "OPERATOR_STOP"
        self._snapshot["result"] = "OPERATOR_STOP"
        self._snapshot["finish_reason"] = "OPERATOR_STOP"

    def _on_error(self, event: SearchEvent) -> None:
        payload = event.payload
        self._snapshot["status"] = STATUS_FAILED
        self._snapshot["result"] = "FAILED"
        if payload.get("schema_version") == "search_error_v1":
            self._snapshot["error"] = copy.deepcopy(payload)
        else:
            try:
                from app.manual_web_demo.search_errors import search_error

                detail = search_error(
                    payload.get("message") or payload.get("reason") or
                    payload.get("error_type") or "搜索异常",
                    code=payload.get("code") or payload.get("cause") or
                    payload.get("error_type"),
                    source=str(payload.get("source") or "autonomous_explorer"),
                    stage=str(payload.get("phase") or self._snapshot.get("phase") or "RUNNING"),
                    detail=payload.get("detail"),
                )
                self._snapshot["error"] = {**copy.deepcopy(payload), **detail}
            except Exception:  # noqa: BLE001 - error rendering must never fail
                self._snapshot["error"] = copy.deepcopy(payload)
        self._snapshot["error"].setdefault(
            "error_type", payload.get("code") or "SEARCH_ERROR"
        )
        self._snapshot["error"].setdefault("message", "")

    def _on_search_finished(self, event: SearchEvent) -> None:
        payload = event.payload
        result = str(payload.get("result") or "")
        self._snapshot["result"] = result
        self._snapshot["finish_reason"] = str(payload.get("finish_reason") or result)
        self._snapshot["finished_at"] = event.timestamp
        self._snapshot["summary"] = dict(payload)
        if result == "TARGET_FOUND":
            self._snapshot["status"] = STATUS_TARGET_FOUND
        elif result == "OPERATOR_STOP":
            self._snapshot["status"] = STATUS_OPERATOR_STOP
        elif result == "SEARCH_EXHAUSTED":
            self._snapshot["status"] = STATUS_SEARCH_EXHAUSTED
        elif result in BUDGET_COMPLETION_RESULTS:
            self._snapshot["status"] = STATUS_FINISHED
            self._snapshot["phase"] = STATUS_FINISHED
            self._snapshot["error"] = None
        elif result in FAILURE_COMPLETION_RESULTS:
            self._snapshot["status"] = STATUS_FAILED
        else:
            self._snapshot["status"] = STATUS_FINISHED
        for key in (
            "planning_cycles", "motion_steps", "observations", "unique_nodes",
            "replans", "navigation_failures", "verify_attempts", "duration_s",
        ):
            if key in payload:
                self._snapshot[key] = payload[key]

    def _append_timeline_locked(self, event: SearchEvent) -> None:
        self._timeline.append(
            {
                "event_type": event.event_type,
                "timestamp": event.timestamp,
                "cycle": event.cycle,
            }
        )
        self._snapshot["timeline"] = list(self._timeline)
        elapsed = 0.0
        started = self._snapshot.get("started_at")
        if started is not None:
            elapsed = max(0.0, event.timestamp - float(started))
        self._snapshot["elapsed_seconds"] = round(elapsed, 2)
