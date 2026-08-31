"""SearchStateStore: authoritative latest-snapshot of the current search
session (plan book §20-§21, §96).

The store is updated exclusively from SearchEvents so the WebUI (page load,
F5, WebSocket reconnect) can always recover the full state without replaying
the whole stream.  All reads take a lock and return deep copies; the store
never half-updates a snapshot.
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
# Keep the public state envelope stable for existing WebUI clients.  The
# graph carried inside it is the unified semantic graph and advertises its
# own ``semantic_navigation_graph_v1`` schema when produced by the explorer.
MAP_SCHEMA_VERSION = "live_exploration_graph_v1"

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
        "map": {
            "schema_version": MAP_SCHEMA_VERSION,
            "revision": 0,
            "map_mode": "topological",
            "graph_mode": "topological",
            "current_node_id": None,
            "robot": None,
            "nodes": [],
            "edges": [],
            "observed_sectors": [],
        },
        "spatial": {
            "rgbd_frame": None,
            "spatial_pose": None,
            "spatial_map": None,
            "frontiers": [],
            "place_graph": None,
            "semantic_objects": [],
            "semantic_graph": None,
            "route_plan": None,
            "map_health": {},
            "association_debug": [],
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
        "health": {},
        "last_warning": None,
        "timeline": [],
        "error": None,
    }


class SearchStateStore:
    """Thread-safe latest-snapshot store for one search session."""

    def __init__(self, *, max_timeline: int = TIMELINE_LIMIT,
                 on_change: Callable[[dict[str, Any], SearchEvent | None], None] | None = None) -> None:
        self._lock = threading.Lock()
        self._snapshot: dict[str, Any] = _empty_snapshot()
        self._map_revision = 0
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
            self._map_revision = 0
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
            MAP_UPDATED: self._on_map_updated,
            RGBD_FRAME_UPDATED: self._on_rgbd_frame,
            SPATIAL_POSE_UPDATED: self._on_spatial_pose,
            SPATIAL_MAP_UPDATED: self._on_spatial_map,
            FRONTIERS_UPDATED: self._on_frontiers,
            PLACE_CREATED: self._on_place_created,
            PLACE_UPDATED: self._on_place_updated,
            SEMANTIC_OBJECT_LOCALIZED: self._on_semantic_object_localized,
            PSG_PRIOR_UPDATED: self._on_psg_prior,
            SEMANTIC_REGION_CREATED: self._on_semantic_region_created,
            LONG_TERM_GOAL_SELECTED: self._on_long_term_goal_selected,
            LOCAL_GOAL_PROGRESS: self._on_local_goal_progress,
            DECISION_RECORDED: self._on_decision_recorded,
            PAUSED: self._on_paused,
            RESUMED: self._on_resumed,
            SEARCH_EXHAUSTED: self._on_search_exhausted,
            OPERATOR_STOP: self._on_operator_stop,
            ERROR: self._on_error,
            SEARCH_FINISHED: self._on_search_finished,
        }.get(event.event_type)
        if handler is None:
            return
        changed: dict[str, Any] | None = None
        with self._lock:
            handler(event)
            self._append_timeline_locked(event)
            changed = copy.deepcopy(self._snapshot)
        if self._on_change is not None and changed is not None:
            self._on_change(changed, event)

    def restore(self, snapshot: dict[str, Any]) -> None:
        """Restore an already materialized snapshot without replaying events."""
        with self._lock:
            base = _empty_snapshot()
            base.update(normalize_search_snapshot(snapshot))
            self._snapshot = base
            self._map_revision = int((base.get("map") or {}).get("revision") or 0)
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
    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._snapshot)

    def map_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._snapshot["map"])

    def spatial_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._snapshot["spatial"])

    def objects_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._snapshot["objects"])

    def decisions_snapshot(self) -> list[dict[str, Any]]:
        with self._lock:
            return copy.deepcopy(self._snapshot["decisions"])

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

    def _on_observation(self, event: SearchEvent) -> None:
        payload = event.payload
        observation = self._snapshot["observation"]
        observation["bundle_id"] = payload.get("bundle_id") or observation["bundle_id"]
        observation["timestamp"] = payload.get("timestamp") or event.timestamp
        observation["objects"] = list(payload.get("scene_objects") or payload.get("objects") or [])
        observation["detections"] = list(payload.get("detections") or [])
        observation["target_present"] = bool(payload.get("target_present", False))
        observation["heading_sector"] = payload.get("heading_sector")
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
                payload.get("pose_quality") or "relative"
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
        match["target_score"] = float(payload.get("target_score", match["target_score"]))
        match["explicit_anchor_found"] = bool(
            payload.get("explicit_anchor_found", match["explicit_anchor_found"])
        )
        anchors = list(payload.get("anchor_labels") or [])
        if anchors:
            match["anchor_labels"] = anchors
        if payload.get("directive") is not None:
            match["directive"] = payload["directive"]
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
        self._snapshot["robot"]["motion_status"] = (
            "SUCCEEDED" if status == "succeeded" else "FAILED"
        )
        self._snapshot["robot"]["last_motion_result"] = {
            "status": status,
            "message": payload.get("message") or "",
            "elapsed_sec": payload.get("elapsed_sec"),
            "requested_motion": copy.deepcopy(payload.get("requested_motion") or {}),
            "observed_motion": copy.deepcopy(payload.get("observed_motion") or {}),
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

    def _on_spatial_map(self, event: SearchEvent) -> None:
        self._snapshot["spatial"]["spatial_map"] = event.payload.get("map")

    def _on_frontiers(self, event: SearchEvent) -> None:
        self._snapshot["spatial"]["frontiers"] = list(event.payload.get("frontiers") or [])

    def _on_place_created(self, event: SearchEvent) -> None:
        place_graph = dict(self._snapshot["spatial"]["place_graph"] or {})
        places = list(place_graph.get("places") or [])
        places.append(event.payload.get("place") or {})
        place_graph["places"] = places
        self._snapshot["spatial"]["place_graph"] = place_graph

    def _on_place_updated(self, event: SearchEvent) -> None:
        place_graph = dict(self._snapshot["spatial"]["place_graph"] or {})
        places = list(place_graph.get("places") or [])
        place = event.payload.get("place") or {}
        place_id = place.get("place_id")
        if place_id is not None:
            places = [p if p.get("place_id") != place_id else place for p in places]
        place_graph["places"] = places
        self._snapshot["spatial"]["place_graph"] = place_graph

    def _on_semantic_object_localized(self, event: SearchEvent) -> None:
        obj = event.payload.get("object") or {}
        objects = list(self._snapshot["spatial"]["semantic_objects"] or [])
        object_id = obj.get("object_id")
        if object_id is not None:
            objects = [item for item in objects if item.get("object_id") != object_id]
        objects.append(obj)
        self._snapshot["spatial"]["semantic_objects"] = objects

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

    def _on_map_updated(self, event: SearchEvent) -> None:
        payload = event.payload
        graph = payload.get("graph") or {}
        self._map_revision += 1
        self._snapshot["map"] = {
            "schema_version": MAP_SCHEMA_VERSION,
            "revision": self._map_revision,
            "map_mode": payload.get("map_mode") or "topological",
            "graph_mode": payload.get("graph_mode") or payload.get("map_mode") or "topological",
            "current_node_id": payload.get("current_node_id"),
            "robot": payload.get("robot"),
            "nodes": list(graph.get("nodes") or []),
            "edges": list(graph.get("edges") or []),
            "observed_sectors": list(graph.get("observed_sectors") or []),
        }
        if graph.get("schema_version") in {
    "semantic_navigation_graph_v1",
    "semantic_entity_graph_v1",
}:
            self._snapshot["map"]["semantic_navigation_graph"] = copy.deepcopy(graph)
            # Keep the dedicated spatial endpoints as projections of the same
            # unified graph; they must never become a second source of truth.
            self._snapshot["spatial"]["spatial_map"] = copy.deepcopy(graph)
            self._snapshot["spatial"]["place_graph"] = {
                "places": copy.deepcopy(graph.get("places") or []),
                "edges": [
                    edge for edge in graph.get("edges") or []
                    if edge.get("from", "").startswith("P")
                    or edge.get("to", "").startswith("P")
                ],
            }
            self._snapshot["spatial"]["semantic_objects"] = copy.deepcopy(
                graph.get("objects") or []
            )
            self._snapshot["spatial"]["frontiers"] = copy.deepcopy(
                graph.get("frontiers") or []
            )
            self._snapshot["spatial"]["semantic_graph"] = copy.deepcopy(graph)
            self._snapshot["spatial"]["route_plan"] = copy.deepcopy(
                graph.get("route_plan") or self._snapshot["spatial"].get("route_plan")
            )
            self._snapshot["spatial"]["map_health"] = dict(
                self._snapshot["spatial"].get("map_health") or {}
            )
            self._snapshot["spatial"]["association_debug"] = copy.deepcopy(
                graph.get("association_debug")
                or self._snapshot["spatial"].get("association_debug")
                or []
            )

    def _on_paused(self, event: SearchEvent) -> None:
        self._snapshot["status"] = STATUS_PAUSED
        self._snapshot["phase"] = "PAUSED"

    def _on_resumed(self, event: SearchEvent) -> None:
        self._snapshot["status"] = STATUS_RUNNING
        self._snapshot["phase"] = "OBSERVE"

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
                    code=payload.get("error_type"),
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
        elif result in {
            "FAILED", "TIMEOUT", "BACKEND_FAILURE",
            "PERCEPTION_FAILURE",
        }:
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
