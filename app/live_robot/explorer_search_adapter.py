"""ExplorerSearchAdapter: bridges AutonomousExplorer's on_event dict stream
into SearchEvents (plan book §35).

The explorer emits flat dicts (``{"event": "...", "state": "...", "host_s",
...}``) from its ``_emit`` hook.  This adapter maps them onto the unified
``SearchEvent`` vocabulary, updates the ``SearchStateStore`` and publishes on
the ``SearchEventBus``.  The adapter itself never talks to FastAPI /
WebSockets, so CLI mode and tests can consume the same stream.
"""

from __future__ import annotations

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
    MEMORY_UPDATED,
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
    SEMANTIC_STATUS,
    SESSION_CREATED,
    SESSION_STARTED,
    SPATIAL_MAP_UPDATED,
    SPATIAL_POSE_UPDATED,
    TARGET_CONFIRMED,
    TARGET_MATCH_UPDATED,
    VERIFICATION_FINISHED,
    VERIFICATION_STARTED,
    SearchEvent,
    make_event,
)
from app.live_robot.search_event_bus import SearchEventBus
from app.live_robot.search_state_store import SearchStateStore

# Explorer events that are map-relevant: their payload may carry a "graph"
# field (injected by the search worker) which becomes MAP_UPDATED.
_MAP_RELEVANT = frozenset({"observation", "memory_update", "navigation_result"})

# Explorer stages that are recoverable by design.  They must remain warnings:
# the explorer either retries or falls back and only ``session_finish`` knows
# whether the whole task ultimately failed.
_RECOVERABLE_EVENTS = {
    "perception_failure": ("PERCEPTION_ERROR", "perception"),
    "perception_retry": ("PERCEPTION_ERROR", "perception_retry"),
    "observer_error": ("PERCEPTION_ERROR", "observer"),
    "observer_retry": ("PERCEPTION_ERROR", "observer_retry"),
    "matcher_error": ("SEARCH_ERROR", "matcher"),
    "verification_error": ("LLM_ERROR", "verifier"),
    # 计划书 §6.2：候选生成异常是 PLANNING_ERROR；第一次可恢复（重新规划），
    # 终局判定由 session_finish 决定，绝不在这里映射成搜索穷尽。
    "candidate_generator_error": ("PLANNING_ERROR", "candidate_generator"),
    "candidates_empty": ("PLANNING_ERROR", "candidates_empty"),
    "planner_returned_no_goal": ("PLANNING_ERROR", "planner"),
}


class ExplorerSearchAdapter:
    """Maps explorer events to SearchEvents and keeps the store in sync."""

    def __init__(
        self,
        bus: SearchEventBus,
        store: SearchStateStore,
        *,
        session_id: str,
        source: str = "autonomous_search",
        now: Callable[[], float] | None = None,
    ) -> None:
        self._bus = bus
        self._store = store
        self._session_id = session_id
        self._source = source
        self._now = now
        self._cycle = 0
        self._pending_verification: dict[str, Any] | None = None

    # ------------------------------------------------------------------ #
    # explorer hook                                                      #
    # ------------------------------------------------------------------ #
    def on_explorer_event(self, event: dict[str, Any]) -> None:
        """Handle one raw explorer event and publish SearchEvents."""
        events = self._convert(event)
        for search_event in events:
            self._bus.publish(search_event)
            self._store.apply(search_event)

    def _emit(self, event_type: str, *, payload: dict[str, Any] | None = None,
              cycle: int | None = None) -> SearchEvent:
        return make_event(
            allocator=self._bus.allocator,
            session_id=self._session_id,
            event_type=event_type,
            cycle=cycle if cycle is not None else (self._cycle or None),
            payload=payload or {},
            now=self._now,
        )

    # ------------------------------------------------------------------ #
    # mapping                                                            #
    # ------------------------------------------------------------------ #
    def _convert(self, event: dict[str, Any]) -> list[SearchEvent]:
        name = str(event.get("event") or "")
        state = str(event.get("state") or "")
        payload = dict(event)
        payload.pop("event", None)
        payload.pop("state", None)
        payload.pop("host_s", None)
        # Preserve the worker envelope inside every SearchEvent payload so
        # downstream logs remain attributable even though SearchEvent keeps a
        # stable session-only top-level schema.
        for key in ("task_id", "executor_id", "worker_generation"):
            if event.get(key) is not None:
                payload[key] = event[key]
        payload["phase"] = state or payload.get("phase")

        if name == "session_start":
            # SESSION_CREATED is emitted by the web service; the explorer's
            # session_start only marks the transition to running.
            return [
                self._emit(SESSION_STARTED, payload={"phase": "BOOTSTRAP"}),
            ]
        if name == "backend_health":
            health = {
                "backend": payload.get("backend"),
                "ready": payload.get("ready"),
                "degraded": payload.get("degraded"),
                "attempt": payload.get("attempt"),
                "health": payload.get("health"),
            }
            return [
                self._emit(SEARCH_STATE_CHANGED, payload={
                    "phase": "BOOTSTRAP", "health": health,
                })
            ]
        if name == "phase_progress":
            return [
                self._emit(SEARCH_STATE_CHANGED, payload={
                    "phase": payload.get("phase") or state or "RUNNING",
                    "phase_detail": payload.get("detail_zh") or "",
                    "operation": payload.get("operation") or "",
                    "phase_started_at": payload.get("phase_started_at")
                    or event.get("host_s"),
                })
            ]
        if name == "observation":
            self._cycle += 1
            return [
                self._emit(OBSERVATION_UPDATED, cycle=self._cycle, payload={
                    "bundle_id": payload.get("bundle_id"),
                    "timestamp": payload.get("host_s") or payload.get("timestamp"),
                    "objects": payload.get("objects") or [],
                    "scene_objects": payload.get("scene_objects") or [],
                    "scene_relations": payload.get("scene_relations") or [],
                    "target_present": payload.get("target_present", False),
                    "target_state": payload.get("target_state") or "ABSENT",
                    "heading_sector": payload.get("heading_sector"),
                    "navigation_heading_sector": payload.get("navigation_heading_sector"),
                    "semantic_status": payload.get("semantic_status"),
                    "semantic_quality": payload.get("semantic_quality"),
                    "semantic_source_frame_id": payload.get("semantic_source_frame_id"),
                    "semantic_age_ms": payload.get("semantic_age_ms"),
                    "pose": payload.get("pose"),
                    "image_ref": payload.get("image_ref"),
                    "depth_ref": payload.get("depth_ref"),
                    "rgbd_frame_id": payload.get("rgbd_frame_id"),
                    "intrinsics": payload.get("intrinsics"),
                    "depth_scale": payload.get("depth_scale"),
                    "spatial_quality": payload.get("spatial_quality"),
                    "camera_xyz": payload.get("camera_xyz"),
                    "sensor_health": payload.get("sensor_health") or {},
                    "detections": payload.get("detections") or [],
                    "phase": "OBSERVE",
                }),
                self._emit(OBJECTS_UPDATED, cycle=self._cycle, payload={
                    "current": payload.get("scene_objects") or [],
                    "phase": "OBSERVE",
                }),
                *self._map_events(event, payload, state),
            ]
        if name == "semantic_status":
            # 计划书 §13：语义健康状态单独透传（Quick VLM / Full Semantic /
            # frame / age / objects / spatial quality）。
            return [
                self._emit(SEMANTIC_STATUS, payload={
                    key: payload.get(key)
                    for key in (
                        "frame_id", "semantic_status", "semantic_quality",
                        "semantic_source_frame_id", "semantic_age_ms",
                        "semantic_object_count", "semantic_error_code",
                    )
                })
            ]
        if name == "match":
            return [
                self._emit(TARGET_MATCH_UPDATED, payload={
                    "has_candidate": payload.get("has_candidate", False),
                    "target_state": payload.get("target_state") or "ABSENT",
                    "target_match_level": payload.get("target_match_level") or "none",
                    "target_score": payload.get("target_score", 0.0),
                    "anchor_labels": payload.get("anchor_labels") or [],
                    "explicit_anchor_found": bool(payload.get("anchor_labels")),
                    "directive": payload.get("directive"),
                    "graph_match": payload.get("graph_match"),
                    "phase": "MATCH",
                })
            ]
        if name == "verification":
            attempt = int(payload.get("attempt") or 1)
            if attempt == 1:
                self._pending_verification = payload
                return [
                    self._emit(VERIFICATION_STARTED, payload={
                        "attempt": attempt, "phase": "VERIFY",
                    })
                ]
            return [
                self._emit(VERIFICATION_FINISHED, payload={
                    "attempt": attempt,
                    "confirmed": bool(payload.get("confirmed", False)),
                    "reason_zh": payload.get("reason_zh") or "",
                    "phase": "VERIFY",
                })
            ]
        if name == "verification_rejected":
            return [
                self._emit(VERIFICATION_FINISHED, payload={
                    "confirmed": False,
                    "reason_zh": payload.get("reason_zh") or "",
                    "phase": "VERIFY",
                })
            ]
        if name == "target_found":
            return [
                self._emit(TARGET_CONFIRMED, payload={
                    "reason_zh": payload.get("reason_zh") or "",
                    "attempts": payload.get("attempts"),
                    "phase": "TARGET_FOUND",
                })
            ]
        if name == "memory_update":
            return [
                self._emit(MEMORY_UPDATED, payload={
                    "node_id": payload.get("node_id"),
                    "new_labels": payload.get("new_labels") or [],
                    "new_relations": payload.get("new_relations") or [],
                    "new_sector": payload.get("new_sector", False),
                    "information_gain": payload.get("information_gain", 0.0),
                    "no_information_cycles": payload.get("no_information_cycles", 0),
                    "unique_nodes": payload.get("unique_nodes", 0),
                    "phase": "UPDATE_MEMORY",
                }),
                *self._map_events(event, payload, state),
            ]
        if name == "candidates":
            candidates = list(payload.get("candidates") or [])
            return [
                self._emit(CANDIDATES_GENERATED, payload={
                    "candidates": candidates,
                    "selected_goal_id": payload.get("selected_goal_id"),
                    "phase": "PLAN",
                })
            ]
        if name == "selected_goal":
            return [
                self._emit(GOAL_SELECTED, payload={
                    "goal": payload.get("goal") or {},
                    "score": payload.get("score"),
                    "components": payload.get("components") or {},
                    "reasons": payload.get("reasons") or [],
                    "planning_cycles": payload.get("planning_cycles"),
                    "phase": "PLAN",
                })
            ]
        if name == "decision_recorded":
            return [
                self._emit(DECISION_RECORDED, payload={
                    "decision": payload.get("decision") or {},
                    "phase": "PLAN",
                })
            ]
        if name == "action_start":
            return [
                self._emit(ACTION_STARTED, payload={
                    "goal": payload.get("goal") or {},
                    "decision_id": payload.get("decision_id"),
                    "next_motion_command": payload.get("next_motion_command"),
                    "phase_detail": "正在等待动作服务器执行并回传结果",
                    "phase": "EXECUTE",
                })
            ]
        if name == "navigation_result":
            return [
                self._emit(ACTION_FINISHED, payload={
                    "goal_id": payload.get("goal_id"),
                    "status": payload.get("status"),
                    "message": payload.get("message") or "",
                    "requested_motion": payload.get("requested_motion") or {},
                    "observed_motion": payload.get("observed_motion") or {},
                    "elapsed_sec": payload.get("elapsed_sec"),
                    "phase_detail": payload.get("message") or "动作已结束",
                    "phase": "WAIT_RESULT",
                }),
                *self._map_events(event, payload, state),
            ]
        if name == "replan":
            return [
                self._emit(REPLAN, payload={
                    "goal_id": payload.get("goal_id"),
                    "status": payload.get("status"),
                    "navigation_failures": payload.get("navigation_failures", 0),
                    "phase": "RECOVER",
                })
            ]
        if name == "rgbd_frame_updated":
            return [self._emit(RGBD_FRAME_UPDATED, payload=payload)]
        if name == "spatial_pose_updated":
            return [self._emit(SPATIAL_POSE_UPDATED, payload=payload)]
        if name == "spatial_map_updated":
            return [self._emit(SPATIAL_MAP_UPDATED, payload=payload)]
        if name == "frontiers_updated":
            return [self._emit(FRONTIERS_UPDATED, payload=payload)]
        if name == "place_created":
            return [self._emit(PLACE_CREATED, payload=payload)]
        if name == "place_updated":
            return [self._emit(PLACE_UPDATED, payload=payload)]
        if name == "semantic_object_localized":
            return [self._emit(SEMANTIC_OBJECT_LOCALIZED, payload=payload)]
        if name == "psg_prior_updated":
            return [self._emit(PSG_PRIOR_UPDATED, payload=payload)]
        if name == "semantic_region_created":
            return [self._emit(SEMANTIC_REGION_CREATED, payload=payload)]
        if name == "long_term_goal_selected":
            return [self._emit(LONG_TERM_GOAL_SELECTED, payload=payload)]
        if name == "local_goal_progress":
            return [self._emit(LOCAL_GOAL_PROGRESS, payload=payload)]
        if name == "paused":
            return [self._emit(PAUSED, payload={"phase": "PAUSED"})]
        if name == "resumed":
            return [self._emit(RESUMED, payload={"phase": "OBSERVE"})]
        if name == "waiting_for_map":
            # 计划书 §6.3：等地图是显式的等待状态，机器狗已停稳。
            return [
                self._emit(SEARCH_STATE_CHANGED, payload={
                    "phase": "WAITING_FOR_MAP",
                    "phase_detail": str(
                        payload.get("detail_zh")
                        or "地图暂时不新鲜，机器狗停稳等待新地图后重新规划"
                    ),
                    "waiting_cycles": payload.get("waiting_cycles"),
                    "reason": payload.get("reason") or "",
                })
            ]
        if name == "search_exhausted":
            return [
                self._emit(SEARCH_EXHAUSTED, payload={
                    "reason": payload.get("reason") or "",
                    # 计划书 §6.5：穷尽必须列出已扫描方向数与 frontier 统计。
                    "scanned_sector_count": payload.get("scanned_sector_count"),
                    "reachable_frontier_count": payload.get("reachable_frontier_count"),
                    "visited_frontier_count": payload.get("visited_frontier_count"),
                    "unreachable_frontier_count": payload.get("unreachable_frontier_count"),
                    "map_fresh": payload.get("map_fresh"),
                    "phase": "SEARCH_EXHAUSTED",
                })
            ]
        if name == "session_finish":
            result = str(payload.get("result") or "")
            finish_payload = {
                "result": result,
                "finish_reason": result,
                "reason": payload.get("reason") or "",
                "planning_cycles": payload.get("planning_cycles"),
                "motion_steps": payload.get("motion_steps"),
                "observations": payload.get("observations"),
                "unique_nodes": payload.get("unique_nodes"),
                "replans": payload.get("replans"),
                "navigation_failures": payload.get("navigation_failures"),
                "verify_attempts": payload.get("verify_attempts"),
                "duration_s": payload.get("duration_s"),
                "phase": result,
            }
            events: list[SearchEvent] = []
            if result == "OPERATOR_STOP":
                events.append(self._emit(OPERATOR_STOP, payload={"phase": "OPERATOR_STOP"}))
            elif result in {"TIMEOUT", "BACKEND_FAILURE",
                            "PERCEPTION_FAILURE", "PLANNING_ERROR",
                            "MAP_UNAVAILABLE"}:
                # 计划书 §8.5：最终失败信息必须精准（cause/attempts 等）。
                perception_detail = dict(payload.get("error_detail") or {})
                events.append(self._emit(ERROR, payload={
                    "error_type": _finish_to_error_type(result),
                    "code": perception_detail.get("code") or payload.get("cause"),
                    "message": (
                        perception_detail.get("message")
                        or payload.get("error")
                        or payload.get("reason")
                        or result
                    ),
                    "cause": payload.get("cause") or perception_detail.get("code"),
                    "attempts": payload.get("attempts"),
                    "last_success_age_s": payload.get("last_success_age_s"),
                    "recoverable": payload.get("recoverable"),
                    "detail": perception_detail.get("detail"),
                    "error_detail": perception_detail,
                    "phase": "FAILED",
                }))
            events.append(self._emit(SEARCH_FINISHED, payload=finish_payload))
            return events
        if name in _RECOVERABLE_EVENTS:
            error_type, source = _RECOVERABLE_EVENTS[name]
            message = str(
                payload.get("error") or payload.get("reason") or name
            )
            return [
                self._emit(SEARCH_STATE_CHANGED, payload={
                    "phase": state or "OBSERVE",
                    "phase_detail": f"可恢复异常，正在重试或降级继续：{message}",
                    "warning": {
                        "warning_type": error_type,
                        "source": source,
                        "message": message,
                        "recoverable": True,
                    },
                })
            ]
        # Unknown explorer events still surface as state changes.
        return [self._emit(SEARCH_STATE_CHANGED, payload=payload)]

    # ------------------------------------------------------------------ #
    # helpers                                                            #
    # ------------------------------------------------------------------ #
    def _map_events(self, raw: dict[str, Any], payload: dict[str, Any],
                    state: str) -> list[SearchEvent]:
        """MAP_UPDATED for map-relevant explorer events (payload carries the
        full graph snapshot injected by the search worker)."""
        graph = payload.get("semantic_navigation_graph")
        semantic = isinstance(graph, dict)
        if not semantic:
            graph = payload.get("graph")
        if not isinstance(graph, dict) or (not semantic and not graph.get("session_id")):
            return []
        nodes = list(graph.get("nodes") or [])
        if not nodes:
            # Pre-memory observation (e.g. target found on the first frame):
            # reflect the current observation node so the map is never empty.
            bundle_id = payload.get("bundle_id")
            pose = payload.get("pose")
            nodes = [
                {
                    "node_id": f"node_{bundle_id}" if bundle_id else "node",
                    "timestamp": payload.get("host_s") or 0.0,
                    "objects": list(
                        payload.get("objects") or payload.get("scene_objects") or []
                    ),
                    "pose": pose,
                    "pose_quality": "relative" if isinstance(pose, dict) else "unavailable",
                    "reachable_state": "OBSERVED",
                    "visited_count": 0,
                    "target_match_level": "none",
                    "semantic_relevance": 0.0,
                    "information_gain": 0.0,
                }
            ]
            graph = {**graph, "nodes": nodes}
        current_node_id = graph.get("current_place_id") if semantic else None
        bundle_id = payload.get("bundle_id")
        if bundle_id and not semantic:
            current_node_id = f"node_{bundle_id}"
        robot = None
        pose = payload.get("pose")
        if isinstance(pose, dict):
            robot = {
                "x": pose.get("x"),
                "y": pose.get("y"),
                "yaw": pose.get("yaw_rad", pose.get("yaw")),
                "pose_quality": "relative",
            }
        # Flatten x/y/yaw to node top level (plan book §47) for consumers that
        # expect the documented node schema; pose stays nested for depth.
        normalized_nodes: list[dict[str, Any]] = []
        for node in nodes:
            item = dict(node)
            node_type = str(item.get("node_type") or "PLACE").upper()
            node_pose = item.get("pose")
            if isinstance(node_pose, dict):
                item["x"] = node_pose.get("x")
                item["y"] = node_pose.get("y")
                item["yaw"] = node_pose.get("yaw")
            # Keep the documented legacy map envelope consumable while the
            # node IDs and graph source are now semantic PLACE/OBJECT/
            # FRONTIER nodes. ``graph_mode`` below identifies the projection.
            item.setdefault("timestamp", item.get("last_seen") or item.get("first_seen") or 0.0)
            item.setdefault("visited_count", item.get("visit_count", 0))
            item.setdefault("objects", [item.get("label")] if node_type == "OBJECT" and item.get("label") else [])
            item.setdefault("reachable_state", "VISITED" if item.get("current") else "OBSERVED")
            item.setdefault(
                "target_match_level",
                "confirmed" if item.get("target_confirmed") else (
                    "candidate" if item.get("target_candidate") else "none"
                ),
            )
            item.setdefault("semantic_relevance", item.get("association_score", 0.0))
            item.setdefault("information_gain", item.get("confidence", 0.0))
            normalized_nodes.append(item)
        normalized_graph = {**graph, "nodes": normalized_nodes}
        return [
            self._emit(MAP_UPDATED, payload={
                "graph": normalized_graph,
                # ``map_mode`` is the stable outer WebUI contract. New
                # clients should use graph_mode/semantic_navigation_graph.
                "map_mode": "topological",
                "graph_mode": "semantic_navigation" if semantic else "topological",
                "semantic_navigation_graph": normalized_graph if semantic else None,
                "current_node_id": current_node_id,
                "robot": robot,
                "phase": state,
            })
        ]


def _finish_to_error_type(result: str) -> str:
    if result in {"BACKEND_FAILURE", "BACKEND_UNAVAILABLE"}:
        return "BACKEND_ERROR"
    if result == "PERCEPTION_FAILURE":
        return "PERCEPTION_ERROR"
    if result in {"PLANNING_ERROR", "MAP_UNAVAILABLE"}:
        return "PLANNING_ERROR"
    return "SEARCH_ERROR"
