"""SearchEvent: the unified, JSON-safe event protocol of the autonomous
semantic search WebUI (plan book §14-§17).

Every piece of live search telemetry that crosses a process or WebSocket
boundary is a ``SearchEvent``.  Payloads must stay JSON-safe (str / number /
bool / list / dict / None); images always travel through the MJPEG endpoint,
never inside an event.

Event ids are monotonically increasing per bus instance so consumers can
detect gaps and ignore stale events (plan book §120).
"""

from __future__ import annotations

import time
from dataclasses import asdict, dataclass, field
from typing import Any, Callable

SCHEMA_VERSION = "search_event_v1"

# ---- event types (plan book §16 / §102) ----------------------------------- #
SESSION_CREATED = "SESSION_CREATED"
TASK_UNDERSTANDING = "TASK_UNDERSTANDING"
TASK_REJECTED = "TASK_REJECTED"
SESSION_STARTED = "SESSION_STARTED"
SEARCH_STATE_CHANGED = "SEARCH_STATE_CHANGED"
OBSERVATION_STARTED = "OBSERVATION_STARTED"
OBSERVATION_UPDATED = "OBSERVATION_UPDATED"
OBJECTS_UPDATED = "OBJECTS_UPDATED"
SCENE_GRAPH_UPDATED = "SCENE_GRAPH_UPDATED"
TARGET_PROFILE_READY = "TARGET_PROFILE_READY"
GOAL_GRAPH_READY = "GOAL_GRAPH_READY"
TARGET_MATCH_UPDATED = "TARGET_MATCH_UPDATED"
TARGET_CANDIDATE = "TARGET_CANDIDATE"
VERIFICATION_STARTED = "VERIFICATION_STARTED"
VERIFICATION_FINISHED = "VERIFICATION_FINISHED"
TARGET_CONFIRMED = "TARGET_CONFIRMED"
MEMORY_UPDATED = "MEMORY_UPDATED"
MAP_UPDATED = "MAP_UPDATED"
CANDIDATES_GENERATED = "CANDIDATES_GENERATED"
GOAL_SELECTED = "GOAL_SELECTED"
ACTION_STARTED = "ACTION_STARTED"
ACTION_PROGRESS = "ACTION_PROGRESS"
ACTION_FINISHED = "ACTION_FINISHED"
REPLAN = "REPLAN"
PAUSED = "PAUSED"
RESUMED = "RESUMED"
OPERATOR_STOP = "OPERATOR_STOP"
ERROR = "ERROR"
SEARCH_EXHAUSTED = "SEARCH_EXHAUSTED"
SEARCH_FINISHED = "SEARCH_FINISHED"
# RGB-D spatial exploration events (§102)
RGBD_FRAME_UPDATED = "RGBD_FRAME_UPDATED"
SPATIAL_POSE_UPDATED = "SPATIAL_POSE_UPDATED"
SPATIAL_MAP_UPDATED = "SPATIAL_MAP_UPDATED"
FRONTIERS_UPDATED = "FRONTIERS_UPDATED"
PLACE_CREATED = "PLACE_CREATED"
PLACE_UPDATED = "PLACE_UPDATED"
SEMANTIC_OBJECT_LOCALIZED = "SEMANTIC_OBJECT_LOCALIZED"
PSG_PRIOR_UPDATED = "PSG_PRIOR_UPDATED"
SEMANTIC_REGION_CREATED = "SEMANTIC_REGION_CREATED"
LONG_TERM_GOAL_SELECTED = "LONG_TERM_GOAL_SELECTED"
LOCAL_GOAL_PROGRESS = "LOCAL_GOAL_PROGRESS"
DECISION_RECORDED = "DECISION_RECORDED"

ALL_EVENT_TYPES = frozenset(
    {
        SESSION_CREATED,
        TASK_UNDERSTANDING,
        TASK_REJECTED,
        SESSION_STARTED,
        SEARCH_STATE_CHANGED,
        OBSERVATION_STARTED,
        OBSERVATION_UPDATED,
        OBJECTS_UPDATED,
        SCENE_GRAPH_UPDATED,
        TARGET_PROFILE_READY,
        GOAL_GRAPH_READY,
        TARGET_MATCH_UPDATED,
        TARGET_CANDIDATE,
        VERIFICATION_STARTED,
        VERIFICATION_FINISHED,
        TARGET_CONFIRMED,
        MEMORY_UPDATED,
        MAP_UPDATED,
        CANDIDATES_GENERATED,
        GOAL_SELECTED,
        ACTION_STARTED,
        ACTION_PROGRESS,
        ACTION_FINISHED,
        REPLAN,
        PAUSED,
        RESUMED,
        OPERATOR_STOP,
        ERROR,
        SEARCH_EXHAUSTED,
        SEARCH_FINISHED,
        RGBD_FRAME_UPDATED,
        SPATIAL_POSE_UPDATED,
        SPATIAL_MAP_UPDATED,
        FRONTIERS_UPDATED,
        PLACE_CREATED,
        PLACE_UPDATED,
        SEMANTIC_OBJECT_LOCALIZED,
        PSG_PRIOR_UPDATED,
        SEMANTIC_REGION_CREATED,
        LONG_TERM_GOAL_SELECTED,
        LOCAL_GOAL_PROGRESS,
        DECISION_RECORDED,
    }
)


@dataclass(frozen=True)
class SearchEvent:
    """One immutable, JSON-safe search telemetry record."""

    event_id: int
    session_id: str
    timestamp: float
    event_type: str
    cycle: int | None = None
    payload: dict[str, Any] = field(default_factory=dict)
    schema_version: str = SCHEMA_VERSION
    source: str = "autonomous_search"

    def __post_init__(self) -> None:
        if self.event_type not in ALL_EVENT_TYPES:
            raise ValueError(f"unknown search event type: {self.event_type}")

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "SearchEvent":
        return cls(
            event_id=int(value.get("event_id", 0)),
            session_id=str(value.get("session_id") or ""),
            timestamp=float(value.get("timestamp", 0.0)),
            event_type=str(value.get("event_type") or ""),
            cycle=value.get("cycle"),
            payload=dict(value.get("payload") or {}),
            schema_version=str(value.get("schema_version") or SCHEMA_VERSION),
            source=str(value.get("source") or "autonomous_search"),
        )


class EventIdAllocator:
    """Monotonic event-id source shared by a bus (plan book §120)."""

    def __init__(self) -> None:
        self._next = 1

    def next_id(self) -> int:
        current = self._next
        self._next += 1
        return current

    def ensure_after(self, event_id: int) -> None:
        """Keep ids monotonic after restoring persisted events."""
        self._next = max(self._next, int(event_id) + 1)


def make_event(
    *,
    allocator: EventIdAllocator,
    session_id: str,
    event_type: str,
    cycle: int | None = None,
    payload: dict[str, Any] | None = None,
    now: Callable[[], float] | None = None,
) -> SearchEvent:
    """Convenience factory used by adapters and executors."""
    timestamp = (now() if now is not None else time.time())
    return SearchEvent(
        event_id=allocator.next_id(),
        session_id=session_id,
        timestamp=timestamp,
        event_type=event_type,
        cycle=cycle,
        payload=payload or {},
    )
