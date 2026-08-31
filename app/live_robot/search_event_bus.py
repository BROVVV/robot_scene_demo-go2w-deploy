"""SearchEventBus: in-process publish/subscribe hub for SearchEvent records
(plan book §18-§19).

The explorer / search worker never talk to FastAPI or WebSockets directly;
they publish here and adapters (WebSocket broadcaster, JSONL logger, session
recorder, tests, replay) subscribe independently.  The bus is thread-safe
because explorer callbacks and the asyncio drain task run on different
threads.
"""

from __future__ import annotations

import threading
from collections import deque
from typing import Any, Callable

from app.live_robot.search_event import SearchEvent, EventIdAllocator

Subscriber = Callable[[SearchEvent], None]


class SearchEventBus:
    """Thread-safe event hub with bounded recent-event history."""

    def __init__(self, *, max_recent: int = 500) -> None:
        self._subscribers: set[Subscriber] = set()
        self._lock = threading.Lock()
        self._recent: deque[SearchEvent] = deque(maxlen=max(1, int(max_recent)))
        self._allocator = EventIdAllocator()

    # ------------------------------------------------------------------ #
    # lifecycle                                                          #
    # ------------------------------------------------------------------ #
    def subscribe(self, callback: Subscriber) -> None:
        with self._lock:
            self._subscribers.add(callback)

    def unsubscribe(self, callback: Subscriber) -> None:
        with self._lock:
            self._subscribers.discard(callback)

    # ------------------------------------------------------------------ #
    # publishing                                                         #
    # ------------------------------------------------------------------ #
    def publish(self, event: SearchEvent) -> None:
        """Fan out one event to every subscriber (exceptions swallowed so a
        slow consumer can never kill the search loop)."""
        with self._lock:
            self._recent.append(event)
            targets = list(self._subscribers)
        for callback in targets:
            try:
                callback(event)
            except Exception:  # noqa: BLE001
                pass

    def publish_many(self, events: list[SearchEvent]) -> None:
        for event in events:
            self.publish(event)

    def clear(self) -> None:
        """Drop recent-event history between sessions.  Subscribers and the
        event-id allocator are kept so the hub keeps receiving events."""
        with self._lock:
            self._recent.clear()

    def recent_events(self, limit: int | None = None) -> list[dict[str, Any]]:
        """JSON-safe recent events, oldest first (plan book §55)."""
        with self._lock:
            items = list(self._recent)
        if limit is not None:
            items = items[-max(0, int(limit)):]
        return [event.to_dict() for event in items]

    def restore_recent(self, values: list[dict[str, Any]]) -> None:
        """Restore the tail of a persisted event stream at web startup."""
        restored: list[SearchEvent] = []
        for value in values:
            try:
                restored.append(SearchEvent.from_dict(value))
            except (TypeError, ValueError):
                continue
        with self._lock:
            self._recent.clear()
            self._recent.extend(restored[-self._recent.maxlen:])
        if restored:
            self._allocator.ensure_after(max(item.event_id for item in restored))

    @property
    def allocator(self) -> EventIdAllocator:
        return self._allocator
