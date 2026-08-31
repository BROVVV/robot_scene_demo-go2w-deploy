"""SearchEvent + SearchEventBus tests (plan book §97: test_search_event)."""

from __future__ import annotations

import pytest

from app.live_robot.search_event import (
    ERROR,
    GOAL_SELECTED,
    SESSION_CREATED,
    SearchEvent,
    make_event,
)
from app.live_robot.search_event import EventIdAllocator
from app.live_robot.search_event_bus import SearchEventBus


def test_event_roundtrip() -> None:
    event = SearchEvent(
        event_id=7,
        session_id="search_20260817_174105",
        timestamp=1786969265.82,
        event_type=GOAL_SELECTED,
        cycle=17,
        payload={"goal": {"goal_id": "g1"}, "score": 0.82},
    )
    payload = event.to_dict()
    assert payload["schema_version"] == "search_event_v1"
    assert payload["event_id"] == 7
    assert payload["cycle"] == 17
    restored = SearchEvent.from_dict(payload)
    assert restored == event


def test_event_rejects_unknown_type() -> None:
    with pytest.raises(ValueError):
        SearchEvent(
            event_id=1,
            session_id="s",
            timestamp=1.0,
            event_type="NOT_A_REAL_EVENT",
        )


def test_make_event_allocates_monotonic_ids() -> None:
    allocator = EventIdAllocator()
    first = make_event(allocator=allocator, session_id="s", event_type=SESSION_CREATED)
    second = make_event(allocator=allocator, session_id="s", event_type=ERROR)
    assert second.event_id > first.event_id
    assert first.event_id >= 1


def test_bus_publish_subscribe_unsubscribe() -> None:
    bus = SearchEventBus(max_recent=10)
    received: list[SearchEvent] = []

    def on_event(event: SearchEvent) -> None:
        received.append(event)

    bus.subscribe(on_event)
    event = make_event(
        allocator=bus.allocator, session_id="s1", event_type=SESSION_CREATED,
        payload={"target": "蓝色垃圾桶"},
    )
    bus.publish(event)
    assert len(received) == 1
    assert received[0].payload["target"] == "蓝色垃圾桶"

    bus.unsubscribe(on_event)
    bus.publish(event)
    assert len(received) == 1


def test_bus_recent_events_bounded_and_ordered() -> None:
    bus = SearchEventBus(max_recent=3)
    for index in range(5):
        bus.publish(
            make_event(
                allocator=bus.allocator, session_id="s",
                event_type=SESSION_CREATED if index == 0 else ERROR,
                payload={"i": index},
            )
        )
    recent = bus.recent_events()
    assert len(recent) == 3
    assert [item["payload"]["i"] for item in recent] == [2, 3, 4]
    assert [item["event_id"] for item in recent] == sorted(
        item["event_id"] for item in recent
    )


def test_bus_subscriber_error_never_kills_publish() -> None:
    bus = SearchEventBus()

    def bad(event: SearchEvent) -> None:
        raise RuntimeError("boom")

    received: list[SearchEvent] = []

    def good(event: SearchEvent) -> None:
        received.append(event)

    bus.subscribe(bad)
    bus.subscribe(good)
    bus.publish(
        make_event(allocator=bus.allocator, session_id="s", event_type=ERROR)
    )
    assert len(received) == 1
