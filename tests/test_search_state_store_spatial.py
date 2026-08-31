"""Tests for spatial event handling in SearchStateStore."""

from __future__ import annotations

from app.live_robot.search_event import (
    FRONTIERS_UPDATED,
    LONG_TERM_GOAL_SELECTED,
    PLACE_CREATED,
    SearchEvent,
)
from app.live_robot.search_state_store import SearchStateStore


def _event(event_type: str, payload: dict, event_id: int = 1) -> SearchEvent:
    return SearchEvent(
        event_id=event_id,
        session_id="s1",
        timestamp=1.0,
        event_type=event_type,
        payload=payload,
    )


def test_spatial_snapshot_updates():
    store = SearchStateStore()
    store.reset(session_id="s1", target="目标")
    store.apply(_event(FRONTIERS_UPDATED, {"frontiers": [{"frontier_id": "F1"}]}))
    store.apply(_event(PLACE_CREATED, {"place": {"place_id": "P1"}}))
    store.apply(_event(LONG_TERM_GOAL_SELECTED, {"intent": {"intent_id": "i1"}}))
    spatial = store.spatial_snapshot()
    assert spatial["frontiers"][0]["frontier_id"] == "F1"
    assert spatial["place_graph"]["places"][0]["place_id"] == "P1"
    assert spatial["long_term_goal"]["intent_id"] == "i1"
