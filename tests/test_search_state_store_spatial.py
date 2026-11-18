"""Tests for spatial event handling in SearchStateStore (计划书 §2.2 之后)。

store 里只剩**一份**图：`set_graph()` 写、`snapshot()["graph"]` 读。曾经分开的
`spatial.place_graph` / `spatial.semantic_objects` / `spatial.frontiers` /
`spatial.spatial_map` 四个投影与它们的 `spatial_snapshot()` 一起删掉了 ——
一份事实一个读口。

本文件仍保留的 `spatial` 只有**不是图**的那几项：RGB-D 帧、位姿、PSG 先验、
长期目标、局部目标进度。
"""

from __future__ import annotations

from app.live_robot.search_event import (
    FRONTIERS_UPDATED,
    LONG_TERM_GOAL_SELECTED,
    MAP_UPDATED,
    PLACE_CREATED,
    SearchEvent,
)
from app.live_robot.search_state_store import SearchStateStore
from app.spatial.semantic_navigation_graph import SemanticNavigationGraph


def _event(event_type: str, payload: dict, event_id: int = 1) -> SearchEvent:
    return SearchEvent(
        event_id=event_id,
        session_id="s1",
        timestamp=1.0,
        event_type=event_type,
        payload=payload,
    )


def _graph_dict(*, with_place: bool) -> dict:
    """走真实生产者，不手写 P1 —— 手写的 id 只是我的假设。"""
    graph = SemanticNavigationGraph(relation_confirm_min_observations=1)
    if with_place:
        graph.update_observation(
            observation_id="obs_0",
            heading_sector=0,
            scene_objects=[
                {"id": "frame_0_1", "label": "办公桌",
                 "map_xyz": [1.0, 0.0, 0.0], "confidence": 0.9},
            ],
            scene_relations=[],
            pose={"x": 0.0, "y": 0.0, "yaw_rad": 0.0},
            timestamp=1.0,
        )
    return graph.to_dict()


def test_spatial_snapshot_keeps_only_non_graph_fields():
    """`spatial` 里剩下的必须**不是图**；图字段一个都不许回潮。"""
    store = SearchStateStore()
    store.reset(session_id="s1", target="目标")
    store.apply(_event(LONG_TERM_GOAL_SELECTED, {"intent": {"intent_id": "i1"}}))
    spatial = store.snapshot()["spatial"]
    assert spatial["long_term_goal"]["intent_id"] == "i1"
    for stale in ("semantic_graph", "place_graph", "semantic_objects",
                  "frontiers", "spatial_map"):
        assert stale not in spatial, f"{stale} 是第二份图，已随 §2.2 删除"


def test_graph_events_do_not_project_a_second_graph():
    """`FRONTIERS_UPDATED` / `PLACE_CREATED` / `MAP_UPDATED` 都不再改 store 的图。

    它们只进时间线。图唯一的来源是 `set_graph()`（worker 的 `graph_snapshot`）。
    """
    store = SearchStateStore()
    store.reset(session_id="s1", target="目标")
    store.apply(_event(FRONTIERS_UPDATED, {"frontiers": [{"frontier_id": "F1"}]}))
    store.apply(_event(PLACE_CREATED, {"place": {"place_id": "P1"}}))
    store.apply(
        _event(MAP_UPDATED, {"graph": {"places": [{"place_id": "P1"}]}}, event_id=2)
    )
    assert store.snapshot()["graph"] is None
    assert store.snapshot()["graph_revision"] == 0


def test_graph_carries_current_place_and_object_topology():
    """一份图同时供三处使用：Place/当前地点、内部导航节点、对象拓扑投影。"""
    store = SearchStateStore()
    store.reset(session_id="s1", target="目标")
    assert store.set_graph(_graph_dict(with_place=True)) is True
    graph = store.snapshot()["graph"]

    assert graph["current_place_id"] == "P1"
    assert [place["place_id"] for place in graph["places"]] == ["P1"]

    topology = graph["object_topology"] or {}
    assert topology["schema_version"] == "semantic_object_topology_v1"
    # 对象拓扑里**带**当前 Place 节点：前端按视图开关（objectStrict / objectOnly）
    # 决定要不要把它画出来，后端不替前端做这个决定。
    assert {node["node_id"] for node in topology["nodes"]} == {"obj_001", "P1"}
    assert [node["node_id"] for node in topology["nodes"]
            if node["node_type"] == "OBJECT"] == ["obj_001"]


def test_object_topology_does_not_swallow_the_internal_nav_graph():
    """计划书 §7.3：内部导航图保留 Place/Frontier，语义拓扑只投影对象。

    `set_graph` 存的是**整份** `web_snapshot()`：对象拓扑是它的一个子视图，
    不是它的替代品。
    """
    store = SearchStateStore()
    store.reset(session_id="s1", target="目标")
    store.set_graph(_graph_dict(with_place=True))
    graph = store.snapshot()["graph"]

    internal_types = {
        node["node_type"] for node in graph["nodes"]
    }
    assert {"PLACE", "OBJECT"} <= internal_types, (
        f"内部导航图必须同时含 Place 与对象节点，实际 {internal_types}"
    )
    assert graph["edges"], "Place↔对象的关系边必须还在"
    assert isinstance(graph["frontiers"], list), (
        "frontier 是图的一部分（未观测的方位角会自动补 frontier），必须是列表键"
    )
