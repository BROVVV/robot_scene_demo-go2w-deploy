"""计划书 §2.1 / §5：一张图每次 revision 只往 Web 发一次。

判据在 worker 的 ``event_hook`` 里：它**每个** explorer 事件都跑一遍，靠
``_STATE["last_graph_revision"]`` 去重。这个去重一旦失效，症状不是报错而是带宽：
长会话里每个 observation 后面都跟一份 122 对象 / 4807 条边的整图。

所以这里数的是**真的发了几条** ``graph_snapshot``，不是"发出来的那条内容对不对"
—— 后者在重复发送时照样通过。两个方向都要测：不变量为 0 条，变一次恰好 1 条。
只测"不重复"的话，把去重写成"永远不发"也能过，而那会让页面永远停在第一帧。
"""

from __future__ import annotations

from typing import Any

import pytest

from scripts.go2w import autonomous_search_worker as worker


class _FakeGraph:
    def __init__(self, revision: int) -> None:
        self.revision = revision

    def web_snapshot(self) -> dict[str, Any]:
        return {
            "schema_version": "semantic_navigation_graph_v1",
            "revision": self.revision,
            "current_place_id": "P1",
            "places": [{"place_id": "P1"}],
            "objects": [{"object_id": "o1"}],
            "frontiers": [{"frontier_id": "f1"}],
            "edges": [{"from": "P1", "to": "o1"}],
            "object_topology": {"nodes": [], "edges": [], "stats": {}},
        }


class _FakeExplorer:
    def __init__(self, revision: int = 0) -> None:
        self.semantic_graph = _FakeGraph(revision)
        self.session_id = "search_20261008_000000_deadbeef"


@pytest.fixture()
def emitted(monkeypatch: pytest.MonkeyPatch) -> list[dict[str, Any]]:
    """拦住 stdout，收集 worker 发出的每一条 IPC 消息，并把会话状态清零。"""
    messages: list[dict[str, Any]] = []
    monkeypatch.setattr(worker, "emit", messages.append)
    # 首帧必发（None → revision 0）；测试之间必须清零，否则第二个测试依赖执行顺序。
    monkeypatch.setitem(worker._STATE, "last_graph_revision", None)
    monkeypatch.setitem(worker._STATE, "session_id", None)
    return messages


def _snapshots(messages: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [item for item in messages if item.get("type") == "graph_snapshot"]


def test_a_hundred_events_without_a_revision_bump_send_no_second_graph(
    emitted: list[dict[str, Any]],
) -> None:
    explorer = _FakeExplorer(revision=0)
    hook = worker.make_event_hook()
    holder = {"explorer": explorer}

    # 第一次事件必然带上图（Web 需要一份初始图，不能指望它自己去要）。
    hook({"event": "observation", "detail_zh": "看到物体"}, holder)
    assert len(_snapshots(emitted)) == 1
    assert len(emitted) == 2, "每条普通事件仍要原样转发"

    for index in range(100):
        hook({"event": "memory_update", "detail_zh": f"第 {index} 次记忆更新"}, holder)

    assert len(_snapshots(emitted)) == 1, "revision 没变却又发了整图"
    assert len([m for m in emitted if m.get("type") == "event"]) == 101


def test_a_revision_bump_sends_exactly_one_more_graph(
    emitted: list[dict[str, Any]],
) -> None:
    explorer = _FakeExplorer(revision=0)
    hook = worker.make_event_hook()
    holder = {"explorer": explorer}
    hook({"event": "observation"}, holder)

    explorer.semantic_graph = _FakeGraph(1)
    hook({"event": "navigation_result"}, holder)
    hook({"event": "navigation_result"}, holder)
    explorer.semantic_graph = _FakeGraph(2)
    hook({"event": "navigation_result"}, holder)

    snapshots = _snapshots(emitted)
    assert [item["revision"] for item in snapshots] == [0, 1, 2]
    # 图和它带来的那条事件同属一次 revision：图必须先到，Web 才不会拿新事件
    # 去配旧图。这里断言的就是这个顺序。
    types = [item.get("type") for item in emitted]
    assert types.index("graph_snapshot") < types.index("event")


def test_ordinary_events_carry_no_graph_arrays(emitted: list[dict[str, Any]]) -> None:
    """§2.1：普通事件只有小字段。整图只走 ``graph_snapshot`` 一条路。

    去掉 ``graph`` 这个键本身是容易的，难的是整张图的数组别从别处漏回来 ——
    所以按名字查 ``places``/``objects``/``frontiers``/``edges``/``nodes``，
    它们在这条消息里出现就说明有人把整图塞进了普通事件。
    """
    explorer = _FakeExplorer(revision=0)
    hook = worker.make_event_hook()
    hook({"event": "observation", "detail_zh": "看到物体"}, {"explorer": explorer})

    ordinary = [item for item in emitted if item.get("type") == "event"]
    assert len(ordinary) == 1
    message = ordinary[0]
    assert set(message) == {
        "type", "event", "session_id", "task_id", "executor_id", "worker_generation",
    }
    leaked = {"places", "objects", "frontiers", "edges", "nodes", "observations",
              "semantic_navigation_graph", "spatial_map", "semantic_graph"}
    assert not leaked & set(message["event"])
