"""§12.4 WebUI session：archive 里有旧 terminal，**新进程的 live graph 必须是空的**
（§30 点名的本文件）。

场景原文（计划书 §12.4）：

    archive 中有 old terminal: P1, F01, F02...
    新 WebUI process 启动：history 可找到旧 session / live graph 必须空
    点击 start：仍先空 / 第一 observation 后才 P1

**不重抄哪些。** 前两句里的 archive 半已被
`tests/test_search_session_archive.py`（断言 fresh 服务端 `session_id==""` /
`status=="IDLE"` 而 history 仍查得到旧 session）与
`tests/test_search_persistent_history_web_contract.py`（JS 层 history 与 live
state 分离）覆盖。本文件只补**后两句** —— 全仓零断言：

* `SearchStateStore.reset()` 之后（= 点击 start）live graph 仍须为空；
* 把已经跑满的 store `reset()` 掉，旧 Places / 物件 / 时间线**一件都不许留**；
* P1 出现的**时机**：启动序列（`SESSION_CREATED` → `TASK_UNDERSTANDING` →
  `SESSION_STARTED` → `SEARCH_STATE_CHANGED` → 第一帧 `OBSERVATION_UPDATED`）
  **一个 Place 都不许造**。

**2026-10-08 计划书 §2.2 之后的机制（本文件随之改写）：**

1. store 里**只有一份图**，装在 `_graph` 槽里，唯一写入口是 `set_graph()`；
   `snapshot()["graph"]` / `["graph_revision"]` 是唯一的读口。
2. **`MAP_UPDATED` 不再携带图**，它只进时间线（`_RECORD_ONLY_EVENTS`）。
   同理 `PLACE_CREATED` / `PLACE_UPDATED` / `SEMANTIC_OBJECT_LOCALIZED` /
   `FRONTIERS_UPDATED` / `SPATIAL_MAP_UPDATED` 在 store 层**都不再改图**。
3. 因此"空图守得住"这件事现在比 §12.4 时代更强：能写图的路径**只剩
   `set_graph()` 一条**，事件流里压根没有别的口子。
4. `reset()` 把 `_graph_revision` 打回 `-1`。它的判别性后果也随 §2.2 变了：
   没清的话，新会话的第一张图（revision 从 1 起）会被 `set_graph()` 的
   "旧的不许回退"判据**拒收**，页面永远停在空图 —— 下面按这个后果来测。

**诚实边界**：本文件是 store 层的契约测试，不驱动 WebUI/HTTP。它证明"给定事件
序列，store 的快照是什么"，不证明"真机跑起来事件序列就是这样"。后者要靠真机
事件流。
"""

from __future__ import annotations

from app.live_robot.search_event import (
    FRONTIERS_UPDATED,
    MAP_UPDATED,
    OBJECTS_UPDATED,
    OBSERVATION_UPDATED,
    PLACE_CREATED,
    PLACE_UPDATED,
    SEARCH_STATE_CHANGED,
    SEMANTIC_OBJECT_LOCALIZED,
    SESSION_CREATED,
    SESSION_STARTED,
    TASK_UNDERSTANDING,
    SearchEvent,
)
from app.live_robot.search_state_store import (
    STATUS_IDLE,
    STATUS_RUNNING,
    STATUS_STARTING,
    SearchStateStore,
)
from app.spatial.place_graph import PlaceGraph
from app.spatial.semantic_navigation_graph import SemanticNavigationGraph

SESSION = "search_20260916_120000_abcd1234"


def _event(event_type: str, payload: dict | None = None, ts: float = 1.0) -> SearchEvent:
    return SearchEvent(
        event_id=1, session_id=SESSION, timestamp=ts,
        event_type=event_type, payload=payload or {},
    )


def _semantic_graph_dict(*, with_place: bool, revision: int = 1) -> dict:
    """用**真实生产者**造 `semantic_navigation_graph_v1`，不手写 P1。

    手写 `{"places": [{"place_id": "P1"}]}` 会让"P1 这个 id 长什么样"变成我的假设；
    走 `PlaceGraph.register_observation` 则钉住真实格式（`place_graph.py`
    的 `f"P{self._sequence}"`）。`revision` 由图的计数器给出，调用方只在需要
    指定新旧时覆盖它。
    """
    place_graph = PlaceGraph()
    if with_place:
        place_graph.register_observation(
            observation_id="obs_0", heading_sector=0, objects=[],
        )
    value = SemanticNavigationGraph(place_graph=place_graph).to_dict()
    value["revision"] = revision
    return value


def _graph(store: SearchStateStore) -> dict | None:
    return store.snapshot()["graph"]


def _places(store: SearchStateStore) -> list[dict]:
    graph = _graph(store) or {}
    return list(graph.get("places") or [])


def _revision(store: SearchStateStore) -> int:
    return int(store.snapshot()["graph_revision"])


def _started_store() -> SearchStateStore:
    """跑到「点击 start 之后、第一帧 observation 之前」的那个时刻。"""
    store = SearchStateStore()
    store.reset(session_id=SESSION, target="垃圾桶")
    for event_type, payload in (
        (SESSION_CREATED, {"target": "垃圾桶"}),
        (TASK_UNDERSTANDING, {"task": {"canonical_target": "垃圾桶"}}),
        (SESSION_STARTED, {"phase": "OBSERVE"}),
        (SEARCH_STATE_CHANGED, {"phase": "OBSERVE"}),
    ):
        store.apply(_event(event_type, payload))
    return store


# --------------------------------------------------------------------------
# A. 新进程 / 点击 start：live graph 必须是空的
# --------------------------------------------------------------------------

def test_a_fresh_store_has_no_live_graph_at_all():
    """全新的 store（= 新 WebUI process）连 `graph` 都是 `None`，不是空图。

    这个区别有意义：`None` 表示"这副图从来没有过"，`{"places": []}` 表示"有过、
    只是眼下没内容"。前端据此区分"还没开始"与"开始了但没建到 Place"。
    """
    store = SearchStateStore()
    assert _graph(store) is None
    assert _revision(store) == 0
    assert store.snapshot()["status"] == STATUS_IDLE


def test_the_startup_sequence_never_creates_a_place():
    """**§12.4「点击 start 仍先空」。** 启动序列 + 第一帧 observation 走完，
    live graph 里**一个 Place 都不许有**。

    危险形状：某个 handler 为了"让页面早点有东西看"而在 `SESSION_STARTED` 时
    凭空塞一个 Place。那样狗还没动，图里就已经有个"当前所在 Place"，而它就是靠
    真实观测融合出来的位姿锚点 —— 一个没观测支撑的锚点会污染后续所有关联。
    """
    store = _started_store()
    store.apply(_event(OBSERVATION_UPDATED, {"bundle_id": "obs_0"}))
    assert _graph(store) is None, "第一帧 observation 本身不是建图的理由"
    assert store.snapshot()["status"] == STATUS_RUNNING, (
        "start 之后状态必须已经 RUNNING —— 空的理由必须是'还没观测到'，"
        "而不是'压根没开始'"
    )


def test_an_empty_graph_projects_empty_places_not_none():
    """worker 第一帧就会推 `graph_snapshot`，哪怕里面还没有 Place。

    此时 `graph["places"]` 是 `[]` 而不是缺键 —— 是"有图但空"。这条钉的是
    `web_snapshot()` 的投影形状：`places` / `edges` / `current_place_id` 三个键
    都在，前端不会因为缺键而渲染出错。
    """
    store = _started_store()
    store.set_graph(_semantic_graph_dict(with_place=False))
    graph = _graph(store) or {}
    for key in ("places", "edges", "current_place_id", "objects", "frontiers",
                "object_topology"):
        assert key in graph, f"web_snapshot() 必须带上 {key}"
    assert graph["places"] == []
    assert graph["current_place_id"] is None


# --------------------------------------------------------------------------
# B. 第一帧真实 observation 之后，P1 才出现
# --------------------------------------------------------------------------

def test_the_first_place_appears_only_when_the_graph_carries_one():
    """**§12.4「第一 observation 后才 P1」。** 从「start 过、观测过、还没 Place」
    出发，把带 P1 的真实语义图送进来 —— `P1` 这时才出现，且 `current_place_id`
    一并跟上。

    §2.2 之后这是**唯一**能建 Place 的路径：事件流里的 `MAP_UPDATED` /
    `PLACE_CREATED` / `PLACE_UPDATED` 在 store 层都不再改图（下面两条在守）。
    """
    store = _started_store()
    store.apply(_event(OBSERVATION_UPDATED, {"bundle_id": "obs_0"}))
    assert _places(store) == []

    assert store.set_graph(_semantic_graph_dict(with_place=True)) is True

    assert [place["place_id"] for place in _places(store)] == ["P1"]
    assert (_graph(store) or {}).get("current_place_id") == "P1"


def test_graph_events_no_longer_write_the_graph_at_all():
    """**§2.2 的判别性证据。** 五条曾经各自往 store 里投影一份图的事件
    （`MAP_UPDATED` 带 `graph`、`PLACE_CREATED`、`PLACE_UPDATED`、
    `SEMANTIC_OBJECT_LOCALIZED`、`FRONTIERS_UPDATED`）现在**一个字都不许改图**。

    危险形状：留着旧的 handler 做兜底。那样同一个事实会有两条写入路径，两边的
    新旧判据还不同（一边按 revision、一边按事件到达顺序），最终页面上的图取决于
    "哪条先到" —— 这正是 §2.1 要消灭的竞态。

    注意这些事件**仍然要进时间线**：它们是原始证据。
    """
    store = _started_store()
    store.set_graph(_semantic_graph_dict(with_place=True))
    before = _graph(store)
    before_revision = _revision(store)

    store.apply(_event(MAP_UPDATED, {"graph": {"places": [{"place_id": "P9"}]}}))
    store.apply(_event(PLACE_CREATED, {"place": {"place_id": "P8"}}))
    store.apply(_event(PLACE_UPDATED, {"place": {"place_id": "P1", "visit_count": 9}}))
    store.apply(_event(SEMANTIC_OBJECT_LOCALIZED, {"object": {"object_id": "O9"}}))
    store.apply(_event(FRONTIERS_UPDATED, {"frontiers": [{"frontier_id": "F09"}]}))

    assert _graph(store) == before, "这五条事件不许改图"
    assert _revision(store) == before_revision, "更不许推进 graph_revision"
    assert [place["place_id"] for place in _places(store)] == ["P1"]
    assert (_places(store)[0]).get("visit_count") != 9, "PLACE_UPDATED 不再是替换路径"

    recorded = {item["event_type"] for item in store.snapshot()["timeline"]}
    for event_type in (MAP_UPDATED, PLACE_CREATED, PLACE_UPDATED,
                       SEMANTIC_OBJECT_LOCALIZED, FRONTIERS_UPDATED):
        assert event_type in recorded, f"{event_type} 仍然必须进时间线"


def test_the_runner_never_emits_place_created():
    """**源码级契约，诚实标注。** runner 发射的事件名里**没有** `place_created`。

    若哪天 runner 开始发 `place_created`，这条会红 —— 那是好事：说明事件流里多了
    一条"看起来能建 Place"的路，需要重新确认它在 store 层是否仍然无害
    （上一条已经证明它无害，但那条用的是我自己造的事件）。
    """
    import re
    from pathlib import Path

    runner = Path(__file__).resolve().parents[1] / "scripts/go2w/run_semantic_exploration.py"
    emitted = set(re.findall(r'"event": "([a-z_]+)"', runner.read_text(encoding="utf-8")))
    assert "place_updated" in emitted, "runner 应当发 place_updated"
    assert "place_created" not in emitted, (
        "runner 开始发 place_created 了 —— 请确认 store 层对它仍然无动于衷"
    )


def test_an_older_graph_is_dropped_instead_of_regressing_the_display():
    """`set_graph()` 只认 revision：迟到的旧图被丢掉，不许把画面拽回去。

    这条替掉了 §2.2 之前那条"PLACE_UPDATED 只替换不追加"的对照实验 —— 那个
    形状已经不存在了，真正需要防的是**乱序到达**：`graph_snapshot` 走的是
    worker stdout，慢一点的那份完全可能后到。
    """
    store = _started_store()
    assert store.set_graph(_semantic_graph_dict(with_place=True, revision=5)) is True
    assert _revision(store) == 5

    assert store.set_graph(_semantic_graph_dict(with_place=False, revision=4)) is False
    assert _revision(store) == 5, "旧图不许把 revision 拽回去"
    assert [place["place_id"] for place in _places(store)] == ["P1"], (
        "旧图更不许把已经有的 Place 抹掉"
    )


# --------------------------------------------------------------------------
# C. 点击 start 是一场真正的「live reset」：旧会话一件都不留
# --------------------------------------------------------------------------

def _populated_store() -> SearchStateStore:
    """跑满一轮：有 Place、有物件、有 frontier、有时间线、有已见计数。"""
    store = _started_store()
    graph = _semantic_graph_dict(with_place=True, revision=3)
    graph["objects"] = [{"object_id": "O1", "label": "办公椅"}]
    graph["frontiers"] = [{"frontier_id": "F01"}]
    store.set_graph(graph)
    store.apply(_event(OBSERVATION_UPDATED, {"bundle_id": "obs_0"}))
    # 旧会话反复看到同一把椅子 —— 让 ``_session_seen`` 攒起来。
    for _ in range(3):
        store.apply(_event(OBJECTS_UPDATED, {"current": [{"label": "办公椅"}]}))
    return store


def test_reset_clears_a_fully_populated_live_graph():
    """已经跑满的 store 被 `reset()` 之后，旧会话的 Place / 物件 / frontier /
    目标**一件都不许留**。任何一处漏清，都会让新会话的页面在第一帧观测之前就显示
    上一个会话的地图 —— 而"上一个会话的地图"在狗已经移动过之后是**错的位置**，
    比空图危险得多。

    **注意这条证据的边界**：这些字段都是 `_empty_snapshot()` 无条件归零的，所以
    它证明的是"快照面干净"，**不证明** reset 有没有清那几个私有状态 —— 后者由
    下面几条各自的可观测后果来证。别把这一条当成"reset 全对"的总结论。
    """
    store = _populated_store()
    # 先确认它真的满了 —— 否则下面"清空"的断言会退化成恒真。
    assert _places(store), "前置条件：reset 之前图里必须有 Place"
    assert (_graph(store) or {}).get("objects"), "前置条件：必须有物件"
    assert (_graph(store) or {}).get("frontiers"), "前置条件：必须有 frontier"
    assert store.snapshot()["timeline"], "前置条件：必须有时间线"
    assert store.snapshot()["objects"]["session_seen"], "前置条件：必须有已见计数"
    assert _revision(store) > 0, "前置条件：revision 必须已经推进过"

    store.reset(session_id=SESSION, target="垃圾桶")

    assert _graph(store) is None
    assert _revision(store) == 0
    assert store.snapshot()["selected_goal"] is None


def test_reset_reidentifies_the_session_and_restarts_the_revision():
    """`reset()` 把 `graph_revision` 打回 0，并换上新的 session/target。

    revision 归零是有意的：前端持有旧 revision 时会看到"变小了"，据以判定
    "这是一张新图"。若 revision 接着往上涨，一个拿着旧图的页面就分不清
    "图更新了"和"换会话了"。
    """
    store = _populated_store()
    assert _revision(store) > 0

    store.reset(session_id="search_20260916_130000_beef0000", target="灭火器")

    assert _revision(store) == 0
    snapshot = store.snapshot()
    assert snapshot["session_id"] == "search_20260916_130000_beef0000"
    assert snapshot["target"] == "灭火器"
    assert snapshot["status"] == STATUS_STARTING, (
        "reset 之后是 STARTING —— 新会话还没跑起来，不该直接报 RUNNING"
    )
    assert snapshot["finished_at"] is None, "上一个会话的结束时间不许留到新会话"
    assert snapshot["result"] == ""
    assert snapshot["finish_reason"] == ""


def test_reset_restarts_the_revision_counter_not_just_the_snapshot_value():
    """**`_graph_revision` 的判别性证据。** 光断言 `reset()` 之后
    `graph_revision == 0` 是**没有分辨力的** —— `snapshot()` 在 `graph is None`
    时本来就把它写成 0，注入"删掉 reset 里的 `self._graph_revision = -1`"
    照样全绿。

    私有计数器只在**新会话的第一张图**上显形。没清的后果是灾难性的：
    `set_graph()` 按"revision 不增就拒收"来防乱序，而新会话的 revision 从 1
    重新数起，于是**新会话的每一张图都被判为旧图而拒收** —— 页面永远空着，
    而 store 自己一切正常，日志里一个字都没有。
    """
    store = _populated_store()
    before = _revision(store)
    assert before > 0

    store.reset(session_id=SESSION, target="垃圾桶")
    accepted = store.set_graph(_semantic_graph_dict(with_place=True, revision=1))

    assert accepted is True, (
        f"新会话的第一张图必须是 revision=1 且被接受（旧值 {before} 不许带过来）"
    )
    assert _revision(store) == 1
    assert [place["place_id"] for place in _places(store)] == ["P1"], (
        "被拒收的话这里会是空的 —— 那是页面永远空白的那个故障"
    )


def test_reset_restarts_the_timeline_not_just_the_snapshot_view():
    """**`_timeline` 的判别性证据。** 同上：`snapshot()["timeline"] == []` 由
    `_empty_snapshot()` 兜底，没有分辨力。

    `_timeline` 是 `deque`，被 `_append_timeline_locked` 持续追加。没清它，新会话
    的第一条事件就会把**上一个会话的整条时间线**带回页面 —— 操作员在新会话里看到
    上一轮的决策过程，会以为狗已经做过那些动作。
    """
    store = _populated_store()
    assert len(store.snapshot()["timeline"]) > 1

    store.reset(session_id=SESSION, target="垃圾桶")
    store.apply(_event(SESSION_CREATED, {"target": "垃圾桶"}))

    assert len(store.snapshot()["timeline"]) == 1, (
        "新会话的时间线必须只含新会话的事件"
    )


def test_reset_restarts_the_object_seen_counters():
    """**`_session_seen` 的判别性证据。** 它是 `Counter`，`_on_objects` 里
    `+= 1` 累加，而累加值直接进 `objects.session_seen`。

    没清它的后果比看起来严重：`session_seen` 的计数是"这个物件我确认过几次"的
    依据，旧会话攒下的次数会让新会话**第一次看到**就把某个标签当成"反复确认过"，
    于是刚开搜就出现一个不该有的高置信物件。
    """
    store = _populated_store()
    seen = {item["label"]: item["observations"]
            for item in store.snapshot()["objects"]["session_seen"]}
    assert seen == {"办公椅": 3}, f"前置条件：旧会话应已看到 3 次，实际 {seen}"

    store.reset(session_id=SESSION, target="垃圾桶")
    store.apply(_event(OBJECTS_UPDATED, {"current": [{"label": "办公椅"}]}))

    seen = {item["label"]: item["observations"]
            for item in store.snapshot()["objects"]["session_seen"]}
    assert seen == {"办公椅": 1}, (
        f"新会话第一次看到就该记 1 次，不是旧会话的 3 次加一：{seen}"
    )
