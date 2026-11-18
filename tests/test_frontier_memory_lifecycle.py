"""Fix #1：前沿记忆链上原本断着的四处，各钉一条有分辨力的判据。

背景（2026-10-14 离线核查）：计划书 §6.3 说"前沿走过就记账、试过到不了就拉黑"，
但整条链两端都断着——

* 生产端：``FrontierExtractor`` 的 frontier_id 里带着**连通分量的枚举序号**，
  位置又只截断到 0.01 m（比 0.05 m 的地图分辨率还细），同一面墙每长大一格就
  换一个 id，按 id 记账的记忆永远是空条目；
* 消费端：``SpatialMemory.mark_frontier_visited`` **零调用点**，
  ``visit_count`` / ``fail_count`` 永远是 0，``blacklisted`` 永远是 False；
* 读键：``LongTermGoalSelector`` 读 ``failure_count``、``run_semantic_exploration``
  读 ``semantic_prior``/``semantic_score``，而 ``FrontierMemoryEntry.to_dict()``
  写的是 ``fail_count`` / ``semantic_gain_after_visit`` —— 读了个空键，恒为 0；
* 诊断：``memory_status`` 用 ``getattr(entry, "status", "OPEN")``，而该 dataclass
  **没有** ``status`` 字段，于是恒返回 "OPEN"，visited/unreachable 永远是 0。

每条判据都配了对照臂：证明它在**该触发时触发**（正控），在该沉默时沉默（负控）。
只报"没检出"而没有注入校准的不算数。
"""

from __future__ import annotations

import builtins
import pathlib

import pytest

from app.navigation.long_term_goal_selector import LongTermGoalSelector
from app.spatial.frontier_extractor import FrontierExtractor
from app.spatial.models import FrontierCandidate, SpatialMapSnapshot
from app.spatial.spatial_memory import FrontierMemoryEntry, SpatialMemory

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
EXPLORATION_SCRIPT = REPO_ROOT / "scripts" / "go2w" / "run_semantic_exploration.py"

RES = 0.05


# --------------------------------------------------------------------------
# 生产端：前沿身份要跨轮稳定
# --------------------------------------------------------------------------

def _snapshot(free, *, width=80, height=40, revision=1) -> SpatialMapSnapshot:
    return SpatialMapSnapshot(
        revision=revision, resolution_m=RES, origin=(0.0, 0.0),
        width=width, height=height, free=list(free), occupied=[], unknown=[],
    )


def _drift_series(*, grow=1, cycles=20):
    """一面墙：free 区每轮向 +x 长 ``grow`` 列，边界前沿随之漂移。"""
    return [
        _snapshot([(x, y) for x in range(6 + grow * t) for y in range(6, 16)],
                  revision=t + 1)
        for t in range(cycles)
    ]


def _two_blobs(gap_cells: int):
    """两个 3x3 free 块，中间留 ``gap_cells`` 格 unknown ⇒ 两个连通分量。

    等大小 ⇒ 块的几何中心就是质心 ⇒ **质心间距 = (3 + gap) x 分辨率**，精确可控。
    """
    a_x0, y0 = 6, 9
    b_x0 = a_x0 + 3 + gap_cells
    free = [(x, y) for x in range(a_x0, a_x0 + 3) for y in range(y0, y0 + 3)]
    free += [(x, y) for x in range(b_x0, b_x0 + 3) for y in range(y0, y0 + 3)]
    return free


def _old_scheme_id(index: int, centroid) -> str:
    """修复前的 id 生成式：枚举序号 + 1 cm 截断（复制自 git 历史）。"""
    return f"frontier_{index:02d}_{int(centroid[0] * 100)}_{int(centroid[1] * 100)}"


def _keep_rate(extractor, snaps, *, scheme: str):
    ids = []
    for snap in snaps:
        found = extractor.extract(snap)
        if not found:
            ids.append(None)
            continue
        # "墙上那个"前沿 = 位置 x 最大的候选
        candidate = max(found, key=lambda c: c.position[0])
        ids.append(
            # The old extractor navigated to its centroid. Its counterfactual
            # ID must use that centroid, not the new free-cell representative.
            _old_scheme_id(0, candidate.provenance["identity_centroid"]) if scheme == "old"
            else candidate.frontier_id
        )
    kept = sum(1 for a, b in zip(ids, ids[1:]) if a == b and a is not None)
    return kept, max(0, len(ids) - 1), ids


def test_frontier_id_survives_map_growth() -> None:
    """正控：同一物理前沿，地图每轮长大一格时 id 必须稳定。

    对照臂是同一条序列上的旧 scheme。**没有对照臂这条测试就不成立** ——
    单看"新 scheme 保持率 0.85"说不清 0.85 是好是坏；摆在一起才知道旧方案
    是 0.00（每轮都换名字）。
    """
    snaps = _drift_series(grow=+1, cycles=20)
    extractor = FrontierExtractor(min_component_size=1)

    new_kept, total, new_ids = _keep_rate(extractor, snaps, scheme="new")
    old_kept, _, old_ids = _keep_rate(
        FrontierExtractor(min_component_size=1), snaps, scheme="old"
    )

    assert total == 19
    assert new_kept / total >= 0.75, f"新 scheme 跨轮保持率过低: {new_ids}"
    assert old_kept / total <= 0.10, f"旧 scheme 竟然稳定: {old_ids}"
    assert new_kept > old_kept


def test_two_frontiers_thirty_cm_apart_keep_distinct_ids() -> None:
    """三段式：两个连通分量、质心相距 0.30 m ⇒ 必须是两个候选、两个 id。

    少了第一段断言，"两个 id"可能只是"提取器漏了一个"的假象。
    """
    found = FrontierExtractor(min_component_size=1).extract(_snapshot(_two_blobs(3)))
    assert len(found) == 2, "前提不成立：提取器没给出两个候选"
    assert len({f.frontier_id for f in found}) == 2


def test_id_quantization_tradeoff_bound() -> None:
    """把「撞 id 的临界间距」钉下来，免得有人随手把 q 调大。

    q=0.25：间距 0.15 m 撞、0.30 m 分开；q=0.50：0.30 m 就撞。
    撞 id 会把两个前沿的记忆合并（比丢掉身份更糟），所以取 0.25 而不是
    保持率更高的 0.50。这条测试的价值在于**它会在阈值被改动时红**。
    """
    q25 = FrontierExtractor(min_component_size=1, id_quantization_m=0.25)
    q50 = FrontierExtractor(min_component_size=1, id_quantization_m=0.50)

    assert len({f.frontier_id for f in q25.extract(_snapshot(_two_blobs(3)))}) == 2
    assert len({f.frontier_id for f in q50.extract(_snapshot(_two_blobs(3)))}) == 1
    # 两根竖条（质心 0.25 / 0.35 m）在 q=0.25 下也撞同一格 —— 这是已知且可
    # 接受的代价，写在测试里而不是只写在注释里：两个前沿挨到十几厘米时合并
    # 记忆，比"每轮都丢身份"轻得多。阈值被改大时上面那行会先红。
    free = [(x, y) for x in (5, 7) for y in range(9, 12)]
    assert len(q25.extract(_snapshot(free))) == 1


def test_id_collisions_are_reported_not_hidden() -> None:
    """同格撞车必须显式丢弃并把数量写进 provenance，不许加后缀区分。

    两根单列竖条（x=5 与 x=7，中间那列是 unknown ⇒ 两个连通分量），质心落在
    同一个 0.25 m 格（0.25 与 0.35 都归 ``frontier_1_2``）。这正是"同一条边界
    被噪声切成两段、两段又落进同一格"的那种情形。
    """
    free = [(x, y) for x in (5, 7) for y in range(9, 12)]
    found = FrontierExtractor(min_component_size=1).extract(_snapshot(free))
    assert len(found) == 1
    assert found[0].frontier_id == "frontier_1_2"
    assert found[0].provenance["id_collisions_dropped"] == 1, \
        "撞车被静默吞掉了 —— 那正是'枚举序号进 id'的病根"
    assert found[0].provenance["source"] == "metric_frontier_extractor"


# --------------------------------------------------------------------------
# 消费端：选择器真的读得懂记忆条目
# --------------------------------------------------------------------------

def _frontier(fid: str) -> FrontierCandidate:
    return FrontierCandidate(
        frontier_id=fid, position=(1.0, 0.0), bearing_deg=0.0,
        distance_m=1.0, spatial_information_gain=0.5,
    )


def test_fail_count_key_is_actually_read() -> None:
    """注入校准：``fail_count`` 必须真的降分。

    修复前读的是 ``failure_count``（不存在于 to_dict），这一项恒为 0。
    分两刀：
    (a) 只有 F1 带失败记录 ⇒ 排序必须翻过来（证明它进了总分，而不只是
        "算出来一个数"）；
    (b) 两边都带 ⇒ 冠军的 component 必须 > 0（``result`` 是**冠军**的
        ScoredIntent，拿它去查败者的 component 正好会读到 0 —— 这正是
        "只写一刀"最容易骗过自己的地方）。
    """
    selector = LongTermGoalSelector()
    flipped = selector.select(
        match_state="ZERO",
        frontiers=[_frontier("F1"), _frontier("F2")],
        frontier_memory={"F1": {"fail_count": 3}, "F2": {"fail_count": 0}},
    )
    assert flipped is not None
    assert flipped.intent.target_frontier_id == "F2"

    both = selector.select(
        match_state="ZERO",
        frontiers=[_frontier("F1"), _frontier("F2")],
        frontier_memory={"F1": {"fail_count": 3}, "F2": {"fail_count": 3}},
    )
    assert both is not None
    assert both.components["traversal_failure_penalty"] > 0.0


def test_blacklisted_frontier_is_penalised_not_filtered() -> None:
    """黑名单是**扣分**不是**剔除**：候选还在，只是排序靠后。

    硬过滤会把候选集缩小，而 ``reachable_frontier_count`` 喂给
    ``_classify_empty_candidates`` 决定 PLANNING_ERROR 还是 SEARCH_EXHAUSTED ——
    过滤掉唯一的候选会让"卡住"被报成"穷尽"。

    判据必须落在**分数**上而不是 component 上：把权重归零时
    ``components["blacklist_penalty"]`` 照样是 1.0（它只是算出来了，没进总分），
    所以只查 component 的测试对"权重接错"完全无感。这条测试的第一版就是
    这样漏掉的，注错校准把它抓了出来。
    """
    selector = LongTermGoalSelector()

    def score(blacklisted: bool) -> float:
        scored = selector.select(
            match_state="ZERO", frontiers=[_frontier("F1")],
            frontier_memory={"F1": {"blacklisted": blacklisted, "fail_count": 0}},
        )
        assert scored is not None
        return scored.score

    drop = score(False) - score(True)
    assert drop > 0.0, "黑名单没有扣分 —— 权重接丢了"
    assert drop == pytest.approx(selector.weights["blacklist"])

    # 排序臂：两边除了 blacklisted 没有任何差别（fail_count 都是 0），
    # 所以名次变化只可能来自黑名单扣分。
    result = selector.select(
        match_state="ZERO",
        frontiers=[_frontier("F1"), _frontier("F2")],
        frontier_memory={
            "F1": {"blacklisted": True, "fail_count": 0},
            "F2": {"blacklisted": False, "fail_count": 0},
        },
    )
    assert result is not None
    assert result.intent.target_frontier_id == "F2"

    # 只剩被拉黑的那一个时，选择器**仍然**要给出意图（不许返回 None）
    only = selector.select(
        match_state="ZERO",
        frontiers=[_frontier("F1")],
        frontier_memory={"F1": {"blacklisted": True, "fail_count": 2}},
    )
    assert only is not None
    assert only.intent.target_frontier_id == "F1"


def test_entry_dict_keys_match_what_the_selector_reads() -> None:
    """契约：选择器读的键必须在 ``to_dict()`` 里真的存在。

    这条是"读了个空键"那一类故障的通式判据 —— 键名对不上时它是静默的
    （恒 0），不会抛异常。
    """
    dumped = FrontierMemoryEntry("F1").to_dict()
    for key in ("visit_count", "fail_count", "blacklisted",
                "semantic_gain_after_visit"):
        assert key in dumped, f"选择器/记忆上下文读的 {key!r} 不在 to_dict() 里"
    assert "failure_count" not in dumped, "旧的错键竟然存在了 —— 检查读端是否也该统一"


# --------------------------------------------------------------------------
# 写入口：唯一的 mark_frontier_visited 调用点
# --------------------------------------------------------------------------

def _load_settle():
    """从脚本源码里取出 ``_settle_frontier_selection``。

    该脚本模块级 import rclpy，不能直接 import；这里把模块级常量与目标函数
    的 AST 摘出来单独 exec，既验证了它确实在模块层（而不是某次重构后掉进
    嵌套作用域变成死代码），也拿到了可直接调用的函数对象。
    """
    import ast

    source = EXPLORATION_SCRIPT.read_text(encoding="utf-8")
    tree = ast.parse(source)
    constants = {
        "_FRONTIER_REACH_RADIUS_M", "_FRONTIER_NO_MOTION_MAX_M",
        "_FRONTIER_FAIL_ATTEMPTS",
    }
    picked = [
        node for node in tree.body
        if (isinstance(node, ast.FunctionDef)
            and node.name == "_settle_frontier_selection")
        or (isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id in constants for t in node.targets))
    ]
    assert len([n for n in picked if isinstance(n, ast.FunctionDef)]) == 1, \
        "``_settle_frontier_selection`` 不在模块层 —— 它从生成器里拿不到了"
    assert len([n for n in picked if isinstance(n, ast.Assign)]) == 3, \
        "三个阈值常量不在模块层"
    func_node = next(n for n in picked if isinstance(n, ast.FunctionDef))
    namespace: dict = {"math": __import__("math")}
    exec(compile(ast.Module(body=picked, type_ignores=[]), "<settle>", "exec"),
         namespace)
    # 该函数引用的**自由名字**必须真的在模块层绑定过。**这一条不是形式主义**：
    # 把常量块插到某个函数体末尾（缩进 4 格）会让它变成那个函数的局部变量，
    # 模块层照样 import 得动、ast.parse 也照样过，只有真跑到那行才 NameError。
    # 仓库里 test_production_entry_wiring.py 钉的就是同一个故障类。
    # 函数内的绑定是**过近似**（含嵌套作用域），可能漏报但不会误报。
    module_names = {
        node.name for node in tree.body if isinstance(node, ast.FunctionDef)
    } | {
        target.id
        for node in tree.body if isinstance(node, ast.Assign)
        for target in node.targets if isinstance(target, ast.Name)
    } | {
        alias.asname or alias.name.split(".")[0]
        for node in tree.body if isinstance(node, (ast.Import, ast.ImportFrom))
        for alias in node.names
    }
    loaded = {
        node.id for node in ast.walk(func_node)
        if isinstance(node, ast.Name) and isinstance(node.ctx, ast.Load)
    }
    local = {node.arg for node in ast.walk(func_node) if isinstance(node, ast.arg)}
    local |= {
        node.id for node in ast.walk(func_node)
        if isinstance(node, ast.Name) and isinstance(node.ctx, (ast.Store, ast.Del))
    }
    unresolved = loaded - local - module_names - set(dir(builtins))
    assert not unresolved, (
        "``_settle_frontier_selection`` 引用了既不是局部、也不在模块层的名字"
        f"（常量块掉进了别的函数体？）: {sorted(unresolved)}"
    )
    return namespace["_settle_frontier_selection"]


def _settle_case(*, frontier_position, current_position, attempts=0, pose=None):
    settle = _load_settle()
    memory = SpatialMemory()
    frontier = FrontierCandidate(
        frontier_id="frontier_0_0", position=frontier_position,
        bearing_deg=0.0, distance_m=1.0, spatial_information_gain=0.5,
    )
    state = {"last_frontier_selection": {
        "frontier_id": "frontier_0_0",
        "pose": pose if pose is not None else current_position,
        "attempts": attempts,
    }}
    settle(state, memory, [frontier], current_position)
    return memory, state


def test_settle_marks_visited_when_reached() -> None:
    memory, state = _settle_case(frontier_position=(1.0, 0.0),
                                 current_position=(0.6, 0.0))
    entry = memory.frontiers["frontier_0_0"]
    assert entry.visit_count == 1
    assert entry.fail_count == 0
    assert "last_frontier_selection" not in state, "结算完应该把记录清掉"


def test_settle_marks_failed_only_after_repeated_zero_motion() -> None:
    """正控 + 阈值负控：一次没挪不够，攒满 3 次才算。"""
    memory, _ = _settle_case(frontier_position=(5.0, 0.0),
                            current_position=(0.0, 0.0), attempts=0)
    assert memory.frontiers == {}, "第一次就记账了 —— 阈值没生效"

    memory, state = _settle_case(frontier_position=(5.0, 0.0),
                                 current_position=(0.0, 0.0), attempts=2)
    entry = memory.frontiers["frontier_0_0"]
    assert entry.fail_count == 1
    assert state["last_frontier_selection"]["attempts"] == 0, \
        "记满一笔后窗口必须清零，否则第二笔永远攒不出来"
    assert state["last_frontier_selection"]["pose"] == (0.0, 0.0)


def test_settle_reaches_blacklist_at_two_failures() -> None:
    """端到端把 ``blacklisted`` 走通 —— 这是黑名单扣分唯一的来源。

    ``SpatialMemory`` 要求 ``fail_count >= 2`` 才拉黑；如果写入口用了
    "记过就不再记"的闩锁，这条链会永远停在 False（接上了但一次不触发）。
    """
    memory = SpatialMemory()
    settle = _load_settle()
    frontier = FrontierCandidate(
        frontier_id="frontier_0_0", position=(5.0, 0.0), bearing_deg=0.0,
        distance_m=5.0, spatial_information_gain=0.5,
    )
    state = {"last_frontier_selection": {
        "frontier_id": "frontier_0_0", "pose": (0.0, 0.0), "attempts": 0}}
    for _ in range(6):        # 两个「3 次规划无一寸位移」的窗口
        settle(state, memory, [frontier], (0.0, 0.0))
    entry = memory.frontiers["frontier_0_0"]
    assert entry.fail_count >= 2
    assert entry.blacklisted is True
    assert "frontier_0_0" in memory.blacklist


def test_explicit_nav2_preflight_rejection_penalizes_immediately() -> None:
    memory = SpatialMemory()
    memory.mark_frontier_preflight_rejected("frontier_0_0")
    entry = memory.frontiers["frontier_0_0"]
    assert entry.fail_count == 1
    assert entry.blacklisted is True
    assert "frontier_0_0" in memory.blacklist


def test_settle_writes_nothing_without_positive_evidence() -> None:
    """负控：三条"没有证据"的路径都不许记账。

    误记的代价不是零 —— 前沿会被错误降权，所以这三条要一起钉：
    有位移但没到（在往那儿走）、前沿已从地图上消失（走穿了还是地图重建，
    分不清）、拿不到位姿。
    """
    # (a) 有位移但还没到
    memory, _ = _settle_case(frontier_position=(5.0, 0.0),
                             current_position=(1.0, 0.0), attempts=2,
                             pose=(0.0, 0.0))
    assert memory.frontiers == {}, "还在往那儿走，不该记成失败"

    # (b) 前沿已不在当前前沿集里
    settle = _load_settle()
    memory = SpatialMemory()
    state = {"last_frontier_selection": {
        "frontier_id": "frontier_9_9", "pose": (0.0, 0.0), "attempts": 5}}
    settle(state, memory, [FrontierCandidate(
        frontier_id="frontier_0_0", position=(5.0, 0.0), bearing_deg=0.0,
        distance_m=5.0, spatial_information_gain=0.5)], (0.0, 0.0))
    assert memory.frontiers == {}

    # (c) 没有位姿
    settle = _load_settle()
    memory = SpatialMemory()
    state = {"last_frontier_selection": {
        "frontier_id": "frontier_0_0", "pose": (0.0, 0.0), "attempts": 9}}
    settle(state, memory, [FrontierCandidate(
        frontier_id="frontier_0_0", position=(5.0, 0.0), bearing_deg=0.0,
        distance_m=5.0, spatial_information_gain=0.5)], None)
    assert memory.frontiers == {}


# --------------------------------------------------------------------------
# 接线本身：写入口与读键不许再退回静默 no-op
# --------------------------------------------------------------------------

def test_mark_frontier_visited_has_a_production_call_site() -> None:
    """``mark_frontier_visited`` 必须有**默认路径上**的生产调用点。

    修复前全仓库零调用点：``SpatialMemory`` 的访存记账整套代码都在，
    却没有一处写它 —— 这种"有读无写"的故障是静默的。

    三层判据，缺一层都能被假绿骗过去：
    1. ``_settle_frontier_selection`` 真的被**调用**（只查 "函数名出现在源码里"
       是没用的 —— 函数定义自己就带着那个名字，把调用点删掉它照样为真，
       第一版就是这么漏的）；
    2. 那个调用发生在 ``spatial_candidate_generator`` 里面（默认路径 = WebUI
       的 ``--spatial-v2`` 走的那个生成器），而不是别处的死代码；
    3. 函数体内确实调了 ``mark_frontier_visited``。
    """
    import ast as _ast

    tree = _ast.parse(EXPLORATION_SCRIPT.read_text(encoding="utf-8"))
    settle_def = next(
        node for node in _ast.walk(tree)
        if isinstance(node, _ast.FunctionDef) and node.name == "_settle_frontier_selection"
    )
    called = {
        node.func.attr for node in _ast.walk(settle_def)
        if isinstance(node, _ast.Call) and isinstance(node.func, _ast.Attribute)
    }
    assert "mark_frontier_visited" in called, \
        "``_settle_frontier_selection`` 里没有 mark_frontier_visited 的调用"

    generators = [
        node for node in _ast.walk(tree)
        if isinstance(node, _ast.FunctionDef) and node.name == "spatial_candidate_generator"
    ]
    assert len(generators) == 1, "找不到唯一的候选生成器 —— 判据失去了目标"
    invoked = [
        node for node in _ast.walk(generators[0])
        if isinstance(node, _ast.Call) and isinstance(node.func, _ast.Name)
        and node.func.id == "_settle_frontier_selection"
    ]
    assert invoked, \
        "写入口只有定义、没有调用点（或调用点不在默认路径上）—— 接上了但不通电"
    assert len(invoked) == 1, "写入口被调用了不止一次 —— 结算会重复记账"


def test_memory_status_does_not_invent_a_status_field() -> None:
    """``FrontierMemoryEntry`` 没有 ``status`` 字段，诊断不许再 getattr 它。

    只看**填充 ``memory_status`` 的那段源码**，不全文搜字符串：脚本里
    ``getattr(entry, "status", "") == "CONFIRMED"`` 是另一套对象上的合法用法，
    而注释里也为说明这件事写着同样的字面量 —— 全文匹配两头都会误报。
    """
    import ast as _ast

    assert not hasattr(FrontierMemoryEntry("F1"), "status")
    source = EXPLORATION_SCRIPT.read_text(encoding="utf-8")
    tree = _ast.parse(source)
    loops = [
        node for node in _ast.walk(tree)
        if isinstance(node, _ast.For) and any(
            isinstance(stmt, _ast.Assign)
            and any(isinstance(t, _ast.Subscript)
                    and isinstance(t.value, _ast.Name)
                    and t.value.id == "memory_status"
                    for t in stmt.targets)
            for stmt in _ast.walk(node)
        )
    ]
    assert len(loops) == 1, "没找到填充 memory_status 的循环 —— 判据失去了目标"
    segment = _ast.get_source_segment(source, loops[0])
    assert "visit_count" in segment, "memory_status 不再由正向证据推导"
    status_attrs = [
        node.args[1].value for node in _ast.walk(loops[0])
        if isinstance(node, _ast.Call)
        and isinstance(node.func, _ast.Name) and node.func.id == "getattr"
        and len(node.args) > 1 and isinstance(node.args[1], _ast.Constant)
    ]
    assert "status" not in status_attrs, \
        "memory_status 又回到了 getattr 一个不存在的 status 字段"


def test_frontier_memory_read_keys_exist_on_the_entry() -> None:
    """脚本从 ``frontier_memory`` 里读的每个键，都必须在 ``to_dict()`` 里。

    通式判据：把脚本里所有 ``fm.get("<key>")`` 静态摘出来逐个核对。
    """
    import re

    source = EXPLORATION_SCRIPT.read_text(encoding="utf-8")
    read_keys = set(re.findall(r'\bfm\.get\(\s*"([^"]+)"', source))
    assert read_keys, "没有摘到任何 fm.get 读键 —— 正则或代码结构变了，这条测试失去了分辨力"
    dumped = FrontierMemoryEntry("F1").to_dict()
    unknown = read_keys - set(dumped)
    assert not unknown, f"脚本读了 to_dict() 里不存在的键: {sorted(unknown)}"


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(pytest.main([__file__, "-q"]))
