"""`ObjectRelationStore` 的**直接**覆盖（§30 点名的本文件）。

**为什么单独建这个文件。** 改动前全仓 `grep -rln "ObjectRelationStore" tests/` 是空的 ——
这个类只有经 `SemanticNavigationGraph` 的**间接**覆盖（`tests/test_semantic_navigation_graph_topology.py`
三个用例），而关系边恰恰是出过真 bug 的地方（full 语义晚一拍导致关系边恒为 0，
见 memory `go2w-relations-dropped-by-frame-race`）。间接覆盖测的是"图里最后有没有边"，
测不出**为什么**某条边没进去 —— 而 `sync()` 有五条各自不同的拒绝理由，把
`source_endpoint_unresolved` 和 `self_relation_rejected` 混成一个"没边"，
下次再出问题时仍然查不出来。

**本文件只测 `sync()` 的准入/拒绝与 `_upsert` 的证据合并**，不重抄
`test_semantic_navigation_graph_topology.py` 已经覆盖的"图投影"部分。

**诚实边界**：`update_result` 用 `SimpleNamespace` 桩件，只带 `associations` 字段
（`_association_mapping` 只读这一个）。它证明"给定这份 association 映射，store 收下
哪些边、以什么理由丢掉哪些"，不证明真机上 `SemanticAssociation` 就是这个形状。
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from app.spatial.object_relation_store import (
    RELATION_ALIASES,
    RELATION_CONFIRMED,
    RELATION_STALE,
    RELATION_TENTATIVE,
    SYMMETRIC_RELATIONS,
    ObjectRelationStore,
    normalize_relation,
)

# 默认地板，与 `__init__` 的默认值保持同一个数：测边界时读它而不是再抄一遍字面量。
DEFAULT_MIN_CONFIDENCE = 0.45


def _update_result(pairs: list[tuple[str, str]]) -> SimpleNamespace:
    """最小 `update_result` 桩：`frame_object_id -> persistent_object_id`。"""
    return SimpleNamespace(
        associations=[
            SimpleNamespace(source_object_id=src, persistent_object_id=pid)
            for src, pid in pairs
        ]
    )


# 一个"两端都解析得动"的映射，绝大多数用例都用它。
TWO = _update_result([("f1", "obj_a"), ("f2", "obj_b")])


def _sync(
    store: ObjectRelationStore,
    relations: list[dict],
    *,
    update_result=TWO,
    observation_id: str = "o1",
    timestamp: float = 1.0,
) -> None:
    store.sync(
        relations=relations,
        update_result=update_result,
        observation_id=observation_id,
        timestamp=timestamp,
    )


def _edge(store: ObjectRelationStore, key: tuple[str, str, str]):
    return store.relations[key]


def _reasons(store: ObjectRelationStore) -> list[str]:
    return [d.get("reason") for d in store.debug if d.get("result") == "rejected"]


# --------------------------------------------------------------------------
# A. 身份契约：边按 persistent_object_id 建，绝不按 label 建
# （模块 docstring 的明文约定，§6/§9/§40）
# --------------------------------------------------------------------------

def test_two_objects_sharing_a_label_stay_two_distinct_edges():
    """**身份契约的核心。** 两个都叫"椅子"的物体必须留下**两条**边。

    这是这个类存在的理由（docstring 原话："Relations are never keyed by label,
    so two chairs / two bins stay distinct"）。曾经有过一版是按 label 查
    persistent id 的（`persistent_id_for_label`），两把椅子会被合并成一把，
    图上表现为"只见过一把椅子" —— 而且完全不会报错。

    这里三个 frame 对象里有两个（`f1`/`f2`）指向**不同**的 persistent id，
    边必须分开。
    """
    store = ObjectRelationStore(relation_confirm_min_observations=1)
    mapping = _update_result([("f1", "obj_001"), ("f2", "obj_002"), ("f3", "obj_003")])

    _sync(
        store,
        [{"subject_id": "f1", "object_id": "f3", "relation": "near", "confidence": 0.9}],
        update_result=mapping,
    )
    _sync(
        store,
        [{"subject_id": "f2", "object_id": "f3", "relation": "near", "confidence": 0.9}],
        update_result=mapping,
        observation_id="o2",
        timestamp=2.0,
    )

    assert sorted(store.relations.keys()) == [
        ("obj_001", "near", "obj_003"),
        ("obj_002", "near", "obj_003"),
    ]


def test_the_identity_key_never_contains_a_label():
    """键里**只许有** persistent id，不许出现任何 label 字样。

    上一条测的是行为，这条把"没有 label 参与"这件事本身钉住：即使两个对象
    带了完全相同的 `label` 字段，键也必须由 `obj_xxx` 组成。
    """
    store = ObjectRelationStore(relation_confirm_min_observations=1)
    mapping = _update_result([("f1", "obj_001"), ("f2", "obj_002")])

    _sync(
        store,
        [{
            "subject_id": "f1", "object_id": "f2", "relation": "near",
            "confidence": 0.9, "subject_label": "办公椅", "object_label": "办公椅",
        }],
        update_result=mapping,
    )

    (key,) = store.relations.keys()
    assert key == ("obj_001", "near", "obj_002")
    assert "办公椅" not in key


# --------------------------------------------------------------------------
# B. 五条拒绝路径各自留痕
# --------------------------------------------------------------------------

def test_each_rejection_reason_is_recorded_separately():
    """四种拒绝各来一次，`debug` 必须记下**四个不同**的 reason，且一条边都不建。

    混成一个"没边"是这份覆盖缺席时最容易发生的事 —— 那时"映射没建立"和
    "模型说物体跟它自己相邻"在日志里长得一模一样。
    """
    store = ObjectRelationStore()

    _sync(store, [{"subject_id": "fX", "object_id": "f2", "relation": "near"}])
    _sync(store, [{"subject_id": "f1", "object_id": "f1", "relation": "near"}])
    _sync(store, [{"subject_id": "f1", "object_id": "f2", "relation": "旁边"}])
    _sync(store, [
        {"subject_id": "f1", "object_id": "f2", "relation": "near", "confidence": 0.2},
    ])

    assert _reasons(store) == [
        "source_endpoint_unresolved",
        "self_relation_rejected",
        "relation_not_allowed",
        "confidence_below_min",
    ]
    assert store.relations == {}


def test_target_endpoint_unresolved_is_not_folded_into_source():
    """只缺**目标**端时必须报 `target_endpoint_unresolved`。

    两端共用一条 `if not source_id or not target_id` 分支，reason 是三元表达式
    算出来的 —— 顺手写成常量就会让"目标物体没关联上"永远显示成"源物体没关联上"，
    而这两种情况指向完全不同的上游缺陷（前者多半是检测漏了，后者多半是关联表没建）。
    """
    store = ObjectRelationStore()
    _sync(store, [{"subject_id": "f1", "object_id": "fX", "relation": "near"}])

    assert _reasons(store) == ["target_endpoint_unresolved"]


def test_a_self_relation_is_rejected_even_when_it_would_otherwise_pass():
    """两个**不同**的 frame 对象映射到**同一个** persistent id ⇒ 拒。

    形状：本帧检测切出了两个框，关联层把它们认成同一个持久物体，而模型说
    "前者在后者左边"。不拦的话图上会出现 `obj_001 --left_of--> obj_001` 自环，
    拓扑页画出来是一个指回自己的箭头。
    """
    store = ObjectRelationStore(relation_confirm_min_observations=1)
    same = _update_result([("f1", "obj_001"), ("f2", "obj_001")])

    _sync(
        store,
        [{"subject_id": "f1", "object_id": "f2", "relation": "left_of", "confidence": 0.99}],
        update_result=same,
    )

    assert _reasons(store) == ["self_relation_rejected"]
    assert store.relations == {}


def test_the_debug_relation_field_is_raw_when_disallowed_and_normalized_when_low():
    """`debug` 里的 `relation` 字段在两条分支上**语义不同**，这是刻意的：

    - 词表外 ⇒ 记**原始文本**（`"旁边"`）—— 不知道模型说了什么就补不进别名表；
    - 置信度低 ⇒ 记**归一后**的词（`"near"`）—— 走到这里已经过词表了。

    这个区别很脆：把 `normalize_relation` 往上提一行就没了，而且没有任何断言会红。
    """
    store = ObjectRelationStore()
    _sync(store, [{"subject_id": "f1", "object_id": "f2", "relation": "旁边"}])
    _sync(store, [
        {"subject_id": "f1", "object_id": "f2", "relation": "close_to", "confidence": 0.1},
    ])

    disallowed, low = store.debug
    assert disallowed["reason"] == "relation_not_allowed"
    assert disallowed["relation"] == "旁边", "词表外必须留原始文本，否则无从补别名"
    assert low["reason"] == "confidence_below_min"
    assert low["relation"] == "near", "过了词表的必须留归一后的词"
    assert low["confidence"] == 0.1


def test_accepted_relations_are_recorded_with_both_persistent_ids():
    """接受路径的 `debug` 必须带**归一前后两套 id** 与归一后的关系词。

    事后想回答"这一帧模型到底说了什么、我把它落成了哪条边"只有这一条记录：
    事件流里的 `observation` 不保证被留档，`debug` 是唯一两套 id 同时在场的
    地方。
    """
    store = ObjectRelationStore(relation_confirm_min_observations=1)
    _sync(
        store,
        [{"subject_id": "f2", "object_id": "f1", "relation": "next_to", "confidence": 0.9}],
    )

    (entry,) = store.debug
    assert entry["result"] == "accepted"
    assert entry["subject_id"] == "f2" and entry["object_id"] == "f1"
    # `next_to` 归一成 `adjacent_to`，且它是对称的 ⇒ 端点被排序过。
    assert entry["relation"] == "adjacent_to"
    assert entry["persistent_source_id"] == "obj_a"
    assert entry["persistent_target_id"] == "obj_b"


def test_non_dict_relations_are_skipped_without_a_trace():
    """`relations` 里混进非 dict（模型返回 `["near"]` 这种）⇒ 跳过，不抛。

    这是唯一一条**不留痕**的跳过，所以额外钉一句：它不能顺手把整个 `sync` 打断，
    同一批里排在后面的合法关系必须照常落库。
    """
    store = ObjectRelationStore(relation_confirm_min_observations=1)
    _sync(
        store,
        [
            "near",
            None,
            {"subject_id": "f1", "object_id": "f2", "relation": "near", "confidence": 0.9},
        ],
    )

    assert list(store.relations.keys()) == [("obj_a", "near", "obj_b")]
    # 且它们**不留痕**：debug 里只有那一条 accepted。若改成"把非 dict 归一成 {}"，
    # 它们会各留一条 `source_endpoint_unresolved` 的拒绝记录 —— 关系边照建，
    # 只有这条断言看得见区别。
    assert [d["result"] for d in store.debug] == ["accepted"]


# --------------------------------------------------------------------------
# C. 关系词归一与对称端点排序
# --------------------------------------------------------------------------

def test_aliases_are_accepted_and_land_on_the_canonical_word():
    """别名表的每一个词都必须能过，且落到规范词。

    `RELATION_ALIASES` 是硬编码的（没有同义词扩展），所以这份表就是唯一的
    "模型可以怎么说"清单。漏一个词 = 那一类关系**静默**永远不建边。
    """
    for alias, canonical in RELATION_ALIASES.items():
        assert normalize_relation(alias) == canonical, alias

    # 别名必须全部落到规范词表里，不许出现指向词表外的孤立词
    # （那样它会先被 `relation_not_allowed` 拒掉，别名写不写都一样）。
    from app.spatial.object_relation_store import OBJECT_RELATIONS

    assert set(RELATION_ALIASES.values()) <= OBJECT_RELATIONS
    # 反向也必须成立：每个别名都得真的建得出边，不只是归一函数认得。
    for alias in RELATION_ALIASES:
        store = ObjectRelationStore(relation_confirm_min_observations=1)
        _sync(store, [
            {"subject_id": "f1", "object_id": "f2", "relation": alias, "confidence": 0.9},
        ])
        assert len(store.relations) == 1, f"别名 {alias} 没能建出边"


def test_unknown_and_empty_relations_normalize_to_none():
    """词表外、空串、`None` 一律 `None` —— 这是 `relation_not_allowed` 的判据来源。"""
    for bad in ("beside_the", "旁边", "", "   ", None, "NEAR_ISH"):
        assert normalize_relation(bad) is None, bad

    # 大小写与空白是容忍的（模型输出不稳定）。
    assert normalize_relation("  NEAR  ") == "near"


def test_symmetric_relations_land_on_one_edge_regardless_of_direction():
    """对称关系正反两说必须落到**同一条边**，且端点按字典序排好。

    不排序的话 `a near b` 与 `b near a` 是两条键，拓扑页上同一个"相邻"关系画成
    两条反向箭头，`observation_count` 也各涨各的、永远升不到 CONFIRMED。
    """
    for relation in SYMMETRIC_RELATIONS:
        store = ObjectRelationStore(relation_confirm_min_observations=1)
        _sync(store, [
            {"subject_id": "f1", "object_id": "f2", "relation": relation, "confidence": 0.9},
        ])
        _sync(
            store,
            [{"subject_id": "f2", "object_id": "f1", "relation": relation, "confidence": 0.9}],
            observation_id="o2",
            timestamp=2.0,
        )

        assert list(store.relations.keys()) == [("obj_a", relation, "obj_b")], relation
        edge = _edge(store, ("obj_a", relation, "obj_b"))
        assert edge.observation_count == 2, f"{relation}: 反向没并进同一条边"
        assert edge.directed is False, f"{relation}: 对称关系不该是有向的"


def test_asymmetric_relations_keep_direction_and_do_not_merge():
    """非对称关系**不许**归一：`a left_of b` 与 `b left_of a` 是两条边，且 `directed`。

    "左"没有对称性，反过来是另一件事。把 `relation_is_symmetric` 写成恒真
    （或把 `SYMMETRIC_RELATIONS` 误当成全部词表）会让图上所有方向信息消失，
    而拓扑页看起来仍然"有边"。
    """
    store = ObjectRelationStore(relation_confirm_min_observations=1)
    _sync(store, [
        {"subject_id": "f1", "object_id": "f2", "relation": "left_of", "confidence": 0.9},
    ])
    _sync(
        store,
        [{"subject_id": "f2", "object_id": "f1", "relation": "left_of", "confidence": 0.9}],
        observation_id="o2",
        timestamp=2.0,
    )

    assert sorted(store.relations.keys()) == [
        ("obj_a", "left_of", "obj_b"),
        ("obj_b", "left_of", "obj_a"),
    ]
    assert all(edge.directed is True for edge in store.relations.values())
    assert "left_of" not in SYMMETRIC_RELATIONS


def test_the_symmetric_reversal_keeps_the_debug_ids_straight():
    """归一发生在 `_upsert` **之前**，但 `debug` 记的是**模型原话**的 frame id。

    也就是说：键被排序过，`debug.subject_id`/`object_id` 没有被排序。这两套必须
    并存 —— 前者是图的身份，后者是"模型当时怎么说的"，事后对账只有它。
    """
    store = ObjectRelationStore(relation_confirm_min_observations=1)
    # f2 -> f1（逆字典序），对称关系 ⇒ 键会被翻成 (obj_a, near, obj_b)。
    _sync(
        store,
        [{"subject_id": "f2", "object_id": "f1", "relation": "near", "confidence": 0.9}],
    )

    assert list(store.relations.keys()) == [("obj_a", "near", "obj_b")]
    (entry,) = store.debug
    assert (entry["subject_id"], entry["object_id"]) == ("f2", "f1")


# --------------------------------------------------------------------------
# D. 证据合并（`_upsert`）
# --------------------------------------------------------------------------

def test_repeated_observations_raise_the_count_and_upgrade_tentative_to_confirmed():
    """`relation_confirm_min_observations=2`：一次 TENTATIVE，两次 CONFIRMED。

    这是"见过一次"与"确认存在"的分界。若把 `>=` 写成 `>`（或忘了升级），
    图上所有边永远 TENTATIVE —— 而 TENTATIVE 边默认是要被过滤掉的
    （`include_tentative_objects`），表现就是"关系边恒为 0"那个老症状复发。
    """
    store = ObjectRelationStore(relation_confirm_min_observations=2)
    relation = {"subject_id": "f1", "object_id": "f2", "relation": "near", "confidence": 0.9}

    _sync(store, [relation])
    edge = _edge(store, ("obj_a", "near", "obj_b"))
    assert edge.observation_count == 1
    assert edge.status == RELATION_TENTATIVE

    _sync(store, [relation], observation_id="o2", timestamp=2.0)
    assert edge.observation_count == 2
    assert edge.status == RELATION_CONFIRMED


def test_confirm_min_one_makes_the_first_observation_confirmed():
    """阈值 1（离线建图/回放常用）⇒ 第一条边直接 CONFIRMED，不走 TENTATIVE。"""
    store = ObjectRelationStore(relation_confirm_min_observations=1)
    _sync(
        store,
        [{"subject_id": "f1", "object_id": "f2", "relation": "near", "confidence": 0.9}],
    )

    assert _edge(store, ("obj_a", "near", "obj_b")).status == RELATION_CONFIRMED


def test_confidence_is_a_running_mean_not_the_last_sample():
    """重复观测的置信度是**样本均值**，不是"取最后一次"。

    取最后一次会让一次 0.30 的低质量观测把一条观测了 10 次、均值 0.9 的边
    拉到 0.30 —— 边的可信度随"最近一帧模型心情"跳变。均值则单调稳定。

    这里 0.9 与 0.3 各一次 ⇒ 0.6。取最后一次会得 0.3，取第一次会得 0.9，
    三个数两两不等，判别力是三值的。
    """
    store = ObjectRelationStore(relation_min_confidence=0.1, relation_confirm_min_observations=1)
    _sync(
        store,
        [{"subject_id": "f1", "object_id": "f2", "relation": "near", "confidence": 0.9}],
    )
    _sync(
        store,
        [{"subject_id": "f1", "object_id": "f2", "relation": "near", "confidence": 0.3}],
        observation_id="o2",
        timestamp=2.0,
    )

    edge = _edge(store, ("obj_a", "near", "obj_b"))
    assert edge.observation_count == 2
    assert edge.confidence == pytest.approx(0.6), "不是样本均值（0.3=取最后 / 0.9=取第一）"


def test_the_mean_is_weighted_by_the_prior_count():
    """三次观测 0.9 / 0.6 / 0.3 ⇒ 0.6，逐次递推必须与算术平均一致。

    只做两次的话，"`prior_count` 用错（比如恒用 1）"这个 bug 恰好看不出来
    （两次时先验计数就是 1）。三次才分得开：正确 0.6，恒用 1 得 0.475。
    """
    store = ObjectRelationStore(relation_min_confidence=0.1, relation_confirm_min_observations=1)
    for index, confidence in enumerate((0.9, 0.6, 0.3)):
        _sync(
            store,
            [{
                "subject_id": "f1", "object_id": "f2",
                "relation": "near", "confidence": confidence,
            }],
            observation_id=f"o{index}",
            timestamp=float(index),
        )

    edge = _edge(store, ("obj_a", "near", "obj_b"))
    assert edge.observation_count == 3
    assert edge.confidence == pytest.approx(0.6)


def test_the_observation_id_list_is_deduplicated_but_the_count_still_rises():
    """同一个 `observation_id` 再来一次：计数照涨，id 列表**不**重复。

    两件事必须分开：计数是"这帧又说了一遍"（同一帧里模型重复输出是常态，
    重复计权会让置信度被同一帧灌水），id 列表是"有哪些帧见过"，重复的 id
    在里面没有信息量。合成一件事（`append` 不查重）会让 id 列表无限膨胀。
    """
    store = ObjectRelationStore(relation_min_confidence=0.1, relation_confirm_min_observations=1)
    relation = {"subject_id": "f1", "object_id": "f2", "relation": "near", "confidence": 0.5}

    _sync(store, [relation], observation_id="o1")
    _sync(store, [relation], observation_id="o2", timestamp=2.0)
    _sync(store, [relation], observation_id="o2", timestamp=3.0)

    edge = _edge(store, ("obj_a", "near", "obj_b"))
    assert edge.observation_count == 3
    assert edge.source_observation_ids == ["o1", "o2"]


def test_descriptions_are_deduplicated_and_capped_from_the_tail():
    """描述去重，且超上限时**丢最早**的那条（保留最近 N 条）。

    截尾方向不是随手定的：越近的描述越贴近物体当前的摆放，早期描述随着
    家具被挪动会变成错的。若改成 `[:max]`（留最早），一条边会永远挂着
    搬家前的旧描述。
    """
    store = ObjectRelationStore(
        relation_min_confidence=0.1, relation_confirm_min_observations=1,
        max_relation_descriptions=2,
    )
    for index, desc in enumerate(("一", "二", "三")):
        _sync(
            store,
            [{
                "subject_id": "f1", "object_id": "f2",
                "relation": "near", "confidence": 0.9, "description_zh": desc,
            }],
            observation_id=f"o{index}",
            timestamp=float(index),
        )

    edge = _edge(store, ("obj_a", "near", "obj_b"))
    assert edge.descriptions_zh == ["二", "三"], "超上限时必须丢最早的"


def test_a_repeated_description_is_not_appended_again():
    """同一句描述重复出现不重复入表 —— 否则上限会很快被同一句话占满。"""
    store = ObjectRelationStore(relation_confirm_min_observations=1)
    for index in range(3):
        _sync(
            store,
            [{
                "subject_id": "f1", "object_id": "f2",
                "relation": "near", "confidence": 0.9, "description_zh": "在椅子旁边",
            }],
            observation_id=f"o{index}",
            timestamp=float(index),
        )

    assert _edge(store, ("obj_a", "near", "obj_b")).descriptions_zh == ["在椅子旁边"]


def test_last_seen_advances_but_first_seen_does_not():
    """`first_seen` 只在建边那一次写；`last_seen` 每次观测都刷新。

    "这条边第一次是哪一帧见到的"是复盘起点，跟着观测一起往前跑就丢了。
    """
    store = ObjectRelationStore(relation_min_confidence=0.1, relation_confirm_min_observations=1)
    relation = {"subject_id": "f1", "object_id": "f2", "relation": "near", "confidence": 0.9}
    _sync(store, [relation], timestamp=100.0)
    _sync(store, [relation], observation_id="o2", timestamp=250.0)

    edge = _edge(store, ("obj_a", "near", "obj_b"))
    assert edge.first_seen == 100.0
    assert edge.last_seen == 250.0


# --------------------------------------------------------------------------
# E. 置信度地板与过期
# --------------------------------------------------------------------------

def test_the_confidence_floor_is_strictly_less_than():
    """地板是开区间下界：恰好等于地板**过**，少一点点拒。

    只测一侧的话，把 `<` 写成 `<=` 会静默丢掉所有"刚好卡在地板上"的关系
    —— 而 0.45 这种整值恰是模型最爱吐的数。
    """
    store = ObjectRelationStore(
        relation_min_confidence=DEFAULT_MIN_CONFIDENCE, relation_confirm_min_observations=1,
    )

    _sync(store, [
        {"subject_id": "f1", "object_id": "f2", "relation": "near",
         "confidence": DEFAULT_MIN_CONFIDENCE},
    ])
    assert list(store.relations.keys()) == [("obj_a", "near", "obj_b")], "恰好等于地板必须过"

    _sync(store, [
        {"subject_id": "f2", "object_id": "f1", "relation": "adjacent_to",
         "confidence": DEFAULT_MIN_CONFIDENCE - 0.0001},
    ], observation_id="o2", timestamp=2.0)
    assert len(store.relations) == 1, "低于地板必须拒"
    assert _reasons(store) == ["confidence_below_min"]


def test_a_missing_confidence_field_defaults_to_the_middle():
    """**默认值 ≠ 生效值**：`confidence` 字段缺失时取 `clamp01(None, 0.5)` = 0.5。

    0.5 高于 0.45 的地板，所以**没写置信度的关系会被收下**。这不是 bug，
    但它是"为什么这条边明明模型没说多确定却进来了"的唯一解释，值得钉住。
    顺带钉住 `clamp01` 的两端：越界值被夹进 [0,1] 而不是被拒或抛。
    """
    store = ObjectRelationStore(relation_confirm_min_observations=1)
    _sync(store, [{"subject_id": "f1", "object_id": "f2", "relation": "near"}])
    assert _edge(store, ("obj_a", "near", "obj_b")).confidence == pytest.approx(0.5)

    store = ObjectRelationStore(relation_min_confidence=0.1, relation_confirm_min_observations=1)
    _sync(store, [
        {"subject_id": "f1", "object_id": "f2", "relation": "near", "confidence": 7.0},
    ])
    assert _edge(store, ("obj_a", "near", "obj_b")).confidence == pytest.approx(1.0)


def test_mark_stale_only_touches_edges_older_than_the_window():
    """`relation_stale_after_seconds` 的**两侧**都必须钉住，含"恰好等于"那一侧。

    只断言"超时的会转 STALE"是不够的：把窗口判据写成恒真，整张图每次
    `mark_stale` 都全量转 STALE，而那条断言仍然绿。所以这里三段都测：
    窗口内不许动、**恰好等于窗口**要有确定答案、超出必须转。

    「恰好等于」落在**不转**这一侧（源码是严格 `>`）。这是**契约选择**，
    不是标定出来的事实 —— 写在这里是为了让下次有人改这个不等式时必须显式
    改一条测试，而不是悄悄挪动一整类边过期的时刻。
    """
    store = ObjectRelationStore(
        relation_min_confidence=0.1, relation_confirm_min_observations=1,
        relation_stale_after_seconds=100.0,
    )
    _sync(
        store,
        [{"subject_id": "f1", "object_id": "f2", "relation": "near", "confidence": 0.9}],
        timestamp=1000.0,
    )
    edge = _edge(store, ("obj_a", "near", "obj_b"))

    store.mark_stale(now=1050.0)  # 差 50s
    assert edge.status == RELATION_CONFIRMED, "窗口内不许转 STALE"

    store.mark_stale(now=1100.0)  # 差 100s，恰好等于窗口
    assert edge.status == RELATION_CONFIRMED, "恰好等于窗口仍算新鲜（严格 >）"

    store.mark_stale(now=1200.0)  # 差 200s
    assert edge.status == RELATION_STALE


def test_mark_stale_does_not_touch_the_last_seen_timestamp():
    """`mark_stale` 只改状态，不刷新 `last_seen`。

    顺手刷新会让 STALE 的边永远 STALE 不掉第二次、也永远查不出"它到底多久
    没被看到了" —— 而这个时长正是调窗口的唯一依据。
    """
    store = ObjectRelationStore(
        relation_min_confidence=0.1, relation_confirm_min_observations=1,
        relation_stale_after_seconds=10.0,
    )
    _sync(
        store,
        [{"subject_id": "f1", "object_id": "f2", "relation": "near", "confidence": 0.9}],
        timestamp=1000.0,
    )

    store.mark_stale(now=5000.0)

    edge = _edge(store, ("obj_a", "near", "obj_b"))
    assert edge.status == RELATION_STALE
    assert edge.last_seen == 1000.0


# --------------------------------------------------------------------------
# F. 与真实生产者的接线（钉住"两套图共用同一份关系库"）
# --------------------------------------------------------------------------

def test_the_navigation_graph_owns_a_store_and_projects_its_edges():
    """`SemanticNavigationGraph` 的关系边必须**取自**同一个 store，不许另建一份。

    这是 docstring 里"WebUI 与 CLI 离线路径看到的是同一份持久关系"的实现方式：
    类里持有 `relation_store`，投影时遍历 `relation_store.relations`。
    两边各建一份的话，WebUI 与离线报告会在同一次运行里给出不同的图。
    """
    from app.spatial.semantic_navigation_graph import SemanticNavigationGraph

    graph = SemanticNavigationGraph()
    assert isinstance(graph.relation_store, ObjectRelationStore)

    _sync(
        graph.relation_store,
        [{"subject_id": "f1", "object_id": "f2", "relation": "near", "confidence": 0.9}],
        observation_id="o1",
    )
    assert ("obj_a", "near", "obj_b") in graph.relation_store.relations
