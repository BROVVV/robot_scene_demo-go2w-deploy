"""结果职责分离：scene_objects / target_candidates / relation_anchors + 三态。

缺口（计划书 §5.1.2）：grounded_sam 那条路（正是网页默认）用
``target_present = bool(detections)`` 判目标，而 ``detections`` 里混着
**RAM 场景词**的框（`_detect_grounded_sam` 把场景词拼进了同一次请求）——
于是画面里随便有件东西，目标就算"存在"。同时 ``relation_anchors`` 这个字段
全仓不存在，且检测项与全场景物体项各带各的编号、对不上号。

本文件锁三件事：ID 稳定、职责拆分、三态判定。ID 那条是核心 ——
用序号做 ID 时，两次调用返回顺序一变同一个实体就换号，"对上号"无从谈起。
"""

from __future__ import annotations

import ast
import random
import textwrap
from pathlib import Path
from types import SimpleNamespace

import pytest

from app.perception.entity_roles import (
    ROLE_ANCHOR,
    ROLE_SCENE,
    ROLE_TARGET,
    assign_entity_ids,
    classify_roles,
    derive_target_state,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ENTRY = PROJECT_ROOT / "scripts" / "go2w" / "run_semantic_exploration.py"
ENTRY_TEXT = ENTRY.read_text(encoding="utf-8")


# --------------------------------------------------------------------------- #
# 夹具
# --------------------------------------------------------------------------- #
def _profile(
    *, direct=("垃圾桶", "trash bin"), anchors=("桌子", "table"), context=()
):
    from app.reasoning.target_profile import TargetProfile

    return TargetProfile(
        raw_query="垃圾桶",
        canonical_name_zh=direct[0],
        primary_labels_en=list(direct[1:]),
        relation_anchor_labels_en=list(anchors),
        context_labels_en=list(context),
    )


def _obj(label, score, bbox, term=None):
    item = {"label": label, "score": score, "bbox_2d": list(bbox)}
    item["source_prompt_term"] = term if term is not None else label
    return item


TRASH = _obj("green trash bin", 0.71, [0.10, 0.20, 0.30, 0.60], "trash bin")
CHAIR = _obj("office chair", 0.95, [0.55, 0.30, 0.80, 0.75], "chair")
TABLE = _obj("desk", 0.64, [0.32, 0.40, 0.54, 0.70], "table")


# --------------------------------------------------------------------------- #
# 1) 实体 ID：与顺序无关
# --------------------------------------------------------------------------- #
def test_ids_are_stable_when_the_detector_returns_objects_in_another_order():
    """核心契约：同一个实体，两次调用顺序不同，必须拿到同一个 ID。"""
    baseline = {item["label"]: item["entity_id"] for item in assign_entity_ids([TRASH, CHAIR, TABLE])}
    for seed in range(5):
        shuffled = [TRASH, CHAIR, TABLE]
        random.Random(seed).shuffle(shuffled)
        got = {item["label"]: item["entity_id"] for item in assign_entity_ids(shuffled)}
        assert got == baseline, f"seed={seed} 打乱顺序后 ID 变了：{got} != {baseline}"


def test_the_same_label_inside_a_slightly_jittered_box_keeps_one_id():
    """浮点末位抖动不该把同一件东西劈成两个实体。"""
    a = assign_entity_ids([_obj("chair", 0.9, [0.1, 0.2, 0.3, 0.4])])[0]
    b = assign_entity_ids([_obj("chair", 0.9, [0.1000004, 0.2, 0.3, 0.4])])[0]
    assert a["entity_id"] == b["entity_id"]


def test_different_boxes_get_different_ids():
    first = assign_entity_ids([TRASH])[0]["entity_id"]
    second = assign_entity_ids([CHAIR])[0]["entity_id"]
    assert first != second


def test_an_existing_id_is_not_overwritten():
    item = dict(TRASH, entity_id="det_keepme")
    assert assign_entity_ids([item])[0]["entity_id"] == "det_keepme"


def test_a_missing_or_malformed_box_still_gets_an_id():
    for bbox in (None, [0.1, 0.2], "nonsense"):
        item = {"label": "chair", "score": 0.5, "bbox_2d": bbox}
        assert assign_entity_ids([item])[0]["entity_id"].startswith("det_")


# --------------------------------------------------------------------------- #
# 2) 职责拆分
# --------------------------------------------------------------------------- #
def test_a_target_box_is_not_also_a_scene_object():
    """与既有 quick 契约一致：目标不混进场景物体。"""
    partition = classify_roles([TRASH, CHAIR], _profile())
    assert [i["label"] for i in partition.target_candidates] == ["green trash bin"]
    assert [i["label"] for i in partition.scene_objects] == ["office chair"]


def test_an_anchor_is_both_a_scene_object_and_an_anchor():
    """「垃圾桶旁边的椅子」里那个桌子是真实的场景物体，不能因为它是锚点就不进图。"""
    partition = classify_roles([TABLE, CHAIR], _profile())
    assert [i["label"] for i in partition.relation_anchors] == ["desk"]
    assert "desk" in [i["label"] for i in partition.scene_objects]


def test_roles_come_from_the_prompt_term_not_the_label():
    """分类依据是"这个框是哪个词生成的"，不是标签字面。

    同一个 label 由不同词生成时，职责不同 —— 这正是 `source_prompt_term`
    存在的意义，也是不能拿 label 做模糊匹配的原因。
    """
    as_target = _obj("object", 0.7, [0.1, 0.1, 0.2, 0.2], "trash bin")
    as_scene = _obj("object", 0.7, [0.1, 0.1, 0.2, 0.2], "floor")
    partition = classify_roles([as_target, as_scene], _profile())
    assert len(partition.target_candidates) == 1
    assert partition.target_candidates[0]["source_prompt_term"] == "trash bin"
    assert partition.scene_objects[0]["source_prompt_term"] == "floor"


def test_the_views_share_one_entity_id_for_the_same_box():
    partition = classify_roles([TABLE], _profile())
    scene_ids = {i["entity_id"] for i in partition.scene_objects}
    anchor_ids = {i["entity_id"] for i in partition.relation_anchors}
    assert scene_ids == anchor_ids and len(scene_ids) == 1


def test_roles_are_recorded_on_every_entity():
    partition = classify_roles([TRASH, TABLE, CHAIR], _profile())
    by_label = {
        i["label"]: i["roles"]
        for view in (
            partition.target_candidates,
            partition.scene_objects,
            partition.relation_anchors,
        )
        for i in view
    }
    assert by_label["green trash bin"] == [ROLE_TARGET]
    assert ROLE_ANCHOR in by_label["desk"]
    assert by_label["office chair"] == [ROLE_SCENE]


def test_without_a_profile_everything_is_a_scene_object():
    """没接线（profile=None）时不猜：不把任何东西当目标。"""
    partition = classify_roles([TRASH, CHAIR], None)
    assert partition.target_candidates == []
    assert len(partition.scene_objects) == 2


# --------------------------------------------------------------------------- #
# 3) 三态判定
# --------------------------------------------------------------------------- #
def test_a_scene_object_alone_does_not_make_the_target_present():
    """这条就是那个 bug 本身：画面里只有椅子，找的是垃圾桶，旧代码判 PRESENT。"""
    assert derive_target_state([CHAIR], _profile(), 0.15) == "ABSENT"
    assert bool([CHAIR]) is True  # 旧行为 `bool(detections)` 会给 PRESENT


def test_a_confident_target_candidate_is_present():
    assert derive_target_state([TRASH, CHAIR], _profile(), 0.15) == "PRESENT"


def test_a_target_candidate_below_the_threshold_is_possible_not_present():
    weak = _obj("trash bin", 0.08, [0.1, 0.1, 0.2, 0.2], "trash bin")
    assert derive_target_state([weak], _profile(), 0.15) == "POSSIBLE"


def test_possible_is_not_positive_evidence():
    """POSSIBLE 不能被当成"找到"——它是三态里刻意留的中间态。"""
    from app.perception.target_state import target_state_is_present

    weak = _obj("trash bin", 0.08, [0.1, 0.1, 0.2, 0.2], "trash bin")
    state = derive_target_state([weak], _profile(), 0.15)
    assert state == "POSSIBLE"
    assert target_state_is_present({"target_state": state, "objects": [weak]}) is False


def test_no_detections_at_all_is_absent():
    assert derive_target_state([], _profile(), 0.15) == "ABSENT"


def test_a_nan_score_does_not_sneak_past_the_threshold():
    """NaN 比较恒为 False，直接拿 max() 会把 NaN 当最大值抛出去。"""
    nan = _obj("trash bin", float("nan"), [0.1, 0.1, 0.2, 0.2], "trash bin")
    assert derive_target_state([nan], _profile(), 0.15) == "POSSIBLE"


# --------------------------------------------------------------------------- #
# 4) 装配：视图真的接到了 LiveObservation 上
# --------------------------------------------------------------------------- #
def _semantic_stub(objects):
    return SimpleNamespace(
        objects=list(objects),
        relations=[],
        scene_graph=None,
        timestamp_sec=1.0,
        frame_id="5465",
        target_state="ABSENT",
        heading_sector=3,
        semantic_source_frame_id="5465",
        semantic_capture_timestamp=1.0,
        semantic_completed_timestamp=1.1,
        semantic_age_ms=100.0,
        semantic_status="fresh",
        semantic_quality="full",
        semantic_error_code=None,
        semantic_error_detail=None,
        semantic_source_pose=None,
        source="grounded_sam_subprocess",
        timestamp_quality="ok",
    )


def _to_live(**overrides):
    from app.live_robot.semantic_observer import semantic_observation_to_live

    # `objects` 是替身 `SemanticObservation` 的入参，不是适配器的形参 ——
    # 必须先摘出来，否则它会连同 `kwargs` 一起传进 `semantic_observation_to_live`。
    objects = overrides.pop("objects", [])
    kwargs = dict(
        bundle_id="bundle_5465",
        detections=[TRASH, CHAIR],
        target_present=True,
        target_state="PRESENT",
        pose=None,
    )
    kwargs.update(overrides)
    return semantic_observation_to_live(_semantic_stub(objects), **kwargs)


def test_the_views_survive_the_adapter():
    obs = _to_live(target_candidates=[TRASH], relation_anchors=[TABLE])
    assert [i["label"] for i in obs.target_candidates] == ["green trash bin"]
    assert [i["label"] for i in obs.relation_anchors] == ["desk"]


def test_the_target_score_is_not_taken_from_scene_objects():
    """目标分此前是在**全部** detections 上取最大值 —— 那是画面里最显眼的
    那件东西的分，与目标无关。有了候选视图就必须只在候选里取。"""
    obs = _to_live(detections=[CHAIR, TRASH], target_candidates=[TRASH])
    assert obs.target_match["score"] == pytest.approx(0.71)


def test_the_adapter_still_falls_back_to_detections_without_candidates():
    # None means the legacy caller did not produce a candidate view. An
    # explicit [] is evidence that classification found no target.
    obs = _to_live(detections=[CHAIR, TRASH])
    assert obs.target_match["score"] == pytest.approx(0.95)


def test_the_adapter_does_not_replace_an_empty_candidate_verdict_with_a_chair():
    obs = _to_live(detections=[CHAIR, TRASH], target_candidates=[])
    assert obs.target_match["score"] == 0.0


def test_the_new_fields_round_trip_through_to_dict():
    from app.navigation.models import LiveObservation

    obs = _to_live(target_candidates=[TRASH], relation_anchors=[TABLE])
    restored = LiveObservation.from_dict(obs.to_dict())
    assert restored.target_candidates == obs.target_candidates
    assert restored.relation_anchors == obs.relation_anchors


def test_an_old_payload_without_the_new_keys_still_loads():
    """回放/存档里的旧 LiveObservation 没有这两个键，必须照常读进来。"""
    from app.navigation.models import LiveObservation

    restored = LiveObservation.from_dict({"bundle_id": "b", "timestamp": 1.0})
    assert restored.target_candidates == []
    assert restored.relation_anchors == []


# --------------------------------------------------------------------------- #
# 5) 入口真的接上了（AST 守卫：挡"新调用点没跟上"）
# --------------------------------------------------------------------------- #
def _observe_functions():
    tree = ast.parse(ENTRY_TEXT)
    found = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "observe"
    ]
    assert found, "入口文件里没有 observe —— 判据锚点失效"
    return found


def test_the_entry_feeds_the_two_views_into_the_adapter():
    calls = [
        node
        for fn in _observe_functions()
        for node in ast.walk(fn)
        if isinstance(node, ast.Call)
        and getattr(node.func, "id", None) == "semantic_observation_to_live"
    ]
    assert calls, "observe 里找不到 semantic_observation_to_live 调用"
    for call in calls:
        names = {kw.arg for kw in call.keywords}
        assert {"target_candidates", "relation_anchors"} <= names, (
            "调用点没把两份视图传下去 —— LiveObservation 上的字段会恒为空表，"
            "而消费方只能回退到旧的、把目标与场景混在一起的 detections"
        )


def test_the_entry_tells_the_adapter_which_frame_the_views_came_from():
    """视图的来源帧必须真的传下去。空串 = 下游只能靠 bbox 哈希猜同一实体。"""
    calls = [
        node
        for fn in _observe_functions()
        for node in ast.walk(fn)
        if isinstance(node, ast.Call)
        and getattr(node.func, "id", None) == "semantic_observation_to_live"
    ]
    assert calls, "observe 里找不到 semantic_observation_to_live 调用"
    for call in calls:
        assert "views_source_frame_id" in {kw.arg for kw in call.keywords}, (
            "调用点没把来源帧传下去 ⇒ `LiveObservation.views_source_frame_id` 恒为空，"
            "而空串的语义是“不知道” —— 谁都不能拿它判同一实体"
        )


def test_the_entry_derives_the_state_instead_of_bool_of_all_detections():
    guarded = []
    for fn in _observe_functions():
        for node in ast.walk(fn):
            if not isinstance(node, ast.If):
                continue
            test_src = ast.get_source_segment(ENTRY_TEXT, node.test) or ""
            if "go2w_result_role_separation" in test_src:
                guarded.append(node)
    assert guarded, "observe 里没有以 go2w_result_role_separation 为条件的判定块"
    for node in guarded:
        # 只看 if 分支：`orelse` 是同一个 If 节点的一部分，而旧判定正是**故意**
        # 留在 else 里当回退路径的（见下一条测试）。把整节点源码拿去断言，
        # 回退路径本身就会把这条判据染红 —— 那是判据写错了，不是代码错了。
        body_src = "\n".join(
            ast.get_source_segment(ENTRY_TEXT, stmt) or "" for stmt in node.body
        )
        assert "classify_roles" in body_src and "derive_target_state" in body_src, (
            "开关打开时没有走职责分离 —— 那这个开关就是空转的"
        )
        assert "target_present = bool(detections)" not in body_src, (
            "开关打开时仍用 bool(detections) 判目标"
        )


def test_the_legacy_path_is_still_there_for_rollback():
    """回退路径必须真的还在，否则"关掉开关"不是回退而是崩溃。"""
    assert "target_present = bool(detections)" in ENTRY_TEXT, (
        "旧判定被删掉了 —— 关掉 GO2W_RESULT_ROLE_SEPARATION 会直接炸"
    )


# --------------------------------------------------------------------------- #
# 6) 集成：跑**入口那段真语句**，再走真的适配器（审查 P1-c）
# --------------------------------------------------------------------------- #
# 上一节的守卫是 AST 字符串匹配，只能证明"字面上接了线"。审查 P1-c 指出的
# 正是这种"只在纯函数单测里成立"：模块的函数对，入口产出的三份视图在
# `LiveObservation` 上却没有共同 ID 契约。所以这一节把 `observe()` 里那段
# 真语句**取出来执行**（替身 settings/args/objects），再把产物喂给真的
# `semantic_observation_to_live`，从**实际产物**上核对 ID、源帧与入库形状。
def _role_separation_block() -> ast.If:
    """observe() 里以 `go2w_result_role_separation` 为条件的那个 if 块。"""
    for fn in _observe_functions():
        for node in ast.walk(fn):
            if not isinstance(node, ast.If):
                continue
            test_src = ast.get_source_segment(ENTRY_TEXT, node.test) or ""
            if "go2w_result_role_separation" in test_src:
                return node
    raise AssertionError("observe 里没有职责分离的判定块 —— 判据锚点失效")


def _run_entry_assembly(objects, *, frame_id="5465", score_min=0.5):
    """执行入口里那段真的装配语句，返回它写出来的局部变量。"""
    from app.perception.target_state import TARGET_ABSENT, TARGET_PRESENT

    block = _role_separation_block()
    body = "\n".join(
        textwrap.dedent(ast.get_source_segment(ENTRY_TEXT, stmt) or "")
        for stmt in block.body
    )
    namespace = {
        "classify_roles": classify_roles,
        "derive_target_state": derive_target_state,
        "TARGET_PRESENT": TARGET_PRESENT,
        "TARGET_ABSENT": TARGET_ABSENT,
        "objects": list(objects),
        "profile_now": _profile(),
        "views_source_frame_id": frame_id,
        "settings": SimpleNamespace(go2w_result_role_separation=True),
        "args": SimpleNamespace(target_score_min=score_min),
        # 块外已有的绑定：入口里它们是先初始化的默认值。
        "target_candidates": [],
        "relation_anchors": [],
        "detections": [],
        "target_present": False,
        "target_state": TARGET_ABSENT,
    }
    exec(compile(body, str(ENTRY), "exec"), namespace)  # noqa: S102 - 就是要跑真源码
    return namespace


def test_the_entrys_three_views_share_one_entity_id_per_box():
    """入口装配出来的三份视图必须**共用同一批 dict**（同一批 entity_id）。"""
    got = _run_entry_assembly([TRASH, CHAIR, TABLE])
    by_label = {item["label"]: item["entity_id"] for item in got["detections"]}
    assert by_label, "装配没产出 detections"

    for view in ("target_candidates", "relation_anchors"):
        for item in got[view]:
            assert item.get("entity_id") == by_label[item["label"]], (
                f"{view} 里的框与 detections 里同一件东西的 ID 不一致 —— "
                "三份视图没有共同 ID 契约"
            )
            assert item.get("entity_source_frame_id") == "5465", (
                f"{view} 没带上来源帧，跨帧就没法判同一实体"
            )
    # 目标候选是 detections 的**子集视图**，不是另一份拷贝。
    assert any(item is det for item in got["target_candidates"] for det in got["detections"]), (
        "候选视图是重建的拷贝，不是同一批 dict"
    )


def test_the_legacy_branch_still_strips_the_boxes_when_the_switch_is_off():
    """回退分支照旧只留 label/score/bbox —— 那条路本来就不带 ID。"""
    block = _role_separation_block()
    assert block.orelse, "职责分离的 if 没有 else 回退分支"
    orelse_src = "\n".join(
        ast.get_source_segment(ENTRY_TEXT, stmt) or "" for stmt in block.orelse
    )
    assert "detections.append" in orelse_src and "bool(detections)" in orelse_src


def test_the_observation_records_which_frame_each_view_came_from():
    """`LiveObservation` 上必须能读出"谁跟谁是同一帧的产物"（审查 P1-c）。"""
    got = _run_entry_assembly([TRASH, CHAIR, TABLE])
    obs = _to_live(
        # 后台全量检测的产物，与前景那次同帧（替身的 semantic_source_frame_id=5465）。
        objects=[CHAIR, TABLE],
        detections=got["detections"],
        target_candidates=got["target_candidates"],
        relation_anchors=got["relation_anchors"],
        views_source_frame_id=got["views_source_frame_id"],
    )
    assert obs.views_source_frame_id == "5465"
    assert obs.scene_objects_source_frame_id == "5465", (
        "scene_objects 的来源帧没记下来 —— 下游只能靠 bbox 哈希猜同一实体"
    )
    assert obs.to_dict()["views_source_frame_id"] == "5465"
    from app.navigation.models import LiveObservation
    assert LiveObservation.from_dict(obs.to_dict()).views_source_frame_id == "5465"


def test_the_same_id_in_two_different_frames_is_not_the_same_entity():
    """跨帧/跨检测的同一 ID **不**构成同一实体 —— 审查 P1-c 点名的那个假设。

    静止办公室里，上一帧的框和这一帧的框常常位位相同，于是 `entity_id`
    （量化框 + 标签的哈希）也一样。把它当"同一件东西"，就是把"位置上又出现了
    一个同标签的物体"当成了"我跟踪到了它"。判同一必须带源帧。
    """
    from app.perception.entity_roles import assign_entity_ids, entities_are_the_same

    frame_a = assign_entity_ids([CHAIR], source_frame_id="frame_a")[0]
    frame_b = assign_entity_ids([CHAIR], source_frame_id="frame_b")[0]
    assert frame_a["entity_id"] == frame_b["entity_id"], "前提：哈希只看框和标签"
    assert entities_are_the_same(frame_a, frame_b) is False, (
        "跨帧只凭哈希认了同一实体"
    )
    assert entities_are_the_same(frame_a, dict(frame_a)) is True
    # 源帧未知时**一律不认** —— 不知道就别认。
    assert entities_are_the_same(
        assign_entity_ids([CHAIR])[0], assign_entity_ids([CHAIR])[0]
    ) is False
