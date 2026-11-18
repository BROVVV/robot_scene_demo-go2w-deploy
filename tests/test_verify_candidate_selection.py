"""复核候选选择（计划书 §5.1.3）：复核的必须是**目标候选**，不是画面里最显眼的东西。

缺口：入口里**两处** argmax 都在全量 ``detections`` 上取最高分 ——
``observe`` 里那个决定相机画面上锁哪个框，``verifier`` 里那个决定把哪个框
裁出来送 Verify。而 grounded_sam 那条路（网页默认）的 ``detections`` 里混着
RAM 场景词，于是一把 0.95 的椅子会把 0.32 的目标桶挤出复核位：裁的是椅子，
报出来的却是"目标确认为真"。

本文件既测纯函数 ``select_candidate``，也**执行入口里 verifier 的真源码**
（``_run_go2w_explorer`` 的闭包取出来单独跑）—— 只测纯函数的话，
"入口到底传没传"这一跳没人管，而那正是上一轮翻过车的地方。
"""

from __future__ import annotations

import ast
import importlib.util
import sys
import textwrap
from pathlib import Path

import pytest

from app.live_robot.verify_cache import VerificationCache
from app.navigation.models import LiveObservation
from app.perception.entity_roles import (
    VERIFY_SCOPE_ALL_DETECTIONS,
    VERIFY_SCOPE_TARGET_CANDIDATES,
    classify_roles,
    select_candidate,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE = PROJECT_ROOT / "scripts" / "go2w" / "run_semantic_exploration.py"
SOURCE_TEXT = SOURCE.read_text(encoding="utf-8")


# --------------------------------------------------------------------------- #
# 夹具
# --------------------------------------------------------------------------- #
def _bin(score=0.32):
    return {"label": "green trash bin", "score": score,
            "bbox_2d": [0.10, 0.20, 0.30, 0.60], "source_prompt_term": "trash bin"}


def _chair(score=0.95):
    return {"label": "office chair", "score": score,
            "bbox_2d": [0.55, 0.30, 0.80, 0.75], "source_prompt_term": "chair"}


# --------------------------------------------------------------------------- #
# 1) 纯函数
# --------------------------------------------------------------------------- #
def test_a_high_score_scene_object_does_not_win_the_slot():
    best, authoritative = select_candidate(
        [_bin()], [_chair(), _bin()],
        scope=VERIFY_SCOPE_TARGET_CANDIDATES, candidates_authoritative=True,
    )
    assert authoritative is True
    assert best is not None and best["label"] == "green trash bin"


def test_without_an_authoritative_view_the_old_behaviour_stands():
    """候选视图没产出时（职责分离没开）不能凭空拒绝——否则整个旧链路瘫掉。"""
    best, authoritative = select_candidate(
        [], [_chair(), _bin()],
        scope=VERIFY_SCOPE_TARGET_CANDIDATES, candidates_authoritative=False,
    )
    assert authoritative is False
    assert best is not None and best["label"] == "office chair"


def test_the_legacy_scope_ignores_the_candidate_view():
    best, authoritative = select_candidate(
        [_bin()], [_chair(), _bin()],
        scope=VERIFY_SCOPE_ALL_DETECTIONS, candidates_authoritative=True,
    )
    assert authoritative is False
    assert best is not None and best["label"] == "office chair"


def test_an_empty_authoritative_view_is_not_a_fallback_signal():
    """空表的两种含义必须分得开：这是"确实没有候选"，不是"没人产视图"。"""
    best, authoritative = select_candidate(
        [], [_chair()],
        scope=VERIFY_SCOPE_TARGET_CANDIDATES, candidates_authoritative=True,
    )
    assert best is None and authoritative is True


def test_a_misspelled_scope_prefers_the_new_semantics():
    """拼错的取值不能悄悄退回旧缺陷 —— 复核目标的语义才是本意。"""
    best, _ = select_candidate(
        [_bin()], [_chair(), _bin()],
        scope="Target_Candidates ", candidates_authoritative=True,
    )
    assert best is not None and best["label"] == "green trash bin"


def test_a_nan_score_does_not_win_the_argmax():
    """`max()` 遇到 NaN 会把它当最大值吐出来，于是"分数没算出来"的框反而中选。"""
    nan_bin = _bin(float("nan"))
    best, _ = select_candidate(
        [nan_bin, _bin(0.32)], [], candidates_authoritative=True,
    )
    assert best is not None and best["score"] == pytest.approx(0.32)


def test_a_candidate_without_a_score_field_does_not_crash():
    best, _ = select_candidate(
        [{"label": "bin", "bbox_2d": [0, 0, 1, 1]}], [],
        candidates_authoritative=True,
    )
    assert best is not None


def test_nothing_at_all_selects_nothing():
    assert select_candidate([], [], candidates_authoritative=True) == (None, True)
    assert select_candidate([], [], candidates_authoritative=False) == (None, False)


# --------------------------------------------------------------------------- #
# 2) 入口的 verifier 真源码
# --------------------------------------------------------------------------- #
def _entry_namespace() -> dict:
    spec = importlib.util.spec_from_file_location("_rse_verify_under_test", SOURCE)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return dict(vars(module))


def _verifier_source() -> str:
    """取实际裁剪/缓存函数，身份包装层由独立行为回归覆盖。

    真实入口由verifier包装层检查采集身份，_verify_candidate_unbound执行
    裁剪/缓存。保留正文锚点，避免抽到其他后端的同名函数。
    """
    for node in ast.walk(ast.parse(SOURCE_TEXT)):
        if isinstance(node, ast.FunctionDef) and node.name == "_verify_candidate_unbound":
            segment = ast.get_source_segment(SOURCE_TEXT, node)
            if "no detection to verify" in segment:
                return textwrap.dedent(segment)
    raise AssertionError("入口里找不到真 verifier —— 判据锚点失效了")


class _StubNode:
    def __init__(self) -> None:
        self.verify_calls: list[dict] = []

    def _verify_target(self, image_path, bbox, env, *, frame_id=None):
        self.verify_calls.append(
            {"image_path": image_path, "bbox": list(bbox), "frame_id": frame_id}
        )
        return {"is_target": True, "confidence": 0.9, "reason_zh": "ok"}


class _Args:
    target = "垃圾桶"
    verify_min_confidence = 0.5


class _Settings:
    def __init__(self, *, rolesep: bool, scope: str) -> None:
        self.go2w_result_role_separation = rolesep
        self.go2w_verify_candidate_scope = scope


def _run_verifier(*, observation, rolesep=True, scope=VERIFY_SCOPE_TARGET_CANDIDATES):
    ns = _entry_namespace()
    node = _StubNode()
    ns.update(
        node=node,
        args=_Args(),
        env={},
        settings=_Settings(rolesep=rolesep, scope=scope),
        verify_cache=VerificationCache(),
        # §5.1.5 的异步复核是 `verifier` **外层作用域**里的名字：置 None 就是
        # "开关关着"，`verifier` 逐字走旧的同步路径 —— 本文件测的就是那条路径。
        async_verifier=None,
    )
    exec(compile(_verifier_source(), str(SOURCE), "exec"), ns)
    outcome = ns["_verify_candidate_unbound"](observation, None)
    return outcome, node


def _observation(candidates, detections):
    return LiveObservation(
        bundle_id="bundle_5465",
        timestamp=1.0,
        image_ref="/tmp/frame_5465.jpg",
        detections=list(detections),
        target_candidates=list(candidates),
    )


def test_the_real_verifier_crops_the_target_candidate_not_the_chair():
    outcome, node = _run_verifier(
        observation=_observation([_bin()], [_chair(), _bin()]),
    )
    assert len(node.verify_calls) == 1
    assert node.verify_calls[0]["bbox"] == pytest.approx([0.10, 0.20, 0.30, 0.60])
    assert outcome.confirmed is True


def test_the_real_verifier_binds_the_frame_id_to_this_observation():
    _, node = _run_verifier(observation=_observation([_bin()], [_bin()]))
    assert node.verify_calls[0]["frame_id"] == "bundle_5465"


def test_scene_objects_alone_are_refused_and_the_reason_says_why():
    """§5.2 第 1 条：只有场景物体时不得裁框送验，也不得判成"找到"。"""
    outcome, node = _run_verifier(
        observation=_observation([], [_chair()]),
    )
    assert node.verify_calls == [], "明知没有目标候选还去裁场景物体复核"
    assert outcome.confirmed is False
    assert "目标候选" in outcome.reason_zh


def test_the_legacy_scope_still_crops_the_top_detection():
    _, node = _run_verifier(
        observation=_observation([_bin()], [_chair(), _bin()]),
        scope=VERIFY_SCOPE_ALL_DETECTIONS,
    )
    assert node.verify_calls[0]["bbox"] == pytest.approx([0.55, 0.30, 0.80, 0.75])


def test_with_role_separation_off_the_verifier_keeps_working():
    """回退路径：关掉开关后不得变成"永远拒绝复核"。"""
    _, node = _run_verifier(
        observation=_observation([], [_chair()]), rolesep=False,
    )
    assert len(node.verify_calls) == 1
    assert node.verify_calls[0]["bbox"] == pytest.approx([0.55, 0.30, 0.80, 0.75])


# --------------------------------------------------------------------------- #
# 3) 与任务 2 的接口：别名命中 / 锚点场景
# --------------------------------------------------------------------------- #
def _profile(*, canonical="椅子", direct=("chair",), anchors=("垃圾桶",), aliases=()):
    from app.reasoning.target_profile import TargetProfile

    return TargetProfile(
        raw_query=canonical,
        canonical_name_zh=canonical,
        primary_labels_en=list(direct),
        aliases_en=list(aliases),
        relation_anchor_labels_en=list(anchors),
    )


def test_a_candidate_hit_through_an_alias_is_still_a_candidate():
    alias_bin = {"label": "waste bin", "score": 0.4,
                 "bbox_2d": [0.1, 0.1, 0.2, 0.2], "source_prompt_term": "waste bin"}
    profile = _profile(canonical="垃圾桶", direct=("trash bin",), anchors=(),
                       aliases=("waste bin",))
    partition = classify_roles(
        [alias_bin, _chair()], profile
    )
    best, authoritative = select_candidate(
        partition.target_candidates, [_chair()],
        candidates_authoritative=True,
    )
    assert authoritative is True
    assert best is not None and best["label"] == "waste bin"


def test_the_anchor_never_steals_the_verification_slot():
    """「垃圾桶旁边的椅子」：复核的是**椅子**，垃圾桶只是关系锚点。"""
    anchor_bin = _bin(score=0.99)  # 锚点分更高，正因如此才要挡
    profile = _profile(canonical="椅子", direct=("chair",), anchors=("垃圾桶", "trash bin"))
    partition = classify_roles([anchor_bin, _chair(0.41)], profile)
    assert [i["label"] for i in partition.relation_anchors] == ["green trash bin"]

    _, node = _run_verifier(
        observation=_observation(partition.target_candidates, [anchor_bin, _chair(0.41)]),
    )
    assert node.verify_calls[0]["bbox"] == pytest.approx([0.55, 0.30, 0.80, 0.75])


# --------------------------------------------------------------------------- #
# 4) AST 守卫：第二个 argmax 点（相机锁框）也没漏
# --------------------------------------------------------------------------- #
def test_no_bare_argmax_over_detections_remains_in_the_entry():
    """挡"新调用点没跟上"：入口里不许再出现对 detections 的裸 argmax。"""
    offenders = []
    for node in ast.walk(ast.parse(SOURCE_TEXT)):
        if not isinstance(node, ast.Call):
            continue
        if getattr(node.func, "id", None) != "max":
            continue
        for arg in node.args:
            if isinstance(arg, ast.Attribute) and arg.attr == "detections":
                offenders.append(ast.get_source_segment(SOURCE_TEXT, node))
            elif isinstance(arg, ast.Name) and arg.id == "detections":
                offenders.append(ast.get_source_segment(SOURCE_TEXT, node))
    assert not offenders, f"仍有对全量 detections 的 argmax：{offenders}"


def test_all_three_call_sites_go_through_select_candidate():
    """锁框、裁剪与身份包装层都必须走同一个选择器。

    第二条（``observe`` 里派给 ``_feed_detection`` 即相机锁框的那个）**没法**
    行为化：它套在一个巨大的闭包里，单独执行不起来。所以这里退一步做结构守卫
    —— 报告里要如实写明这一条是结构判据、不是行为判据。计数而不是"存在"，
    是因为只断言"存在"的话，改掉其中一处仍会全绿。
    """
    owners = []
    def visit(node, owner=None):
        if isinstance(node, ast.FunctionDef):
            owner = node.name
        if isinstance(node, ast.Call) and getattr(node.func, "id", None) == "select_candidate":
            owners.append(owner)
        for child in ast.iter_child_nodes(node):
            visit(child, owner)
    visit(ast.parse(SOURCE_TEXT))
    assert sorted(owners) == sorted(["observe", "_verify_candidate_unbound", "verifier"])
