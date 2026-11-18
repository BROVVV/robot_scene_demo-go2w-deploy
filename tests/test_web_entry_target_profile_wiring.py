"""网页主入口 → 检测器的**目标画像**接线（计划书 §5.1.1）。

缺口（本轮才修）：``node._target_profile`` 只在
``run_autonomous_loop.py`` 的 ``_run_state_machine_search`` 里赋值，而那是
**CLI/legacy 专用**。网页路径是::

    run_go2w → _run_go2w_explorer → AutonomousExplorer(...).run()

它**从不经过**那个函数，于是 ``_ensure_gsam_detector`` 读到的
``getattr(self, "_target_profile", None)`` 恒为 ``None``，
``GroundedSAMSubprocessDetector(target_profile=None)``，
``_build_prompts`` 丢掉 ``detector_terms()`` / ``context_labels_en`` /
``relation_anchor_labels_en``。

这条缺陷**不会报错**：少几个词而已，日志里看不出来。所以判据不能靠读源码
找字符串，要**把入口那段真语句取出来执行**，用替身 node 看画像到底有没有
落到 ``node._target_profile`` 上。

``run_autonomous_loop.py`` 顶层 ``import rclpy``（:123），在宿主 .venv 里
import 不进来，所以本文件只 import ``run_semantic_exploration``（它的 rclpy
全在函数体内按需导入），对另一个文件只用 AST 读文本 —— 与
``tests/test_web_entry_frame_id_wiring.py`` 同一套路。
"""

from __future__ import annotations

import ast
import importlib.util
import sys
import textwrap
import types
from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ENTRY = PROJECT_ROOT / "scripts" / "go2w" / "run_semantic_exploration.py"
LOOP = PROJECT_ROOT / "scripts" / "go2w" / "run_autonomous_loop.py"
ENTRY_TEXT = ENTRY.read_text(encoding="utf-8")
LOOP_TEXT = LOOP.read_text(encoding="utf-8")

WIRING_SWITCH = "go2w_web_target_profile_wiring"
USE_LLM_SWITCH = "go2w_web_target_profile_use_llm"


# --------------------------------------------------------------------------- #
# 取入口里的真语句
# --------------------------------------------------------------------------- #
@pytest.fixture(scope="module")
def entry_ns() -> dict:
    spec = importlib.util.spec_from_file_location("_rse_under_test_profile", ENTRY)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return dict(vars(module))


def _explorer_statements() -> list[ast.stmt]:
    """``_run_go2w_explorer`` 的**直接**语句（不下钻进嵌套闭包）。

    只取直接语句是刻意的：接线块与画像解析都在这个函数的顶层。钻进嵌套函数
    会捞到同名局部（``observe`` 里也有自己的赋值），判据就对着另一份代码空转。
    """
    tree = ast.parse(ENTRY_TEXT)
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == "_run_go2w_explorer":
            return list(node.body)
    raise AssertionError(f"{ENTRY.name} 里没有 _run_go2w_explorer —— 判据锚点失效了")


def _source_of(node: ast.stmt) -> str:
    return textwrap.dedent(ast.get_source_segment(ENTRY_TEXT, node) or "")


def _wiring_block_source() -> str:
    """以 ``go2w_web_target_profile_wiring`` 为条件的那段接线块。"""
    for stmt in _explorer_statements():
        if isinstance(stmt, ast.If):
            test_src = ast.get_source_segment(ENTRY_TEXT, stmt.test) or ""
            if WIRING_SWITCH in test_src:
                return _source_of(stmt)
    raise AssertionError(
        f"_run_go2w_explorer 里找不到以 {WIRING_SWITCH} 为条件的接线块 —— "
        "判据锚点失效（接线被删或被挪进闭包了）"
    )


def _profile_resolution_source() -> str:
    """``profile = TargetProfileResolver().resolve(...)`` 那一条赋值。"""
    for stmt in _explorer_statements():
        if isinstance(stmt, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id == "profile" for t in stmt.targets
        ):
            return _source_of(stmt)
    raise AssertionError(
        "_run_go2w_explorer 里找不到 profile 的赋值语句 —— 判据锚点失效"
    )


# --------------------------------------------------------------------------- #
# 替身
# --------------------------------------------------------------------------- #
class _StubNode:
    """只记录"画像有没有落到我身上 / 检测器有没有被关掉"。"""

    def __init__(self) -> None:
        self._target_profile = None
        self.closed = 0

    def _close_detector(self) -> None:
        self.closed += 1


class _StubResolver:
    def __init__(self) -> None:
        self.calls: list[dict] = []

    def resolve(self, query, use_llm=True):
        self.calls.append({"query": query, "use_llm": use_llm})
        return f"profile::{query}::use_llm={use_llm}"


def _settings(*, wiring: bool, use_llm: bool) -> types.SimpleNamespace:
    return types.SimpleNamespace(
        **{WIRING_SWITCH: wiring, USE_LLM_SWITCH: use_llm}
    )


def _run_wiring_block(*, wiring: bool, profile: object):
    node = _StubNode()
    ns = {"settings": _settings(wiring=wiring, use_llm=False), "node": node,
          "profile": profile}
    exec(compile(_wiring_block_source(), "<wiring>", "exec"), ns)  # noqa: S102
    return node


def _run_profile_resolution(*, use_llm_setting: bool, target: str):
    resolver = _StubResolver()
    ns = {
        "TargetProfileResolver": lambda: resolver,
        "args": types.SimpleNamespace(target=target),
        "settings": _settings(wiring=True, use_llm=use_llm_setting),
    }
    exec(compile(_profile_resolution_source(), "<resolve>", "exec"), ns)  # noqa: S102
    return resolver, ns


# --------------------------------------------------------------------------- #
# 1) 接线真的把画像交给 node
# --------------------------------------------------------------------------- #
def test_the_wiring_block_hands_the_profile_to_the_node():
    sentinel = object()
    node = _run_wiring_block(wiring=True, profile=sentinel)
    assert node._target_profile is sentinel, (
        "接线块执行后 node._target_profile 不是解析出来的画像 —— 网页路径的"
        "检测器仍会拿到 None"
    )


def test_the_wiring_block_invalidates_a_cached_detector():
    """换目标必须重建：`_ensure_gsam_detector` 首次调用即缓存实例。"""
    node = _run_wiring_block(wiring=True, profile=object())
    assert node.closed >= 1, (
        "接线块没有关掉可能已缓存的检测器 —— 换目标会继续用上一轮的词表"
    )


def test_turning_the_switch_off_restores_the_old_behaviour():
    """回退路径：开关关掉就完全回到"不设画像"的旧行为。"""
    node = _run_wiring_block(wiring=False, profile=object())
    assert node._target_profile is None
    assert node.closed == 0


# --------------------------------------------------------------------------- #
# 2) use_llm 由开关决定（此前是硬编码 False）
# --------------------------------------------------------------------------- #
def test_the_profile_is_resolved_with_the_switch_not_a_hardcoded_value():
    _, ns = _run_profile_resolution(use_llm_setting=False, target="绿色垃圾桶")
    assert ns["profile"] == "profile::绿色垃圾桶::use_llm=False"

    resolver, ns_on = _run_profile_resolution(
        use_llm_setting=True, target="绿色垃圾桶"
    )
    assert ns_on["profile"] == "profile::绿色垃圾桶::use_llm=True"
    assert [c["use_llm"] for c in resolver.calls] == [True]


def test_the_target_argument_reaches_the_resolver():
    resolver, _ = _run_profile_resolution(use_llm_setting=False, target="椅子")
    assert resolver.calls == [{"query": "椅子", "use_llm": False}]


# --------------------------------------------------------------------------- #
# 3) 消费端：另一端确实把这个属性读出来传给检测器
# --------------------------------------------------------------------------- #
def _detector_ctor_profile_kwarg() -> str:
    tree = ast.parse(LOOP_TEXT)
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "_ensure_gsam_detector":
            for call in ast.walk(node):
                if not isinstance(call, ast.Call):
                    continue
                func = call.func
                name = func.id if isinstance(func, ast.Name) else getattr(func, "attr", "")
                if name != "GroundedSAMSubprocessDetector":
                    continue
                for kw in call.keywords:
                    if kw.arg == "target_profile":
                        return ast.get_source_segment(LOOP_TEXT, kw.value) or ""
    raise AssertionError(
        "run_autonomous_loop.py 的 _ensure_gsam_detector 里找不到把 "
        "target_profile= 传给检测器的构造调用"
    )


def test_the_consumer_reads_the_attribute_the_entry_writes():
    """两端必须是对同一个属性名。这一段就是原来那个 bug 的另一头。"""
    value_src = _detector_ctor_profile_kwarg()
    assert "_target_profile" in value_src, (
        f"检测器构造时传的 target_profile 不是从 _target_profile 读的（{value_src!r}）"
        " —— 接线写一个属性、消费读另一个属性"
    )
    assert "getattr" in value_src, (
        "应当用 getattr(..., None) 兜底：CLI 之外没有 __init__ 初始化这个属性"
    )


# --------------------------------------------------------------------------- #
# 4) 真正变的是什么：画像进了 prompt（跑真的 _build_prompts）
# --------------------------------------------------------------------------- #
def _prompts_for(profile):
    from app.config import get_settings
    from app.detectors.grounded_sam_subprocess import GroundedSAMSubprocessDetector

    detector = GroundedSAMSubprocessDetector(
        settings=get_settings(), target_profile=profile
    )
    return detector._build_prompts("绿色垃圾桶", ["green trash bin"])


def _anchor_profile():
    from app.reasoning.target_profile import TargetProfile

    # 上下文词刻意用一个**不会**从 dynamic_terms / base 词表里冒出来的串：
    # 判据要在"只有画像能带它进来"的词上做，否则测的是别的东西。
    return TargetProfile(
        raw_query="垃圾桶旁边的椅子",
        canonical_name_zh="椅子",
        primary_labels_en=["chair"],
        context_labels_en=["plastic stool"],
        relation_anchor_labels_en=["backpack"],
    )


def test_the_prompt_carries_the_anchor_and_context_terms_when_wired():
    joined = " ".join(_prompts_for(_anchor_profile())).lower()
    assert "chair" in joined
    assert "backpack" in joined, (
        "关系锚点词没进 prompt —— §15：锚点是关系目标的一半，必须一起搜"
    )
    assert "plastic stool" in joined, "上下文词没进 prompt"


def test_without_a_profile_those_terms_are_silently_absent():
    """把缺口本身钉住：没有画像时是**静默**降级，不报错、只是少词。"""
    joined = " ".join(_prompts_for(None)).lower()
    assert "backpack" not in joined
    assert "plastic stool" not in joined


# --------------------------------------------------------------------------- #
# 5) 可观测：生效值落在 detector_diagnostics 上
# --------------------------------------------------------------------------- #
def _detector_stub(*, profile, attempts=None, available=False, diagnostics=None):
    stub = types.SimpleNamespace(
        target_profile=profile,
        detect_attempts=list(attempts or []),
        last_diagnostics_available=available,
        last_diagnostics=diagnostics or {},
    )
    return stub


def test_the_payload_reports_an_absent_profile_explicitly():
    from app.detectors.grounded_sam_subprocess import diagnostics_event_payload

    payload = diagnostics_event_payload(
        _detector_stub(profile=None, available=True, diagnostics={"term_count": 19}),
        frame_id="5465",
    )
    assert payload["target_profile_present"] is False
    assert payload["detector_term_count"] == 0
    assert payload["target_profile_source"] is None


def test_the_payload_reports_the_effective_profile_values():
    from app.detectors.grounded_sam_subprocess import diagnostics_event_payload

    payload = diagnostics_event_payload(
        _detector_stub(
            profile=_anchor_profile(), available=True, diagnostics={"term_count": 19}
        ),
        frame_id="5465",
    )
    assert payload["target_profile_present"] is True
    assert payload["relation_anchor_term_count"] == 1
    assert payload["context_term_count"] == 1
    assert payload["target_profile_has_anchors"] is True
    assert payload["detector_term_count"] >= 1


def test_a_profile_alone_does_not_make_the_event_get_written():
    """画像字段只是搭车，不是新的发射理由。

    ``scripts/go2w/check_perception_rollback_session.sh`` 拿"本会话零条
    ``detector_diagnostics``"当"尚无法验证"的门（第四轮复审刚认证过
    "完全没诊断的帧整条事件都不写"）。如果加一个静态配置就能让事件发出来，
    那道门就再也触发不了 —— 等于偷偷放松了核验。
    """
    from app.detectors.grounded_sam_subprocess import diagnostics_event_payload

    payload = diagnostics_event_payload(_detector_stub(profile=_anchor_profile()))
    assert payload is None, (
        "没有 attempts 也没有 diagnostics 时仍写出了事件 —— 会架空回退核验脚本"
        "的'零条诊断'门"
    )
