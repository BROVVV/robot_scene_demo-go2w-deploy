"""网页主入口 → detector 的**源帧号**接线（独立复审 2026-10-13 的 F1）。

上一轮把 R2（诊断事件绑定源帧）判为"已修"，判据却是给 helper 手工传
``frame_id="5465"`` —— 那只证明 helper 收得到，证明不了**入口真的传了**。
真实情况是：``run_autonomous_loop.py`` 里 5 个 ``_detect`` 调用点全传了，
而网页真正的主入口 ``run_semantic_exploration.py`` 里那 2 处
（后台 ``analyze_semantic``、前台 ``observe``）仍是三参调用，于是一整条
网页搜索链的 ``detector_diagnostics.frame_id`` 恒为空串。

本文件锁的是**入口那一跳**，并且刻意让判据"跑起来"而不是"读源码"：

* 两个闭包都在 ``_run_go2w_explorer`` 里，调不到；但本模块**能 import**
  （rclpy 全在函数体内按需导入），所以把闭包源码取出来单独执行，
  用替身 node 观察帧号到底有没有到达 detector 与事件。
* 另加一条 AST 守卫：入口文件里**任何** ``node._detect(...)`` 都必须带
  ``frame_id``。上一轮漏的正是"新调用点没跟上"，这条挡的就是再漏一次。
"""

from __future__ import annotations

import ast
import importlib.util
import sys
import textwrap
from pathlib import Path
from types import SimpleNamespace

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE = PROJECT_ROOT / "scripts" / "go2w" / "run_semantic_exploration.py"
SOURCE_TEXT = SOURCE.read_text(encoding="utf-8")


# --------------------------------------------------------------------------- #
# 载入真模块：取一份命名空间副本，供取出闭包源码后执行
# --------------------------------------------------------------------------- #
def _entry_namespace() -> dict:
    spec = importlib.util.spec_from_file_location(
        "_rse_under_test", SOURCE
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return dict(vars(module))


@pytest.fixture(scope="module")
def entry_ns() -> dict:
    return _entry_namespace()


def _function_source(name: str, *, must_contain: str = "") -> str:
    """取源码里 ``name`` 那个函数的**真语句**。

    ``must_contain`` 用来消歧：入口文件里 `observe` / `verifier` 各有**两个**
    同名嵌套定义（一个在 mock 后端、一个在真后端），只按名字取会取错，
    判据就变成对着另一份代码空转。给一个只可能出现在目标函数里的锚点即可。
    """
    tree = ast.parse(SOURCE_TEXT)
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            segment = ast.get_source_segment(SOURCE_TEXT, node)
            if must_contain and must_contain not in segment:
                continue
            return textwrap.dedent(segment)
    raise AssertionError(
        f"{name}(must_contain={must_contain!r}) 不在 {SOURCE.name} 里 —— 判据锚点失效了"
    )


# --------------------------------------------------------------------------- #
# 替身：只记录"谁拿到了什么帧号"
# --------------------------------------------------------------------------- #
class _StubNode:
    _detector = "grounded_sam"

    def __init__(self) -> None:
        self.detect_calls: list[dict] = []
        self.events: list[dict] = []

    def _detect(self, image_path, prompt, env, *, frame_id="", slots=None):
        # 替身必须跟着**外层作用域一起长**：入口给 `_detect` 加了 `slots`
        # （§5.1.4 的分频槽集），替身不收就会 TypeError，然后被 quick 段的
        # `except Exception` 吞成另一条与帧号无关的失败。
        self.detect_calls.append(
            {"image_path": image_path, "frame_id": frame_id, "slots": slots}
        )
        return [{"label": "垃圾桶", "score": 0.91, "bbox_2d": [0.1, 0.1, 0.4, 0.5]}]

    def _write(self, payload: dict) -> None:
        self.events.append(dict(payload))

    def _host_s(self) -> float:
        return 0.0


class _StubProfiler:
    def __init__(self) -> None:
        self.marks: list[str] = []

    def mark(self, name: str) -> None:
        self.marks.append(name)


# --------------------------------------------------------------------------- #
# 1) 接线守卫：入口文件里每一处 node.<detector 方法> 都必须带 frame_id
# --------------------------------------------------------------------------- #
def _entry_calls(attr: str) -> list[ast.Call]:
    tree = ast.parse(SOURCE_TEXT)
    calls = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if (
            isinstance(func, ast.Attribute)
            and func.attr == attr
            and isinstance(func.value, ast.Name)
            and func.value.id == "node"
        ):
            calls.append(node)
    return calls


def _missing_frame_id(attr: str) -> list[int]:
    return [
        call.lineno
        for call in _entry_calls(attr)
        if not any(kw.arg == "frame_id" for kw in call.keywords)
    ]


def test_the_entry_file_calls_detect_at_least_twice() -> None:
    """锚点自检：网页入口确实有前后台两处检测调用。

    少于两处说明抽取方式失效（例如调用被改名），后面的守卫会变成空转。
    """
    assert len(_entry_calls("_detect")) >= 2


def test_every_entry_detect_call_passes_a_frame_id() -> None:
    """**这条就是上一轮漏掉的那一跳。**

    三参调用（只传 image_path/prompt/env）时，`frame_id` 落到 `_detect` 的
    默认空串，worker 回填的诊断事件与请求体帧号全是空的 —— 事件还在、
    格式还对、只是帧号没用。改成必填 kwarg 后，任何新加的入口调用点
    漏传都会在这里变红。
    """
    missing = _missing_frame_id("_detect")
    assert not missing, (
        f"入口文件第 {missing} 行的 node._detect(...) 没传 frame_id —— "
        "这些调用的诊断事件帧号会恒为空串"
    )


def test_every_entry_verify_target_call_passes_a_frame_id() -> None:
    """Verify 这一跳同样要有帧号（复审 F1 明说"确需帧号的 Verify/事件优先用
    本次请求上下文"）。

    上一轮只盯 `_detect`，于是入口里 `verifier()` 那处 `_verify_target(...)`
    被漏掉，退回默认空串。它与 `_detect` 是**同类**缺陷，所以用同一条守卫
    一起挡：入口里任何 detector 调用点漏传帧号都变红。
    """
    missing = _missing_frame_id("_verify_target")
    assert not missing, (
        f"入口文件第 {missing} 行的 node._verify_target(...) 没传 frame_id —— "
        "Verify 请求里的帧号会退回默认空串"
    )


# --------------------------------------------------------------------------- #
# 2) 行为：后台语义闭包只复用前台同帧检测，不再重复调用 detector
# --------------------------------------------------------------------------- #
def _call_analyze_semantic(entry_ns: dict, node, state, frame_id):
    ns = dict(entry_ns)
    # 这三个在原文件里是 `_run_go2w_explorer` 的闭包变量，取出来的函数靠
    # 全局查找解析它们，所以补进命名空间即可。
    ns["node"] = node
    effective_frame = str(frame_id or state.get("frame_id", "semantic_live"))
    state.setdefault("local_detection_by_frame", {})[effective_frame] = {
        "objects": [
            {"label": "垃圾桶", "score": .91, "bbox_2d": [.1, .1, .4, .5]}
        ],
        "source_frame_id": effective_frame,
        "capture_timestamp": 1.0,
    }
    ns["state"] = state
    ns["prompt"] = "垃圾桶"
    ns["env"] = {}
    exec(  # noqa: S102 - 执行的是本仓源文件里的真函数
        _function_source("analyze_semantic"), ns
    )
    return ns["analyze_semantic"](
        "runtime/go2w/spool/bundle_1.jpg", None,
        frame_id=frame_id, robot_pose=None,
    )


def test_the_background_semantic_request_binds_its_own_frame(entry_ns) -> None:
    """后台语义：请求帧号 → 同帧缓存 → 记录里的源帧字段。

    跑真源码、给两个不同帧号，两处必须**各自**对上（前后台不同帧时不能串）。
    """
    node = _StubNode()
    state = {"frame_id": "state_never_used"}

    first = _call_analyze_semantic(entry_ns, node, state, "bg_100")
    second = _call_analyze_semantic(entry_ns, node, state, "bg_200")

    assert node.detect_calls == []
    assert first["frame_id"] == "bg_100"
    assert first["semantic_source_frame_id"] == "bg_100"
    assert second["frame_id"] == "bg_200"
    assert second["semantic_source_frame_id"] == "bg_200"


def test_the_background_semantic_request_falls_back_to_the_session_frame(
    entry_ns,
) -> None:
    """调用点没给帧号时才回落到会话状态 —— 且交出去的与记下来的必须是同一个值。

    两处各算各的（比如 detector 拿回落值、记录拿空串）会长成"事件有帧号、
    请求没有"的错帧，正是 R2 要修的那一类。
    """
    node = _StubNode()
    result = _call_analyze_semantic(entry_ns, node, {}, None)

    assert node.detect_calls == []
    assert result["frame_id"] == "semantic_live"
    assert result["semantic_source_frame_id"] == "semantic_live"


def test_the_background_semantic_request_prefers_the_caller_frame(entry_ns) -> None:
    """给了帧号就不能被会话状态盖掉（`or` 的两边顺序写反就会这样）。"""
    node = _StubNode()
    result = _call_analyze_semantic(entry_ns, node, {"frame_id": "sess_9"}, "bg_1")

    assert node.detect_calls == []
    assert result["frame_id"] == "bg_1"


# --------------------------------------------------------------------------- #
# 3) 行为：前台观察闭包（observe）的 quick 段把本次观测帧号交给 detector 与事件
# --------------------------------------------------------------------------- #
_QUICK_START = 'profiler.mark("quick_start")'
_QUICK_END = 'quick_latency_ms = (time.perf_counter() - quick_start_perf)'


def _quick_block_source() -> str:
    """取出 `observe` 里 quick 检测那一段的**真实语句**。

    闭包（970 行）整体跑不起来，但这一段基本自足：它只用 node / state /
    profiler / stable_image / prompt / env / frame_id，外加检测分频（§5.1.4）
    留在外层作用域的三个只读名字（见 `_CADENCE_OFF`）。取真源码而不是重写
    一份，是为了让"改了源码、判据没跟上"这种情况直接炸出来。
    """
    if _QUICK_START not in SOURCE_TEXT or _QUICK_END not in SOURCE_TEXT:
        raise AssertionError("quick 段的锚点不在源文件里了 —— 判据需要跟着改")
    # 从**行首**起切：锚点是行中间的一段文本，直接 index 会丢掉这一行的缩进，
    # 后面 textwrap.dedent 就找不到公共缩进、拼出来的函数体语法不合法。
    start = SOURCE_TEXT.rindex("\n", 0, SOURCE_TEXT.index(_QUICK_START)) + 1
    end = SOURCE_TEXT.rindex("\n", 0, SOURCE_TEXT.index(_QUICK_END)) + 1
    return textwrap.indent(textwrap.dedent(SOURCE_TEXT[start:end]), "    ")


# 检测分频（§5.1.4）在 quick 段**之前**决策，结果以一组只读名字进到段内：
# `slots`（这一轮跑哪些词槽）、`reuse_detection`（复用上一轮产物）、
# `cadence_entries`/`cadence_ttl_by_slot`（按槽保留的产物缓存）与 `settings`。
# 这里给的是"分频关闭"的取值，段的行为就与改造前逐字一致 —— 本文件测的是
# 帧号接线，不是分频。分频自己的判据在 `tests/test_detection_scheduler.py`。
def _cadence_off_namespace() -> dict:
    from app.perception.detection_scheduler import (
        ALL_SLOTS,
        SlotAssembly,
        SlotCacheEntry,
        assemble_slot_objects,
        put_slot_entry,
    )

    return {
        "slots": None,
        "reuse_detection": False,
        "cadence_entries": [],
        "cadence_ttl_by_slot": {},
        "settings": SimpleNamespace(go2w_detection_cadence_enabled=False),
        "ALL_SLOTS": ALL_SLOTS,
        "SlotAssembly": SlotAssembly,
        "SlotCacheEntry": SlotCacheEntry,
        "assemble_slot_objects": assemble_slot_objects,
        "put_slot_entry": put_slot_entry,
    }


def _run_quick_block(entry_ns: dict, node, state, frame_id, image="frame.jpg"):
    ns = dict(entry_ns)
    ns["time"] = __import__("time")
    ns.update(_cadence_off_namespace())
    wrapper = (
        "def _quick_block(node, state, profiler, stable_image, prompt, env, frame_id):\n"
        + _quick_block_source()
        + "    return objects\n"
    )
    exec(wrapper, ns)  # noqa: S102 - 包的是本仓源文件里的真语句
    profiler = _StubProfiler()
    return ns["_quick_block"](
        node, state, profiler, image, "垃圾桶", {}, frame_id
    )


def test_the_foreground_observation_binds_its_own_frame(entry_ns) -> None:
    """前台 quick：同一次观测的帧号要同时出现在**事件**和 **detector 请求**里。"""
    node = _StubNode()
    state: dict = {}

    _run_quick_block(entry_ns, node, state, "fg_11")
    _run_quick_block(entry_ns, node, state, "fg_22")

    assert [c["frame_id"] for c in node.detect_calls] == ["fg_11", "fg_22"]
    started = [e for e in node.events if e.get("event") == "quick_vlm_started"]
    assert [e["frame_id"] for e in started] == ["fg_11", "fg_22"]
    assert node.detect_calls[0]["frame_id"] == started[0]["frame_id"]


def test_the_quick_retry_event_carries_both_frames(entry_ns) -> None:
    """补跑那一次要同时说清"这一次是哪帧""上次失败的是哪帧"。

    只有一个 retry 记号、没有帧号，事后查不出补的是哪一帧（§16 的原始诉求）。
    """
    node = _StubNode()
    state = {"quick_vlm_last_failure": {"frame_id": "fg_10", "code": "X"}}

    _run_quick_block(entry_ns, node, state, "fg_11")

    retry = [e for e in node.events if e.get("event") == "quick_vlm_retry"]
    assert len(retry) == 1
    assert retry[0]["frame_id"] == "fg_11"
    assert retry[0]["previous_frame_id"] == "fg_10"
    assert node.detect_calls[-1]["frame_id"] == "fg_11"


# --------------------------------------------------------------------------- #
# 4) 行为：验证闭包（verifier）把本次观测自己的身份交给 Verify 请求
# --------------------------------------------------------------------------- #
class _VerifyStubNode:
    def __init__(self) -> None:
        self.verify_calls: list[dict] = []

    def _verify_target(self, image_path, bbox, env, *, frame_id=""):
        self.verify_calls.append(
            {"image_path": image_path, "bbox": list(bbox), "frame_id": frame_id}
        )
        return {"is_target": True, "confidence": 0.92, "reason_zh": "像垃圾桶"}


class _ArgsStub:
    target = "垃圾桶"
    verify_min_confidence = 0.5


class _SettingsStub:
    """verifier 闭包读的那两个开关（阶段二 §5.1.3）。

    这里取**回退口径**：职责分离关、复核池为全量 detections —— 即本文件
    测试的 F1（帧号绑定）与候选选择无关，不该被后者的默认值牵动。
    """

    go2w_result_role_separation = False
    go2w_verify_candidate_scope = "all_detections"


class _ObservationStub:
    """只带 verifier 真正用到的字段（够用即可，不模拟整个 LiveObservation）。"""

    def __init__(self, bundle_id: str, image_ref: str = "runtime/obs.jpg") -> None:
        self.bundle_id = bundle_id
        self.image_ref = image_ref
        self.detections = [
            {"label": "垃圾桶", "score": 0.9, "bbox_2d": [0.10, 0.10, 0.40, 0.50]}
        ]
        # 阶段二 §5.1.2 之后 LiveObservation 多了这份视图。stub 必须跟着长，
        # 否则测的是"这个替身缺字段"，不是帧号接线。
        self.target_candidates: list[dict] = []


def _run_verifier(entry_ns: dict, node, observation, cache=None):
    """exec 入口文件里 `verifier` 闭包的**真源码**，返回结果与它用的缓存。

    同名函数在文件里有两处（mock 后端一处、真后端一处），用
    `must_contain="node._verify_target"` 只取真后端那一处。

    `cache` 可传入以跨多次调用复用 —— 真代码里 `verify_cache` 是会话级对象，
    不复用就测不到"同帧不重复验"。
    """
    ns = dict(entry_ns)
    ns["node"] = node
    ns["args"] = _ArgsStub()
    ns["env"] = {}
    ns["settings"] = _SettingsStub()
    ns["verify_cache"] = cache if cache is not None else entry_ns["VerificationCache"]()
    # §5.1.5 的异步复核是 `verifier` **外层作用域**里的名字。置 None = 开关关着，
    # `verifier` 逐字走旧的同步路径 —— 本文件测的是帧号接线，不是异步。
    ns["async_verifier"] = None
    exec(  # noqa: S102 - 执行的是本仓源文件里的真函数
        _function_source("verifier", must_contain="node._verify_target"), ns
    )
    return ns["verifier"](observation, None), ns["verify_cache"]


def test_the_verify_request_binds_the_observations_own_frame(entry_ns) -> None:
    """Verify 请求带的帧号必须是**这一次观测自己的** bundle_id。

    两帧同一位置、同一个框（分数也一样）时，若帧号取的是"最近一帧"那个共享
    字段，两次会写出同一个帧号，事后根本分不清验的是哪一帧。
    """
    node = _VerifyStubNode()

    _run_verifier(entry_ns, node, _ObservationStub("bundle_aaa"))
    _run_verifier(entry_ns, node, _ObservationStub("bundle_bbb"))

    assert [c["frame_id"] for c in node.verify_calls] == [
        "bundle_aaa",
        "bundle_bbb",
    ]
    # 框与图像路径两帧相同，唯一变的就是帧号 —— 证明分辨力来自帧号本身。
    assert node.verify_calls[0]["bbox"] == node.verify_calls[1]["bbox"]
    assert node.verify_calls[0]["image_path"] == node.verify_calls[1]["image_path"]


def test_a_repeated_verify_on_the_same_frame_does_not_reach_the_detector(
    entry_ns,
) -> None:
    """同一帧同一框重复验证走缓存，不再打 detector。

    缓存的键是 `(bundle_id, target, bbox)` —— **正是**上面那条测试要求下传的
    那个身份。所以这条是它的配套：证明"帧号"在这个入口里不只是个日志字段，
    而是验证语义的一部分（§16 的同帧验证缓存）。诚实地说：它的键取自
    `observation.bundle_id` 而不是入参 `frame_id`，因此把 `frame_id` 传成
    空串时**这条仍是绿的**，抓到那种错的是上一条测试。
    """
    node = _VerifyStubNode()
    observation = _ObservationStub("bundle_aaa")
    cache = entry_ns["VerificationCache"]()

    _run_verifier(entry_ns, node, observation, cache)
    outcome, cache = _run_verifier(entry_ns, node, observation, cache)

    assert len(node.verify_calls) == 1, "同帧第二次验证应走缓存"
    assert outcome.confirmed is True
    assert len(cache) == 1
