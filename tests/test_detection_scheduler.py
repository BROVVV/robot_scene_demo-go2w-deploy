"""检测分频（计划书 §5.1.4）：这一轮该跑哪些词槽。

缺口：`observe()` 每轮都拿整套 prompt 打一次 GroundingDINO，全无节流 ——
只有 RAM++ 的 tag 是 1.8 s 一次。而"站着没动、朝向没变、地点没变"的时候，
场景词那半张词表的结果不会变。

本文件分三层：

1. 纯函数 `decide_detection_refresh`：`now` 注入，逐条判到点/没到点。
2. 真 `_build_prompts` 的词槽切分：只开场景槽时**目标词必须真的不在** prompt 里
   —— 这是分频唯一能省钱的地方，切不动就等于没做。
3. 缓存键：同一个 prompt 的"只跑场景槽"与"全跑"必须是两个键。
"""

from __future__ import annotations

import ast
import os
import random
import textwrap
from pathlib import Path

import pytest

from app.config import Settings
from app.detectors.backend import is_local_structured_backend
from app.detectors.grounded_sam_subprocess import (
    GroundedSAMSubprocessDetector,
    detection_cache_key,
)
from app.perception.detection_scheduler import (
    DetectionScheduler,
    SLOT_ANCHOR,
    SLOT_SCENE,
    SLOT_TARGET,
    decide_detection_refresh,
)
from app.reasoning.target_profile import TargetProfile

FROZEN = (
    Path(__file__).resolve().parents[1]
    / "artifacts" / "perception_profile_20261013" / "frames"
)


# --------------------------------------------------------------------------- #
# 1) 纯函数
# --------------------------------------------------------------------------- #
def test_unified_scheduler_initial_time_translation_rotation_and_debounce():
    scheduler = DetectionScheduler(
        interval_s=3.0,
        translation_refresh_m=.8,
        rotation_refresh_deg=30.,
        min_interval_s=1.,
    )
    initial = scheduler.evaluate(now=0., pose=(0., 0., 0.))
    assert initial.should_detect and initial.reason == "initial"
    scheduler.mark_detected(now=0., pose=(0., 0., 0.))

    assert not scheduler.evaluate(now=.5, pose=(.4, 0., 0.)).should_detect
    translation = scheduler.evaluate(now=1.1, pose=(.8, 0., 0.))
    assert translation.should_detect and "translation" in translation.reasons
    scheduler.mark_detected(now=1.1, pose=(.8, 0., 0.))

    scheduler.evaluate(now=1.5, pose=(.8, 0., .3))
    rotation = scheduler.evaluate(now=2.2, pose=(.8, 0., .55))
    assert rotation.should_detect and "rotation" in rotation.reasons
    scheduler.mark_detected(now=2.2, pose=(.8, 0., .55))

    time_due = scheduler.evaluate(now=5.2, pose=(.8, 0., .55))
    assert time_due.should_detect and time_due.reason == "time"


def test_unified_scheduler_debounces_simultaneous_critical_events():
    scheduler = DetectionScheduler(min_interval_s=1.)
    scheduler.mark_detected(now=10., pose=(0., 0., 0.))
    decision = scheduler.evaluate(
        now=10.2, pose=(1., 0., 1.), event="nav_failure"
    )
    assert not decision.should_detect
    assert decision.debounced is True
    assert "nav_failure" in decision.reasons


@pytest.mark.parametrize("backend", ["grounded_sam", "yoloe_seg"])
def test_both_local_backends_share_unified_schedule_and_single_frame_reuse(backend):
    assert is_local_structured_backend(backend)
    scheduler = DetectionScheduler()
    first = scheduler.evaluate(now=0., pose=(0., 0., 0.))
    assert first.should_detect
    scheduler.mark_detected(now=0., pose=(0., 0., 0.))
    cached = [{"id": "det_1", "label": "bin", "detection_slots": ["target", "scene"]}]
    target_view = [item for item in cached if "target" in item["detection_slots"]]
    scene_view = [item for item in cached if "scene" in item["detection_slots"]]
    assert target_view[0] is scene_view[0]
    assert not scheduler.evaluate(now=.5, pose=(0., 0., 0.)).should_detect


def _decide(**overrides):
    kwargs = dict(
        now=100.0,
        last_target_s=100.0,
        last_anchor_s=100.0,
        last_scene_s=100.0,
        ttl_target_s=0.0,
        ttl_anchor_s=0.0,
        ttl_scene_s=4.0,
        scene_max_stale_s=20.0,
        place_id="p1",
        last_place_id="p1",
        sector=3,
        last_sector=3,
        translation_m=0.0,
        translation_refresh_m=0.25,
    )
    kwargs.update(overrides)
    return decide_detection_refresh(**kwargs)


def test_cold_start_refreshes_everything():
    decision = _decide(last_target_s=None, last_anchor_s=None, last_scene_s=None)
    assert decision.slots == frozenset({SLOT_TARGET, SLOT_ANCHOR, SLOT_SCENE})
    assert "cold_start" in decision.reasons


def test_a_ttl_that_has_not_elapsed_does_not_refresh():
    decision = _decide(last_scene_s=100.0, now=102.0, ttl_scene_s=4.0)
    assert decision.refresh_scene is False
    # 目标槽与锚点槽的默认 TTL 都是 0 ⇒ 每轮都刷（只有场景槽被节流）。
    assert decision.slots == frozenset({SLOT_TARGET, SLOT_ANCHOR})


def test_a_ttl_that_has_elapsed_refreshes():
    decision = _decide(last_scene_s=100.0, now=104.0, ttl_scene_s=4.0)
    assert decision.refresh_scene is True
    assert "scene_ttl" in decision.reasons


def test_target_cadence_is_independent_of_scene_cadence():
    """两类时间戳分开记、分别判 —— 共用一个时间戳就退化成"一起刷/一起不刷"。"""
    decision = _decide(
        last_target_s=100.0, last_scene_s=100.0, now=101.0,
        ttl_target_s=0.5, ttl_scene_s=10.0,
    )
    assert decision.refresh_target is True
    assert decision.refresh_scene is False


@pytest.mark.parametrize(
    "overrides, expected_reason",
    [
        ({"place_id": "p2", "last_place_id": "p1"}, "place_changed"),
        ({"sector": 5, "last_sector": 3}, "sector_changed"),
        ({"translation_m": 0.4}, "translation"),
    ],
)
def test_a_changed_view_forces_every_slot(overrides, expected_reason):
    """换了地方/朝向还沿用旧结果，就是拿旧视角的词表解释新画面。"""
    decision = _decide(
        last_target_s=100.0, last_anchor_s=100.0, last_scene_s=100.0,
        now=100.1, ttl_scene_s=600.0, **overrides,
    )
    assert expected_reason in decision.reasons
    assert decision.slots == frozenset({SLOT_TARGET, SLOT_ANCHOR, SLOT_SCENE})


def test_a_place_switch_refreshes_the_scene_even_when_nothing_else_is_due():
    decision = _decide(
        last_scene_s=100.0, now=100.2, ttl_scene_s=600.0,
        place_id="kitchen", last_place_id="hallway",
    )
    assert decision.refresh_scene is True


def test_starvation_max_wait_forces_a_scene_refresh():
    """运营可以把场景 TTL 调得很大（"只在换地方时刷"），但到硬顶必须刷一次。"""
    decision = _decide(
        last_scene_s=100.0, now=125.0,
        ttl_scene_s=1000.0, scene_max_stale_s=20.0,
    )
    assert decision.refresh_scene is True
    assert decision.forced_scene is True
    assert decision.reasons[-1] == "scene_max_stale"


def test_the_max_wait_is_reported_with_its_actual_duration():
    """§5.2：被硬顶逼出来时要记下**实际等了多久**，否则只能靠推断。"""
    decision = _decide(
        last_scene_s=100.0, now=137.5,
        ttl_scene_s=1000.0, scene_max_stale_s=20.0,
    )
    assert decision.scene_wait_s == pytest.approx(37.5)


def test_below_the_max_wait_nothing_is_forced():
    decision = _decide(
        last_scene_s=100.0, now=105.0,
        ttl_scene_s=1000.0, scene_max_stale_s=20.0,
    )
    assert decision.refresh_scene is False
    assert decision.forced_scene is False
    assert decision.scene_wait_s is None


def test_the_max_wait_is_not_reported_on_the_view_change_path():
    """视图变了不是被"逼"的，别把两种成因混成一个字段。"""
    decision = _decide(
        last_scene_s=100.0, now=137.5, ttl_scene_s=1000.0,
        place_id="p2", last_place_id="p1",
    )
    assert decision.refresh_scene is True
    assert decision.forced_scene is False
    assert decision.scene_wait_s is None


def test_a_zero_ttl_means_every_cycle():
    decision = _decide(last_scene_s=100.0, now=100.0, ttl_scene_s=0.0)
    assert decision.refresh_scene is True


def test_no_slot_due_yields_an_empty_slot_set():
    decision = _decide(
        now=100.5, ttl_target_s=10.0, ttl_anchor_s=10.0, ttl_scene_s=10.0,
    )
    assert decision.slots == frozenset()


def test_a_disabled_translation_gate_does_not_force_a_refresh():
    """`translation_refresh_m <= 0` = 关掉位移判据；`0.0 >= 0` 不能变成恒真。"""
    decision = _decide(
        now=100.1, ttl_scene_s=600.0, translation_m=0.0, translation_refresh_m=0.0,
    )
    assert decision.refresh_scene is False


def test_a_first_cycle_of_silence_does_not_loop_forever():
    """模拟 30 轮：target 每轮刷、scene 按 TTL 刷 —— 两个计数必须**不同**。

    如果分频实现成"共用一个时间戳"，这里两个计数会相等，
    而那正是"分频看起来生效、实际一点没省"的样子。
    """
    target_count = 0
    scene_count = 0
    last_target_s = last_scene_s = None
    for step in range(30):
        now = 100.0 + step
        decision = decide_detection_refresh(
            now=now,
            last_target_s=last_target_s,
            last_anchor_s=last_target_s,
            last_scene_s=last_scene_s,
            ttl_target_s=0.0,
            ttl_anchor_s=0.0,
            ttl_scene_s=4.0,
            scene_max_stale_s=20.0,
        )
        if decision.refresh_target:
            target_count += 1
            last_target_s = now  # 只有真跑了才推进时间戳
        if decision.refresh_scene:
            scene_count += 1
            last_scene_s = now
    assert target_count == 30
    assert scene_count == 8  # 冷启动 + 每 4 秒一次
    assert target_count != scene_count


# --------------------------------------------------------------------------- #
# 2) 真 _build_prompts 的词槽切分
# --------------------------------------------------------------------------- #
def _profile() -> TargetProfile:
    return TargetProfile(
        raw_query="垃圾桶旁边的椅子",
        canonical_name_zh="椅子",
        primary_labels_en=["chair"],
        aliases_en=["seat"],
        context_labels_en=["backpack"],
        # 第二个锚点词刻意取 static base 词表里没有的：base 词表属于**场景槽**，
        # 拿 "trash bin"（base 里有）当"锚点槽独有的判据"会误判。
        relation_anchor_labels_en=["trash bin", "waste receptacle"],
    )


def _detector() -> GroundedSAMSubprocessDetector:
    return GroundedSAMSubprocessDetector(
        settings=Settings(
            siliconflow_api_key="",
            grounding_prompt_llm_expansion_enabled=False,
            static_object_prompts_enabled=True,
        ),
        target_profile=_profile(),
    )


def _joined(prompts) -> str:
    return " || ".join(prompts)


def test_all_slots_is_byte_identical_to_the_old_call():
    """`slots=None` 必须与改造前逐字一致 —— 否则默认关闭的开关也在改行为。"""
    detector = _detector()
    assert detector._build_prompts("椅子", ["chair"], ["stool"]) == \
        detector._build_prompts("椅子", ["chair"], ["stool"], slots=None)


def test_the_scene_slot_alone_carries_no_target_terms():
    """只开场景槽时，**画像里的**目标/锚点/上下文词一个都不该出现。

    判据刻意用画像独有的词（`seat` = 别名、`trash bin` = 锚点），不用
    "chair"：static base 词表里本来就有 chair，而 base 词表属于场景槽，
    拿它当判据会把正确行为判成错。
    """
    detector = _detector()
    joined = _joined(
        detector._build_prompts("椅子", ["chair"], ["stool"], slots=frozenset({SLOT_SCENE}))
    )
    assert "stool" in joined
    assert "seat" not in joined, "画像别名属于目标槽"
    assert "waste receptacle" not in joined, "锚点词属于 anchor 槽"


def test_the_target_slot_alone_carries_no_scene_terms():
    detector = _detector()
    prompts = detector._build_prompts(
        "椅子", ["chair"], ["stool"], slots=frozenset({SLOT_TARGET})
    )
    joined = _joined(prompts)
    assert "chair" in joined
    assert "stool" not in joined
    assert "trash bin" not in joined, "锚点词属于 anchor 槽，不该跟着目标槽一起跑"


def test_the_anchor_slot_is_separable_from_the_target_slot():
    detector = _detector()
    joined = _joined(
        detector._build_prompts("椅子", ["chair"], [], slots=frozenset({SLOT_ANCHOR}))
    )
    assert "trash bin" in joined
    assert "chair" not in joined


def test_an_empty_slot_set_builds_nothing():
    """空槽集必须返回空表：拿 static base 词表兜底就是偷偷又跑一次场景检测。"""
    detector = _detector()
    assert detector._build_prompts("椅子", ["chair"], ["stool"], slots=frozenset()) == []


def test_dropping_the_scene_slot_actually_shrinks_the_prompt():
    """切分要真的少词，而不是换个写法把词又拼回去。"""
    detector = _detector()
    full = _joined(detector._build_prompts("椅子", ["chair"], ["stool", "plant"]))
    without_scene = _joined(
        detector._build_prompts(
            "椅子", ["chair"], ["stool", "plant"], slots=frozenset({SLOT_TARGET, SLOT_ANCHOR})
        )
    )
    assert len(without_scene) < len(full)
    assert "plant" not in without_scene


# --------------------------------------------------------------------------- #
# 3) 缓存键：词槽必须参与
# --------------------------------------------------------------------------- #
def test_the_same_prompt_under_different_slots_gets_different_keys(tmp_path):
    """共用键会让"只跑场景槽"的结果喂给"全跑"那一轮 —— 目标时有时无。"""
    image = tmp_path / "frame.jpg"
    image.write_bytes(b"jpeg")
    only_scene = detection_cache_key(str(image), "chair", frozenset({SLOT_SCENE}))
    everything = detection_cache_key(str(image), "chair", None)
    assert only_scene != everything


def test_slot_order_does_not_change_the_key(tmp_path):
    image = tmp_path / "frame.jpg"
    image.write_bytes(b"jpeg")
    first = detection_cache_key(
        str(image), "chair", frozenset({SLOT_TARGET, SLOT_SCENE})
    )
    second = detection_cache_key(
        str(image), "chair", [SLOT_SCENE, SLOT_TARGET, SLOT_TARGET]
    )
    assert first == second


def test_a_missing_image_has_no_cache_key(tmp_path):
    missing = tmp_path / "nope.jpg"
    assert detection_cache_key(str(missing), "chair") is None


def test_a_changed_file_invalidates_the_key(tmp_path):
    image = tmp_path / "frame.jpg"
    image.write_bytes(b"jpeg")
    before = detection_cache_key(str(image), "chair")
    image.write_bytes(b"jpeg2")
    os.utime(str(image), ns=(0, 1_700_000_000_000_000_000))
    after = detection_cache_key(str(image), "chair")
    assert before != after


# --------------------------------------------------------------------------- #
# 4) 结构守卫：分频真的接到了主循环上
# --------------------------------------------------------------------------- #
ENTRY = Path(__file__).resolve().parents[1] / "scripts" / "go2w" / "run_semantic_exploration.py"


def test_live_observer_binds_shared_session_state_nonlocal():
    """Prevent the first live frame from reading unbound closure state."""
    tree = ast.parse(ENTRY.read_text(encoding="utf-8"))
    observers = [
        node
        for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == "observe"
    ]
    declared = {
        name
        for observe in observers
        for statement in observe.body
        if isinstance(statement, ast.Nonlocal)
        for name in statement.names
    }
    assert {"cadence_entries", "last_semantic_map_odom_pose"} <= declared
NODE = Path(__file__).resolve().parents[1] / "scripts" / "go2w" / "run_autonomous_loop.py"


def _node_detect_calls(text: str) -> list:
    """`node._detect(...)` —— 是 Attribute 调用（`func.attr`），不是 `func.id`。"""
    import ast

    return [
        node
        for node in ast.walk(ast.parse(text))
        if isinstance(node, ast.Call)
        and getattr(node.func, "attr", None) == "_detect"
    ]


def test_the_entry_hands_the_slot_set_to_the_detector():
    """分频算出槽集却不让 `_detect` 知道，那就是一个空转的开关（§3.1.2）。

    这一条是结构判据、不是行为判据：`observe` 套在巨大闭包里没法单独执行，
    报告里要如实写明。
    """
    detect_calls = _node_detect_calls(ENTRY.read_text(encoding="utf-8"))
    assert detect_calls, "入口里找不到 node._detect 调用 —— 判据锚点失效"
    for call in detect_calls:
        assert "slots" in {kw.arg for kw in call.keywords}, (
            "有 node._detect 调用点没带 slots —— 分频对它无效"
        )


def test_the_node_routes_its_cache_key_through_the_testable_helper():
    """缓存键一旦写回内联，词槽就可能被漏掉，而那条判据也就无处可测。"""
    text = NODE.read_text(encoding="utf-8")
    assert "detection_cache_key(" in text, "缓存键没走 detection_cache_key"


def test_the_boxes_come_back_carrying_the_slot_that_asked_for_them(tmp_path):
    """端到端一段：真 `detect_with_dynamic_terms`（假 worker）⇒ 框上带着词槽归属。

    上游算好了归属表却忘了往对象上放，下游看到的是"来源未知"⇒ 按旧行为留着 ——
    静默、不报错。所以这段必须真跑一遍，把"归属表 → `DetectedObject` → dict"
    整条接上；词表归属本身由上面那条单测盯。
    """
    import json
    from types import SimpleNamespace

    from app.detectors.grounded_sam_subprocess import GroundedSAMSubprocessDetector

    # 帧里返回两个框：一个只由**目标槽**（画像的上下文词 paper tray）问出来，
    # 一个只由**场景槽**（RAM 的 desk）问出来。
    reply = json.dumps({
        "objects": [
            {"label": "paper tray", "source_prompt_term": "paper tray",
             "bbox_2d": [0.1, 0.1, 0.2, 0.2]},
            {"label": "desk", "source_prompt_term": "desk",
             "bbox_2d": [0.6, 0.6, 0.8, 0.8]},
        ],
        "diagnostics": {},
    }) + "\n"

    class _FakeWorker:
        stdin = SimpleNamespace(write=lambda text: None, flush=lambda: None)
        stdout = SimpleNamespace(readline=lambda: reply)
        poll = staticmethod(lambda: None)

    detector = GroundedSAMSubprocessDetector(
        settings=Settings(
            siliconflow_api_key="",
            grounding_prompt_llm_expansion_enabled=False,
            static_object_prompts_enabled=False,
        ),
        target_profile=_ctx_profile(),
    )
    detector._worker = _FakeWorker()  # type: ignore[assignment]

    frame = tmp_path / "frame.jpg"
    frame.write_bytes(b"\xff\xd8\xff")  # 假 worker 不读图，客户端只查 is_file()
    objects = detector.detect_with_dynamic_terms(
        str(frame), "打印机", [], ["desk"], frame_id="f1"
    )

    slots_by_label = {item.label: item.detection_slots for item in objects}
    assert slots_by_label == {"paper tray": ("target",), "desk": ("scene",)}, (
        f"框上的词槽归属不对：{slots_by_label}"
    )


def test_a_scene_short_word_does_not_claim_a_longer_target_query():
    """来源归属按完整查询词；角色分类的子串规则不适用于词槽来源。"""
    from app.detectors.grounded_sam_subprocess import _slots_for_term
    from app.reasoning.target_profile import TargetProfileResolver

    profile = TargetProfileResolver().resolve("打印机", use_llm=False)
    detector = GroundedSAMSubprocessDetector(
        settings=Settings(
            siliconflow_api_key="",
            grounding_prompt_llm_expansion_enabled=False,
            static_object_prompts_enabled=False,
        ),
        target_profile=profile,
    )
    prompts, vocab = detector._slot_prompts("打印机", [], ["tray"])
    assert any("paper tray" in prompt for prompt in prompts)
    assert any("tray" in prompt for prompt in prompts)
    assert _slots_for_term("paper tray", vocab) == ("target",)
    assert _slots_for_term("PAPER-TRAY", vocab) == ("target",)
    assert _slots_for_term("tray", vocab) == ("scene",)


def test_the_frozen_frames_still_exist_so_this_file_cannot_silently_rot():
    """素材是冻结的 3 帧；它们没了，评估集（任务 6）就无从复现。"""
    if not FROZEN.is_dir():
        pytest.skip("冻结素材不在本机 —— 标 skip 而不是假装通过")
    frames = sorted(FROZEN.glob("color_*.jpg"))
    assert len(frames) >= 3
    assert random.Random(0).choice(frames).is_file()


# --------------------------------------------------------------------------- #
# 5) 按槽保留结果（审查 P2）
# --------------------------------------------------------------------------- #
# 病根：分频切得动词槽，却留不住词槽 —— 本轮 `_detect` 的返回值**整体覆盖**
# 缓存，于是"只刷 target"的那一轮把场景物体丢掉、"只刷 scene"的那一轮把目标
# 候选丢掉。分频不是"少算一点"，是"每轮只看半张画面"。
def _obj(term, score=0.5, bbox=(0.1, 0.1, 0.2, 0.2), *, slots=None):
    """一个检测结果。`slots` = 检测器给它记的 `detection_slots`（"由哪个槽问出来"）。

    默认 `None` = **不写这个键**，代表"检测器没记"（quick 路径、手工构造的
    dict）。要测按词槽归属就必须显式写 —— 这正是本轮新增的那枚字段。
    """
    item = {"label": term, "score": score, "bbox_2d": list(bbox),
            "source_prompt_term": term}
    if slots is not None:
        item["detection_slots"] = list(slots)
    return item


def _entry(slots, objects, *, frame_id="f1", at=0.0):
    from app.perception.detection_scheduler import SlotCacheEntry

    return SlotCacheEntry(
        slots=frozenset(slots), objects=list(objects), frame_id=frame_id, at=at
    )


def _assemble(entries, *, now=10.0):
    from app.perception.detection_scheduler import assemble_slot_objects

    return assemble_slot_objects(entries, now=now)


def test_refreshing_one_slot_does_not_drop_the_other_slots_objects():
    """核心：只刷 target 的那一轮，场景物体的结果**还在**。

    整体覆盖的旧写法下这一条必红 —— 那正是审查 P2 指出的病根。
    """
    entries = [
        _entry(["target", "anchor", "scene"], [_obj("trash bin"), _obj("desk")], at=0.0),
        _entry(["target"], [_obj("trash bin", 0.9)], at=4.0, frame_id="f2"),
    ]
    got = _assemble(entries, now=4.0)
    terms = sorted(item["source_prompt_term"] for item in got.objects)
    assert terms == ["desk", "trash bin"], (
        "只刷目标槽的那一轮把场景物体丢了 —— 观测会少半张画面"
    )


def test_the_freshest_result_wins_for_the_slot_that_was_refreshed():
    """同一件东西刷了两次时，留下的必须是**新的那次**（分数是新的）。"""
    entries = [
        _entry(["target"], [_obj("trash bin", 0.2)], at=0.0),
        _entry(["target"], [_obj("trash bin", 0.9)], at=4.0, frame_id="f2"),
    ]
    got = _assemble(entries, now=4.0)
    assert [item["score"] for item in got.objects] == [0.9]


def test_a_reused_object_carries_its_own_source_frame():
    """旧帧的框必须带着自己那一帧 —— 不写下来，下游只能当它是当前帧的。"""
    entries = [
        _entry(["target"], [_obj("trash bin")], at=0.0, frame_id="f_old"),
        _entry(["anchor", "scene"], [_obj("desk")], at=4.0, frame_id="f_new"),
    ]
    # 目标槽停在 f_old 那次的结果上，锚点/场景槽已经被 f_new 那次刷新。
    got = _assemble(entries, now=4.0)
    frames = {item["source_prompt_term"]: item["entity_source_frame_id"]
              for item in got.objects}
    assert frames == {"trash bin": "f_old", "desk": "f_new"}


def test_a_mixed_frame_assembly_reports_an_unknown_frame_not_the_current_one():
    """来自不止一帧时，`frame_id` 必须给空串（"不知道"）。

    把当前帧号按在旧框上，正是审查说的"过期结果冒充当前帧事实"。
    """
    entries = [
        _entry(["target"], [_obj("trash bin")], at=0.0, frame_id="f_old"),
        _entry(["scene"], [_obj("desk")], at=4.0, frame_id="f_new"),
    ]
    got = _assemble(entries, now=4.0)
    assert got.mixed_frames is True
    assert got.frame_id == ""


def test_a_single_frame_assembly_reports_that_frame():
    entries = [_entry(["target", "anchor", "scene"], [_obj("trash bin")], frame_id="f9")]
    got = _assemble(entries, now=1.0)
    assert got.mixed_frames is False
    assert got.frame_id == "f9"


def test_the_same_box_seen_by_two_entries_is_counted_once():
    """一条检测覆盖 target+scene、另一条只覆盖 target 时，目标框只能出现一次。"""
    shared = _obj("trash bin")
    entries = [
        _entry(["target", "scene"], [shared, _obj("desk")], at=0.0),
        _entry(["target"], [shared], at=4.0, frame_id="f2"),
    ]
    got = _assemble(entries, now=4.0)
    terms = [item["source_prompt_term"] for item in got.objects]
    assert sorted(terms) == ["desk", "trash bin"], f"同一件东西出现了两次：{terms}"


def test_the_provenance_says_how_old_each_source_is():
    """§5.2 要的可见性：逐条说出"哪个槽、几秒前、来自哪一帧"。"""
    entries = [
        _entry(["target"], [_obj("trash bin")], at=0.0, frame_id="f_old"),
        _entry(["scene"], [_obj("desk")], at=7.5, frame_id="f_new"),
    ]
    got = _assemble(entries, now=10.0)
    prov = {tuple(item["slots"]): item for item in got.provenance(
        now=10.0, ttl_by_slot={"target": 0.0, "anchor": 1.0, "scene": 4.0}
    )}
    assert prov[("target",)]["age_s"] == pytest.approx(10.0)
    assert prov[("scene",)]["age_s"] == pytest.approx(2.5)
    assert prov[("scene",)]["frame_id"] == "f_new"
    assert prov[("target",)]["stale"] is True   # 10 s > 0 s 的 TTL
    assert prov[("scene",)]["stale"] is False   # 2.5 s < 4 s


def test_putting_the_same_slot_set_twice_keeps_only_the_newer_run():
    """同槽集合的新一次检测已经覆盖旧的，旧的没有留的必要（也是内存有界的保证）。"""
    from app.perception.detection_scheduler import put_slot_entry

    entries = put_slot_entry([], _entry(["target"], [_obj("trash bin", 0.1)], at=0.0))
    entries = put_slot_entry(entries, _entry(["target"], [_obj("trash bin", 0.9)], at=5.0))
    assert len(entries) == 1
    assert entries[0].objects[0]["score"] == 0.9


# --------------------------------------------------------------------------- #
# 5b) 按槽归属（2026-10-14 补充复审必修 1）：旧全量条目不能把过期目标框
#     带回当前轮
# --------------------------------------------------------------------------- #
# 病根：一条检测常覆盖多个槽。旧的 `target+anchor+scene` 全量条目为了保住
# `scene` 被选中时，它**里面那个过期的 target 框**也一并进了观测。目标动一点点
# 就逃过去重（键是词 + 4 位小数 bbox），于是旧帧的高分框在 `select_candidate`
# 里把新帧的低分真框顶掉 —— 新帧**完全没有**目标时也能报出 PRESENT。
# `mixed_frames` 如实标记了跨帧，但它拦不住这个。
#
# 判据必须走完 assemble → classify → select/derive 整条链：只看组装出来的物体
# 列表，分不出"该丢的旧框还在不在候选池里"（它**确实**还在 objects 里，只是不
# 该再算作当前的目标证据）。

TARGET_TERM = "trash bin"
ANCHOR_TERM = "table"


def _role_profile() -> TargetProfile:
    """与生产画像同构：目标词表含 trash bin、锚点词表含 table。"""
    return TargetProfile(
        raw_query="垃圾桶",
        canonical_name_zh="垃圾桶",
        primary_labels_en=[TARGET_TERM],
        relation_anchor_labels_en=[ANCHOR_TERM],
        context_labels_en=[],
    )


def _downstream(entries, *, now=10.0, target_score_min=0.30):
    """跑完真链路：组装 → 分角色 → 选复核候选 → 判三态。"""
    from app.perception.entity_roles import (
        classify_roles,
        derive_target_state,
        select_candidate,
    )

    assembly = _assemble(entries, now=now)
    profile = _role_profile()
    partition = classify_roles(
        assembly.objects, profile, source_frame_id=assembly.frame_id
    )
    best, _ = select_candidate(
        partition.target_candidates,
        partition.detections,
        scope="target_candidates",
        candidates_authoritative=True,
    )
    state = derive_target_state(assembly.objects, profile, target_score_min)
    return assembly, partition, best, state


def test_a_stale_full_entry_cannot_overwrite_the_fresher_target_verdict():
    """复审给的例子：旧帧 0.90 分（位置略移）→ 新帧 0.20 分 ⇒ 必须选**新**框。

    这条在修之前是红的：旧全量条目为了保住 scene 被选中，它的 0.90 目标框
    跟着进候选池，压过新帧的真框，狗会朝旧位置去。
    """
    entries = [
        _entry(
            ["target", "anchor", "scene"],
            [
                _obj(TARGET_TERM, 0.90, (0.10, 0.10, 0.20, 0.30)),
                _obj("office chair", 0.55, (0.60, 0.20, 0.90, 0.80)),
            ],
            at=0.0,
            frame_id="f_old",
        ),
        _entry(
            ["target", "anchor"],
            [_obj(TARGET_TERM, 0.20, (0.11, 0.11, 0.21, 0.31))],
            at=4.0,
            frame_id="f_new",
        ),
    ]
    assembly, partition, best, state = _downstream(entries)

    assert assembly.mixed_frames is True  # 跨帧是真的，标记没被绕过
    assert [item["score"] for item in partition.target_candidates] == [0.20], (
        "旧帧的 0.90 分框还在候选池里 —— 目标位置已经变了"
    )
    assert best is not None and best["score"] == 0.20
    assert best["entity_source_frame_id"] == "f_new"
    assert state == "POSSIBLE"  # 0.20 未过 0.30 的门 ⇒ 可能，不是确证
    # 保住 scene 正是这次组装的原意：旧条目的**场景**物体照旧在。
    assert "office chair" in [
        item["source_prompt_term"] for item in partition.scene_objects
    ]


def test_a_disappeared_target_is_cleared_by_the_newest_target_slot():
    """新帧**没有**目标 ⇒ 候选池必须空、三态必须是 ABSENT。

    这是复审点名的第二组，也是比它举的例子更坏的一种：修之前这里报 PRESENT，
    等于对着一帧没有目标的画面说"找到了"（§3.2 明令不许新增的假"找到"）。
    """
    entries = [
        _entry(
            ["target", "anchor", "scene"],
            [_obj(TARGET_TERM, 0.90, (0.10, 0.10, 0.20, 0.30))],
            at=0.0,
            frame_id="f_old",
        ),
        _entry(
            ["target", "anchor"],
            [_obj(ANCHOR_TERM, 0.60, (0.61, 0.21, 0.91, 0.81))],
            at=4.0,
            frame_id="f_new",
        ),
    ]
    _, partition, best, state = _downstream(entries)

    assert partition.target_candidates == []
    assert best is None
    assert state == "ABSENT", "新帧没有目标，旧框却把它判成了存在"


def test_a_word_in_both_word_lists_does_not_come_back_as_a_scene_object():
    """同一个词既在目标词表、又是那次检测的场景槽产物时，旧框不能换个身份活下来。

    它命中目标词 ⇒ 目标槽的最新一次用的词表里就含它、没报出来 ⇒ 出局。不能因为
    "没命中角色就当归场景"把它从后门放回物体地图（那是旧框冒充当前帧事实）。
    """
    stale = _obj(TARGET_TERM, 0.90, (0.10, 0.10, 0.20, 0.30))
    entries = [
        _entry(["target", "anchor", "scene"], [stale], at=0.0, frame_id="f_old"),
        _entry(
            ["target", "anchor"],
            [_obj(ANCHOR_TERM, 0.60, (0.61, 0.21, 0.91, 0.81))],
            at=4.0,
            frame_id="f_new",
        ),
    ]
    _, partition, _, state = _downstream(entries)

    from app.perception.entity_roles import entity_id_for

    assert partition.target_candidates == []
    assert entity_id_for(stale) not in [
        item["entity_id"] for item in partition.scene_objects
    ], "旧目标框从场景视图的后门回到了观测里"
    assert state == "ABSENT"


def test_the_newest_box_wins_even_when_the_old_box_sits_at_the_same_coordinates():
    """新旧框坐标**完全相同**时，留下的必须是新那次。

    去重键是"词 + 4 位小数 bbox"，同坐标就是同一个键。旧写法按时间**升序**并、
    先到先得，于是旧框先入 `seen`、新框被当成重复丢掉 —— 现有那批判据全用同一组
    坐标，刚好被这一点掩盖。
    """
    box = (0.10, 0.10, 0.20, 0.30)
    entries = [
        _entry(["target", "scene"], [_obj(TARGET_TERM, 0.90, box)],
               at=0.0, frame_id="f_old"),
        _entry(["target"], [_obj(TARGET_TERM, 0.20, box)],
               at=4.0, frame_id="f_new"),
    ]
    assembly, _, best, _ = _downstream(entries)

    same = [item for item in assembly.objects
            if item["source_prompt_term"] == TARGET_TERM]
    assert len(same) == 1, "同一件东西被并成了两条"
    assert same[0]["score"] == 0.20
    assert same[0]["entity_source_frame_id"] == "f_new"
    assert best is not None and best["score"] == 0.20


def test_objects_without_a_freshness_tag_keep_their_old_roles():
    """quick 路径 / 手工构造的 dict 没有 `detection_fresh_slots` ⇒ 照旧分类。

    没有"它属于哪个槽"的信息时凭空判人家过期，会把这条链之外的所有调用方
    一起改掉 —— 护栏：不带标记的物体一律按旧行为处理。
    """
    from app.perception.entity_roles import classify_roles

    objs = [_obj(TARGET_TERM, 0.80, (0.1, 0.1, 0.2, 0.3)), _obj("desk", 0.4, (0.5, 0.5, 0.6, 0.6))]
    assert all("detection_fresh_slots" not in item for item in objs)
    partition = classify_roles(objs, _role_profile())

    assert [item["score"] for item in partition.target_candidates] == [0.80]
    assert "desk" in [item["source_prompt_term"] for item in partition.scene_objects]


def test_the_entry_no_longer_keeps_one_whole_round_objects_blob():
    """结构判据：整块 `cadence_cache` 必须消失（它就是审查 P2 的病根）。

    "有没有真的组装"不能只靠这一条 —— 结构匹配分不出"组装"与"只留本轮"，
    那件事由下面的行为判据负责。
    """
    text = ENTRY.read_text(encoding="utf-8")
    assert "cadence_cache" not in text, (
        "整块 `cadence_cache` 还在 —— 整体覆盖就是审查 P2 的病根"
    )


# --------------------------------------------------------------------------- #
# 6) 行为：跑**入口那一段真语句**，连跑三轮，看观测里还剩什么
# --------------------------------------------------------------------------- #
# 上面那些是纯函数判据，只能证明"组装函数对"。审查 P2 要的是从网页
# `observe()` 的**实际产物**上看：只刷一个槽的那一轮，观测里是不是还留着
# 别的槽上次的结果。所以这一段把 `observe` 里 quick 检测那段真语句取出来执行。
_QUICK_START = 'profiler.mark("quick_start")'
_QUICK_END = 'quick_latency_ms = (time.perf_counter() - quick_start_perf)'


def _quick_block_source() -> str:
    text = ENTRY.read_text(encoding="utf-8")
    if _QUICK_START not in text or _QUICK_END not in text:
        raise AssertionError("quick 段的锚点不在源文件里了 —— 判据需要跟着改")
    start = text.rindex("\n", 0, text.index(_QUICK_START)) + 1
    end = text.rindex("\n", 0, text.index(_QUICK_END)) + 1
    return textwrap.indent(textwrap.dedent(text[start:end]), "    ")


class _CadenceNode:
    """按槽集返回不同的检测结果 —— 就像真的 `_build_prompts` 切词槽那样。"""

    _detector = "grounded_sam"

    def __init__(self, by_slot: dict) -> None:
        self.by_slot = by_slot
        self.detect_calls: list[dict] = []
        self.events: list[dict] = []
        # 画面上锁的那个框（计划书 §5.1.3）。复审要的"完整 observe →
        # _feed_detection"就看这里锁到了谁。
        self.feed_calls: list[tuple] = []

    def _feed_detection(self, label, score, bbox) -> None:
        self.feed_calls.append((label, float(score), list(bbox or [])))

    def _detect(self, image_path, prompt, env, *, frame_id="", slots=None):
        self.detect_calls.append({"frame_id": frame_id, "slots": slots})
        key = None if slots is None else tuple(sorted(slots))
        return [dict(item) for item in self.by_slot[key]]

    def _write(self, payload: dict) -> None:
        self.events.append(dict(payload))

    def _host_s(self) -> float:
        return 0.0


class _Profiler:
    def mark(self, name: str) -> None:
        pass


@pytest.fixture(scope="module")
def entry_ns() -> dict:
    import importlib.util
    import sys

    spec = importlib.util.spec_from_file_location("_rse_under_test", ENTRY)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return dict(vars(module))


def _round_runner(entry_ns: dict):
    import time as _time

    ns = dict(entry_ns)
    ns["time"] = _time
    wrapper = (
        "def _quick_round(node, state, profiler, stable_image, prompt, env, frame_id,\n"
        "                 slots, reuse_detection, stale_assembly, cadence_entries,\n"
        "                 cadence_ttl_by_slot, settings):\n"
        + _quick_block_source()
        + "    return objects, cadence_entries\n"
    )
    exec(wrapper, ns)  # noqa: S102 - 包的是本仓源文件里的真语句
    return ns["_quick_round"]


def _run_rounds(entry_ns, node, rounds):
    """按给定的 (slots) 序列连跑几轮，每轮返回这一轮的观测物体。"""
    from types import SimpleNamespace

    run = _round_runner(entry_ns)
    settings = SimpleNamespace(go2w_detection_cadence_enabled=True)
    ttl = {"target": 0.0, "anchor": 0.0, "scene": 4.0}
    entries: list = []
    out = []
    for index, slots in enumerate(rounds, start=1):
        objects, entries = run(
            node, {}, _Profiler(), "frame.jpg", "垃圾桶", {}, f"f{index}",
            slots, False, None, entries, ttl, settings,
        )
        out.append(objects)
    return out


TRASH = _obj("trash bin", 0.2, (0.10, 0.20, 0.30, 0.60))
CHAIR = _obj("office chair", 0.95, (0.55, 0.30, 0.80, 0.75))
DESK = _obj("desk", 0.64, (0.32, 0.40, 0.54, 0.70))


def test_a_target_only_round_still_sees_the_scene_objects(entry_ns):
    """**这一条就是审查 P2 的现场**：只刷目标槽的那一轮，观测里必须还有场景物体。

    整体覆盖的旧写法下，第二轮只剩目标框 —— 物体地图/拓扑每轮掉一半。
    """
    node = _CadenceNode({
        None: [TRASH, CHAIR, DESK],          # 冷启动：全槽
        ("target",): [_obj("trash bin", 0.9, (0.10, 0.20, 0.30, 0.60))],
    })
    rounds = _run_rounds(entry_ns, node, [None, frozenset({"target"})])
    terms = sorted(item["source_prompt_term"] for item in rounds[1])
    assert terms == ["desk", "office chair", "trash bin"], (
        "只刷目标槽的那一轮把场景物体丢了 —— 观测少半张画面"
    )


def test_a_scene_only_round_still_sees_the_target_candidate(entry_ns):
    """反过来也一样：只刷场景槽的那一轮，目标候选不能消失。"""
    node = _CadenceNode({
        None: [TRASH, CHAIR, DESK],
        ("scene",): [DESK],
    })
    rounds = _run_rounds(entry_ns, node, [None, frozenset({"scene"})])
    terms = sorted(item["source_prompt_term"] for item in rounds[1])
    assert terms == ["desk", "office chair", "trash bin"]


def test_the_round_reports_which_frame_each_slot_came_from(entry_ns):
    """§5.2：逐槽说出"来自哪一帧、多久之前"，不然"过期"这件事根本查不出来。"""
    node = _CadenceNode({
        None: [TRASH, CHAIR, DESK],
        ("target",): [_obj("trash bin", 0.9, (0.10, 0.20, 0.30, 0.60))],
    })
    _run_rounds(entry_ns, node, [None, frozenset({"target"})])
    assembled = [e for e in node.events if e.get("event") == "detection_cadence_assembled"]
    assert assembled, "没有 `detection_cadence_assembled` 事件 —— 按槽保留是看不见的"
    last = assembled[-1]
    sources = {frozenset(item["slots"]): item for item in last["sources"]}
    assert sources[frozenset({"target"})]["frame_id"] == "f2"
    assert sources[frozenset({"target", "anchor", "scene"})]["frame_id"] == "f1"
    assert last["mixed_frames"] is True
    assert last["views_source_frame_id"] == ""


# --------------------------------------------------------------------------- #
# 6b) 行为：再往下走 —— 从分类段一直跑到 `_feed_detection`（画面上锁的那个框）
# --------------------------------------------------------------------------- #
# 2026-10-14 补充复审明说："单看 mixed_frames 事件不足以验收"，要"跑完整
# observe → _feed_detection/verify 行为测试"。上面那段只证明**观测里还剩什么**，
# 这一段证明**锁框锁到了谁**：旧帧那个高分框会不会把复核目标抢过去。
# 取的是 `observe()` 里从"三份视图"到 `_feed_detection(...)` 的**真语句**。
_FEED_START = "        # §5.1.2：一次检测的产物按职责拆成三份视图。默认空表，两个分支各自填。"
_FEED_END = "        # ---- 计划书 §4.1：Quick 快路径场景物体（轻量 fallback）-------------"


def _feed_block_source() -> str:
    text = ENTRY.read_text(encoding="utf-8")
    if _FEED_START not in text or _FEED_END not in text:
        raise AssertionError("分类段/锁框段的锚点不在源文件里了 —— 判据要跟着改")
    start = text.index(_FEED_START)
    end = text.index(_FEED_END)
    assert end > start
    return textwrap.indent(textwrap.dedent(text[start:end]), "    ")


def _run_observe_feed(entry_ns, node, objects, *, frame_id="f_cur",
                      assembly_frame_id="", profile=None, target_score_min=0.30,
                      role_separation=True, cadence=True, scope="target_candidates"):
    """跑真分类段 + 真锁框段。`quick_payload=None` ⇒ 走 grounded_sam 那条分支。

    `assembly_frame_id` 就是生产里 `assembly.frame_id` 的值：跨帧组装时它是空串，
    于是 `views_source_frame_id` 也是空串（"不知道"）—— 这里照着喂，不美化。

    三个开关都**从参数喂**（不再是硬编码）：第三次补充复审指出原来的写法把
    `go2w_result_role_separation` 固定成 `True`，于是"职责分离**默认关着**"
    那个组合一条都没覆盖 —— 而这恰好是单独开任务 4 时的真实现场。
    返回 `(target_candidates, target_state, best_feed, detections)`；`detections`
    多还一个，是因为默认路径的缺陷长在那张表上，只断言锁框看不出"旧框还在池里"。
    """
    from types import SimpleNamespace

    ns = dict(entry_ns)
    wrapper = (
        "def _observe_feed(node, objects, settings, frame_id, assembly,\n"
        "                  controller, args, quick_payload):\n"
        + _feed_block_source()
        + "    return target_candidates, target_state, best_feed, detections\n"
    )
    exec(wrapper, ns)  # noqa: S102 - 包的是本仓源文件里的真语句
    return ns["_observe_feed"](
        node,
        objects,
        SimpleNamespace(
            go2w_detection_cadence_enabled=cadence,
            go2w_result_role_separation=role_separation,
            go2w_verify_candidate_scope=scope,
        ),
        frame_id,
        SimpleNamespace(frame_id=assembly_frame_id),
        SimpleNamespace(target_profile=profile or _role_profile()),
        SimpleNamespace(target_score_min=target_score_min),
        None,
    )


def test_the_verification_lock_goes_to_the_new_box_not_the_stale_one(entry_ns):
    """锁框必须落在**新帧**那个 0.20 的框上。

    修之前：旧全量条目的 0.90 框还在候选池里，`select_candidate` 取最高分 ⇒
    画面锁在旧位置。这是"跨槽组装把过期目标框带回当前轮"的**最终症状**，也是
    复审要的 observe → _feed_detection 那一段。
    """
    node = _CadenceNode({
        None: [_obj("trash bin", 0.90, (0.10, 0.10, 0.20, 0.30)), CHAIR],
        ("target",): [_obj("trash bin", 0.20, (0.11, 0.11, 0.21, 0.31))],
    })
    rounds = _run_rounds(entry_ns, node, [None, frozenset({"target"})])
    node.feed_calls.clear()

    candidates, state, best, _ = _run_observe_feed(entry_ns, node, rounds[1])

    assert [item["score"] for item in candidates] == [0.20]
    assert best is not None and best["score"] == 0.20
    assert node.feed_calls == [("trash bin", 0.20, [0.11, 0.11, 0.21, 0.31])], (
        f"画面锁到别处去了：{node.feed_calls}（旧帧那个 0.90 的框又赢了）"
    )
    assert state == "POSSIBLE"  # 0.20 没过 0.30 的门


def test_a_round_whose_target_vanished_does_not_lock_on_the_old_box(entry_ns):
    """新帧没有目标 ⇒ **一个框都不许锁**，三态 ABSENT。

    修之前这一轮会锁在旧帧的 0.90 框上并报 PRESENT —— 对着一帧没有目标的画面
    说"找到了"，正是 §3.2 明令不许新增的假"找到"。
    """
    node = _CadenceNode({
        None: [_obj("trash bin", 0.90, (0.10, 0.10, 0.20, 0.30)), CHAIR],
        ("target",): [_obj("office chair", 0.60, (0.61, 0.21, 0.91, 0.81))],
    })
    rounds = _run_rounds(entry_ns, node, [None, frozenset({"target"})])
    node.feed_calls.clear()

    candidates, state, best, _ = _run_observe_feed(entry_ns, node, rounds[1])

    assert candidates == []
    assert best is None
    assert node.feed_calls == [], f"新帧没有目标，却还是锁了框：{node.feed_calls}"
    assert state == "ABSENT"


# --------------------------------------------------------------------------- #
# 6c) **默认开关组合**：分频开着、职责分离关着（第三次补充复审·必修）
# --------------------------------------------------------------------------- #
# 上面 6b 那段把 `go2w_result_role_separation` 固定成 `True` 了。而"单独开任务 4、
# 任务 2 保持默认关"才是**运营上最可能先试的那个组合** —— 也正是缺陷还在的那个
# 组合：默认分支从**未过滤的** `objects` 重建全量 `detections`，装配层为保住
# `scene` 而并进来的那个旧目标框于是原样回到选框池，`select_candidate` 取最高分
# ⇒ 锁框与复核都指着旧位置。必修 1 的那一半（`classify_roles` 的槽归属闸门）
# 只管候选视图，**这条默认回退路径它管不着**。
#
# 下面两条就把这两个场景钉在默认组合上：**旧框不许进池、更不许被锁**。
def test_default_switches_the_stale_box_never_reaches_the_rounds_detection_pool(entry_ns):
    """分频开、职责分离关：旧帧那个 0.90 的框**不许出现在** `detections` 里。

    这一条看的不是"锁到谁"，而是**池子里有没有它** —— 只断言锁框的话，
    池里有旧框但恰好没赢过的情形会漏过去，而复核入口（`verifier`）读的
    正是 `observation.detections` 这张表。

    场子里另外放一个低分场景物体（0.15 < 新框的 0.20），这样"最高分是谁"
    只由新旧目标框决定，不被任务 3 尚未生效的"椅子抢复核"岔开。
    """
    new_box = [0.11, 0.11, 0.21, 0.31]
    node = _CadenceNode({
        None: [_obj("trash bin", 0.90, (0.10, 0.10, 0.20, 0.30)),
               _obj("desk", 0.15, (0.60, 0.60, 0.80, 0.80))],
        ("target",): [_obj("trash bin", 0.20, new_box)],
    })
    rounds = _run_rounds(entry_ns, node, [None, frozenset({"target"})])
    node.feed_calls.clear()

    candidates, _, best, detections = _run_observe_feed(
        entry_ns, node, rounds[1], role_separation=False
    )

    # 候选视图本来就干净（必修 1 那一半有效）—— 但默认组合**不读**它。
    assert candidates == []
    scores = sorted(round(float(item["score"]), 2) for item in detections)
    assert 0.90 not in scores, (
        f"旧帧那个 0.90 的框还在默认选框池里：{detections}"
    )
    assert scores == [0.15, 0.20]
    assert best is not None and best["score"] == 0.20
    assert node.feed_calls == [("trash bin", 0.20, new_box)], (
        f"默认组合下画面锁到别处去了：{node.feed_calls}"
    )


def test_default_switches_a_vanished_target_leaves_no_target_box_in_the_pool(entry_ns):
    """分频开、职责分离关：新帧目标消失 ⇒ 池子里**一个目标框都不许剩**。

    旧写法下这一轮锁在旧帧的 0.90 框上并把它送进复核 —— 对着没有目标的画面
    报"目标确认为真"，是 §3.2 明令不许新增的假"找到"。

    **如实标边界**：默认组合下 `target_state` 仍按旧口径 `bool(detections)` 走
    （场景物体在就算 PRESENT），那是任务 2 要修、且默认关着的另一件事，本条
    **不**声称修好了它 —— 本条钉的是"旧目标框不再冒充当前帧的目标证据"。
    """
    node = _CadenceNode({
        None: [_obj("trash bin", 0.90, (0.10, 0.10, 0.20, 0.30))],
        ("target",): [],
    })
    rounds = _run_rounds(entry_ns, node, [None, frozenset({"target"})])
    node.feed_calls.clear()

    _, _, best, detections = _run_observe_feed(
        entry_ns, node, rounds[1], role_separation=False
    )

    assert [item["label"] for item in detections] == []
    assert best is None
    assert node.feed_calls == [], f"新帧没有目标，却还是锁了框：{node.feed_calls}"


def test_with_cadence_off_the_default_path_still_keeps_every_object(entry_ns):
    """分频**关着**时代码路径必须与从前逐字一致 —— 这是本次修复的containment。

    `classify_roles` 的槽归属闸门认的是 `detection_fresh_slots` 那枚标记，而它
    **只有按槽组装才会写**。分频关着时物体身上没有它 ⇒ 一条都不许丢。
    少了这条，"修默认组合"就可能悄悄改掉默认行为（审查口径里最忌讳的那种）。
    """
    plain = [
        {"label": "trash bin", "score": 0.90, "bbox_2d": [0.10, 0.10, 0.20, 0.30],
         "source_prompt_term": "trash bin"},
        {"label": "desk", "score": 0.15, "bbox_2d": [0.60, 0.60, 0.80, 0.80],
         "source_prompt_term": "desk"},
    ]
    node = _CadenceNode({})
    node.feed_calls.clear()

    _, _, best, detections = _run_observe_feed(
        entry_ns, node, plain, role_separation=False, cadence=False
    )

    assert sorted(round(float(item["score"]), 2) for item in detections) == [0.15, 0.90]
    assert best is not None and best["score"] == 0.90  # 旧行为：全量里取最高分


# --------------------------------------------------------------------------- #
# 6d) 目标槽的**上下文词**（第四次补充复审·必修）
# --------------------------------------------------------------------------- #
# 上一段修的是"旧**目标词**框借场景槽的新鲜度留下来"。但同一个洞还漏另一种词：
# 画像的**上下文词**（`context_labels_en`）是拼进 **target 槽** prompt 的
# （`grounded_sam_subprocess._slot_prompts`），项目现成的「打印机」画像就带
# `paper tray` / `office desk`。这类词**不是目标词也不是锚点词** ⇒ 角色过滤
# （`direct` / `anchors`）挡不住 ⇒ 旧全量条目为了保住 `scene` 被选中时，它里面
# 那个 0.95 的 `paper tray` 原样留在默认选框池里，压过新帧 0.20 的 `printer`；
# 新帧连打印机都没检出时还会报 PRESENT 并锁住旧纸盒。
#
# 修法在检测器一侧：随框记 `detection_slots`（"这个框由哪个槽问出来"），
# `classify_roles` 拿它与 `detection_fresh_slots` 求交，**一个都不交就丢掉**。
# 下面这组把复审点名的两个场景钉在**四种开关组合**上（职责分离 × 复核范围），
# 并单独钉住"场景词照旧可供使用"与"不知道来源就不判"这两条边界。
def _ctx_profile() -> TargetProfile:
    """与项目现成的「打印机」画像同构：目标词 printer，上下文词 paper tray。"""
    return TargetProfile(
        raw_query="打印机",
        canonical_name_zh="打印机",
        primary_labels_en=["printer", "copier"],
        context_labels_en=["paper tray"],
    )


CTX_OLD_BOX = (0.10, 0.10, 0.20, 0.30)
CTX_NEW_BOX = (0.11, 0.11, 0.21, 0.31)
# 场景物体压到新目标框之下（0.15 < 0.20）：这一组测的是"旧上下文词会不会赢"，
# 不该被"任务 3 在默认组合下还没生效"那件**另一码事**岔开。
SCENE_LOW = _obj("desk", 0.15, (0.60, 0.60, 0.80, 0.80), slots=("scene",))


@pytest.mark.parametrize("role_separation", [False, True])
@pytest.mark.parametrize("scope", ["target_candidates", "all_detections"])
def test_a_stale_context_word_never_beats_the_new_target_box(
    entry_ns, role_separation, scope
):
    """① 旧全量有高分目标槽上下文词 → 新目标槽只有低分目标词。

    四种开关组合**都**要锁在新帧那个 0.20 的 `printer` 上。修之前四种组合里
    有三种锁在旧帧 0.95 的 `paper tray` 上（复审报告的四行表）。
    """
    node = _CadenceNode({
        None: [
            _obj("paper tray", 0.95, CTX_OLD_BOX, slots=("target",)),
            SCENE_LOW,
        ],
        ("target",): [_obj("printer", 0.20, CTX_NEW_BOX, slots=("target",))],
    })
    rounds = _run_rounds(entry_ns, node, [None, frozenset({"target"})])
    node.feed_calls.clear()

    candidates, state, best, detections = _run_observe_feed(
        entry_ns, node, rounds[1], profile=_ctx_profile(),
        role_separation=role_separation, scope=scope,
    )

    scores = [round(float(item["score"]), 2) for item in detections]
    assert 0.95 not in scores, (
        f"旧帧那个 0.95 的上下文词还在选框池里：{detections}"
    )
    assert best is not None and best["label"] == "printer", (
        f"锁框没落在新帧的目标上：{best}"
    )
    assert node.feed_calls == [("printer", 0.20, list(CTX_NEW_BOX))], (
        f"画面锁到别处去了：{node.feed_calls}"
    )
    # 复核池（`observation.detections` 就是 verifier 读的那张表）也不许有它。
    assert "paper tray" not in [item["label"] for item in detections]
    assert "paper tray" not in [item["label"] for item in candidates]
    # 三态口径随开关不同，如实分列：职责分离开着才走"只看目标词候选"的三态；
    # 关着时仍是旧口径 `bool(detections)` —— 那是任务 2 的另一件事，本组不声称
    # 修好了它（池子里还有场景物体，所以关着时报 PRESENT）。
    assert state == ("POSSIBLE" if role_separation else "PRESENT")


@pytest.mark.parametrize("role_separation", [False, True])
@pytest.mark.parametrize("scope", ["target_candidates", "all_detections"])
def test_a_stale_context_word_never_survives_a_round_with_no_target(
    entry_ns, role_separation, scope
):
    """② 旧上下文词 → 新目标槽返回**空**。

    这是最凶的那一格：新帧**连打印机都没检出**，旧帧的 0.95 `paper tray` 之前
    会原样冒充当前目标、被锁住、被送进复核 —— 对着一帧没有目标的画面报
    "找到"，正是 §3.2 明令不许新增的假"找到"。

    这一轮旧条目里**只**放上下文词，不放别的场景物体：默认组合的三态是
    `bool(detections)`，混进一件场景物体就会把"池子空了"这件事糊掉。
    """
    node = _CadenceNode({
        None: [_obj("paper tray", 0.95, CTX_OLD_BOX, slots=("target",))],
        ("target",): [],
    })
    rounds = _run_rounds(entry_ns, node, [None, frozenset({"target"})])
    node.feed_calls.clear()

    candidates, state, best, detections = _run_observe_feed(
        entry_ns, node, rounds[1], profile=_ctx_profile(),
        role_separation=role_separation, scope=scope,
    )

    assert detections == [], f"旧上下文词还留在选框池里：{detections}"
    assert candidates == []
    assert best is None
    assert node.feed_calls == [], f"新帧没有目标，却还是锁了框：{node.feed_calls}"
    assert state == "ABSENT"


def test_a_scene_word_from_the_same_stale_entry_is_still_usable(entry_ns):
    """同一条旧条目里的**场景词**照旧可用 —— 修复只丢"来源槽已经过期"的框。

    否则就是把"保留地图语义"这件正事一起丢了（复审原话："同时测试场景词依旧
    可供建图"）。这里钉两件事：它还在组装出来的物体里、还在选框池里；并且它
    带着**真实来源帧**，没有冒充当前帧。

    **如实标边界**：物体地图本身走的是后台全量语义那条链
    （`observation.scene_objects` ← `semantic.objects`），不经过这里；本条证明的是
    "分频的按词槽归属没有把场景框一起砍掉"，不是"地图内容已验证"。
    """
    node = _CadenceNode({
        None: [
            _obj("paper tray", 0.95, CTX_OLD_BOX, slots=("target",)),
            _obj("desk", 0.64, (0.60, 0.60, 0.80, 0.80), slots=("scene",)),
        ],
        ("target",): [],
    })
    rounds = _run_rounds(entry_ns, node, [None, frozenset({"target"})])
    node.feed_calls.clear()

    # 职责分离**开着**跑：这条路 `detections` 直接就是带 ID 的实体 dict，
    # 能看清来源帧；关着时会把 dict 削成 label/score/bbox 三件套（旧行为）。
    _, _, _, detections = _run_observe_feed(
        entry_ns, node, rounds[1], profile=_ctx_profile(), role_separation=True
    )

    by_label = {item["label"]: item for item in detections}
    assert set(by_label) == {"desk"}, f"场景框被一起砍掉了：{detections}"
    assert by_label["desk"]["entity_source_frame_id"] == "f1", (
        "场景框被按上了当前帧号 —— 旧框冒充当前帧正是要防的事"
    )
    assert "paper tray" not in by_label


def _ctx_pool(entry_ns, old_objects):
    """组装 → 按词槽归属分类，返回这一轮**选框池**里的 label。

    丢弃发生在 `classify_roles`，不在组装：组装如实把旧条目的物体都并进来，
    是分类这一步按"这个框由哪个槽问出来"决定它还作不作数。
    """
    from app.perception.entity_roles import classify_roles

    node = _CadenceNode({None: list(old_objects), ("target",): []})
    rounds = _run_rounds(entry_ns, node, [None, frozenset({"target"})])
    return sorted(
        item["label"]
        for item in classify_roles(rounds[1], _ctx_profile()).detections
    )


def test_the_context_word_is_kept_when_the_box_carries_no_slot_record(entry_ns):
    """**不知道就不判**：框上没有 `detection_slots` ⇒ 按旧行为留着。

    那枚字段是这一轮才加的，quick 路径、旧缓存、别处手工构造的 dict 都没有它。
    拿"没有记录"当"来源已过期"会把这条链之外的所有调用方一起改掉。
    （对照：下一条同样的输入、只多写了 `slots=("target",)` ⇒ 必须被丢掉 ——
    两条一起才说明这枚字段**有分辨力**，不是恒丢或恒留。）
    """
    kept = _ctx_pool(entry_ns, [
        _obj("paper tray", 0.95, CTX_OLD_BOX),
        _obj("desk", 0.64, (0.60, 0.60, 0.80, 0.80)),
    ])
    assert kept == ["desk", "paper tray"], (
        f"没有词槽来源的框被当成过期丢掉了：{kept}"
    )


def test_the_same_context_word_is_dropped_once_the_slot_record_is_present(entry_ns):
    """上一条的对照：唯一的差别是框上记了 `detection_slots=("target",)`。

    两条合起来才是判据 —— 只留一条的话，"恒丢"和"恒留"都能骗过它。
    """
    kept = _ctx_pool(entry_ns, [
        _obj("paper tray", 0.95, CTX_OLD_BOX, slots=("target",)),
        _obj("desk", 0.64, (0.60, 0.60, 0.80, 0.80)),
    ])
    assert kept == ["desk"], f"目标槽问出来的旧框没被丢掉：{kept}"


@pytest.mark.parametrize("new_printer", [False, True])
def test_overlapping_scene_word_cannot_keep_an_old_target_context_box(
    entry_ns, tmp_path, new_printer
):
    """真检测器算来源 → 两轮组装 → 真入口锁框；旧框的槽不是测试手填的。

    RAM 场景词 `tray` 是画像上下文查询 `paper tray` 的子串。若来源归属用
    子串匹配，旧 0.95 框会被误记为 scene，随后借场景槽留在默认选框池。
    """
    import json
    from types import SimpleNamespace

    from app.reasoning.target_profile import TargetProfileResolver

    profile = TargetProfileResolver().resolve("打印机", use_llm=False)
    first_objects = [
        {"label": "paper tray", "source_prompt_term": "paper tray",
         "score": 0.95, "bbox_2d": list(CTX_OLD_BOX)},
    ]
    if new_printer:
        first_objects.append({
            "label": "tray", "source_prompt_term": "tray", "score": 0.15,
            "bbox_2d": [0.60, 0.60, 0.80, 0.80],
        })
    second_objects = ([{
        "label": "printer", "source_prompt_term": "printer", "score": 0.20,
        "bbox_2d": list(CTX_NEW_BOX),
    }] if new_printer else [])
    replies = iter(
        json.dumps({"objects": objects, "diagnostics": {}}) + "\n"
        for objects in (first_objects, second_objects)
    )
    writes = []
    detector = GroundedSAMSubprocessDetector(
        settings=Settings(
            siliconflow_api_key="",
            grounding_prompt_llm_expansion_enabled=False,
            grounding_prompt_retry_on_empty=False,
            enable_gdino_high_recall=False,
            static_object_prompts_enabled=False,
        ),
        target_profile=profile,
    )
    detector._worker = SimpleNamespace(
        stdin=SimpleNamespace(write=writes.append, flush=lambda: None),
        stdout=SimpleNamespace(readline=lambda: next(replies)),
        poll=lambda: None,
    )
    frame = tmp_path / "frame.jpg"
    frame.write_bytes(b"\xff\xd8\xff")
    full = detector.detect_with_dynamic_terms(
        str(frame), "打印机", [], ["tray"], frame_id="f1"
    )
    fresh = detector.detect_with_dynamic_terms(
        str(frame), "打印机", [], ["tray"], frame_id="f2",
        slots=frozenset({"target"}),
    )
    assert len(writes) == 2
    assert [item.detection_slots for item in full] == (
        [("target",), ("scene",)] if new_printer else [("target",)]
    )
    assert [item.detection_slots for item in fresh] == (
        [("target",)] if new_printer else []
    )

    def as_entry(item):
        return _obj(
            item.source_prompt_term or item.label, item.score, item.bbox_2d,
            slots=item.detection_slots,
        )

    node = _CadenceNode({
        None: [as_entry(item) for item in full],
        ("target",): [as_entry(item) for item in fresh],
    })
    rounds = _run_rounds(entry_ns, node, [None, frozenset({"target"})])
    old = next(item for item in rounds[1] if item["label"] == "paper tray")
    assert old["entity_source_frame_id"] == "f1"
    assert "target" not in old["detection_fresh_slots"]

    for role_separation in (False, True):
        for scope in ("target_candidates", "all_detections"):
            node.feed_calls.clear()
            candidates, state, best, detections = _run_observe_feed(
                entry_ns, node, rounds[1], profile=profile,
                role_separation=role_separation, scope=scope,
            )
            assert "paper tray" not in [item["label"] for item in detections]
            assert "paper tray" not in [item["label"] for item in candidates]
            if new_printer:
                assert "tray" in [item["label"] for item in detections]
                assert best is not None and best["label"] == "printer"
                assert node.feed_calls == [("printer", 0.20, list(CTX_NEW_BOX))]
            else:
                assert detections == [] and candidates == []
                assert best is None and state == "ABSENT"
                assert node.feed_calls == []


def test_a_box_is_attributed_to_the_slots_whose_word_lists_asked_for_it():
    """词→槽映射本身（`_slots_for_term`）—— 这是本次修复的**唯一**信息来源。

    上游漏记一个词，下游看到的就是"来源未知"⇒ 按旧行为留着，**不报错**。
    所以映射要单独钉住，不能只靠"端到端跑出来是对的"。
    """
    from app.detectors.grounded_sam_subprocess import _slots_for_term

    _, vocab = _detector()._slot_prompts("椅子", ["chair"], ["stool"])

    # 上下文词（backpack）是拼进**目标槽** prompt 的 —— 这正是第四次补充复审
    # 那个洞的来源，必须归 target，不能因为"它不在目标词表里"就归成场景。
    assert _slots_for_term("backpack", vocab) == ("target",)
    # 锚点词归 anchor；RAM 场景词归 scene。
    assert _slots_for_term("waste receptacle", vocab) == ("anchor",)
    assert _slots_for_term("stool", vocab) == ("scene",)
    # `TARGET_TERM_RULES` 顺带拼进目标 prompt 的词（office chair）也是目标词。
    assert _slots_for_term("office chair", vocab) == ("target",)
    # static base 词表里、这一轮**根本没发出去**的词（画像有目标词 ⇒ 不发 base
    # prompt）不许被安上场景槽的归属：那等于给一批没问过的词背书，目标槽问出来
    # 的框就能借场景槽的新鲜度留下来 —— 正是这次要修的毛病换个入口复发。
    assert _slots_for_term("desk", vocab) == ()
    assert _slots_for_term("cabinet", vocab) == ()
    # 认不出来的词、空词 ⇒ 空元组 = "不知道"，下游按旧行为处理。
    assert _slots_for_term("printer", vocab) == ()
    assert _slots_for_term("", vocab) == ()
    assert _slots_for_term(None, vocab) == ()


@pytest.mark.parametrize("with_profile", [True, False])
def test_every_term_in_the_rendered_prompts_is_attributed_to_a_slot(with_profile):
    """漂移守卫：分频发的**每个 prompt 里的每个词**，都必须能在归属表里找到。

    词表一共四支会进 prompt：画像的目标/上下文/锚点词、RAM 场景词、
    `TARGET_TERM_RULES` 顺带拼进来的目标词、以及 static base 词表（场景槽）。
    漏记任何一支，那批词就会以"来源未知"的身份恒留在选框池里 —— 静默、不报错。
    所以这里不看某个词对不对，而是**逐词对账**：prompt 里出现过的，归属表里
    必须都有。

    `with_profile=False` 走的是**没有画像**那条支（static base 词表真的会被
    渲染出来），专门盯住 base 词的记账 —— 它是"没发出去就不许记账"那条判据的
    另一面。
    """
    if with_profile:
        detector = _detector()
        args = ("椅子", ["chair"], ["stool"])
    else:
        detector = GroundedSAMSubprocessDetector(
            settings=Settings(
                siliconflow_api_key="",
                grounding_prompt_llm_expansion_enabled=False,
                static_object_prompts_enabled=True,
            ),
            target_profile=None,
        )
        args = ("垃圾桶", [], ["stool"])

    prompts, vocab = detector._slot_prompts(*args)
    assert prompts, "这一组没渲染出任何 prompt —— 判据没在考东西"
    attributed: set[str] = set()
    for terms in vocab.values():
        attributed.update(str(term).strip().lower() for term in terms)
    for prompt in prompts:
        for term in [t.strip() for t in prompt.split(".") if t.strip()]:
            assert term.lower() in attributed, (
                f"prompt 里的 `{term}` 在词槽归属表里找不到 —— "
                f"它到了下游只会是「来源未知」，恒被留着"
            )
