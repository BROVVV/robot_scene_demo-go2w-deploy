"""计划书 §9 的三态目标契约（§30 点名的 `tests/test_target_tri_state.py`）。

**为什么需要这个文件。** `app/perception/target_state.py` 的三个公开函数
（`normalize_target_state` / `target_state_from_payload` / `target_state_is_present`）
是"这一帧到底算不算看见目标"的唯一裁决处 —— 它决定狗停不停、动不动。在本文件之前
**没有任何测试文件 import 过这个模块**（`grep perception.target_state tests/` 零命中），
于是"POSSIBLE 不能被当成停止证据"这条最要命的不变量，只靠代码注释扛着。

**§9.8 四个 Case 里，有两个不能照原文写：**

* **Case A（"白色垃圾桶 / category 匹配但颜色 uncertain → POSSIBLE"）的链路不存在。**
  grep 过全仓：`uncertain_attributes` 只出现在计划书自己的示例 JSON 里，零个 `.py`
  引用；颜色/属性判定（`crop_verifier.py:114-117`、`semantic_observer.py:223`）
  **都不参与 target_state 推导**。真实存在的是两层更粗的机制：(1)
  `siliconflow_vision_worker.py:321-330` 用 prompt 要求 VLM 自己在属性不确定时吐
  `POSSIBLE`；(2) `target_state.py` 的**显式状态优先于 found 布尔**。所以本文件测的是
  第 (2) 条 —— 那是可以让"不确定压过 found=true"成立的真实代码，而不是去测一段
  prompt 字符串（那属于假绿）。
* **Case D（`found=true` 但 bbox 无效 → 不许立刻 ABSENT）** 链路存在且零覆盖，
  落点在 `siliconflow_vision_worker.py:193-196`，本文件直接打它。

**诚实边界**：本文件是**契约测试**，不是端到端。它证明的是"三态的判定与降级规则在
给定输入下成立"，不证明"真机上 VLM 会给出这些输入"——后者要靠真机事件流。
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest
from PIL import Image

from app.detectors.siliconflow_vision_worker import (
    _quick_detect,
    quick_target_present,
    quick_target_state,
)
from app.live_robot.autonomous_explorer import (
    AutonomousExplorer,
    SemanticMatch,
    VerificationOutcome,
)
from app.navigation.backend_factory import MockBackend
from app.navigation.exploration_config import ExplorationBudget, load_exploration_policy
from app.navigation.exploration_graph import ExplorationGraph
from app.navigation.models import LiveObservation
from app.perception.target_state import (
    TARGET_ABSENT,
    TARGET_POSSIBLE,
    TARGET_PRESENT,
    normalize_target_state,
    target_state_from_payload,
    target_state_is_present,
)


# --------------------------------------------------------------------------
# A. 三态规范化：旧布尔与新文本的兼容
# --------------------------------------------------------------------------

def test_bool_is_still_the_legacy_two_state_contract():
    """旧调用方还在传布尔；True/False 必须映射到 PRESENT/ABSENT，不是 POSSIBLE。"""
    assert normalize_target_state(True) == TARGET_PRESENT
    assert normalize_target_state(False) == TARGET_ABSENT


def test_textual_aliases_land_on_the_three_states():
    """别名表：FOUND/TRUE/YES -> PRESENT，NOT_FOUND/FALSE/NO -> ABSENT，
    UNCERTAIN/MAYBE/CANDIDATE -> **POSSIBLE**（不是 ABSENT —— 这条最容易写错）。"""
    for alias in ("FOUND", "true", "yes"):
        assert normalize_target_state(alias) == TARGET_PRESENT, alias
    for alias in ("NOT_FOUND", "false", "no"):
        assert normalize_target_state(alias) == TARGET_ABSENT, alias
    for alias in ("UNCERTAIN", "maybe", "candidate"):
        assert normalize_target_state(alias) == TARGET_POSSIBLE, alias


def test_unknown_and_empty_fall_back_to_absent_not_possible():
    """认不出来的状态落 ``default``（ABSENT）。**不能落 POSSIBLE**：那会让每个
    VLM 拼错的词都变成一次"停下来重观测"，把搜索拖死。"""
    assert normalize_target_state("WAT") == TARGET_ABSENT
    assert normalize_target_state(None) == TARGET_ABSENT
    assert normalize_target_state("") == TARGET_ABSENT
    # 但 default 是可覆盖的 —— 调用方要保守时可以要 POSSIBLE。
    assert normalize_target_state("WAT", default=TARGET_POSSIBLE) == TARGET_POSSIBLE


def test_hyphen_is_normalized_so_model_spelling_variants_agree():
    """VLM 有时吐 ``NOT-FOUND``。规范化必须把 ``-`` 折成 ``_``。"""
    assert normalize_target_state("not-found") == TARGET_ABSENT


# --------------------------------------------------------------------------
# B. 显式状态优先于 found 布尔 —— §9.8 Case A 真正成立的那一半
# --------------------------------------------------------------------------

def test_explicit_possible_wins_over_found_true():
    """**本文件最重要的一条。** ``target_state=POSSIBLE`` 与 ``found=true`` 同时出现时，
    判决必须是 POSSIBLE。

    这正是 Case A 的形状：模型认得类别（所以 found 为真），但对属性没把握（所以显式
    报 POSSIBLE）。如果这里让 ``found`` 翻盘成 PRESENT，"属性不确定"就会被当成停止
    证据 —— 狗会停在一个只是颜色像的垃圾桶前，并报 TARGET_FOUND。
    """
    payload = {"target_state": "POSSIBLE", "found": True, "confidence": 0.9}
    assert target_state_from_payload(payload) == TARGET_POSSIBLE
    # 连带：它必须不能通过停止闸门。
    assert target_state_is_present(payload) is False
    assert quick_target_present(payload) is False


def test_explicit_state_wins_over_the_legacy_decision_block():
    """显式键的优先级高于旧 ``target_decision`` 块，三层顺序都要保住：
    payload.target_state > payload.state > decision.*。"""
    payload = {
        "target_state": "ABSENT",
        "state": "PRESENT",
        "target_decision": {"target_state": "PRESENT", "is_present": True},
        "found": True,
    }
    assert target_state_from_payload(payload) == TARGET_ABSENT


def test_non_dict_payload_is_absent_not_an_exception():
    """喂进去 None / 字符串 / 列表都不许抛 —— 它在实时循环里跑。"""
    for payload in (None, "PRESENT", [], 0):
        assert target_state_from_payload(payload) == TARGET_ABSENT


def test_legacy_uncertain_flags_still_reach_possible():
    """老 payload 没有 target_state，只有 ``uncertain``/``possible`` 这类布尔。"""
    assert target_state_from_payload({"uncertain": True}) == TARGET_POSSIBLE
    assert target_state_from_payload({"possible": True}) == TARGET_POSSIBLE
    assert target_state_from_payload(
        {"target_decision": {"is_possible": True}}) == TARGET_POSSIBLE
    # 旧二进制契约：只有 found。
    assert target_state_from_payload({"found": True}) == TARGET_PRESENT
    assert target_state_from_payload({"found": False}) == TARGET_ABSENT


# --------------------------------------------------------------------------
# C. PRESENT 还不够 —— 分数与候选两道门
# --------------------------------------------------------------------------

def test_present_without_any_candidate_is_not_stoppable():
    """PRESENT 但没有任何候选物体 -> 不是停止证据。"""
    assert target_state_is_present({"target_state": "PRESENT"}) is False
    assert target_state_is_present(
        {"target_state": "PRESENT", "target_objects": []}) is False


def test_present_below_the_score_floor_is_not_stoppable():
    """PRESENT + 有候选但分数低于地板 -> 不是停止证据（默认地板 0.15）。"""
    low = {"target_state": "PRESENT", "target_objects": [{"score": 0.14}]}
    assert target_state_is_present(low) is False
    # 恰好等于地板算过（>= 而不是 >）—— 边界写死，免得日后被"顺手改成 >"。
    at = {"target_state": "PRESENT", "target_objects": [{"score": 0.15}]}
    assert target_state_is_present(at) is True
    # 多个候选取最大，不是取第一个。
    best_of = {"target_state": "PRESENT",
               "target_objects": [{"score": 0.02}, {"score": 0.80}]}
    assert target_state_is_present(best_of) is True


def test_scene_only_objects_never_authorize_a_stop():
    """场景物体（``scene_objects``）不能冒充目标 —— 只有 ``target_objects`` 算数。"""
    payload = {"target_state": "PRESENT",
               "scene_objects": [{"score": 0.99, "label": "办公椅"}]}
    assert target_state_is_present(payload) is False


def test_junk_score_does_not_crash_the_gate():
    """VLM 可能给字符串分数。跳过它，不要抛。"""
    payload = {"target_state": "PRESENT",
               "target_objects": [{"score": "很高"}, {"score": 0.5}]}
    assert target_state_is_present(payload) is True


# --------------------------------------------------------------------------
# D. quick_target_state：PRESENT 掉到 POSSIBLE 的降级总闸
# --------------------------------------------------------------------------

def test_quick_state_downgrades_unstoppable_present_to_possible():
    """``quick_target_state`` 是活路径用的那道闸：PRESENT 但过不了停止闸门时
    **降级成 POSSIBLE**，而不是原样放行 PRESENT。"""
    payload = {"target_state": "PRESENT", "target_objects": [{"score": 0.01}]}
    assert quick_target_state(payload) == TARGET_POSSIBLE
    # 过得了闸门就保持 PRESENT。
    assert quick_target_state(
        {"target_state": "PRESENT", "target_objects": [{"score": 0.9}]}
    ) == TARGET_PRESENT
    # POSSIBLE 与 ABSENT 不参与降级。
    assert quick_target_state({"target_state": "POSSIBLE"}) == TARGET_POSSIBLE
    assert quick_target_state({"target_state": "ABSENT"}) == TARGET_ABSENT


# --------------------------------------------------------------------------
# E. §9.8 Case D：found=true 但框无效 -> POSSIBLE，不许立刻 ABSENT
# --------------------------------------------------------------------------

def _quick_settings() -> SimpleNamespace:
    """只造 ``_quick_detect`` 真正读的那几个字段，避免拉进整个 Settings。"""
    return SimpleNamespace(
        siliconflow_api_key="k",
        siliconflow_base_url="http://local",
        siliconflow_timeout_seconds=1,
        vision_model="m",
        image_max_side=64,
        image_detail="low",
        vlm_runtime_quick_max_tokens=256,
    )


def _quick_client(content: str) -> SimpleNamespace:
    def _create(**_kwargs):
        message = SimpleNamespace(content=content)
        return SimpleNamespace(choices=[SimpleNamespace(message=message)])

    return SimpleNamespace(
        chat=SimpleNamespace(completions=SimpleNamespace(create=_create))
    )


def _frame(tmp_path):
    image_path = tmp_path / "frame.jpg"
    Image.new("RGB", (64, 48), (10, 120, 10)).save(image_path)
    return image_path


@pytest.mark.parametrize("bbox_json", ["null", "[0.4, 0.3]", '"不是框"'])
def test_case_d_present_without_a_usable_bbox_becomes_possible(tmp_path, bbox_json):
    """**Case D**：``found=true`` + ``bbox_2d`` 无效 -> POSSIBLE。

    这是 §9.8 明写要求"不应立即 ABSENT"的那条。三种无效框都试：null、长度不足、
    非数值类型 —— 它们走的是 ``_normalize_quick_bbox`` 返回 ``None`` 的同一条路。
    """
    content = (
        '{"target_state": "PRESENT", "found": true, "bbox_2d": ' + bbox_json + ','
        ' "confidence": 0.9, "name_zh": "白色垃圾桶", "reason_zh": "看到桶身"}'
    )
    result = _quick_detect(
        _quick_settings(), str(_frame(tmp_path)), "垃圾桶", "",
        client=_quick_client(content),
    )

    assert result["target_state"] == TARGET_POSSIBLE, (
        "框无效的 PRESENT 必须降级为 POSSIBLE：它是待重观测的候选，"
        "不是可以据以停车的确认"
    )
    # 降级的候选**仍然保留**（否则重观测的对象就丢了），但绝不能通过停止闸门。
    assert result["target_objects"], "降级不等于丢弃候选"
    assert result["target_objects"][0]["bbox_2d"] is None
    assert quick_target_present(result) is False
    assert quick_target_state(result) == TARGET_POSSIBLE


def test_present_with_a_usable_bbox_stays_present(tmp_path):
    """对照组：框有效时**不许**降级 —— 否则这道门会把真目标也吃掉。"""
    content = (
        '{"target_state": "PRESENT", "found": true,'
        ' "bbox_2d": [0.4, 0.3, 0.6, 0.7], "confidence": 0.9,'
        ' "name_zh": "白色垃圾桶", "reason_zh": "看到桶身"}'
    )
    result = _quick_detect(
        _quick_settings(), str(_frame(tmp_path)), "垃圾桶", "",
        client=_quick_client(content),
    )
    assert result["target_state"] == TARGET_PRESENT
    assert quick_target_present(result) is True


# --------------------------------------------------------------------------
# F. §9.8 Case B / Case C：POSSIBLE 在 explorer 里的两个出口
# --------------------------------------------------------------------------

def _explorer(observations, policy, verifier):
    cursor = [0]

    def observe() -> LiveObservation:
        index = min(cursor[0], len(observations) - 1)
        cursor[0] += 1
        return observations[index]

    return AutonomousExplorer(
        target="目标",
        observer=observe,
        matcher=lambda observation: SemanticMatch(
            has_candidate=observation.target_state == TARGET_PRESENT),
        verifier=verifier,
        backend=MockBackend(),
        policy=policy,
        graph=ExplorationGraph(session_id="tri_state_test"),
    )


def _observation(index: int, state: str) -> LiveObservation:
    return LiveObservation(
        bundle_id=f"obs_{index}", timestamp=float(index),
        target_match={"target_state": state, "target_present": state == TARGET_PRESENT},
    )


def _events(explorer) -> list[str]:
    return [item["event"] for item in explorer.events]


def test_case_b_possible_confirmed_on_the_first_verification_is_target_found():
    """**Case B**：POSSIBLE -> verify 通过 -> ``TARGET_FOUND``，且 ``motion_steps`` 不增。

    **机制（实测出来的，不是猜的）**：POSSIBLE 帧会被强制标成
    ``match.has_candidate=True``（``autonomous_explorer.py:635-640``），所以它**照样
    进 VERIFY**；验证一旦通过就走 ``target_found`` 并 ``break``（``:735-746``），
    **不会再花掉一次重观测**。所以这里断言的是"重观测没有发生"，而不是"发生了两次"
    —— 后者是验证**被否决**时的形状（见下一个用例）。

    这一格同时是"不确定的候选也能走向确认、但全程一步没动"的正面证据。
    """
    policy = load_exploration_policy()
    policy.budget = ExplorationBudget(max_planning_cycles=1, max_motion_steps=1)
    explorer = _explorer(
        [_observation(0, TARGET_POSSIBLE), _observation(1, TARGET_PRESENT)],
        policy,
        lambda _observation, _match: VerificationOutcome(
            confirmed=True, attempts=1, reason_zh="白桶确认"),
    )
    result = explorer.run()
    events = _events(explorer)

    assert result.result == "TARGET_FOUND"
    assert result.motion_steps == 0, "POSSIBLE 未决期间以及确认之后都不该有动作"
    assert "action_start" not in events
    assert "target_possible_reobserve" not in events, (
        "当场确认就不该再重观测：多花一次重观测会让确认路径莫名其妙变慢"
    )


def test_case_c_exhausted_possible_emits_resolved_absent_and_frees_the_planner():
    """**Case C**：POSSIBLE 连续验证都被否决 -> 发 ``target_possible_resolved_absent``
    事件、状态落到 ABSENT，**然后** planner 才能动。

    事件的 ``attempts`` 是"一共复核了几次"，用于事后判断预算是否够 —— 它在
    本文件之前零覆盖（`grep target_possible_resolved_absent tests/` 零命中）。
    """
    policy = load_exploration_policy()
    policy.budget = ExplorationBudget(max_planning_cycles=1, max_motion_steps=1)
    explorer = _explorer(
        [_observation(index, TARGET_POSSIBLE) for index in range(4)],
        policy,
        lambda _observation, _match: VerificationOutcome(
            confirmed=False, attempts=1, reason_zh="不是它"),
    )
    result = explorer.run()
    events = _events(explorer)

    resolved = [item for item in explorer.events
                if item["event"] == "target_possible_resolved_absent"]
    assert len(resolved) == 1, "连续 POSSIBLE 必须收敛成一次 ABSENT，不能每帧都发"
    assert resolved[0]["attempts"] >= 2, "至少复核过两次才允许判 ABSENT"
    # 收敛之后 planner 才有机会动作（排序与 test_target_state_explorer 同向）。
    assert "action_start" in events
    assert events.index("target_possible_resolved_absent") < events.index("action_start")
    assert result.motion_steps <= 1
