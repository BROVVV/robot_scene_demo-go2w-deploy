"""``motion_gate_state_core`` 的折算规则测试。

这些用例全部是"**坏输入必须得到拒绝**"型的，因为这段代码唯一的价值就是
fail-closed。分辨力说明写在每个用例里：去掉对应的那句守卫，哪个断言会挂。
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts" / "go2w"))

from motion_gate_state_core import (  # noqa: E402
    GATE_TOPICS,
    MAX_STATE_AGE_SECONDS,
    GateState,
    gate_state_from_payload,
)


def _payload(**state_overrides) -> dict:
    state = {
        "error_code": 0,
        "error_age_sec": 0.05,
        "lease_alive": True,
        "lease_age_sec": 0.2,
        "armed": True,
        "emergency_stop_latched": False,
    }
    state.update(state_overrides)
    return {"ok": True, "state": state}


def test_default_gate_state_blocks_everything() -> None:
    """没有状态时的默认值必须是"全拒"。

    分辨力：把 ``GateState`` 里任一字段的默认值改成放行，本用例挂。这条是
    ``bridge_node`` 那八个成员的同一口径 —— 桥的默认值也是全拒。
    """
    gate = GateState()
    assert gate.software_arm is False
    assert gate.sport_lease_alive is False
    assert gate.robot_error_zero is False
    assert gate.emergency_stop is True
    assert gate.remote_override is True


def test_publish_values_covers_exactly_the_five_bridge_topics() -> None:
    """话题名必须与 ``bridge_node`` 订阅的那五个逐字相同。"""
    assert dict(GATE_TOPICS) == {
        "/go2w/software_arm": "software_arm",
        "/go2w/sport_lease/alive": "sport_lease_alive",
        "/go2w/robot_error_zero": "robot_error_zero",
        "/go2w/emergency_stop": "emergency_stop",
        "/go2w/remote_override": "remote_override",
    }
    assert set(GateState().publish_values()) == {topic for topic, _ in GATE_TOPICS}


def test_complete_fresh_state_opens_the_four_measurable_gates() -> None:
    """全部读数都新鲜且正常时，四个可测量的闸门放行。"""
    gate = gate_state_from_payload(_payload())
    assert gate.software_arm is True
    assert gate.sport_lease_alive is True
    assert gate.robot_error_zero is True
    assert gate.emergency_stop is False


def test_remote_override_stays_blocking_unless_asserted() -> None:
    """``remote_override`` 没有来源，所以默认仍然拦着。

    分辨力：把 ``remote_override = not remote_override_cleared`` 改成
    ``False`` 常量，第一条断言挂。这条是本模块唯一一个"断言"型输入，必须显式。
    """
    assert gate_state_from_payload(_payload()).remote_override is True
    assert (
        gate_state_from_payload(_payload(), remote_override_cleared=True).remote_override
        is False
    )


@pytest.mark.parametrize("payload", [None, {}, {"ok": False, "error": "boom"}, 42, []])
def test_missing_or_failed_gateway_response_falls_closed(payload: object) -> None:
    """网关没应答 / 应答不是成功时，五个闸门一个都不放行。

    分辨力：删掉 ``_state_response`` 之外的那句 ``payload.get("ok") is not True``
    提前返回，``{"ok": False,...}`` 这条会因为读不到 ``state`` 而走另一条分支
    —— 断言仍然过，但 notes 里会丢掉 ``gateway error`` 这条线索；
    ``42`` / ``[]`` 两条则会直接抛 TypeError。
    """
    gate = gate_state_from_payload(payload)
    assert gate.software_arm is False
    assert gate.sport_lease_alive is False
    assert gate.robot_error_zero is False
    assert gate.emergency_stop is True
    assert gate.remote_override is True


def test_gateway_error_text_reaches_the_notes() -> None:
    """失败原因要能读出来 —— 上一次真机验收里，卡住的原因在日志里一个字都没有。"""
    gate = gate_state_from_payload({"ok": False, "error": "local gateway timeout"})
    assert any("local gateway timeout" in note for note in gate.notes)


def test_stale_error_reading_is_not_zero() -> None:
    """报错码读数过期就当作"有错"，不是"没错"。

    分辨力：把 ``error_age`` 那一段改成不看 age，第二条断言挂。这条最容易写错成
    "拿最后一次读到的值继续用"，那样一次掉线就会让门永远绿。
    """
    gate = gate_state_from_payload(
        _payload(error_age_sec=MAX_STATE_AGE_SECONDS + 0.01)
    )
    assert gate.robot_error_zero is False
    assert any("stale" in note for note in gate.notes)


def test_stale_or_absent_lease_is_unavailable() -> None:
    """租约过期、字段缺失、值为假，三种都得判成不可用。"""
    assert gate_state_from_payload(_payload(lease_age_sec=9.9)).sport_lease_alive is False
    assert gate_state_from_payload(_payload(lease_alive=None)).sport_lease_alive is False
    assert gate_state_from_payload(_payload(lease_alive=False)).sport_lease_alive is False
    del_lease = gate_state_from_payload(_payload())
    assert del_lease.sport_lease_alive is True  # 对照：完整那份是放行的
    payload = _payload()
    del payload["state"]["lease_age_sec"]
    assert gate_state_from_payload(payload).sport_lease_alive is False


def test_nonzero_error_code_blocks() -> None:
    gate = gate_state_from_payload(_payload(error_code=1002))
    assert gate.robot_error_zero is False
    assert any("1002" in note for note in gate.notes)


@pytest.mark.parametrize("bad", ["false", 0, 1, None, [], {}])
def test_non_boolean_flags_are_type_errors_not_truthiness(bad: object) -> None:
    """字符串 ``"false"`` 必须判成错误，而不是"非空即真"。

    分辨力：把 ``_as_bool`` 换成 ``bool(value)``，``"false"`` 这条挂
    （``bool("false") is True``，会放行一个明确写着 false 的状态）。
    """
    gate = gate_state_from_payload(_payload(armed=bad))
    assert gate.software_arm is False


@pytest.mark.parametrize("bad", [True, False, "0", 0.0, None, [0]])
def test_non_integer_error_code_is_unknown_not_zero(bad: object) -> None:
    """报错码类型不对 = 未知 = 拒。``False`` 尤其危险：``False == 0`` 是真的。"""
    gate = gate_state_from_payload(_payload(error_code=bad))
    assert gate.robot_error_zero is False


def test_one_broken_field_does_not_poison_the_others() -> None:
    """单个字段坏掉只让那一项退回拒绝，其余照常 —— 否则真机上根本查不出是哪一项。

    分辨力：把折算改成"任一处不对就 return GateState()"，后三条断言挂。
    """
    gate = gate_state_from_payload(_payload(error_age_sec="soon"))
    assert gate.robot_error_zero is False
    assert gate.software_arm is True
    assert gate.sport_lease_alive is True
    assert gate.emergency_stop is False


def test_absent_estop_latch_blocks() -> None:
    """读不到急停锁存时按"已按下"处理。

    分辨力：把 ``emergency_stop = True`` 的默认改成 False，第一条断言挂。
    """
    assert gate_state_from_payload(_payload(emergency_stop_latched=None)).emergency_stop
    assert gate_state_from_payload(_payload(emergency_stop_latched=True)).emergency_stop
    # 对照：明确报 false 时才是放行
    assert not gate_state_from_payload(
        _payload(emergency_stop_latched=False)
    ).emergency_stop


@pytest.mark.parametrize("age", [-0.1, float("nan"), float("inf"), "0.1", None])
def test_unusable_ages_are_rejected(age: object) -> None:
    """负数、NaN、无穷、字符串、缺失，一律不是"新鲜"。"""
    gate = gate_state_from_payload(_payload(error_age_sec=age))
    assert gate.robot_error_zero is False


def test_age_boundary_is_inclusive_at_the_limit() -> None:
    """正好等于上限算新鲜，超过一点就不算 —— 边界要钉住，不然"门限"是橡皮筋。"""
    assert gate_state_from_payload(
        _payload(error_age_sec=MAX_STATE_AGE_SECONDS)
    ).robot_error_zero
    assert not gate_state_from_payload(
        _payload(error_age_sec=MAX_STATE_AGE_SECONDS + 1e-6)
    ).robot_error_zero
