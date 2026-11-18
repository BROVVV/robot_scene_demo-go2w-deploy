"""WebUI SLAM 桥的**运动段漂移闸门**（§30 点名的 `tests/test_plain_slam_web_bridge_gate.py`）。

被测对象：`app/spatial/slam_web_map_state.py` 的 `MotionEpisodeGate` / `drift_reason`。

**这个文件补的是哪一块。** 闸门已有的覆盖在 `tests/test_plain_slam_webui_3d.py`
（`drift_reason` 两条判据的三组正负例、`started`/`finished` 的基本转换、一次转向
episode 的端点断言）。本文件只补**那三处真缺口**，不重抄已经覆盖的部分：

1. **"不是计时器"这一条从来没有被证过。** §9.6 明写"不要用固定 sleep 冒充停稳"，
   但现有测试里**唯一一帧运动中数据连返回值都没断言**，而且所有测试都在喂完运动帧
   之后立刻转静止 —— 于是"只要时间到了就 finished"这种错误实现照样全绿。本文件把
   狗一直喂在"动着"的状态并跨过好几个 settle 周期，断言它**永远不 finished**。
2. **默认阈值 0.02 / 0.03 从来没被跑过。** 现有 3 处 `MotionEpisodeGate(...)` 全都
   显式传了值，默认值一旦被改坏（比如 0.02 手滑成 0.2）没有任何测试会红。本文件用
   默认构造钉住"刚过阈值算动、刚不到不算动"。
3. **"以整段运动为单位"从来没被判别过。** 现有 episode 里 LIO 只被喂过一次，
   "从起点累计"与"看上一帧"数值恰好相同 —— 分不开。本文件构造中间量 ≠ 总量的
   多采样 episode。

另外补 `stationary()`（全仓零断言）与 `drift_reason` 的**等号边界**。

**写这个文件时踩到的、必须记住的闸门语义（实测，不是读出来的）：**
判"动没动"有**三条**并列判据，不只是 speed/yaw —— 还有
``travelled = 帧间位移 / 帧间耗时 > speed_max * 1.5``（默认 0.03 m/s）。于是
**两个时间点之间位置跳一下、哪怕 speed 传 0，也算"在动"**。后果是：想结束一段
运动，必须**连续喂同一位置**让 ``travelled`` 归零；那一帧才把 ``stationary_since``
置位，``finished`` 从**它**起算 settle 时长，而不是从"最后一次真运动"起算。
第一版测试就是在这里错的（凭空以为静止帧会重置计时），所以这条写进 docstring。

**诚实边界**：这些是纯逻辑契约测试，不碰 ROS、不碰话题。它证明闸门的判定规则，
不证明"真机上轮式里程计与 LIO 的数值会长这样"——后者只能靠真机事件流。
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from app.spatial.slam_web_map_state import (
    HEALTHY,
    ROTATION_PSLAM_XY_M,
    ROTATION_WHEEL_XY_M,
    MotionEpisodeGate,
    SlamWebMapState,
    drift_reason,
)


def _state(gate: MotionEpisodeGate) -> SlamWebMapState:
    return SlamWebMapState(gate=gate)


# ---------------------------------------------------------------------------
# 1. 不是计时器：动着的狗永远不许被判「停稳」
# ---------------------------------------------------------------------------

def test_a_still_moving_robot_is_never_reported_as_finished():
    """**本文件最重要的一条**：§9.6 的"不要用固定 sleep 冒充停稳"。

    狗从 ``now=0`` 起持续运动，喂到 ``now=5.0``（= 10 个 settle 周期），途中每一帧
    都超速。一个"计时器式"实现（只看 ``now - stationary_since``）会在 0.5 s 后
    就返回 ``finished``，于是一段根本没结束的运动被当成结束、被拿去算假平移。
    """
    gate = MotionEpisodeGate(settle_seconds=0.5)
    gate.note_lio(0.0, 0.0, 0.0)

    assert gate.note_wheel(0.0, 0.0, 0.0, speed=0.5, yaw_rate=0.0, now=0.0) == "started"
    for tick in range(1, 11):
        # 每 0.5 s 前进 0.1 m —— speed 与"位置变动率"两个判据都超阈。
        returned = gate.note_wheel(0.1 * tick, 0.0, 0.0,
                                   speed=0.5, yaw_rate=0.0, now=0.5 * tick)
        assert returned == "", f"第 {tick} 帧狗还在动，不该返回 {returned!r}"
    assert gate.moving is True
    assert gate.episode_id == 1, "一整段连续运动只算一个 episode"
    # 静止判定同样不许在运动中说"已停稳"。
    assert gate.stationary(now=100.0) is False


def test_settle_duration_is_required_not_a_single_quiet_frame():
    """一帧安静不够 —— 必须**持续** settle 时长才算这段运动结束。"""
    gate = MotionEpisodeGate(settle_seconds=0.5)
    gate.note_lio(0.0, 0.0, 0.0)
    assert gate.note_wheel(0.0, 0.0, 0.0, speed=0.5, yaw_rate=0.0, now=0.0) == "started"
    # 第一帧安静（同位、零速）：它把静止起点定在 0.4。
    assert gate.note_wheel(0.0, 0.0, 0.0, speed=0.0, yaw_rate=0.0, now=0.4) == ""
    assert gate.moving is True
    assert gate.stationary(now=0.4) is False
    # 0.8 距静止起点只有 0.4 s < 0.5 s：仍不许结束。
    assert gate.note_wheel(0.0, 0.0, 0.0, speed=0.0, yaw_rate=0.0, now=0.8) == ""
    assert gate.moving is True
    assert gate.stationary(now=0.8) is False
    # 恰好 0.9 = 0.4 + 0.5：>= settle_seconds，判结束。
    assert gate.note_wheel(0.0, 0.0, 0.0, speed=0.0, yaw_rate=0.0, now=0.9) == "finished"
    assert gate.moving is False
    assert gate.stationary(now=0.9) is True


def test_settle_seconds_is_clamped_so_zero_cannot_instant_finish():
    """``settle_seconds=0`` 会被夹到 0.1 —— 否则"一帧就结束"又是一台计时器。"""
    assert MotionEpisodeGate(settle_seconds=0.0).settle_seconds == 0.1
    assert MotionEpisodeGate(settle_seconds=-3.0).settle_seconds == 0.1


def test_a_position_jump_alone_counts_as_motion_even_at_zero_speed():
    """第三条判据：帧间位移率超 ``speed_max * 1.5`` 就算动，即使 speed 报 0。

    这是实测出来的语义。它救的是"轮式速度话题偶尔报 0 而位置其实在跳"的情形 ——
    反过来说，它意味着**不能靠"连续两次相同位置"以外的方式结束一段运动**。
    """
    gate = MotionEpisodeGate()  # 默认 speed_max=0.02 -> 位移率门 0.03
    gate.note_lio(0.0, 0.0, 0.0)
    assert gate.note_wheel(0.0, 0.0, 0.0, speed=0.5, yaw_rate=0.0, now=0.0) == "started"
    # 位置跳 1 m / 1 s = 1 m/s，远超 0.03 —— 尽管 speed 传 0，仍算在动。
    assert gate.note_wheel(1.0, 0.0, 0.0, speed=0.0, yaw_rate=0.0, now=1.0) == ""
    assert gate.moving is True
    # 同位帧才真正安静下来。
    assert gate.note_wheel(1.0, 0.0, 0.0, speed=0.0, yaw_rate=0.0, now=1.1) == ""
    assert gate.note_wheel(1.0, 0.0, 0.0, speed=0.0, yaw_rate=0.0, now=1.6) == "finished"


# ---------------------------------------------------------------------------
# 2. 默认阈值 0.02 / 0.03 真正生效
# ---------------------------------------------------------------------------

def test_default_thresholds_decide_what_counts_as_motion():
    """默认构造下：speed 刚过 0.02 或 yaw_rate 刚过 0.03 才算动。

    位置在帧间**完全不变**（travelled = 0），把第三条判据排除掉，单看默认阈值。
    """
    gate = MotionEpisodeGate()
    assert (gate.speed_max, gate.yaw_rate_max) == (0.02, 0.03)
    gate.note_lio(0.0, 0.0, 0.0)

    # 0.019 < 0.02：不算动。
    assert gate.note_wheel(0.0, 0.0, 0.0, speed=0.019, yaw_rate=0.0, now=0.0) == ""
    assert gate.moving is False
    # 0.029 < 0.03：仍不算动。
    assert gate.note_wheel(0.0, 0.0, 0.0, speed=0.0, yaw_rate=0.029, now=0.1) == ""
    assert gate.moving is False
    # 0.021 > 0.02：算动。
    assert gate.note_wheel(0.0, 0.0, 0.0, speed=0.021, yaw_rate=0.0, now=0.2) == "started"
    assert gate.moving is True

    # 换一段，单看 yaw_rate：0.031 > 0.03 也算动。
    other = MotionEpisodeGate()
    other.note_lio(0.0, 0.0, 0.0)
    assert other.note_wheel(0.0, 0.0, 0.0, speed=0.0, yaw_rate=0.031, now=0.0) == "started"


def test_negative_speed_counts_as_motion_by_magnitude():
    """倒着走也是动：判据取绝对值。"""
    gate = MotionEpisodeGate()
    gate.note_lio(0.0, 0.0, 0.0)
    assert gate.note_wheel(0.0, 0.0, 0.0, speed=-0.5, yaw_rate=0.0, now=0.0) == "started"
    assert gate.note_wheel(0.0, 0.0, 0.0, speed=0.0, yaw_rate=-0.5, now=0.1) == ""


# ---------------------------------------------------------------------------
# 3. 「整段运动」= 从起点累计，不是最后一帧的瞬时量
# ---------------------------------------------------------------------------

def test_episode_delta_is_the_whole_leg_not_the_last_frame():
    """L 形轨迹：起点 (0,0) -> 中间 (1,0) -> 终点 (1,1)。

    轮式侧总量必须是 ``hypot(1,1)=1.414``（从起点算），**不是**最后一帧那一小步、
    也不是中间量。LIO 同步给一条更长的假轨迹，好让"总量"与"最后一帧"分得开。
    """
    gate = MotionEpisodeGate(settle_seconds=0.5)
    gate.note_lio(0.0, 0.0, 0.0)
    assert gate.note_wheel(0.0, 0.0, 0.0, speed=0.5, yaw_rate=0.0, now=0.0) == "started"
    # 中段：LIO 已经漂到 (2.0, 0.0)，轮子才走到 (1.0, 0.0)。
    gate.note_lio(2.0, 0.0, 0.0)
    assert gate.note_wheel(1.0, 0.0, 0.0, speed=0.5, yaw_rate=0.0, now=1.0) == ""
    # 终点：LIO 漂到 (2.0, 1.5)，轮子到 (1.0, 1.0)。
    gate.note_lio(2.0, 1.5, 0.0)
    assert gate.note_wheel(1.0, 1.0, 0.0, speed=0.0, yaw_rate=0.0, now=1.5) == ""
    # 必须同位再喂一帧才算安静（见 docstring 的 travelled 语义）。
    assert gate.note_wheel(1.0, 1.0, 0.0, speed=0.0, yaw_rate=0.0, now=2.0) == ""
    assert gate.note_wheel(1.0, 1.0, 0.0, speed=0.0, yaw_rate=0.0, now=2.5) == "finished"

    wheel_xy, wheel_yaw = gate.wheel_delta
    lio_xy, lio_yaw = gate.lio_delta
    assert wheel_xy == pytest.approx(math.hypot(1.0, 1.0)), wheel_xy
    assert lio_xy == pytest.approx(math.hypot(2.0, 1.5)), lio_xy
    assert wheel_yaw == pytest.approx(0.0)
    assert lio_yaw == pytest.approx(0.0)


def test_episode_start_is_latched_at_the_started_frame():
    """起点锁在 ``started`` 那一帧，**不随后续帧移动** —— 否则总量会被越算越小。"""
    gate = MotionEpisodeGate(settle_seconds=0.5)
    gate.note_lio(5.0, 5.0, 0.0)
    assert gate.note_wheel(10.0, 10.0, 0.0, speed=0.5, yaw_rate=0.0, now=0.0) == "started"
    assert gate.note_wheel(13.0, 14.0, 0.0, speed=0.5, yaw_rate=0.0, now=0.5) == ""
    assert gate.note_wheel(13.0, 14.0, 0.0, speed=0.0, yaw_rate=0.0, now=1.0) == ""
    assert gate.note_wheel(13.0, 14.0, 0.0, speed=0.0, yaw_rate=0.0, now=1.5) == "finished"
    # 从 (10,10) 到 (13,14) = 5.0；若起点被改成中段帧，这里会变小。
    assert gate.wheel_delta[0] == pytest.approx(5.0)


def test_second_episode_measures_only_its_own_leg():
    """两段运动：第二段的总量只算第二段，不累加第一段。"""
    gate = MotionEpisodeGate(settle_seconds=0.5)
    gate.note_lio(0.0, 0.0, 0.0)

    assert gate.note_wheel(0.0, 0.0, 0.0, speed=0.5, yaw_rate=0.0, now=0.0) == "started"
    assert gate.note_wheel(3.0, 0.0, 0.0, speed=0.5, yaw_rate=0.0, now=0.5) == ""
    assert gate.note_wheel(3.0, 0.0, 0.0, speed=0.0, yaw_rate=0.0, now=1.0) == ""
    assert gate.note_wheel(3.0, 0.0, 0.0, speed=0.0, yaw_rate=0.0, now=1.5) == "finished"
    assert gate.episode_id == 1
    assert gate.wheel_delta[0] == pytest.approx(3.0)

    assert gate.note_wheel(3.0, 0.0, 0.0, speed=0.5, yaw_rate=0.0, now=2.0) == "started"
    assert gate.note_wheel(4.0, 0.0, 0.0, speed=0.5, yaw_rate=0.0, now=2.5) == ""
    assert gate.note_wheel(4.0, 0.0, 0.0, speed=0.0, yaw_rate=0.0, now=3.0) == ""
    assert gate.note_wheel(4.0, 0.0, 0.0, speed=0.0, yaw_rate=0.0, now=3.5) == "finished"
    assert gate.episode_id == 2
    assert gate.wheel_delta[0] == pytest.approx(1.0), "第二段只有 1 m，不是 4 m"


# ---------------------------------------------------------------------------
# 4. stationary() 的三态语义（全仓此前零断言）
# ---------------------------------------------------------------------------

def test_stationary_is_false_before_any_odometry_arrives():
    """还没收到任何轮式里程计 -> **不是**"已停稳"（不知道 ≠ 静止）。"""
    gate = MotionEpisodeGate(settle_seconds=0.5)
    assert gate.stationary(now=999.0) is False


def test_stationary_needs_a_frame_to_close_the_episode_first():
    """``stationary(now)`` 有两个条件，缺一不可：``not moving`` **且** 距静止起点
    已过 settle 时长。

    **非显然之处（实测）**：``moving`` 只在"收尾那一帧"翻成 False，所以
    **光有时间流逝不够** —— 没有 ``note_wheel`` 来收尾，``stationary`` 永远 False，
    哪怕时钟已经走了 10 秒。也就是说这个函数不是"现在是否静止"的纯时间查询，
    它同时依赖里程计流还在推进。调用方如果拿它做"等它停稳"的轮询条件，必须保证
    期间仍在喂帧，否则会一直等到超时。
    """
    gate = MotionEpisodeGate(settle_seconds=0.5)
    gate.note_lio(0.0, 0.0, 0.0)
    assert gate.note_wheel(0.0, 0.0, 0.0, speed=0.5, yaw_rate=0.0, now=0.0) == "started"
    assert gate.stationary(now=10.0) is False, "还在动，时间再久也不是静止"

    # 安静帧只把静止起点定下来，`moving` 仍是 True -> 还不算停稳。
    assert gate.note_wheel(0.0, 0.0, 0.0, speed=0.0, yaw_rate=0.0, now=1.0) == ""
    assert gate.moving is True
    assert gate.stationary(now=1.0) is False
    assert gate.stationary(now=1.4) is False, "差 0.1 s 也不许提前说停稳"
    # 关键一格：时间早已越过 1.0+0.5，但**没有任何一帧收尾**，moving 仍为真，
    # 所以仍不许说停稳。这一条把 `not moving` 这个闸门真正钉住 ——
    # 去掉它（只看 elapsed）会让这里变成 True。
    assert gate.stationary(now=5.0) is False, "没有收尾帧就不算这段运动结束"

    # 收尾帧：episode 关闭，此后 stationary 才为真。
    assert gate.note_wheel(0.0, 0.0, 0.0, speed=0.0, yaw_rate=0.0, now=1.5) == "finished"
    assert gate.moving is False
    assert gate.stationary(now=1.5) is True
    # 关闭之后即使不再喂帧，时间继续走它照样是真 —— 这才是"已停稳"。
    assert gate.stationary(now=99.0) is True


# ---------------------------------------------------------------------------
# 5. 快照字段：episode id / motion_active / 两侧 episode 位移
# ---------------------------------------------------------------------------

def test_snapshot_reports_the_episode_fields_the_webui_reads():
    """快照里的 ``motion_episode_id`` / ``motion_active`` / ``stationary`` /
    两侧 ``*_episode_delta_*`` 是 §10.3 要求输出的诊断字段。

    数值容差取 1e-4：快照对这几个量做了 ``round(..., 4)``，比它更严的容差是在
    测四舍五入而不是测语义。
    """
    gate = MotionEpisodeGate(settle_seconds=0.5)
    state = _state(gate)
    fixed = np.array([[1.0, 1.0, 0.0]], dtype=np.float32)
    assert state.accept_map(fixed, frame_id="pslam_map", stamp=1.0, wall_time=1.0) is True
    state.note_lio(0.0, 0.0, 0.0)

    assert state.note_wheel(0.0, 0.0, 0.0, speed=0.5, yaw_rate=0.0, now=0.0) == "started"
    moving = state.snapshot(now=0.2, generated_at=0.2)
    assert moving["motion_episode_id"] == 1
    assert moving["motion_active"] is True
    assert moving["stationary"] is False

    # 正确的原地转向：轮子几乎没平移，但转了 1.04 rad ≈ 59.6°。
    state.note_lio(0.04, 0.02, 1.05)
    assert state.note_wheel(0.02, 0.005, 1.04, speed=0.0, yaw_rate=0.0, now=1.2) == ""
    assert state.note_wheel(0.02, 0.005, 1.04, speed=0.0, yaw_rate=0.0,
                            now=1.7) == "finished"

    done = state.snapshot(now=1.7, generated_at=1.7)
    assert done["motion_episode_id"] == 1
    assert done["motion_active"] is False
    assert done["stationary"] is True
    assert done["wheel_episode_delta_xy_m"] == pytest.approx(
        math.hypot(0.02, 0.005), abs=1e-4)
    assert done["wheel_episode_delta_yaw_deg"] == pytest.approx(
        math.degrees(1.04), abs=1e-2)
    assert done["pslam_episode_delta_xy_m"] == pytest.approx(
        math.hypot(0.04, 0.02), abs=1e-4)
    assert done["pslam_episode_delta_yaw_deg"] == pytest.approx(
        math.degrees(1.05), abs=1e-2)
    assert state.health == HEALTHY


# ---------------------------------------------------------------------------
# 6. drift_reason 的等号边界
# ---------------------------------------------------------------------------

def test_drift_reason_boundaries_are_pinned():
    """两条判据的等号侧写死，免得日后被"顺手"改成 >= 而放开一个假平移。"""
    # 判据一：轮子 <= 0.10 且 LIO **>** 0.20。恰好 0.20 不算。
    assert drift_reason((ROTATION_WHEEL_XY_M, 30.0),
                        (ROTATION_PSLAM_XY_M, 29.0)) == ""
    assert "假平移" in drift_reason((ROTATION_WHEEL_XY_M, 30.0),
                                    (ROTATION_PSLAM_XY_M + 1e-6, 29.0))
    # 判据二：LIO **>** 3*轮子 + 0.30。恰好等于不算。
    assert drift_reason((0.40, 0.0), (3.0 * 0.40 + 0.30, 0.0)) == ""
    assert "轮式里程计只有0.40米" in drift_reason((0.40, 0.0),
                                                  (3.0 * 0.40 + 0.31, 0.0))
    # 判据一只在轮子几乎没动时生效：轮子 0.11 时，LIO 再大也不走判据一。
    assert drift_reason((0.11, 0.0), (0.5, 0.0)) == ""


def test_drift_reason_prefers_the_rotation_wording_when_both_apply():
    """两条判据同时成立时报"原地转向"那条 —— 它更具体，操作员更容易看懂。"""
    reason = drift_reason((0.02, 0.0), (5.0, 0.0))
    assert "原地转向中产生" in reason
