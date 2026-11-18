#!/usr/bin/env python3
"""把动作服务器侧的真实状态折算成 ``go2w_cmd_vel_bridge`` 的五道闸门输入。

## 为什么需要这个文件

``go2w_cmd_vel_bridge`` 订阅八个 ``std_msgs/Bool`` 话题，五个没有发布者
（``/go2w/software_arm`` / ``/go2w/sport_lease/alive`` / ``/go2w/robot_error_zero``
/ ``/go2w/emergency_stop`` / ``/go2w/remote_override``），于是恒拒、狗一步不动，
而 ``/go2w/cmd_vel_bridge/status`` 只会说 ``operator_not_armed,lease_unavailable,…``
—— 失败信息里没有任何一处指向"这五个话题没人发"。

真实的锁存状态全都在**动作服务器**那边（``armed_`` 原子量、STOP 验证、
``SportModeState.error_code``），而它按设计只跑在机器人本地的 ROS participant 上；
公网可见的只有 ``motion_network_proxy`` 那一个门面。所以接线只能走
**本地网关 → Unix socket → 网络代理 → 话题** 这一条路。

本模块是那条路上唯一需要判断的部分：给定网关报来的一小段状态，决定五个话题
各自该发什么。它**不含 rclpy**，所以能在没有 ROS 的解释器里直接测
（与 ``bridge_core`` / ``arbiter_core`` 同一个模式）。

## 三个必须说清楚的口径

1. **一律 fail-closed。** 缺字段、类型不对、读数过期、socket 没连上，全部当成
   "不满足"（``False``，急停/遥控那条是 ``True``）。默认值取 ``GateState()``
   就等于"全拒"，这是刻意的。
2. **``software_arm`` 是重建，不是读数。** 动作服务器的 ``armed_`` 是 60 秒空闲
   租约（``arm_timeout_sec``，见 ``motion_action_server.cpp`` 的 ``CheckArmTimeout``），
   网关按同一条规则重建。它**可能偏乐观**（服务器因别的原因提前解除武装时网关不知道）。
   这不构成解锁：动作服务器自己的 ``GoalContext{!require_arm || armed_}`` 才是权威，
   一个乐观的 ``software_arm`` 只会让 bridge 把 goal 发出去，然后被服务器拒掉。
   **门是纵深，不是唯一一道。**
3. **``remote_override`` 没有来源。** 本部署里没有遥控操作者这一路输入，而
   "遥控器没被按"和"遥控器没接"在数据上分不开。所以这里**不猜**：只有操作员
   显式声明遥控器已清（``motion_network_proxy.py --remote-override-cleared``，
   launch 参数 ``remote_override_cleared:=true``）时才发 ``False``，否则保留
   ``True``（继续拦）。它是本模块里唯一一个"断言"而非"测量"的字段，所以
   notes 里会显式写 ``asserted``，让读日志的人一眼看出这一条不是测出来的。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

#: 网关报来的读数最多允许这么旧。动作服务器的 ``state_timeout_sec`` 是同一量级；
#: 取 0.5 s 是为了让 socket 轮询（5 Hz）抖一两次不至于把门拍死。
MAX_STATE_AGE_SECONDS = 0.5

#: 话题名 → ``GateState`` 字段。顺序即 ``/go2w/cmd_vel_bridge/status`` 里
#: blockers 的排列顺序，改这里要回头看 ``bridge_core.decide_velocity``。
GATE_TOPICS: tuple[tuple[str, str], ...] = (
    ("/go2w/software_arm", "software_arm"),
    ("/go2w/sport_lease/alive", "sport_lease_alive"),
    ("/go2w/robot_error_zero", "robot_error_zero"),
    ("/go2w/emergency_stop", "emergency_stop"),
    ("/go2w/remote_override", "remote_override"),
)


@dataclass(frozen=True)
class GateState:
    """五个话题各自该发的布尔值。默认值 = 全拒（fail-closed）。"""

    software_arm: bool = False
    sport_lease_alive: bool = False
    robot_error_zero: bool = False
    emergency_stop: bool = True
    remote_override: bool = True
    #: 人类可读的成因，逐条对应上面某个字段为什么是这个值。
    notes: tuple[str, ...] = ("no gateway state yet",)

    def publish_values(self) -> dict[str, bool]:
        return {topic: getattr(self, field) for topic, field in GATE_TOPICS}


def _as_bool(value: Any) -> bool | None:
    """只接受真正的 JSON 布尔。``0`` / ``"false"`` 一律当类型错误。

    这里刻意不用 ``bool(value)``：``bool("false") is True``，而这一段代码的
    全部价值就在于"报错时拒绝"，宽松转换会把一条格式错误的状态读成放行。
    """
    return value if isinstance(value, bool) else None


def _as_fresh_age(value: Any) -> float | None:
    """把 age 字段读成"可用的新鲜度"。缺失、非数、负数、超时都返回 None。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    age = float(value)
    if not math.isfinite(age) or age < 0.0 or age > MAX_STATE_AGE_SECONDS:
        return None
    return age


def gate_state_from_payload(
    payload: Any,
    *,
    remote_override_cleared: bool = False,
) -> GateState:
    """把网关 ``op:"state"`` 的应答折成五个话题的值。

    ``payload`` 是 ``motion_local_gateway`` 原样回的那份 JSON。任何一处不对
    —— ``ok`` 不是 True、``state`` 不是对象、某个字段缺失或类型错 —— 都只让
    **对应那一项**退回 fail-closed，而不是整包丢掉：一个字段坏掉不该让另外四个
    本来正常的闸门也跟着变得不可读（那样更查不出问题）。
    """
    notes: list[str] = []
    remote_override = not remote_override_cleared
    if remote_override:
        notes.append(
            "remote_override asserted active: no remote-teleop input exists in this "
            "deployment and silence is indistinguishable from 'remote off'; pass "
            "--remote-override-cleared only with an operator decision"
        )
    else:
        notes.append("remote_override asserted cleared by deployment config")

    if not isinstance(payload, dict) or payload.get("ok") is not True:
        reason = "gateway unavailable"
        if isinstance(payload, dict):
            reason = f"gateway error: {payload.get('error', 'unknown')}"
        notes.append(reason)
        return GateState(notes=tuple(notes))

    state = payload.get("state")
    if not isinstance(state, dict):
        notes.append("gateway state missing or not an object")
        return GateState(notes=tuple(notes))

    # --- 机器人报错码：必须"新鲜且为 0"，过期或缺失都算有错 ------------------ #
    error_code = state.get("error_code")
    error_age = _as_fresh_age(state.get("error_age_sec"))
    if error_age is None:
        notes.append("robot error_code stale or missing -> robot_error_or_unknown")
        robot_error_zero = False
    elif isinstance(error_code, bool) or not isinstance(error_code, int):
        notes.append("robot error_code not an integer -> robot_error_or_unknown")
        robot_error_zero = False
    else:
        robot_error_zero = error_code == 0
        if not robot_error_zero:
            notes.append(f"robot error_code={error_code}")

    # --- 运动租约：由狗上的 lease_status_bridge 写文件、网关读文件 ---------- #
    lease_alive = _as_bool(state.get("lease_alive"))
    lease_age = _as_fresh_age(state.get("lease_age_sec"))
    if lease_alive is None or lease_age is None:
        notes.append("sport lease state stale or missing -> lease_unavailable")
        sport_lease_alive = False
    else:
        sport_lease_alive = lease_alive
        if not sport_lease_alive:
            notes.append("sport lease not held by the computer")

    # --- 武装：动作服务器的 60 s 空闲租约，由网关按同一规则重建 -------------- #
    armed = _as_bool(state.get("armed"))
    if armed is None:
        notes.append("armed flag missing or not a boolean -> operator_not_armed")
        software_arm = False
    else:
        software_arm = armed
        if not software_arm:
            notes.append(
                "arm lease expired or never taken (action server holds it for 60 s idle)"
            )

    # --- 急停：Trigger 是事件，这里锁存；没有任何东西会替它自动清除 ---------- #
    estop = _as_bool(state.get("emergency_stop_latched"))
    if estop is None:
        notes.append("estop latch missing or not a boolean -> treated as triggered")
        emergency_stop = True
    else:
        emergency_stop = estop
        if emergency_stop:
            notes.append("emergency stop latched on the action server")

    return GateState(
        software_arm=software_arm,
        sport_lease_alive=sport_lease_alive,
        robot_error_zero=robot_error_zero,
        emergency_stop=emergency_stop,
        remote_override=remote_override,
        notes=tuple(notes),
    )
