"""LOCAL_SCAN 的 **配额稳定性** 与 **sector 取实际 yaw**（§30 点名的本文件）。

**为什么不重抄 `tests/test_local_scan.py`。** 那个文件已经覆盖了
`select_local_scan_goal` 的六条规则（配额耗尽 / 与 heading coverage 解耦 / 非左优先 /
同向惩罚 / 死循环保护 / `max_local_rotations=0` 关闭）。再写一遍等于新建重复，§10.1
禁止。本文件只补它**没有**、且全仓零断言的三件事：

1. **同一个 Place 内 `goal_id` 必须单调递增、绝不回到 `001`。**
   全仓 `grep local_scan_0 tests/` 零命中 —— `goal_id` 的“不回退”从没被钉过。
   它若不回退失效，同一个 Place 会反复产出同名的 `local_scan_001`，而下游
   `local_scan_states` / 事件流 / 归档都按 `goal_id` 认目标，于是"狗原地转了三圈"
   在日志里长得像"只转了一圈"。
2. **配额是「每个 Place 一份」，不是全局一份。**
   这条的落地在 runner 里（`local_scan_states[place_id]`），靠 `to_dict`/`from_dict`
   往返承载。`from_dict` 若丢掉 `steps`，每轮都从 0 起算 —— 狗会在每个 Place 无限
   旋转，而 `tests/test_local_scan.py` 的单轮配额测试**照样全绿**（它每轮都新建
   `state`）。所以这里单测往返契约，再对 runner 做源码级绑定断言。
3. **§8.4：reached sector 必须由运动后的真实 yaw 算出，不许拿 requested 冒充。**
   本文件之前，`observed_dyaw` / `sector_before` / `sector_reached` 在 `tests/` 里
   被引用 **0** 次。

**诚实边界。** 第 1、2（前半）节是行为断言；第 2（后半）、3（后半）节是**源码级契约**
断言 —— 它们证明"runner 里那段代码是这么写的"，**不证明真机上跑出来是什么**。这两处
只能靠源码断言，因为 `run_semantic_exploration.py` 的主体是 ROS 节点主循环
（`rclpy.init()` 在 `:722`），测试环境里跑不起来。该脚本的**模块级函数**
（`_navigation_sector`）确实 import 得动（实测 0.43 s、顶层无 rclpy 引用），
所以第 3 节前半是真正跑出来的行为断言，不是读源码读出来的。
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from app.navigation.exploration_config import load_exploration_policy
from app.navigation.local_scan import LocalScanState, select_local_scan_goal
from scripts.go2w.run_semantic_exploration import _navigation_sector


RUNNER = Path(__file__).resolve().parents[1] / "scripts/go2w/run_semantic_exploration.py"


def _scan(state: LocalScanState | None, *, yaw_deg: float = 0.0, quota: int = 5):
    """跑一轮 LOCAL_SCAN，返回 (goal_fields, next_state)。"""
    return select_local_scan_goal(
        current_yaw_deg=yaw_deg, max_local_rotations=quota, state=state,
    )


# --------------------------------------------------------------------------
# 1. 同一个 Place 内 goal_id 单调、不回到 001
# --------------------------------------------------------------------------

def test_goal_id_is_monotonic_within_one_place_and_never_returns_to_001():
    """**本文件最重要的一条。** 连续 5 轮把上一轮的 `next_state` 喂回去，`goal_id`
    必须是 `local_scan_001..005`，`steps` 严格递增。

    为什么危险：`goal_id` 是下半段认目标的钥匙（`local_scan_states` 的写、
    `local_scan_selected` 事件、归档）。一旦它回退，同一个 Place 的第二次旋转会
    顶着和第一次一样的 `goal_id`，事后统计"转了几次"就会少算。
    """
    state: LocalScanState | None = None
    seen: list[str] = []
    for _ in range(5):
        goal, state = _scan(state)
        assert goal is not None, "配额 5 轮内不该提前耗尽"
        seen.append(goal["goal_id"])

    assert seen == ["local_scan_001", "local_scan_002", "local_scan_003",
                    "local_scan_004", "local_scan_005"]
    assert len(set(seen)) == len(seen), "同 Place 内不许出现重复 goal_id"


def test_goal_id_encodes_the_post_increment_step():
    """`goal_id` 里的序号 == 自增**之后**的 `steps`，且零填充到 3 位。

    零填充不是装饰：`local_scan_010` 与 `local_scan_9` 混排会让字符串排序与
    数值排序分家，而下游是按字符串存档的。
    """
    state: LocalScanState | None = None
    for expected_step in range(1, 4):
        goal, state = _scan(state)
        assert state.steps == expected_step
        assert goal["goal_id"] == f"local_scan_{state.steps:03d}"


def test_goal_id_formula_agrees_with_the_runner_override():
    """**两处同值公式，必须一致。** 调用方
    （`run_semantic_exploration.py:2505`）在 `ExplorationGoal(**goal_fields)` 之后
    **又显式覆写了一次** `goal_id`，用的是同一个 f-string。

    于是模块内 `local_scan.py:120` 那份在真机路径上是**死写入**（被同值覆盖）。
    后果：只改模块那份的格式，`tests/` 里的行为断言会红，真机却仍吐旧格式。
    这条源码级断言把两个站点钉在一起，任何一边单独改动都会被指出来。
    """
    goal, _ = _scan(None)
    assert goal["goal_id"] == "local_scan_001"  # 模块侧的实际产物

    src = RUNNER.read_text(encoding="utf-8")
    # 调用方覆写点：与模块侧同一个 f-string 形状。
    assert '"goal_id": f"local_scan_{next_state.steps:03d}"' in src, (
        "runner 里对 goal_id 的覆写式样变了 —— 请同步 local_scan.py:120，"
        "否则模块内那份会变成静默分叉的死写入"
    )


# --------------------------------------------------------------------------
# 2. 配额按 Place 绑定（不是全局）
# --------------------------------------------------------------------------

def test_a_new_place_starts_the_quota_over():
    """换 Place = 换一份 `LocalScanState` = 配额从头开始。

    这是"一个 Place 最多转 N 次"的正面一半：新 Place 必须能重新扫。
    反面的危险在下一个用例（同一 Place 不许回 001）。
    """
    exhausted: LocalScanState | None = None
    for _ in range(5):
        _, exhausted = _scan(exhausted)
    assert exhausted is not None and exhausted.steps == 5

    # 新 Place 拿到的是一份全新的状态 —— 与 runner 的
    # ``scan_states.get(current_place.place_id)`` 未命中时的行为一致。
    goal, fresh = _scan(LocalScanState())
    assert goal is not None
    assert goal["goal_id"] == "local_scan_001"
    assert fresh.steps == 1


def test_state_survives_the_dict_roundtrip_that_carries_the_quota():
    """`to_dict`/`from_dict` 是配额跨轮次存活的唯一通道
    （runner：`scan_states[place_id] = next_state.to_dict()`）。

    **这里丢字段 = 每个 Place 无限旋转。** 而且 `tests/test_local_scan.py` 抓不到：
    它每轮都新建 state，`steps` 归零在单轮视角下完全正常。
    """
    original = LocalScanState(
        steps=3, last_direction=-1, same_direction_count=2,
        last_sector=9, last_scan_cycle_info_gain=True,
    )
    restored = LocalScanState.from_dict(original.to_dict())
    assert restored == original
    assert restored.steps == 3, "steps 丢了配额就永远耗不尽"

    # 缺失/空 -> 全新状态，不是抛异常（首轮就是这条路）。
    assert LocalScanState.from_dict(None).steps == 0
    assert LocalScanState.from_dict({}).steps == 0


def test_runner_keys_the_scan_quota_by_place_id():
    """**源码级契约。** runner 必须按 `place_id` 取/存 `local_scan_states`。

    若改成一份全局状态（不按 place 分键），狗跑满 `max_local_rotations` 个 Place
    之后就**再也扫不动了** —— 表现为"搜索走到一半突然不转圈了"，而单轮测试全绿。
    """
    src = RUNNER.read_text(encoding="utf-8")
    assert 'scan_states = state.setdefault("local_scan_states", {})' in src
    assert "scan_states.get(current_place.place_id)" in src, (
        "取配额时必须按 place_id 取，否则换 Place 继承上一个 Place 的已用量"
    )
    assert "scan_states[current_place.place_id] = next_state.to_dict()" in src, (
        "存配额时必须按 place_id 存"
    )


# --------------------------------------------------------------------------
# 3. §8.4：sector 取自运动后的真实 yaw
# --------------------------------------------------------------------------

def test_navigation_sector_is_a_pure_function_of_the_yaw_it_is_given():
    """**行为断言**（该函数在测试环境 import 得动，实测 0.43 s、无 rclpy 依赖）。

    它是 `motion_sector_reached` 事件里 `sector_before` / `sector_reached` 的唯一
    算法。这里钉住三件事：
    * 只依赖传入的 yaw —— 同一个 yaw 恒得同一个 sector；
    * **接受未卷绕的 yaw**（`±2π` 平移不变）。`before[2]`/`after[2]` 是 pslam odom
      的原始 yaw，可以累积超过 `±π` 而不回卷；若有人"顺手"加一句
      `assert -pi <= yaw <= pi`，狗会在运动中途把 runner 抛死；
    * 边界 `±15°` 落回 sector 0（`round` 的取偶行为，与 `local_scan.py` 的
      `int(round(yaw_deg/30.0)) % 12` 一致）。
    """
    sector = _navigation_sector
    assert sector(yaw_rad=0.0, sectors=12) == 0
    assert sector(yaw_rad=math.radians(20.0), sectors=12) == 1
    assert sector(yaw_rad=math.radians(90.0), sectors=12) == 3
    # 负角回卷到 0..11（-45° -> -2 -> 10），不是负数。
    assert sector(yaw_rad=math.radians(-45.0), sectors=12) == 10

    # 未卷绕的 yaw：+2π 平移不改变 sector。
    for yaw in (0.5, 3.0, 7.0, -7.0):
        assert sector(yaw_rad=yaw, sectors=12) == sector(
            yaw_rad=yaw + 2 * math.pi, sectors=12), yaw
        assert 0 <= sector(yaw_rad=yaw, sectors=12) < 12

    # 边界：±15° 恰好落在 round 的取偶点上 -> sector 0。
    assert sector(yaw_rad=math.radians(15.0), sectors=12) == 0
    assert sector(yaw_rad=math.radians(-15.0), sectors=12) == 0


def test_the_two_sector_conventions_agree_on_degrees_vs_radians():
    """两个模块各有一套 sector 约定，**且必须一致**：
    `local_scan.py:68` 吃**度**（`current_yaw_deg / sector_deg`），
    `run_semantic_exploration._navigation_sector` 吃**弧度**。

    它们描述的是同一个 heading sector（`local_scan.py` 的 `heading_coverage` 用的
    就是这套下标），一旦有人在其中一处把度当弧度传，两边就会对不上，而**两边各自的
    单测都会照样绿**（各测各的公式）。这条是跨模块的一致性钉子。
    """
    for yaw_deg in (0.0, 60.0, 120.0, 200.0, -45.0, 359.0):
        from_local_scan = int(round(yaw_deg / 30.0)) % 12
        from_runner = _navigation_sector(
            yaw_rad=math.radians(yaw_deg), sectors=12
        )
        assert from_local_scan == from_runner, (
            f"{yaw_deg}° 两套 sector 约定不一致："
            f"local_scan={from_local_scan} runner={from_runner}"
        )


def test_runner_derives_reached_sector_from_actual_yaw_not_the_request():
    """**源码级契约 —— §8.4 的核心。** `motion_sector_reached` 事件里：

    * `sector_reached` 必须由**运动之后**的真实 yaw（`after[2]`）算出；
    * `sector_before` 由**运动之前**的真实 yaw（`before[2]`）算出；
    * `sector_requested` 才是"计划想去"的那个 sector，由
      `before[2] + radians(requested_dyaw)` 推算 —— 它是**另一个独立字段**。

    危险形状：把 `sector_reached` 写成 `sector_requested` 的同义词（或直接拿
    `goal.heading_sector` 冒充）。那样"请求转 30° 但实际只转了 3°"会被记成成功，
    heading coverage 会假装覆盖了一个从没看过的方向。这个断言之前零覆盖
    （`grep observed_dyaw tests/` 零命中）。
    """
    src = RUNNER.read_text(encoding="utf-8")
    assert 'observed_dyaw = math.degrees(_wrap_pi(after[2] - before[2]))' in src, (
        "observed_dyaw 必须由运动前后的真实 yaw 之差算出"
    )
    assert '"sector_reached": sector_reached' in src
    assert '"sector_before": sector_before' in src
    assert "sector_before = _navigation_sector(yaw_rad=before[2], sectors=sectors)" in src
    assert "sector_reached = _navigation_sector(yaw_rad=after[2], sectors=sectors)" in src
    # requested 是第三个、独立的字段 —— 不许和 reached 合并。
    assert '"sector_requested": _navigation_sector(' in src
    assert "yaw_rad=before[2] + math.radians(requested_dyaw)" in src


def test_sectors_come_from_the_policy_not_a_hardcoded_twelve():
    """`sectors` 必须取自 `policy.candidates.heading_sectors`（默认 12）。

    写死 12 会在配置改成 8/16 时静默错位：`local_scan.py` 拿到的仍是
    `sectors=12`（调用方显式传的），而事件里的 sector 换成别的分档，同一份
    `heading_coverage` 下标就再也对不上了。
    """
    policy = load_exploration_policy()
    assert policy.candidates.heading_sectors == 12, (
        "默认值变了：local_scan 的调用点显式写死 sectors=12，"
        "两处会分家，需要一起改"
    )
    src = RUNNER.read_text(encoding="utf-8")
    assert "sectors = max(1, int(policy.candidates.heading_sectors))" in src
