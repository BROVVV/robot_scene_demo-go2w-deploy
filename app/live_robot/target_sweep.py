"""§20：什么时候允许问一次整帧 VLM 目标扫描。

纯函数，不持有状态 —— 「连续空手几次」「上次扫描时刻」留在调用方，
所以 ROS 节点里那三个字段就是全部状态，不需要额外的类（§33）。
"""

from __future__ import annotations


def decide_target_sweep(
    *,
    has_target_like: bool,
    empty_cycles: int,
    last_sweep_time: float,
    now: float,
    empty_cycles_threshold: int,
    interval_seconds: float,
) -> tuple[bool, int, float]:
    """返回 (是否扫描, 新的连续空手次数, 新的上次扫描时刻)。

    §44.7：连续 `empty_cycles_threshold` 次没有 target-like 候选，且距上次扫描
    `interval_seconds` 以上，才允许扫；扫完计数清零，所以下一周期不会马上再扫。
    §44.8：只要出现 target-like 候选，计数立刻清零。
    """
    if has_target_like:
        return False, 0, last_sweep_time
    empty_cycles += 1
    if empty_cycles < empty_cycles_threshold:
        return False, empty_cycles, last_sweep_time
    if now - last_sweep_time < interval_seconds:
        return False, empty_cycles, last_sweep_time
    return True, 0, now
