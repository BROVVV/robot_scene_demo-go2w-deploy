"""Route policy helpers for high-level task plans."""

from __future__ import annotations


def approach_description(target: str) -> str:
    if any(keyword in target for keyword in ["门", "房间", "走廊"]):
        return f"沿可通行方向靠近{target}，保持侧向观察门牌或门缝。"
    return f"缓慢靠近{target}，保持避障距离并准备重新观察。"


def fallback_observation_description() -> str:
    return "如果当前验证点没有结果，退回到能覆盖全局的位置重新观察。"
