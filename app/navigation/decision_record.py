"""Structured DecisionRecord: first-class explainable decision data model.

This is the non-placeholder successor of the old ``components={"spatial_v2":
1.0}`` style.  A decision record contains the selected long-term intent, the
local goal / next motion command, real score breakdown, evidence and
alternatives with concrete rejection reasons.

The module is intentionally self-contained and JSON-safe so it can be unit
tested offline and used by both the CLI runner and the WebUI.
"""

from __future__ import annotations

import time
from dataclasses import asdict, dataclass, field
from typing import Any
from uuid import uuid4

REASON_CODE_EXPLORE_SEMANTIC_FRONTIER = "EXPLORE_SEMANTIC_FRONTIER"
REASON_CODE_INSPECT_ANCHOR = "INSPECT_ANCHOR_REGION"
REASON_CODE_APPROACH_TARGET = "APPROACH_TARGET"
REASON_CODE_VERIFY_TARGET = "VERIFY_TARGET"
REASON_CODE_REVISIT_PLACE = "REVISIT_PLACE"


@dataclass
class StructuredDecisionRecord:
    decision_id: str
    cycle: int
    timestamp: float
    map_revision: int
    match_state: str
    selected_intent: dict[str, Any]
    selected_goal: dict[str, Any]
    next_motion_command: dict[str, Any]
    reason_code: str
    reason_zh: str
    score: float
    score_breakdown: dict[str, float]
    evidence: dict[str, Any] = field(default_factory=dict)
    alternatives: list[dict[str, Any]] = field(default_factory=list)
    session_id: str = ""
    task_text: str = ""
    canonical_target: str = ""
    current_place_id: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "StructuredDecisionRecord":
        return cls(**value)


def build_decision_record(
    *,
    cycle: int,
    match_state: str,
    selected_intent: dict[str, Any],
    selected_goal: dict[str, Any] | None,
    next_motion_command: dict[str, Any] | None,
    score: float,
    score_breakdown: dict[str, float],
    evidence: dict[str, Any] | None = None,
    alternatives: list[dict[str, Any]] | None = None,
    map_revision: int = 0,
    session_id: str = "",
    task_text: str = "",
    canonical_target: str = "",
    current_place_id: str | None = None,
    timestamp: float | None = None,
) -> StructuredDecisionRecord:
    """Build a record whose explanations come from structured rules.

    The reason text has two sources:

    * a fixed code/rule template chosen from ``match_state``,
    * a data-driven reason assembled from the selected candidate's components.

    It never contains hidden LLM chain-of-thought.
    """
    now = timestamp if timestamp is not None else time.time()
    goal = selected_goal or {}
    command = next_motion_command or {}
    reason_code = _choose_reason_code(match_state, selected_intent, goal)
    reason_zh = build_reason_zh(
        match_state=match_state,
        selected_intent=selected_intent,
        goal=goal,
        command=command,
        score=score,
        score_breakdown=score_breakdown,
        evidence=evidence or {},
        alternatives=alternatives or [],
    )
    return StructuredDecisionRecord(
        decision_id=f"D{uuid4().hex[:10].upper()}",
        cycle=int(cycle),
        timestamp=float(now),
        map_revision=int(map_revision),
        match_state=str(match_state).upper(),
        selected_intent=selected_intent,
        selected_goal=goal,
        next_motion_command=command,
        reason_code=reason_code,
        reason_zh=reason_zh,
        score=round(float(score), 4),
        score_breakdown={k: round(float(v), 4) for k, v in score_breakdown.items()},
        evidence=evidence or {},
        alternatives=list(alternatives or []),
        session_id=session_id,
        task_text=task_text,
        canonical_target=canonical_target,
        current_place_id=current_place_id,
    )


def build_reason_zh(
    *,
    match_state: str,
    selected_intent: dict[str, Any],
    goal: dict[str, Any],
    command: dict[str, Any],
    score: float,
    score_breakdown: dict[str, float],
    evidence: dict[str, Any],
    alternatives: list[dict[str, Any]],
) -> str:
    """Rule-template Chinese reason.

    The method is kept deterministic and free of LLM chain-of-thought.
    """
    state = str(match_state).upper()
    intent_type = str(selected_intent.get("intent_type") or "")
    parts: list[str] = []
    if state in {"STRONG", "VERIFY"}:
        parts.append("当前检测到高置信目标候选，下一步转向并接近该候选进行视觉确认。")
    elif state == "PARTIAL":
        anchors = evidence.get("anchor_labels") or []
        if anchors:
            parts.append(f"已发现与目标相关的锚点“{'、'.join(str(a) for a in anchors)}”，因此优先探索其附近尚未覆盖区域。")
        else:
            parts.append("已发现部分语义证据，因此优先探索相关语义区域。")
    else:
        parts.append("当前没有发现目标或可靠锚点，因此优先探索未访问且信息增益最高的前沿。")
    if intent_type == "INSPECT_ANCHOR_REGION":
        parts.append("选择语义锚点区域进行局部检查。")
    elif intent_type == "VERIFY_TARGET":
        parts.append("选择对已发现目标候选进行验证。")
    elif intent_type == "EXPLORE_FRONTIER":
        parts.append("选择综合评分最高的前沿方向。")
    command_text = ""
    if command.get("instruction_zh"):
        command_text = str(command["instruction_zh"])
    elif goal.get("goal_type"):
        command_text = str(goal.get("goal_type") or "")
    if command_text:
        parts.append(f"下一步动作：{command_text}。")
    top_penalty = _top_penalty(score_breakdown)
    if top_penalty:
        parts.append(top_penalty)
    if alternatives:
        first = alternatives[0]
        parts.append(
            f"首选评分 {score:.2f}，次选 {first.get('score', 0.0):.2f}"
            f"（{first.get('rejected_reason_zh', '综合得分较低')}）。"
        )
    return "".join(parts)


def _choose_reason_code(
    match_state: str, selected_intent: dict[str, Any], goal: dict[str, Any]
) -> str:
    state = str(match_state).upper()
    intent_type = str(selected_intent.get("intent_type") or "")
    if intent_type == "VERIFY_TARGET":
        return REASON_CODE_VERIFY_TARGET
    if intent_type == "APPROACH_TARGET":
        return REASON_CODE_APPROACH_TARGET
    if intent_type == "INSPECT_ANCHOR_REGION":
        return REASON_CODE_INSPECT_ANCHOR
    if intent_type == "REVISIT_PLACE":
        return REASON_CODE_REVISIT_PLACE
    if state in {"STRONG", "VERIFY"}:
        return REASON_CODE_APPROACH_TARGET
    return REASON_CODE_EXPLORE_SEMANTIC_FRONTIER


def _top_penalty(breakdown: dict[str, float]) -> str:
    items = [item for item in breakdown.items() if item[1] < 0]
    if not items:
        return ""
    key, value = min(items, key=lambda item: item[1])
    label = {
        "route_cost_penalty": "路线代价",
        "visited_penalty": "已访问惩罚",
        "negative_evidence_penalty": "负证据惩罚",
        "estimated_motion_cost": "运动代价",
    }.get(key, key)
    return f"主要代价为{label} {-value:.2f}。"


def alternative_from_candidate(
    *,
    candidate_id: str,
    score: float,
    top_penalty_name: str,
    top_penalty_value: float,
) -> dict[str, Any]:
    """Build a rejected candidate entry with a concrete reason."""
    return {
        "candidate_id": candidate_id,
        "score": round(float(score), 4),
        "rejected_reason_zh": (
            f"{top_penalty_name} {top_penalty_value:.2f} 导致综合分低于首选"
        ),
    }