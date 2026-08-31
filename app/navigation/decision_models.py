"""Explainable local-motion and planning decision contracts."""

from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass, field
from typing import Any
from uuid import uuid4


@dataclass(frozen=True)
class NextMotionCommand:
    command_id: str
    plan_id: str
    decision_id: str
    action_type: str
    turn_deg: float = 0.0
    forward_m: float = 0.0
    lateral_m: float = 0.0
    stop_and_reobserve: bool = True
    instruction_zh: str = ""
    reason_zh: str = ""
    target_place_id: str | None = None
    target_frontier_id: str | None = None
    target_object_id: str | None = None
    safety_limited: bool = False
    requested_motion: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class DecisionRecord:
    decision_id: str
    session_id: str
    task_id: str
    cycle: int
    timestamp: float
    raw_task_text: str
    canonical_target: str
    map_revision: int
    current_place_id: str | None
    current_pose: dict[str, Any] | None
    target_match_level: str
    selected_long_term_goal: dict[str, Any]
    candidate_ranking: list[dict[str, Any]]
    navigation_plan: dict[str, Any]
    next_motion_command: dict[str, Any]
    reason_zh: str
    evidence: list[dict[str, Any]] = field(default_factory=list)
    negative_evidence: list[dict[str, Any]] = field(default_factory=list)
    requested_motion: dict[str, Any] | None = None
    observed_motion: dict[str, Any] | None = None
    execution_status: str = "PLANNED"
    execution_message: str = ""
    replan_reason: str | None = None

    @classmethod
    def create(
        cls,
        *,
        session_id: str,
        task_id: str,
        cycle: int,
        raw_task_text: str,
        canonical_target: str,
        current_place_id: str | None,
        current_pose: dict[str, Any] | None,
        target_match_level: str,
        selected_long_term_goal: dict[str, Any],
        candidate_ranking: list[dict[str, Any]],
        navigation_plan: dict[str, Any],
        next_motion_command: NextMotionCommand,
        reason_zh: str,
        map_revision: int = 0,
        evidence: list[dict[str, Any]] | None = None,
        negative_evidence: list[dict[str, Any]] | None = None,
    ) -> "DecisionRecord":
        return cls(
            decision_id=f"decision_{uuid4().hex[:12]}",
            session_id=session_id,
            task_id=task_id,
            cycle=int(cycle),
            timestamp=time.time(),
            raw_task_text=raw_task_text,
            canonical_target=canonical_target,
            map_revision=int(map_revision),
            current_place_id=current_place_id,
            current_pose=current_pose,
            target_match_level=target_match_level,
            selected_long_term_goal=selected_long_term_goal,
            candidate_ranking=candidate_ranking,
            navigation_plan=navigation_plan,
            next_motion_command=next_motion_command.to_dict(),
            reason_zh=reason_zh,
            evidence=list(evidence or []),
            negative_evidence=list(negative_evidence or []),
            requested_motion=dict(next_motion_command.requested_motion),
        )

    def with_execution(
        self,
        *,
        status: str,
        message: str = "",
        requested_motion: dict[str, Any] | None = None,
        observed_motion: dict[str, Any] | None = None,
        replan_reason: str | None = None,
    ) -> "DecisionRecord":
        values = asdict(self)
        values.update(
            {
                "execution_status": status,
                "execution_message": message,
                "requested_motion": requested_motion if requested_motion is not None else self.requested_motion,
                "observed_motion": observed_motion if observed_motion is not None else self.observed_motion,
                "replan_reason": replan_reason,
            }
        )
        return type(self)(**values)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def signed_yaw_instruction(turn_deg: float, forward_m: float = 0.0) -> str:
    """Render the single global convention: positive yaw means left.

    A negative ``forward_m`` means breadcrumb-safe backward recovery; it is
    never masked into a forward instruction.
    """
    turn = float(turn_deg)
    parts: list[str] = []
    if abs(turn) >= 0.05:
        side = "左转" if turn > 0 else "右转"
        parts.append(f"{side} {abs(turn):.0f}°")
    forward = float(forward_m)
    if abs(forward) >= 0.005:
        direction = "前进" if forward > 0.0 else "后退"
        parts.append(f"{direction} {abs(forward):.2f} m")
    if not parts:
        parts.append("原地停止")
    return "，然后".join(parts) + "，停止并重新观察"


def make_motion_command(
    *,
    plan_id: str,
    decision_id: str,
    turn_deg: float = 0.0,
    forward_m: float = 0.0,
    reason_zh: str = "",
    action_type: str = "TURN_AND_MOVE",
    target_place_id: str | None = None,
    target_frontier_id: str | None = None,
    target_object_id: str | None = None,
    safety_limited: bool = False,
) -> NextMotionCommand:
    turn = float(turn_deg)
    forward = float(forward_m)
    return NextMotionCommand(
        command_id=f"motion_{uuid4().hex[:12]}",
        plan_id=plan_id,
        decision_id=decision_id,
        action_type=action_type,
        turn_deg=turn,
        forward_m=forward,
        instruction_zh=signed_yaw_instruction(turn, forward),
        reason_zh=reason_zh,
        target_place_id=target_place_id,
        target_frontier_id=target_frontier_id,
        target_object_id=target_object_id,
        safety_limited=bool(safety_limited),
        requested_motion={"turn_deg": turn, "forward_m": forward},
    )
