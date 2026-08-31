"""RecoveryManager: central recovery decision for the real Go2-W.

It keeps all reverse-recovery policy in one place instead of splitting it
between the explorer, backend and step executor.  It is fail-closed: without
a valid breadcrumb there is no autonomous backward.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from app.live_robot.recovery_motion import (
    BackwardSafetyDecision,
    SafeMotionSegment,
    evaluate_backward_safety,
)
from app.navigation.models import GOAL_RELATIVE_MOVE, ExplorationGoal
from app.navigation.robot_backend import NavigationResult, NavigationStatus


@dataclass
class RecoveryDecision:
    trigger: str = ""
    should_stop: bool = False
    backward_allowed: bool = False
    backward_goal: ExplorationGoal | None = None
    reason: str = ""
    safety: BackwardSafetyDecision = field(
        default_factory=lambda: BackwardSafetyDecision(False)
    )

    def to_dict(self) -> dict[str, Any]:
        return {
            "trigger": self.trigger,
            "should_stop": self.should_stop,
            "backward_allowed": self.backward_allowed,
            "backward_goal": (
                self.backward_goal.to_dict() if self.backward_goal else None
            ),
            "reason": self.reason,
            "safety": {
                "allowed": self.safety.allowed,
                "distance_m": self.safety.distance_m,
                "reason": self.safety.reason,
                "source": self.safety.source,
            },
        }


@dataclass
class RecoveryManagerConfig:
    enabled: bool = True
    max_backward_step_m: float = 0.12
    min_backward_step_m: float = 0.05
    max_age_sec: float = 8.0
    heading_tolerance_deg: float = 8.0
    max_consecutive: int = 2
    max_total_m: float = 0.36


class RecoveryManager:
    def __init__(self, config: RecoveryManagerConfig | None = None) -> None:
        self.config = config or RecoveryManagerConfig()
        self._consecutive = 0
        self._total_m = 0.0

    def reset_budget(self) -> None:
        self._consecutive = 0
        self._total_m = 0.0

    def decide(
        self,
        *,
        result: NavigationResult,
        breadcrumb: SafeMotionSegment | None,
        current_pose: tuple[float, float, float],
        front_clearance_m: float | None = None,
        forward_min_clearance_m: float | None = None,
        camera_occluded: bool = False,
        goal_id: str = "recovery",
        recovery_reason: str = "",
    ) -> RecoveryDecision:
        if not self.config.enabled:
            return RecoveryDecision(reason="recovery disabled")
        trigger = self._trigger(result, front_clearance_m, forward_min_clearance_m, camera_occluded)
        if not trigger:
            return RecoveryDecision(reason="no recovery trigger")
        if result.status == NavigationStatus.OPERATOR_STOP:
            return RecoveryDecision(trigger=trigger, should_stop=True, reason="operator stop")
        if self._consecutive >= self.config.max_consecutive:
            return RecoveryDecision(
                trigger=trigger,
                reason="RECOVERY_BUDGET_EXCEEDED: too many consecutive recoveries",
            )
        if self._total_m >= self.config.max_total_m:
            return RecoveryDecision(
                trigger=trigger,
                reason="RECOVERY_BUDGET_EXCEEDED: cumulative recovery distance exceeded",
            )
        safety = evaluate_backward_safety(
            breadcrumb,
            current_pose=current_pose,
            requested_distance_m=self.config.max_backward_step_m,
            max_backward_step_m=self.config.max_backward_step_m,
            min_backward_step_m=self.config.min_backward_step_m,
            max_age_sec=self.config.max_age_sec,
            heading_tolerance_deg=self.config.heading_tolerance_deg,
        )
        if not safety.allowed:
            return RecoveryDecision(
                trigger=trigger,
                reason=safety.reason,
                safety=safety,
            )
        self._consecutive += 1
        self._total_m = round(self._total_m + safety.distance_m, 4)
        goal = ExplorationGoal(
            goal_id=f"{goal_id}_{self._consecutive:03d}",
            goal_type=GOAL_RELATIVE_MOVE,
            relative_dx=-safety.distance_m,
            semantic_reason=f"recovery {trigger} -> backward {safety.distance_m:.2f}m",
            provenance={"recovery_reason": trigger or recovery_reason},
        )
        return RecoveryDecision(
            trigger=trigger,
            should_stop=False,
            backward_allowed=True,
            backward_goal=goal,
            reason="backward_recovery_allowed",
            safety=safety,
        )

    @staticmethod
    def _trigger(
        result: NavigationResult,
        front_clearance_m: float | None,
        forward_min_clearance_m: float | None,
        camera_occluded: bool,
    ) -> str:
        if result.status == NavigationStatus.TIMEOUT:
            return "FORWARD_NOT_CONFIRMED"
        if result.failed and "FORWARD_NOT_CONFIRMED" in (result.message or ""):
            return "FORWARD_NOT_CONFIRMED"
        if (
            result.failed
            and "FRONT" in (result.message or "").upper()
            and "CLEARANCE" in (result.message or "").upper()
        ):
            return "FRONT_BLOCKED"
        if camera_occluded:
            return "CAMERA_OCCLUDED_BY_NEAR_WALL"
        if (
            front_clearance_m is not None
            and forward_min_clearance_m is not None
            and front_clearance_m < forward_min_clearance_m
        ):
            return "FRONT_BLOCKED"
        return ""