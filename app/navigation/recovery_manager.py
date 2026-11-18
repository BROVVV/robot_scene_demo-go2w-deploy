"""RecoveryManager: central recovery decision for the real Go2-W.

It keeps all reverse-recovery policy in one place instead of splitting it
between the explorer, backend and step executor.  It is fail-closed: without
a valid breadcrumb there is no autonomous backward.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Any

from app.live_robot.recovery_motion import (
    BackwardSafetyDecision,
    SafeMotionSegment,
    evaluate_backward_safety,
)
from app.navigation.models import (
    GOAL_NAVIGATE_POSE,
    GOAL_RELATIVE_MOVE,
    ExplorationGoal,
)
from app.navigation.robot_backend import NavigationResult, NavigationStatus
from app.navigation.safety_models import SWEPT_HALF_ANGLE_DEG, LocalSafetySnapshot


# Plan section 13.2: a stuck classification is a first-class recovery trigger,
# even when no navigation goal failed.  SENSOR_DEGRADED is deliberately not
# here - blind reversing is forbidden (plan section 44).
STUCK_STATE_TRIGGERS = {
    "CORNERED": "CORNERED",
    "CAMERA_OCCLUDED": "CAMERA_OCCLUDED_BY_NEAR_WALL",
    "FRONT_BLOCKED": "FRONT_BLOCKED",
    "NO_PROGRESS": "NO_PROGRESS",
    "RECOVERY_REQUIRED": "RECOVERY_REQUIRED",
}


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
        result: NavigationResult | None = None,
        breadcrumb: SafeMotionSegment | None,
        current_pose: tuple[float, float, float],
        front_clearance_m: float | None = None,
        forward_min_clearance_m: float | None = None,
        camera_occluded: bool = False,
        stuck_state: Any = None,
        goal_id: str = "recovery",
        recovery_reason: str = "",
        metric_navigation: bool = False,
    ) -> RecoveryDecision:
        if not self.config.enabled:
            return RecoveryDecision(reason="recovery disabled")
        trigger = self._trigger(
            result, front_clearance_m, forward_min_clearance_m, camera_occluded, stuck_state
        )
        if not trigger:
            return RecoveryDecision(reason="no recovery trigger")
        if result is not None and result.status == NavigationStatus.OPERATOR_STOP:
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
        goal = self._backward_goal(
            breadcrumb=breadcrumb,
            distance_m=safety.distance_m,
            trigger=trigger,
            recovery_reason=recovery_reason,
            goal_id=goal_id,
            metric_navigation=metric_navigation,
        )
        if goal is None:
            # 计量不到目标就不算用掉预算：没动过，不该扣次数。
            return RecoveryDecision(
                trigger=trigger,
                reason="NO_METRIC_BACKWARD_TARGET: metric 后端下表达不出后退目标位姿",
                safety=safety,
            )
        self._consecutive += 1
        self._total_m = round(self._total_m + safety.distance_m, 4)
        return RecoveryDecision(
            trigger=trigger,
            should_stop=False,
            backward_allowed=True,
            backward_goal=goal,
            reason="backward_recovery_allowed",
            safety=safety,
        )

    def _backward_goal(
        self,
        *,
        breadcrumb: SafeMotionSegment | None,
        distance_m: float,
        trigger: str,
        recovery_reason: str,
        goal_id: str,
        metric_navigation: bool,
    ) -> ExplorationGoal | None:
        """计划书 §6.7：倒车恢复也要表达成地图系 NAVIGATE_POSE。

        倒车走的正是刚刚成功走过的那段（breadcrumb），所以它的 ``start_pose``
        就是合法的地图系目标；只有拿不到这段面包屑时才返回 ``None``，宁可
        停在原地重选，也不发相对后退。
        """
        sequence = f"{goal_id}_{self._consecutive + 1:03d}"
        if metric_navigation:
            if breadcrumb is None:
                return None
            x, y, yaw = breadcrumb.start_pose
            return ExplorationGoal(
                goal_id=sequence,
                goal_type=GOAL_NAVIGATE_POSE,
                position=(float(x), float(y)),
                yaw=math.degrees(float(yaw)),
                frame="map",
                semantic_reason=f"recovery {trigger} -> back out {distance_m:.2f}m",
                provenance={"recovery_reason": trigger or recovery_reason},
            )
        return ExplorationGoal(
            goal_id=sequence,
            goal_type=GOAL_RELATIVE_MOVE,
            relative_dx=-distance_m,
            semantic_reason=f"recovery {trigger} -> backward {distance_m:.2f}m",
            provenance={"recovery_reason": trigger or recovery_reason},
        )

    @staticmethod
    def _trigger(
        result: NavigationResult | None,
        front_clearance_m: float | None,
        forward_min_clearance_m: float | None,
        camera_occluded: bool,
        stuck_state: Any = None,
    ) -> str:
        if result is not None:
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
        state = str(getattr(stuck_state, "value", stuck_state) or "")
        if state in STUCK_STATE_TRIGGERS:
            return STUCK_STATE_TRIGGERS[state]
        if camera_occluded:
            return "CAMERA_OCCLUDED_BY_NEAR_WALL"
        if (
            front_clearance_m is not None
            and forward_min_clearance_m is not None
            and front_clearance_m < forward_min_clearance_m
        ):
            return "FRONT_BLOCKED"
        return ""

@dataclass(frozen=True)
class EscapeRotation:
    """Plan section 14: the in-place turn used when backward is unavailable."""

    yaw_deg: float = 0.0
    allowed: bool = False
    post_turn_clearance_m: float | None = None
    view_quality: float = 0.0
    reason: str = ""
    rejected: tuple[dict[str, Any], ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return {
            "yaw_deg": round(self.yaw_deg, 1),
            "allowed": self.allowed,
            "post_turn_clearance_m": (
                None
                if self.post_turn_clearance_m is None
                else round(self.post_turn_clearance_m, 3)
            ),
            "view_quality": round(self.view_quality, 3),
            "reason": self.reason,
            "rejected": [dict(item) for item in self.rejected],
        }


def select_escape_rotation(
    safety: LocalSafetySnapshot,
    *,
    candidates_deg: Any = (10.0, -10.0, 15.0, -15.0, 20.0, -20.0, 30.0, -30.0),
    rotation_clearance_m: float = 0.40,
    swept_half_angle_deg: float = SWEPT_HALF_ANGLE_DEG,
    view_quality: Any = None,
) -> EscapeRotation:
    """Pick the smallest safe turn that also opens up the camera.

    A candidate must clear three gates: the rotation swept volume up to that
    yaw, observable geometry on the post-turn heading, and enough post-turn
    clearance.  If none passes, the caller must stop - never turn blind.
    """
    if not safety.lidar_fresh or not safety.geometry_validated:
        return EscapeRotation(
            reason="RECOVERY_BLOCKED: rotation needs fresh validated geometry"
        )
    rejected: list[dict[str, Any]] = []
    best: EscapeRotation | None = None
    for candidate in candidates_deg:
        yaw = float(candidate)
        span = (
            safety.safe_turn_left_deg if yaw >= 0.0 else safety.safe_turn_right_deg
        )
        if abs(yaw) > span:
            rejected.append(
                {
                    "yaw_deg": yaw,
                    "reason": f"ROTATION_SWEPT_BLOCKED: safe span {span:.0f}deg",
                }
            )
            continue
        clearance = safety.min_clearance(yaw, swept_half_angle_deg)
        if clearance is None:
            rejected.append({"yaw_deg": yaw, "reason": "POST_TURN_UNOBSERVABLE"})
            continue
        if clearance < rotation_clearance_m:
            rejected.append(
                {
                    "yaw_deg": yaw,
                    "reason": f"POST_TURN_CLEARANCE_TOO_LOW: {clearance:.2f}m",
                }
            )
            continue
        quality = (
            float(view_quality(yaw)) if view_quality is not None else min(1.0, clearance / 2.5)
        )
        candidate_result = EscapeRotation(
            yaw_deg=yaw,
            allowed=True,
            post_turn_clearance_m=clearance,
            view_quality=quality,
            reason="escape_rotation_allowed",
        )
        if best is None or (quality, clearance) > (best.view_quality, best.post_turn_clearance_m):
            best = candidate_result
    if best is None:
        return EscapeRotation(
            reason="RECOVERY_BLOCKED: no safe escape rotation", rejected=tuple(rejected)
        )
    return replace(best, rejected=tuple(rejected))
