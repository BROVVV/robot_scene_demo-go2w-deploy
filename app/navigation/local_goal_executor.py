"""LocalGoalExecutor: translate long-term spatial intents into platform
primitives (ROTATE_VIEW / RELATIVE_MOVE / NAVIGATE_POSE)."""

from __future__ import annotations

import math
from typing import Any

from app.navigation.models import (
    GOAL_NAVIGATE_POSE,
    GOAL_RELATIVE_MOVE,
    GOAL_ROTATE_VIEW,
    ExplorationGoal,
)
from app.navigation.robot_backend import RobotCapabilities
from app.spatial.models import (
    INTENT_APPROACH_TARGET,
    INTENT_EXPLORE_FRONTIER,
    INTENT_INSPECT_ANCHOR_REGION,
    INTENT_REVISIT_PLACE,
    INTENT_VERIFY_TARGET,
    ExplorationIntent,
)


class LocalGoalExecutor:
    """Stateful local executor for one long-term spatial intent.

    For the current Go2-W (relative backend) an EXPLORE_FRONTIER intent is
    decomposed as: rotate toward the frontier bearing -> short forward -> done.
    The executor exposes ``next_goal()``; the caller is responsible for calling
    it after each navigation result.
    """

    def __init__(
        self,
        *,
        forward_step_m: float = 1.5,
        max_forward_step_m: float | None = None,
        max_turn_deg: float = 30.0,
        turn_only: bool = False,
    ) -> None:
        self.forward_step_m = float(forward_step_m)
        self.max_forward_step_m = float(
            forward_step_m if max_forward_step_m is None else max_forward_step_m
        )
        self._metric_step_limit = max_forward_step_m
        self.max_turn_deg = float(max_turn_deg)
        self.turn_only = bool(turn_only)
        self._intent: ExplorationIntent | None = None
        self._phase = "idle"
        self._goal_seq = 0

    def begin(self, intent: ExplorationIntent) -> None:
        self._intent = intent
        self._phase = "rotate"
        self._goal_seq = 0

    @property
    def active(self) -> bool:
        return self._intent is not None

    def next_goal(
        self,
        *,
        current_yaw_deg: float = 0.0,
        current_position: tuple[float, float] | None = None,
        capabilities: RobotCapabilities | None = None,
    ) -> ExplorationGoal | None:
        if self._intent is None:
            return None
        if self._phase == "done":
            return None
        capabilities = capabilities or RobotCapabilities()
        allowed = {
            str(item).upper()
            for item in (capabilities.allowed_motion_primitives or ())
        }
        intent = self._intent
        self._goal_seq += 1
        if capabilities.supports_metric_navigation:
            return self._metric_goal(
                intent, current_yaw_deg=current_yaw_deg, current_position=current_position
            )
        if self._phase == "rotate":
            bearing = self._preferred_bearing(intent, current_yaw_deg)
            delta = self._normalize_deg(bearing - current_yaw_deg)
            if abs(delta) < 5.0:
                self._phase = "move"
                return self.next_goal(
                    current_yaw_deg=current_yaw_deg,
                    current_position=current_position,
                    capabilities=capabilities,
                )
            self._phase = "move"
            return ExplorationGoal(
                goal_id=f"local_{self._goal_seq:03d}",
                goal_type=GOAL_ROTATE_VIEW,
                relative_dyaw=max(-self.max_turn_deg, min(self.max_turn_deg, delta)),
                semantic_reason=intent.semantic_reason,
                heading_sector=None,
                expected_information_gain=intent.spatial_gain,
                provenance={"source": "local_goal_executor", "intent": intent.intent_type},
            )
        if self._phase == "move":
            self._phase = "done"
            if (
                self.turn_only
                or not capabilities.supports_relative_translation
                or (allowed and "FORWARD" not in allowed)
            ):
                return None
            # Frontier range is relative to the observation, so it can be used
            # without subtracting positions from potentially different frames.
            frontier = intent.provenance.get("frontier") or {}
            distance = frontier.get("distance_m")
            distance = self.forward_step_m if distance is None else float(distance)
            if not math.isfinite(distance) or distance <= 0:
                return None
            return ExplorationGoal(
                goal_id=f"local_{self._goal_seq:03d}",
                goal_type=GOAL_RELATIVE_MOVE,
                relative_dx=min(distance, self.max_forward_step_m),
                semantic_reason=intent.semantic_reason,
                expected_information_gain=intent.spatial_gain * 0.5,
                provenance={"source": "local_goal_executor", "intent": intent.intent_type},
            )
        return None

    def _metric_goal(
        self,
        intent: ExplorationIntent,
        *,
        current_yaw_deg: float,
        current_position: tuple[float, float] | None,
    ) -> ExplorationGoal | None:
        """计划书 §6.7：Nav2 模式下每个动作都表达成地图系 ``NAVIGATE_POSE``。

        观察转向也一样——写成「当前位置 + 目标朝向」而不是相对转动。这样每一
        次位移都要经过 Nav2 的规划、走廊分析和碰撞监视，没有旁路；相对后端那套
        「先转 30°、再前进一段」的两段式在这里整个消失。
        """
        bearing = self._preferred_bearing(intent, current_yaw_deg)
        target = intent.preferred_position
        if self.turn_only:
            target = current_position
        if target is None:
            # 没有偏好位置时，观察转向退化成「站在原地转到目标朝向」。
            target = current_position
        self._phase = "done"
        if target is None:
            # 既无偏好位置又拿不到当前位姿：表达不出地图系目标，让上层换候选，
            # 绝不退回相对转动。
            return None
        if current_position is not None and self._metric_step_limit is not None:
            dx, dy = target[0] - current_position[0], target[1] - current_position[1]
            distance = math.hypot(dx, dy)
            if distance > self.max_forward_step_m:
                scale = self.max_forward_step_m / distance
                target = (current_position[0] + dx * scale, current_position[1] + dy * scale)
        frame = "map"
        if isinstance(intent.provenance, dict):
            frame = str(intent.provenance.get("frame") or "map")
        return ExplorationGoal(
            goal_id=f"local_{self._goal_seq:03d}",
            goal_type=GOAL_NAVIGATE_POSE,
            position=(float(target[0]), float(target[1])),
            yaw=bearing,
            frame=frame,
            semantic_reason=intent.semantic_reason,
            expected_information_gain=intent.spatial_gain,
            provenance={"source": "local_goal_executor", "intent": intent.intent_type},
        )

    def finish(self) -> None:
        self._intent = None
        self._phase = "idle"

    @staticmethod
    def _preferred_bearing(intent: ExplorationIntent, current_yaw_deg: float) -> float:
        if intent.preferred_bearing_deg is not None:
            return float(intent.preferred_bearing_deg)
        return current_yaw_deg

    @staticmethod
    def _normalize_deg(value: float) -> float:
        return (value + 180.0) % 360.0 - 180.0
