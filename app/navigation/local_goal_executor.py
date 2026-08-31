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
        max_turn_deg: float = 30.0,
        turn_only: bool = False,
    ) -> None:
        self.forward_step_m = float(forward_step_m)
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
        capabilities: RobotCapabilities | None = None,
    ) -> ExplorationGoal | None:
        if self._intent is None:
            return None
        capabilities = capabilities or RobotCapabilities()
        allowed = {
            str(item).upper()
            for item in (capabilities.allowed_motion_primitives or ())
        }
        intent = self._intent
        self._goal_seq += 1
        if self._phase == "rotate":
            bearing = self._preferred_bearing(intent, current_yaw_deg)
            delta = self._normalize_deg(bearing - current_yaw_deg)
            if abs(delta) < 5.0:
                self._phase = "move"
                return self.next_goal(current_yaw_deg=current_yaw_deg, capabilities=capabilities)
            if capabilities.supports_metric_navigation and intent.preferred_position is not None:
                self._phase = "done"
                return ExplorationGoal(
                    goal_id=f"local_{self._goal_seq:03d}",
                    goal_type=GOAL_NAVIGATE_POSE,
                    position=tuple(intent.preferred_position),
                    yaw=bearing,
                    frame=intent.provenance.get("frame", "map") if isinstance(intent.provenance, dict) else "map",
                    semantic_reason=intent.semantic_reason,
                    expected_information_gain=intent.spatial_gain,
                    provenance={"source": "local_goal_executor", "intent": intent.intent_type},
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
            return ExplorationGoal(
                goal_id=f"local_{self._goal_seq:03d}",
                goal_type=GOAL_RELATIVE_MOVE,
                relative_dx=self.forward_step_m,
                semantic_reason=intent.semantic_reason,
                expected_information_gain=intent.spatial_gain * 0.5,
                provenance={"source": "local_goal_executor", "intent": intent.intent_type},
            )
        return None

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
