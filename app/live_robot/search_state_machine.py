"""Fail-closed live search state machine independent of ROS and motion APIs."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any


class SearchState(str, Enum):
    INITIALIZE = "INITIALIZE"
    WAIT_FOR_SENSORS = "WAIT_FOR_SENSORS"
    OBSERVE = "OBSERVE"
    UNDERSTAND_SCENE = "UNDERSTAND_SCENE"
    DETECT_TARGET = "DETECT_TARGET"
    VERIFY_TARGET = "VERIFY_TARGET"
    LOCALIZE_TARGET = "LOCALIZE_TARGET"
    TARGET_CONFIRMED = "TARGET_CONFIRMED"
    SELECT_NEXT_VIEW = "SELECT_NEXT_VIEW"
    CHECK_SAFETY = "CHECK_SAFETY"
    PLAN_STEP = "PLAN_STEP"
    MOVE = "MOVE"
    WAIT_FOR_STOP = "WAIT_FOR_STOP"
    REOBSERVE = "REOBSERVE"
    NAV2_PLAN = "NAV2_PLAN"
    NAV2_EXECUTE = "NAV2_EXECUTE"
    FINISH = "FINISH"
    FAILED = "FAILED"
    EMERGENCY_STOP = "EMERGENCY_STOP"


class SearchMode(str, Enum):
    OBSERVE_ONLY = "observe_only"
    STEP_SEARCH = "step_search"
    NAV2_PLAN_ONLY = "nav2_plan_only"
    NAV2_EXECUTE = "nav2_execute"


@dataclass(frozen=True)
class SensorSnapshot:
    camera_fresh: bool
    lidar_fresh: bool
    robot_stationary: bool
    tf_ready: bool = False
    extrinsics_ready: bool = False
    lio_fresh: bool = False
    # Fail-closed rotation/dual-LiDAR gate data (optional, default False so
    # existing callers that do not provide them remain safe).
    rotation_clearance_valid: bool = False
    dual_lidar_clearance_valid: bool = False
    pandar_raw_fresh: bool = False


@dataclass(frozen=True)
class VisualEvidence:
    bbox: bool
    mask: bool
    crop_verify: bool
    track_vote: bool
    evidence_gate: bool
    frame_available: bool = True
    source: str = "visual_detector"
    require_mask: bool = True
    require_track_vote: bool = True

    @property
    def confirmed(self) -> bool:
        return bool(
            self.source != "llm_context"
            and self.frame_available
            and self.bbox
            and (not self.require_mask or self.mask)
            and self.crop_verify
            and (not self.require_track_vote or self.track_vote)
            and self.evidence_gate
        )


class InvalidTransition(RuntimeError):
    pass


class SearchStateMachine:
    def __init__(
        self,
        mode: SearchMode | str = SearchMode.OBSERVE_ONLY,
        *,
        motion_allowed: bool = False,
        stop_settle_seconds: float = 0.8,
    ) -> None:
        self.mode = SearchMode(mode)
        self.motion_allowed = bool(motion_allowed)
        self.stop_settle_seconds = float(stop_settle_seconds)
        self.state = SearchState.INITIALIZE
        self.spatial_state = "target_2d_only"
        self.trace: list[dict[str, Any]] = []
        self._record("created")

    def start(self) -> SearchState:
        self._require(SearchState.INITIALIZE)
        return self._transition(SearchState.WAIT_FOR_SENSORS, "initialized")

    def sensors(self, snapshot: SensorSnapshot, *, require_3d: bool = False) -> SearchState:
        self._require(SearchState.WAIT_FOR_SENSORS, SearchState.REOBSERVE)
        ready = snapshot.camera_fresh and snapshot.lidar_fresh and snapshot.robot_stationary
        if require_3d:
            ready = ready and snapshot.tf_ready and snapshot.extrinsics_ready
        if not ready:
            self._record("sensor_gate_closed", snapshot=asdict(snapshot))
            return self.state
        return self._transition(SearchState.OBSERVE, "sensor_gate_passed")

    def observation_elapsed(self, seconds: float) -> SearchState:
        self._require(SearchState.OBSERVE)
        if seconds < self.stop_settle_seconds:
            self._record("waiting_for_sensor_settle", elapsed_seconds=seconds)
            return self.state
        return self._transition(SearchState.UNDERSTAND_SCENE, "observation_stable")

    def scene_understood(self) -> SearchState:
        self._require(SearchState.UNDERSTAND_SCENE)
        return self._transition(SearchState.DETECT_TARGET, "scene_understood")

    def detection(self, detected: bool) -> SearchState:
        self._require(SearchState.DETECT_TARGET)
        return self._transition(
            SearchState.VERIFY_TARGET if detected else SearchState.SELECT_NEXT_VIEW,
            "visual_candidate" if detected else "target_not_seen",
        )

    def verify(self, evidence: VisualEvidence) -> SearchState:
        self._require(SearchState.VERIFY_TARGET)
        if not evidence.confirmed:
            return self._transition(
                SearchState.SELECT_NEXT_VIEW,
                "visual_evidence_gate_failed",
                evidence=asdict(evidence),
            )
        return self._transition(
            SearchState.LOCALIZE_TARGET,
            "visual_evidence_confirmed",
            evidence=asdict(evidence),
        )

    def localization(self, localized_3d: bool, pose_frame: str | None = None) -> SearchState:
        self._require(SearchState.LOCALIZE_TARGET)
        if localized_3d:
            if pose_frame not in {"base_link", "odom", "map"}:
                raise ValueError("3D localization requires base_link, odom, or map frame")
            self.spatial_state = {
                "base_link": "target_3d_relative",
                "odom": "target_3d_odom",
                "map": "target_3d_map",
            }[pose_frame]
        else:
            self.spatial_state = "target_2d_only"
        return self._transition(
            SearchState.TARGET_CONFIRMED,
            "target_confirmed_with_spatial_provenance",
            spatial_state=self.spatial_state,
        )

    def finish_confirmed(self) -> SearchState:
        self._require(SearchState.TARGET_CONFIRMED)
        return self._transition(SearchState.FINISH, "evidence_saved_robot_remains_stopped")

    def next_view_unavailable(self) -> SearchState:
        self._require(SearchState.SELECT_NEXT_VIEW)
        if self.mode == SearchMode.OBSERVE_ONLY:
            return self._transition(SearchState.FINISH, "observe_only_no_motion")
        return self.next_view_selected(
            source="legacy", directive_id="legacy_unavailable", confidence=1.0
        )

    def next_view_selected(
        self, *, source: str, directive_id: str, confidence: float
    ) -> SearchState:
        """Record an auditable semantic/legacy next-view choice."""
        self._require(SearchState.SELECT_NEXT_VIEW)
        if self.mode == SearchMode.OBSERVE_ONLY:
            return self._transition(SearchState.FINISH, "observe_only_no_motion")
        return self._transition(
            SearchState.CHECK_SAFETY,
            "next_view_selected",
            source=str(source),
            directive_id=str(directive_id),
            confidence=max(0.0, min(1.0, float(confidence))),
        )

    def safety_checked(self, safe: bool) -> SearchState:
        self._require(SearchState.CHECK_SAFETY)
        if not safe:
            return self._transition(SearchState.FAILED, "safety_gate_failed")
        if not self.motion_allowed:
            return self._transition(SearchState.FAILED, "motion_not_authorized")
        return self._transition(SearchState.PLAN_STEP, "safety_gate_passed")

    def plan_step(self, step: str) -> SearchState:
        """Accept a concrete short step (f/lN/rN) and enter MOVE."""
        self._require(SearchState.PLAN_STEP)
        if not isinstance(step, str) or not step.strip():
            raise ValueError("planned step must be a non-empty string")
        return self._transition(SearchState.MOVE, "step_planned", step=step)

    def motion_started(self, step: str) -> SearchState:
        """Record that the planned step has been handed to the executor."""
        self._require(SearchState.MOVE)
        return self._transition(SearchState.WAIT_FOR_STOP, "motion_started",
                                step=step)

    def motion_completed(self, step: str, verified: bool = True) -> SearchState:
        """Record that the step ended and the robot is stationary again."""
        self._require(SearchState.WAIT_FOR_STOP)
        return self._transition(
            SearchState.REOBSERVE,
            "motion_completed" if verified else "motion_not_verified",
            step=step,
            verified=bool(verified),
        )

    def emergency_stop(self, reason: str) -> SearchState:
        return self._transition(SearchState.EMERGENCY_STOP, reason)

    def _require(self, *states: SearchState) -> None:
        if self.state not in states:
            expected = ", ".join(item.value for item in states)
            raise InvalidTransition(f"state {self.state.value}; expected {expected}")

    def _transition(self, state: SearchState, reason: str, **details) -> SearchState:
        self.state = state
        self._record(reason, **details)
        return state

    def _record(self, reason: str, **details) -> None:
        self.trace.append(
            {"sequence": len(self.trace), "state": self.state.value, "reason": reason, **details}
        )
