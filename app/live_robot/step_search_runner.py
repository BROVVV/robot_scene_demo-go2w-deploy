"""Framework-level short-step search runner for the Go2-W.

The runner drives the fail-closed ``SearchStateMachine`` and the pure
``step_planner`` helpers. All hardware access is injected as callables, so the
module stays ROS-independent and unit-testable; the real Go2-W executor wires
these callables to the LLM quick worker, the LLM verify worker and the
``/go2w/motion`` action server.
"""

from __future__ import annotations

from copy import deepcopy
import time
from dataclasses import dataclass
from typing import Any, Callable

from app.live_robot.search_state_machine import (
    SearchMode,
    SearchState,
    SearchStateMachine,
    SensorSnapshot,
    VisualEvidence,
)
from app.live_robot.step_planner import (
    PlanKind,
    StepPlan,
    plan_approach_step,
    plan_scan_step,
    verify_rejection_step,
)
from app.live_robot.search_directive_adapter import directive_to_step_plan
from app.reasoning.semantic_navigation.models import SearchDirective, SearchReasoningContext


@dataclass(frozen=True)
class Detection:
    label: str
    score: float
    bbox: tuple[float, float, float, float]

    @property
    def center_x(self) -> float:
        return (self.bbox[0] + self.bbox[2]) / 2.0

    @property
    def area_ratio(self) -> float:
        x1, y1, x2, y2 = self.bbox
        return max(0.0, (x2 - x1) * (y2 - y1))


@dataclass(frozen=True)
class VerificationResult:
    object_name_zh: str
    is_target: bool
    confidence: float
    reason_zh: str = ""


@dataclass(frozen=True)
class StepSearchConfig:
    target: str
    max_seconds: float = 420.0
    max_radius_m: float = 1.0
    score_min: float = 0.45
    align_threshold: float = 0.08
    align_yaw_max_deg: float = 25.0
    reach_area_ratio: float = 0.15
    scan_turn_deg: float = 30.0
    scan_span: int = 3
    stop_settle_seconds: float = 0.8
    forward_estimate_m: float = 0.15
    verify_rejection_turn_deg: float = 15.0
    semantic_reasoning_enabled: bool = False
    search_reasoner_backend: str = "legacy"
    search_reasoner_mode: str = "shadow"
    reasoner_min_confidence: float = 0.55
    reasoner_allow_forward: bool = False
    reasoner_max_turn_deg: float = 30.0
    reasoner_min_replan_seconds: float = 0.0
    max_motion_steps: int = 0


class StepSearchRunner:
    def __init__(
        self,
        config: StepSearchConfig,
        *,
        detect: Callable[[], list[Detection]],
        verify: Callable[[tuple[float, float, float, float]],
                         VerificationResult],
        execute_step: Callable[[str], tuple[bool, str]],
        snapshot: Callable[[], SensorSnapshot],
        odometry: Callable[[], tuple[float, float, float]],
        reason_next_view: Callable[[SearchReasoningContext], SearchDirective] | None = None,
        semantic_observe: Callable[[], Any] | None = None,
        on_reasoning_artifact: Callable[[dict[str, Any]], None] | None = None,
        observation_memory: list[dict[str, Any]] | None = None,
        negative_memory: Any = None,
        build_auxiliary_hints: Callable[[Any], dict[str, Any]] | None = None,
    ) -> None:
        self.config = config
        self._detect = detect
        self._verify = verify
        self._execute_step = execute_step
        self._snapshot = snapshot
        self._odometry = odometry
        self._reason_next_view = reason_next_view
        self._semantic_observe = semantic_observe
        self._on_reasoning_artifact = on_reasoning_artifact
        # Long-term observations are an immutable input to this session. Short
        # lived search failures remain in ``context.negative_memory`` and must
        # never be appended to this list or its backing JSONL store.
        self._observation_memory = deepcopy(list(observation_memory or []))
        self._negative_memory = negative_memory
        self._build_auxiliary_hints = build_auxiliary_hints
        self.events: list[dict[str, Any]] = []
        self._observed_heading_sectors: set[int] = set()
        self._last_reasoner_key: tuple[Any, ...] | None = None
        self._last_reasoner_time: float | None = None
        self._last_reasoner_directive: SearchDirective | None = None
        self._last_graph_match: Any = None

    def _event(self, name: str, **details: Any) -> None:
        self.events.append(
            {"event": name, "host_s": round(time.monotonic(), 6), **details}
        )

    def _best(self, detections: list[Detection]) -> Detection | None:
        return max(
            (item for item in detections
             if item.score >= self.config.score_min),
            key=lambda item: item.score,
            default=None,
        )

    def _execute_planned(self, machine: SearchStateMachine,
                         plan: StepPlan) -> tuple[bool, str]:
        machine.plan_step(plan.step)
        machine.motion_started(plan.step)
        self._event("motion_start", step=plan.step,
                    description=plan.description_zh, phase=plan.phase)
        ok, reason = self._execute_step(plan.step)
        machine.motion_completed(plan.step, ok)
        self._event("motion_result", step=plan.step, ok=ok, reason=reason)
        return ok, reason

    def _enter_plan_step(self, machine: SearchStateMachine,
                         detected: bool, evidence_ok: bool = False,
                         *, directive: SearchDirective | None = None) -> None:
        """Drive the state machine from DETECT_TARGET to PLAN_STEP."""
        machine.detection(detected)
        if detected:
            machine.verify(
                VisualEvidence(
                    bbox=True,
                    mask=True,
                    crop_verify=evidence_ok,
                    track_vote=True,
                    evidence_gate=True,
                    source="llm_quick",
                    require_mask=False,
                    require_track_vote=False,
                )
            )
        if directive is None:
            machine.next_view_unavailable()
        else:
            machine.next_view_selected(
                source=directive.source_backend,
                directive_id=directive.directive_id,
                confidence=directive.confidence,
            )
        machine.safety_checked(True)

    def _next_search_plan(
        self, *, scan_index: int, distance: float,
        current: tuple[float, float, float], recoverable_error: str | None = None,
    ) -> tuple[StepPlan, SearchDirective | None]:
        legacy = plan_scan_step(
            scan_index,
            scan_turn_deg=self.config.scan_turn_deg,
            scan_span=self.config.scan_span,
            distance_m=distance,
            max_radius_m=self.config.max_radius_m,
            forward_estimate_m=self.config.forward_estimate_m,
        )
        enabled = (
            self.config.semantic_reasoning_enabled
            and self.config.search_reasoner_backend in {"semantic_navigation", "hybrid"}
            and self._reason_next_view is not None
        )
        if not enabled:
            return legacy, None
        semantic = None
        try:
            if self._semantic_observe is not None:
                semantic = self._semantic_observe()
            scene_graph = getattr(semantic, "scene_graph", None)
            if scene_graph is None and isinstance(semantic, dict):
                scene_graph = semantic.get("scene_graph")
            auxiliary_payload = (
                self._build_auxiliary_hints(semantic)
                if self._build_auxiliary_hints is not None
                else {}
            )
            if not isinstance(auxiliary_payload, dict):
                raise TypeError("build_auxiliary_hints must return a mapping")
            auxiliary_hints = list(auxiliary_payload.get("hints") or [])
            auxiliary_status = dict(auxiliary_payload.get("status") or {})
            context = SearchReasoningContext(
                scene_graph=scene_graph,
                semantic_observation=semantic,
                observation_memory=deepcopy(self._observation_memory),
                negative_memory=self._negative_memory,
                auxiliary_hints=auxiliary_hints,
                auxiliary_status=auxiliary_status,
                robot_pose={
                    "x": current[0], "y": current[1],
                    "yaw_rad": current[2], "yaw_deg": current[2] * 180.0 / 3.141592653589793,
                },
                robot_yaw_deg=current[2] * 180.0 / 3.141592653589793,
                distance_from_origin_m=distance,
                scan_index=scan_index,
                safety_context={
                    "max_radius_m": self.config.max_radius_m,
                    "allow_forward": self.config.reasoner_allow_forward,
                    "recoverable_detection_error": recoverable_error,
                },
                legacy_scan_candidate=legacy,
                observed_heading_sectors=sorted(self._observed_heading_sectors),
            )
            now = time.monotonic()
            reasoner_key = _semantic_replan_key(
                semantic,
                current=current,
                recoverable_error=recoverable_error,
            )
            replan_age = (
                None if self._last_reasoner_time is None
                else max(0.0, now - self._last_reasoner_time)
            )
            replan_throttled = bool(
                reasoner_key is not None
                and _semantic_cache_hit(semantic)
                and reasoner_key == self._last_reasoner_key
                and self._last_reasoner_directive is not None
                and replan_age is not None
                and replan_age < max(
                    0.0, self.config.reasoner_min_replan_seconds
                )
            )
            if replan_throttled:
                directive = self._last_reasoner_directive
                context.graph_match = deepcopy(self._last_graph_match)
                self._event(
                    "semantic_replan_throttled",
                    directive_id=directive.directive_id,
                    age_seconds=round(float(replan_age), 6),
                    minimum_seconds=self.config.reasoner_min_replan_seconds,
                )
            else:
                directive = self._reason_next_view(context)
                self._last_reasoner_key = reasoner_key
                self._last_reasoner_time = now
                self._last_reasoner_directive = directive
                self._last_graph_match = deepcopy(context.graph_match)
            if not isinstance(directive, SearchDirective):
                raise TypeError("reason_next_view must return SearchDirective")
            semantic_plan = directive_to_step_plan(
                directive, scan_index=scan_index, distance_m=distance,
                step_config=self.config,
                min_confidence=self.config.reasoner_min_confidence,
                allow_forward=self.config.reasoner_allow_forward,
                max_turn_deg=self.config.reasoner_max_turn_deg,
            )
            memory_payload = None
            if hasattr(context.negative_memory, "to_dict"):
                memory_payload = context.negative_memory.to_dict()
            artifact = {
                "mode": self.config.search_reasoner_mode,
                "backend": self.config.search_reasoner_backend,
                "legacy_step": legacy.step,
                "semantic_step": semantic_plan.step,
                "semantic_disagrees_with_legacy": semantic_plan.step != legacy.step,
                "dangerous_forward_request": bool(
                    directive.allow_forward
                    or directive.preferred_distance_m is not None
                ),
                "semantic_directive": directive.to_dict(),
                "graph_match": context.graph_match.to_dict() if context.graph_match else None,
                "observation_memory": {
                    "retrieved_count": len(self._observation_memory),
                    "memory_ids": [
                        str(item.get("memory_id"))
                        for item in self._observation_memory
                        if isinstance(item, dict) and item.get("memory_id")
                    ],
                    "persistent_write_attempted": False,
                },
                "negative_memory": memory_payload,
                "auxiliary_reasoning": {
                    "status": auxiliary_status,
                    "hints": auxiliary_hints,
                    "used_refs": [
                        reference
                        for reference in directive.evidence_refs
                        if str(reference).startswith(("psg:", "situated_prior:"))
                    ],
                    "can_confirm_target": False,
                },
                "replan": {
                    "minimum_seconds": self.config.reasoner_min_replan_seconds,
                    "throttled": replan_throttled,
                    "cached_directive_id": (
                        directive.directive_id if replan_throttled else None
                    ),
                },
            }
            self._event("reasoner_shadow_compare" if self.config.search_reasoner_mode == "shadow" else "reasoner_active_decision", **artifact)
            if self._on_reasoning_artifact is not None:
                self._on_reasoning_artifact(artifact)
            if self.config.search_reasoner_mode == "shadow":
                return legacy, None
            if self.config.search_reasoner_mode != "active":
                raise ValueError("search_reasoner_mode must be shadow or active")
            plan = semantic_plan
            if plan == legacy:
                self._event(
                    "semantic_directive_fallback",
                    directive_id=directive.directive_id,
                    reason="low_confidence_or_not_convertible",
                    legacy_step=legacy.step,
                )
                return legacy, None
            return plan, directive
        except Exception as exc:
            self._event("semantic_reasoner_error", error=f"{type(exc).__name__}: {exc}", fallback_step=legacy.step)
            return legacy, None
    def run(self) -> dict[str, Any]:
        machine = SearchStateMachine(
            mode=SearchMode.STEP_SEARCH,
            motion_allowed=True,
            stop_settle_seconds=self.config.stop_settle_seconds,
        )
        machine.start()
        origin = self._odometry()
        started = time.monotonic()
        scan_index = 0
        index = 0
        status = "finished"
        finish_reason = ""
        steps_executed = 0

        self._event("step_search_start", target=self.config.target,
                    max_radius_m=self.config.max_radius_m,
                    max_seconds=self.config.max_seconds,
                    max_motion_steps=self.config.max_motion_steps)
        while time.monotonic() - started < self.config.max_seconds:
            if (
                self.config.max_motion_steps > 0
                and steps_executed >= self.config.max_motion_steps
            ):
                status = "motion_step_limit"
                finish_reason = (
                    f"operator motion-step limit reached: "
                    f"{self.config.max_motion_steps}"
                )
                self._event(
                    "motion_step_limit",
                    steps_executed=steps_executed,
                    limit=self.config.max_motion_steps,
                )
                break
            snapshot = self._snapshot()
            machine.sensors(snapshot)
            if machine.state == SearchState.WAIT_FOR_SENSORS:
                status = "sensor_gate_closed"
                finish_reason = "camera/lidar not fresh or robot not stationary"
                self._event("sensor_gate_closed")
                break
            machine.observation_elapsed(self.config.stop_settle_seconds)
            machine.scene_understood()

            current = self._odometry()
            self._observed_heading_sectors.add(
                int(round((current[2] * 180.0 / 3.141592653589793) / 30.0))
            )
            distance = (
                (current[0] - origin[0]) ** 2
                + (current[1] - origin[1]) ** 2
            ) ** 0.5
            if (self.config.max_radius_m > 0.0
                    and distance > self.config.max_radius_m):
                status = "range_limit"
                finish_reason = (
                    f"radius {distance:.2f}m > "
                    f"{self.config.max_radius_m:.1f}m"
                )
                self._event("range_limit", distance_m=round(distance, 3))
                break

            try:
                detections = self._detect()
            except Exception as exc:
                self._event("detection_error", error=str(exc))
                if "stale" in str(exc).lower():
                    status = "camera_stale"
                    finish_reason = str(exc)
                    self._event("abort", reason=finish_reason)
                    break
                plan, directive = self._next_search_plan(
                    scan_index=scan_index, distance=distance, current=current,
                    recoverable_error=str(exc),
                )
                scan_index += 1
                self._enter_plan_step(machine, detected=False, directive=directive)
                ok, reason = self._execute_planned(machine, plan)
                if not ok:
                    status = "motion_failed"
                    finish_reason = reason
                    break
                steps_executed += 1
                index += 1
                continue

            best = self._best(detections)
            if best is None:
                plan, directive = self._next_search_plan(
                    scan_index=scan_index, distance=distance, current=current,
                )
                scan_index += 1
                self._event("target_not_found", objects=len(detections))
                self._enter_plan_step(machine, detected=False, directive=directive)
                ok, reason = self._execute_planned(machine, plan)
                if not ok:
                    status = "motion_failed"
                    finish_reason = reason
                    break
                steps_executed += 1
                index += 1
                continue

            self._event("target_found", label=best.label,
                        score=round(best.score, 3),
                        center_x=round(best.center_x, 3),
                        area_ratio=round(best.area_ratio, 4),
                        distance_m=round(distance, 3))
            plan = plan_approach_step(
                center_x=best.center_x,
                area_ratio=best.area_ratio,
                distance_m=distance,
                align_threshold=self.config.align_threshold,
                align_yaw_max_deg=self.config.align_yaw_max_deg,
                reach_area_ratio=self.config.reach_area_ratio,
                max_radius_m=self.config.max_radius_m,
                forward_estimate_m=self.config.forward_estimate_m,
            )
            if plan.kind == PlanKind.ABORT_RADIUS:
                status = "range_limit"
                finish_reason = plan.description_zh
                self._event("range_limit", phase="APPROACH")
                break
            if plan.kind == PlanKind.VERIFY:
                verification = self._verify(best.bbox)
                self._event("target_verification",
                            label=best.label,
                            area_ratio=round(best.area_ratio, 4),
                            verification={
                                "object_name_zh": verification.object_name_zh,
                                "is_target": verification.is_target,
                                "confidence": round(
                                    verification.confidence, 3
                                ),
                                "reason_zh": verification.reason_zh,
                            })
                machine.detection(True)
                machine.verify(
                    VisualEvidence(
                        bbox=True,
                        mask=True,
                        crop_verify=verification.is_target,
                        track_vote=True,
                        evidence_gate=True,
                        source="llm_quick",
                        require_mask=False,
                        require_track_vote=False,
                    )
                )
                if machine.state == SearchState.LOCALIZE_TARGET:
                    machine.localization(False)
                    machine.finish_confirmed()
                    status = "target_reached"
                    finish_reason = (
                        f"{verification.object_name_zh} "
                        f"({verification.reason_zh})"
                    )
                    self._event("target_reached",
                                verification=verification.object_name_zh)
                    break
                plan = verify_rejection_step(
                    self.config.verify_rejection_turn_deg
                )
                self._event("verification_rejected",
                            object_name=verification.object_name_zh,
                            reason=verification.reason_zh)
                machine.next_view_unavailable()
                machine.safety_checked(True)
                ok, reason = self._execute_planned(machine, plan)
                if not ok:
                    status = "motion_failed"
                    finish_reason = reason
                    break
                steps_executed += 1
                index += 1
                continue

            self._event("approach_step", step=plan.step,
                        phase=plan.phase)
            self._enter_plan_step(machine, detected=True)
            ok, reason = self._execute_planned(machine, plan)
            if not ok:
                status = "motion_failed"
                finish_reason = reason
                break
            steps_executed += 1
            index += 1
        else:
            status = "time_limit"
            finish_reason = f"max_seconds={self.config.max_seconds:.0f}s"
            self._event("time_limit")

        self._event("step_search_finish", status=status,
                    reason=finish_reason, steps_executed=steps_executed)
        return {
            "status": status,
            "finish_reason": finish_reason,
            "steps_executed": steps_executed,
            "events": self.events,
            "state_machine_trace": machine.trace,
            "final_state": machine.state.value,
        }


def _semantic_replan_key(
    semantic: Any,
    *,
    current: tuple[float, float, float],
    recoverable_error: str | None,
) -> tuple[Any, ...] | None:
    """Identify a semantic view and the robot pose from which it was used."""

    if isinstance(semantic, dict):
        frame_id = semantic.get("frame_id")
        heading_sector = semantic.get("heading_sector")
    else:
        frame_id = getattr(semantic, "frame_id", None)
        heading_sector = getattr(semantic, "heading_sector", None)
    if frame_id is None:
        return None
    return (
        str(frame_id),
        heading_sector,
        round(float(current[0]), 2),
        round(float(current[1]), 2),
        round(float(current[2]), 3),
        str(recoverable_error or ""),
    )


def _semantic_cache_hit(semantic: Any) -> bool:
    if isinstance(semantic, dict):
        return semantic.get("cache_hit") is True
    return getattr(semantic, "cache_hit", False) is True
