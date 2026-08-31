"""AutonomousExplorer: the high-level long-cycle orchestrator.

It owns the continuous loop

    OBSERVE -> MATCH -> VERIFY -> UPDATE_MEMORY -> PLAN -> EXECUTE
    -> WAIT_RESULT -> RECOVER/REPLAN -> OBSERVE -> ... -> TARGET_FOUND

The explorer only knows the platform-independent contracts
(``RobotBackend``, ``ExplorationGoal``, ``ExplorationGraph``,
``LiveObservation``); perception details and robot specifics are injected as
callables, so the same class runs on the Go2-W experimental backend, mock
backends and a future production metric backend.
"""

from __future__ import annotations

import json
import math
import time
import traceback
from dataclasses import asdict, dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Callable

from app.navigation.candidate_goal_generator import (
    generate_live_exploration_candidates,
)
from app.navigation.exploration_config import (
    CandidateConfig,
    ExplorationBudget,
    ExplorationPolicy,
    MemoryConfig,
    RecoveryConfig,
    ScoringWeights,
    load_exploration_policy,
)
from app.navigation.exploration_graph import ExplorationGraph, ObservationNode
from app.navigation.exploration_planner import (
    ScoredGoal,
    score_exploration_goal,
    select_exploration_goal,
)
from app.navigation.models import LiveObservation
from app.navigation.decision_models import DecisionRecord, make_motion_command
from app.navigation.live_graph_path_planner import plan_live_graph_path
from app.navigation.robot_backend import (
    NavigationResult,
    NavigationStatus,
    RobotBackend,
    TERMINAL_NAVIGATION_STATUSES,
)
from app.spatial.semantic_navigation_graph import SemanticNavigationGraph
from app.task_understanding.search_task_context import SearchTaskContext


class ExplorerState(str, Enum):
    BOOTSTRAP = "BOOTSTRAP"
    OBSERVE = "OBSERVE"
    MATCH = "MATCH"
    VERIFY = "VERIFY"
    UPDATE_MEMORY = "UPDATE_MEMORY"
    PLAN = "PLAN"
    EXECUTE = "EXECUTE"
    WAIT_RESULT = "WAIT_RESULT"
    RECOVER = "RECOVER"
    PAUSED = "PAUSED"
    TARGET_FOUND = "TARGET_FOUND"
    SEARCH_EXHAUSTED = "SEARCH_EXHAUSTED"
    OPERATOR_STOP = "OPERATOR_STOP"
    FAILED = "FAILED"
    FINISHED = "FINISHED"


EXPLORER_FINISH_REASONS = (
    "TARGET_FOUND",
    "TIMEOUT",
    "SEARCH_EXHAUSTED",
    "OPERATOR_STOP",
    "BACKEND_FAILURE",
    "PERCEPTION_FAILURE",
    "MAX_STEPS_REACHED",
)


@dataclass
class SemanticMatch:
    """Matcher output handed to the verifier / memory / planner."""

    has_candidate: bool
    graph_match: Any | None = None
    target_match: dict[str, Any] | None = None
    target_profile: Any | None = None
    anchor_labels: list[str] = field(default_factory=list)
    directive: Any | None = None
    target_score: float = 0.0
    target_match_level: str = "none"
    provenance: dict[str, Any] = field(default_factory=dict)


@dataclass
class VerificationOutcome:
    confirmed: bool
    attempts: int
    reason_zh: str = ""
    details: dict[str, Any] = field(default_factory=dict)


@dataclass
class SessionResult:
    result: str
    target: str = ""
    duration_s: float = 0.0
    planning_cycles: int = 0
    motion_steps: int = 0
    observations: int = 0
    unique_nodes: int = 0
    unique_places: int = 0
    unique_objects: int = 0
    frontiers_discovered: int = 0
    map_nodes_total: int = 0
    replans: int = 0
    navigation_failures: int = 0
    verify_attempts: int = 0
    semantic_goal_selection_count: int = 0
    fallback_goal_selection_count: int = 0
    finish_reason: str = ""
    session_id: str = ""
    summary: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "result": self.result,
            "target": self.target,
            "duration_s": round(self.duration_s, 3),
            "planning_cycles": self.planning_cycles,
            "motion_steps": self.motion_steps,
            "observations": self.observations,
            "unique_nodes": self.unique_nodes,
            "unique_places": self.unique_places,
            "unique_objects": self.unique_objects,
            "frontiers_discovered": self.frontiers_discovered,
            "map_nodes_total": self.map_nodes_total,
            "replans": self.replans,
            "navigation_failures": self.navigation_failures,
            "verify_attempts": self.verify_attempts,
            "semantic_goal_selection_count": self.semantic_goal_selection_count,
            "fallback_goal_selection_count": self.fallback_goal_selection_count,
            "finish_reason": self.finish_reason,
            "session_id": self.session_id,
            "summary": self.summary,
        }


class PerceptionFailure(RuntimeError):
    """Observer could not produce a valid observation (recoverable)."""


class AutonomousExplorer:
    """Long-cycle operator-supervised semantic exploration orchestrator."""

    def __init__(
        self,
        *,
        target: str = "",
        task_context: SearchTaskContext | None = None,
        observer: Callable[[], LiveObservation | None],
        matcher: Callable[[LiveObservation], SemanticMatch],
        verifier: Callable[[LiveObservation, SemanticMatch], VerificationOutcome],
        backend: RobotBackend,
        policy: ExplorationPolicy | None = None,
        graph: ExplorationGraph | None = None,
        negative_memory: Any = None,
        negative_target_key: str = "target",
        candidate_generator: Callable[..., list[Any]] | None = None,
        planner: Callable[..., ScoredGoal | None] | None = None,
        on_event: Callable[[dict[str, Any]], None] | None = None,
        finish_on_visual_confirmation: bool = True,
        turn_only: bool = False,
        session_id: str | None = None,
        max_perception_retries: int = 2,
        now: Callable[[], float] = time.time,
        semantic_graph: SemanticNavigationGraph | None = None,
        executor_id: str | None = None,
        worker_generation: int | None = None,
    ) -> None:
        self.task_context = task_context
        self.target = task_context.canonical_target if task_context else target
        self._observer = observer
        self._matcher = matcher
        self._verifier = verifier
        self._backend = backend
        self.policy = policy or load_exploration_policy()
        self.graph = graph or ExplorationGraph(session_id=session_id or "exploration_session")
        self.semantic_graph = semantic_graph or SemanticNavigationGraph()
        self.negative_memory = negative_memory
        self.negative_target_key = negative_target_key
        self._candidate_generator = candidate_generator or generate_live_exploration_candidates
        self._planner = planner or select_exploration_goal
        self._on_event = on_event or (lambda event: None)
        self.finish_on_visual_confirmation = bool(finish_on_visual_confirmation)
        self.turn_only = bool(turn_only)
        self.session_id = session_id or time.strftime("explore_%Y%m%d_%H%M%S")
        self.max_perception_retries = max(0, int(max_perception_retries))
        self._now = now
        self._state = ExplorerState.BOOTSTRAP
        self._operator_stop_requested = False
        self._pause_requested = False
        self._paused = False
        self.events: list[dict[str, Any]] = []
        self._seen_labels: set[str] = set()
        self._no_information_cycles = 0
        # True when the previous EXECUTE step actually moved/turned the robot.
        # As long as it keeps moving we keep exploring and do not count "no new
        # information" against the exhaust budget (a sofa may just not be
        # visible from this office corner yet).
        self._last_execution_moved = False
        self._last_goal_source: str | None = None
        self._current_place_id: str | None = None
        self.executor_id = executor_id
        self.worker_generation = worker_generation
        self.decision_records: list[DecisionRecord] = []

    # ---- operator interface ----------------------------------------------

    def request_stop(self) -> None:
        """Operator stop: halt motion and finish with OPERATOR_STOP."""
        self._operator_stop_requested = True
        self._pause_requested = False
        try:
            self._backend.stop()
        except Exception:
            pass

    def request_pause(self) -> None:
        """Operator pause (plan book §40): stop generating new goals and stop
        the current motion; memory and the exploration graph are kept."""
        self._pause_requested = True
        try:
            self._backend.stop()
        except Exception:
            pass

    def request_resume(self) -> None:
        """Operator resume: continue the loop; the next cycle re-observes and
        replans (no stale instruction is restored)."""
        self._pause_requested = False

    @property
    def paused(self) -> bool:
        return self._paused

    @property
    def state(self) -> str:
        """Public terminal/current state used by session artifact writers."""
        return self._state.value

    @property
    def operator_stop_requested(self) -> bool:
        return self._operator_stop_requested

    # ---- main loop ---------------------------------------------------------

    def run(self) -> SessionResult:
        started = float(self._now())
        budget = self.policy.budget
        planning_cycles = 0
        motion_steps = 0
        observations = 0
        replans = 0
        navigation_failures = 0
        verify_attempts = 0
        semantic_goal_selection_count = 0
        fallback_goal_selection_count = 0
        finish_reason = ""

        self._state = ExplorerState.BOOTSTRAP
        self._emit("session_start", target=self.target, session_id=self.session_id,
                   task_id=self.task_context.task_id if self.task_context else None,
                   task_context=self.task_context.to_dict() if self.task_context else None,
                   budget=budget.to_dict(), policy=asdict(self.policy) if hasattr(self.policy, "__dataclass_fields__") else self.policy.to_dict())

        # ---- BOOTSTRAP ------------------------------------------------------
        backend_ready = False
        for attempt in range(1, max(1, self.policy.recovery.backend_reconnect_attempts) + 1):
            health = self._backend.health()
            self._emit("backend_health", attempt=attempt, ready=health.ready,
                       degraded=health.degraded, health=health.to_dict())
            if health.ready:
                backend_ready = True
                break
            if attempt < self.policy.recovery.backend_reconnect_attempts:
                time.sleep(self.policy.recovery.backend_reconnect_delay_seconds)
        if not backend_ready:
            finish_reason = "BACKEND_FAILURE"
            self._state = ExplorerState.FAILED
            self._emit("session_finish", result=finish_reason,
                       reason="backend unhealthy after reconnect attempts")
            return self._result(finish_reason, started, planning_cycles, motion_steps,
                                observations, replans, navigation_failures,
                                verify_attempts, semantic_goal_selection_count,
                                fallback_goal_selection_count, finish_reason)

        self._state = ExplorerState.OBSERVE
        elapsed = float(self._now()) - started
        while budget.remaining(
            elapsed_sec=elapsed, planning_cycles=planning_cycles,
            motion_steps=motion_steps,
        ):
            if self._operator_stop_requested:
                finish_reason = "OPERATOR_STOP"
                self._state = ExplorerState.OPERATOR_STOP
                try:
                    self._backend.stop()
                except Exception:
                    pass
                self._emit("session_finish", result=finish_reason, reason="operator stop")
                return self._result(finish_reason, started, planning_cycles, motion_steps,
                                    observations, replans, navigation_failures,
                                    verify_attempts, semantic_goal_selection_count,
                                    fallback_goal_selection_count, finish_reason)

            # ---- PAUSE gate (plan book §40) -------------------------------
            # While paused the loop parks without generating new goals or
            # motion; memory and the graph stay intact. Resume re-enters the
            # normal OBSERVE -> ... -> PLAN pipeline, so the next decision is
            # always a fresh replan, never a restored instruction.
            if self._pause_requested:
                self._state = ExplorerState.PAUSED
                self._paused = True
                try:
                    self._backend.stop()
                except Exception:
                    pass
                self._emit("paused")
                while self._pause_requested and not self._operator_stop_requested:
                    time.sleep(0.2)
                self._paused = False
                self._emit("resumed")

            # ---- OBSERVE ----------------------------------------------------
            self._state = ExplorerState.OBSERVE
            self._emit(
                "phase_progress",
                phase="OBSERVE",
                operation="capture_and_analyze",
                detail_zh="正在获取最新画面并分析目标与场景；机器狗会原地等待分析完成",
            )
            perception_retries = 0
            observation: LiveObservation | None = None
            while observation is None:
                try:
                    observation = self._observer()
                except PerceptionFailure as exc:
                    self._emit("perception_failure", error=str(exc))
                    if observations == 0 and perception_retries < self.max_perception_retries:
                        perception_retries += 1
                        self._emit("perception_retry", attempt=perception_retries,
                                   reason=str(exc))
                        time.sleep(2.0)
                        continue
                    finish_reason = "PERCEPTION_FAILURE"
                    self._state = ExplorerState.FAILED
                    break
                except Exception as exc:
                    self._emit("observer_error", error=f"{type(exc).__name__}: {exc}",
                               traceback=traceback.format_exc())
                    if observations == 0 and perception_retries < self.max_perception_retries:
                        perception_retries += 1
                        self._emit("observer_retry", attempt=perception_retries,
                                   error=f"{type(exc).__name__}: {exc}")
                        time.sleep(2.0)
                        continue
                    finish_reason = "PERCEPTION_FAILURE"
                    self._state = ExplorerState.FAILED
                    break
            if finish_reason:
                break
            observations += 1
            pose = self._backend.get_pose()
            observation.pose = pose.to_dict() if pose is not None else None
            if observation.heading_sector is None and pose is not None:
                sector_deg = 360.0 / max(1, self.policy.candidates.heading_sectors)
                observation.heading_sector = int(
                    round(pose.yaw * 180.0 / 3.141592653589793 / sector_deg)
                ) % max(1, self.policy.candidates.heading_sectors)
            self._emit("observation", bundle_id=observation.bundle_id,
                       objects=observation.object_labels,
                       scene_objects=observation.scene_objects,
                       scene_relations=observation.scene_relations,
                       target_present=observation.target_present,
                       heading_sector=observation.heading_sector,
                       pose=observation.pose,
                       sensor_health=observation.sensor_health,
                       image_ref=observation.image_ref)

            # ---- MATCH ------------------------------------------------------
            self._state = ExplorerState.MATCH
            try:
                match = self._matcher(observation)
            except Exception as exc:
                self._emit("matcher_error", error=f"{type(exc).__name__}: {exc}")
                match = SemanticMatch(has_candidate=False)
            if not isinstance(match, SemanticMatch):
                match = SemanticMatch(has_candidate=bool(getattr(match, "has_candidate", False)))
            self._emit("match", has_candidate=match.has_candidate,
                       target_match_level=match.target_match_level,
                       target_score=match.target_score,
                       anchor_labels=match.anchor_labels,
                       directive=match.directive.to_dict() if match.directive is not None and hasattr(match.directive, "to_dict") else None,
                       graph_match=match.graph_match.to_dict() if match.graph_match is not None and hasattr(match.graph_match, "to_dict") else None)

            # ---- VERIFY -----------------------------------------------------
            verification: VerificationOutcome | None = None
            if match.has_candidate:
                self._state = ExplorerState.VERIFY
                for attempt in range(1, max(1, budget.verify_attempts) + 1):
                    try:
                        verification = self._verifier(observation, match)
                    except Exception as exc:
                        # A verification failure must never kill the session:
                        # record it as unconfirmed and keep exploring.
                        verification = VerificationOutcome(
                            confirmed=False, attempts=attempt,
                            reason_zh=f"verifier error: {type(exc).__name__}: {exc}",
                            details={"error": traceback.format_exc()},
                        )
                        self._emit("verification_error", attempt=attempt,
                                   error=f"{type(exc).__name__}: {exc}")
                    verify_attempts += 1
                    self._emit("verification", attempt=attempt,
                               confirmed=verification.confirmed,
                               reason_zh=verification.reason_zh)
                    if verification.confirmed:
                        break
                    if attempt < budget.verify_attempts:
                        # small re-observe between verify attempts
                        time.sleep(1.0)
                if verification is not None and verification.confirmed:
                    # A target can be confirmed on the first frame, before the
                    # normal UPDATE_MEMORY phase.  Persist that observation in
                    # the semantic graph first so the terminal state still
                    # contains a Place, Object evidence and map provenance.
                    spatial_update = self.semantic_graph.update_observation(
                        observation_id=observation.bundle_id,
                        heading_sector=observation.heading_sector,
                        scene_objects=observation.scene_objects,
                        scene_relations=observation.scene_relations,
                        pose=observation.pose,
                        timestamp=observation.timestamp,
                        target_candidate=True,
                    )
                    self._current_place_id = spatial_update["place_id"]
                    semantic_map = self.semantic_graph.to_dict()
                    self._emit(
                        "memory_update",
                        node_id=self._current_node_id(observation),
                        place_id=self._current_place_id,
                        new_labels=observation.object_labels,
                        new_relations=[],
                        new_sector=True,
                        information_gain=1.0,
                        no_information_cycles=0,
                        unique_nodes=len(self.graph.nodes),
                        unique_places=len(self.semantic_graph.place_graph.places),
                        unique_objects=len(self.semantic_graph.object_map.objects),
                        frontiers_discovered=len(self.semantic_graph.frontiers),
                        map_nodes_total=len(semantic_map.get("nodes") or []),
                        semantic_navigation_graph=semantic_map,
                    )
                    self.graph.mark_target_confirmed(self._current_node_id(observation))
                    self.semantic_graph.mark_target_confirmed(
                        object_id=self._target_object_id(observation),
                        observation_id=observation.bundle_id,
                    )
                    self._state = ExplorerState.TARGET_FOUND
                    finish_reason = "TARGET_FOUND"
                    self._emit("target_found",
                               reason_zh=verification.reason_zh,
                               attempts=verification.attempts,
                               place_id=self._current_place_id,
                               object_id=self._target_object_id(observation),
                               observation_id=observation.bundle_id)
                    try:
                        self._backend.stop()
                    except Exception:
                        pass
                    break
                if verification is not None:
                    self._emit("verification_rejected", reason_zh=verification.reason_zh)

            # ---- UPDATE_MEMORY ----------------------------------------------
            self._state = ExplorerState.UPDATE_MEMORY
            info_gain, new_labels, new_relations, new_sector = self._update_memory(
                observation, match, pose
            )
            if match.has_candidate and not (verification and verification.confirmed):
                node = self.graph.get_node(self._current_node_id(observation))
                if node is not None:
                    self.graph.mark_target_candidate(node.node_id)
            if new_labels or new_relations or new_sector or self._last_execution_moved:
                self._no_information_cycles = 0
            else:
                self._no_information_cycles += 1
            # Per-cycle flag: the next cycle's UPDATE_MEMORY only sees a
            # movement that happened in the cycle just before it.
            self._last_execution_moved = False
            semantic_map = self.semantic_graph.to_dict()
            self._emit("memory_update", node_id=self._current_node_id(observation),
                       place_id=self._current_place_id,
                       new_labels=new_labels, new_relations=new_relations,
                       new_sector=new_sector,
                       information_gain=info_gain,
                       no_information_cycles=self._no_information_cycles,
                       unique_nodes=len(self.graph.nodes),
                       unique_places=len(self.semantic_graph.place_graph.places),
                       unique_objects=len(self.semantic_graph.object_map.objects),
                       frontiers_discovered=len(self.semantic_graph.frontiers),
                       map_nodes_total=len(semantic_map.get("nodes") or []),
                       semantic_navigation_graph=semantic_map)
            if self._no_information_cycles >= max(1, budget.max_consecutive_no_information_cycles):
                finish_reason = "SEARCH_EXHAUSTED"
                self._state = ExplorerState.SEARCH_EXHAUSTED
                self._emit("search_exhausted",
                           reason=f"{self._no_information_cycles} consecutive no-information cycles")
                break

            # ---- PLAN --------------------------------------------------------
            self._state = ExplorerState.PLAN
            planning_cycles += 1
            current_yaw_deg = (pose.yaw * 180.0 / 3.141592653589793) if pose is not None else 0.0
            anchor_labels = match.anchor_labels
            try:
                candidates = self._candidate_generator(
                    observation=observation,
                    graph=self.graph,
                    directive=getattr(match, "directive", None),
                    anchor_labels=anchor_labels,
                    negative_memory=self.negative_memory,
                    negative_target_key=self.negative_target_key,
                    capabilities=self._backend.capabilities(),
                    config=self.policy.candidates,
                    current_yaw_deg=current_yaw_deg,
                    turn_only=self.turn_only,
                    max_candidates=self.policy.candidates.max_candidates,
                )
            except Exception as exc:
                self._emit("candidate_generator_error", error=f"{type(exc).__name__}: {exc}")
                candidates = []
            if not candidates:
                finish_reason = "SEARCH_EXHAUSTED"
                self._state = ExplorerState.SEARCH_EXHAUSTED
                self._emit("search_exhausted", reason="no exploration candidates")
                break
            # Failed goals: exclude sectors that failed >= N times (section
            # 10.5) while alternatives exist; recent-goal tabu avoids
            # immediate repeats.
            recent = self.graph.recent_goal_sequence()
            tabu_sectors = {
                item.get("heading_sector") for item in recent[-2:]
                if item.get("heading_sector") is not None
            }
            tabu_nodes = {
                item.get("target_node_id") for item in recent[-2:]
                if item.get("target_node_id") is not None
            }
            max_failures = max(1, budget.max_navigation_failures_per_goal)
            failed_sectors = {
                sector for sector in range(self.policy.candidates.heading_sectors)
                if self.graph.sector_failure_count(sector) >= max_failures
            }
            exclude_sectors = (tabu_sectors | failed_sectors) if len(candidates) > 2 else set()
            exclude_nodes = tabu_nodes if len(candidates) > 2 else set()
            scored = self._planner(
                candidates,
                graph=self.graph,
                weights=self.policy.scoring,
                current_yaw_deg=current_yaw_deg,
                exclude_node_ids=exclude_nodes,
                exclude_sectors=exclude_sectors,
            )
            if scored is None:
                finish_reason = "SEARCH_EXHAUSTED"
                self._state = ExplorerState.SEARCH_EXHAUSTED
                self._emit("search_exhausted", reason="planner returned no goal")
                break
            goal = scored.goal
            # Emit the full scored ranking (plan book §33-§34) so the WebUI
            # can render the candidate list with per-component scores.
            all_scored: list[ScoredGoal] = []
            for candidate in candidates:
                if candidate.target_node_id in exclude_nodes:
                    continue
                if candidate.heading_sector in exclude_sectors:
                    continue
                try:
                    all_scored.append(
                        score_exploration_goal(
                            candidate,
                            graph=self.graph,
                            weights=self.policy.scoring,
                            current_yaw_deg=current_yaw_deg,
                        )
                    )
                except Exception:  # noqa: BLE001 - one bad goal never kills PLAN
                    continue
            self._emit("candidates",
                       candidates=[item.to_dict() for item in all_scored],
                       selected_goal_id=scored.goal.goal_id,
                       planning_cycles=planning_cycles)
            source = str(goal.provenance.get("source", "unknown"))
            if source in {"semantic_navigation_directive", "semantic_anchor"}:
                semantic_goal_selection_count += 1
            else:
                fallback_goal_selection_count += 1
            self._emit("selected_goal", goal=goal.to_dict(),
                       score=round(scored.score, 4),
                       components=scored.components,
                       reasons=scored.reasons,
                       planning_cycles=planning_cycles)

            decision = self._make_decision_record(
                observation=observation,
                match=match,
                goal=goal,
                scored=scored,
                candidates=all_scored,
                cycle=planning_cycles,
            )
            self.decision_records.append(decision)
            self._emit("decision_recorded", decision=decision.to_dict(),
                       next_motion_command=decision.next_motion_command)

            # ---- EXECUTE -----------------------------------------------------
            self._state = ExplorerState.EXECUTE
            self._emit("action_start", goal=goal.to_dict(),
                       planning_cycles=planning_cycles,
                       decision_id=decision.decision_id,
                       next_motion_command=decision.next_motion_command)
            handle = self._backend.execute_goal(goal)
            self._state = ExplorerState.WAIT_RESULT
            result = self._wait_result(handle, timeout_sec=max(10.0, self.policy.recovery.timeout_retry_count * 15.0))
            self._emit("navigation_result", goal_id=goal.goal_id,
                       status=result.status.value, message=result.message,
                       requested_motion=result.requested_motion,
                       observed_motion=result.observed_motion,
                       elapsed_sec=result.elapsed_sec)
            # If this goal physically moved the robot (even without new visual
            # info), it is still active exploration - don't penalize it.
            self._last_execution_moved = bool(result.succeeded)
            updated_decision = decision.with_execution(
                status="SUCCEEDED" if result.succeeded else "FAILED",
                message=result.message,
                requested_motion=result.requested_motion,
                observed_motion=result.observed_motion,
                replan_reason=None if result.succeeded else result.message,
            )
            self.decision_records.append(updated_decision)
            self._emit("decision_recorded", decision=updated_decision.to_dict())
            self.graph.record_navigation(
                result,
                goal_type=goal.goal_type,
                requested_motion=result.requested_motion,
                observed_motion=result.observed_motion,
                target_node_id=goal.target_node_id or self._current_node_id(observation),
                source_node_id=self._current_node_id(observation),
                heading_sector=goal.heading_sector,
            )
            if result.succeeded:
                motion_steps += 1
                node_id = goal.target_node_id or self._current_node_id(observation)
                node = self.graph.get_node(node_id)
                if node is not None:
                    self.graph.mark_visited(node.node_id)
                continue
            if result.status == NavigationStatus.OPERATOR_STOP:
                finish_reason = "OPERATOR_STOP"
                self._state = ExplorerState.OPERATOR_STOP
                self._emit("session_finish", result=finish_reason, reason=result.message)
                return self._result(finish_reason, started, planning_cycles, motion_steps,
                                    observations, replans, navigation_failures,
                                    verify_attempts, semantic_goal_selection_count,
                                    fallback_goal_selection_count, finish_reason)
            if result.status == NavigationStatus.TIMEOUT:
                self._backend.cancel(handle)
                if result.attempt <= self.policy.recovery.timeout_retry_count:
                    replans += 1
                    continue
            if result.failed:
                navigation_failures += 1
                node_id = goal.target_node_id or self._current_node_id(observation)
                node = self.graph.get_node(node_id)
                if node is not None and node.navigation_fail_count >= budget.max_navigation_failures_per_goal:
                    self.graph.mark_unreachable(node_id, reason=result.message)
            if self._state != ExplorerState.OPERATOR_STOP:
                self._state = ExplorerState.RECOVER
                replans += 1
                self._emit("replan", goal_id=goal.goal_id,
                           status=result.status.value,
                           navigation_failures=navigation_failures)
        else:
            elapsed = float(self._now()) - started
            if finish_reason == "":
                if elapsed >= budget.max_search_seconds:
                    finish_reason = "TIMEOUT"
                elif budget.max_motion_steps > 0 and motion_steps >= budget.max_motion_steps:
                    finish_reason = "MAX_STEPS_REACHED"
                else:
                    finish_reason = "MAX_PLANNING_CYCLES_REACHED"

        if self._state not in {ExplorerState.TARGET_FOUND, ExplorerState.OPERATOR_STOP}:
            self._state = ExplorerState.FINISHED
        if finish_reason == "":
            finish_reason = "SEARCH_EXHAUSTED"
        self._emit("session_finish", result=finish_reason,
                   planning_cycles=planning_cycles, motion_steps=motion_steps,
                   observations=observations, unique_nodes=len(self.graph.nodes),
                   replans=replans, navigation_failures=navigation_failures,
                   verify_attempts=verify_attempts)
        return self._result(finish_reason, started, planning_cycles, motion_steps,
                            observations, replans, navigation_failures,
                            verify_attempts, semantic_goal_selection_count,
                            fallback_goal_selection_count, finish_reason)

    # ---- internals ---------------------------------------------------------

    def _wait_result(self, handle: Any, *, timeout_sec: float) -> NavigationResult:
        deadline = float(self._now()) + timeout_sec
        while float(self._now()) < deadline:
            if self._operator_stop_requested:
                return NavigationResult(
                    goal_id=getattr(handle, "goal_id", "goal"),
                    status=NavigationStatus.OPERATOR_STOP,
                    message="operator stop during navigation",
                )
            if self._pause_requested:
                # Pause halts the wait; the loop parks at the pause gate.
                return NavigationResult(
                    goal_id=getattr(handle, "goal_id", "goal"),
                    status=NavigationStatus.CANCELLED,
                    message="operator pause during navigation",
                )
            result = self._backend.get_navigation_status(handle)
            if result.status in TERMINAL_NAVIGATION_STATUSES:
                return result
            time.sleep(0.2)
        return NavigationResult(
            goal_id=getattr(handle, "goal_id", "goal"),
            status=NavigationStatus.TIMEOUT,
            message=f"navigation wait timeout after {timeout_sec:.0f}s",
        )

    def _current_node_id(self, observation: LiveObservation) -> str:
        # ExplorationGraph remains an internal scoring/recovery ledger keyed
        # by observation bundles.  The exposed navigation map is the separate
        # semantic graph, whose current stable node is ``current_place_id``.
        bundle_id = str(getattr(observation, "bundle_id", "") or "")
        return f"node_{bundle_id}" if bundle_id else "node_unknown"

    def _update_memory(
        self,
        observation: LiveObservation,
        match: SemanticMatch,
        pose: Any,
    ) -> tuple[float, list[str], list[str], bool]:
        spatial_update = self.semantic_graph.update_observation(
            observation_id=observation.bundle_id,
            heading_sector=observation.heading_sector,
            scene_objects=observation.scene_objects,
            scene_relations=observation.scene_relations,
            pose=observation.pose,
            timestamp=observation.timestamp,
            target_candidate=observation.target_present,
        )
        self._current_place_id = spatial_update["place_id"]
        node_id = self._current_node_id(observation)
        labels = observation.object_labels
        new_labels = [label for label in labels if label not in self._seen_labels]
        self._seen_labels.update(labels)
        relations = [str(item.get("relation") or "") for item in observation.scene_relations]
        existing_relations = {
            item for node in self.graph.nodes.values() for item in node.relations
        }
        new_relations = [rel for rel in relations if rel and rel not in existing_relations]
        new_sector = (
            observation.heading_sector is not None
            and self.graph.sector_visited_count(observation.heading_sector) == 0
        )
        info_gain = min(
            1.0,
            len(new_labels) * 0.25 + len(new_relations) * 0.2 + (0.3 if new_sector else 0.0),
        )
        node = self.graph.node_or_create(
            node_id,
            pose=pose,
            pose_quality="relative" if pose is not None else "unavailable",
            heading=(pose.yaw * 180.0 / 3.141592653589793) if pose is not None else None,
            heading_sector=observation.heading_sector,
            objects=labels,
            relations=relations,
            scene_graph=observation.scene_graph,
            target_match_level=match.target_match_level,
            target_score=match.target_score,
            semantic_relevance=match.target_score,
            information_gain=info_gain,
            source_bundle_id=observation.bundle_id,
            provenance={"source": "live_observation", "place_id": self._current_place_id},
        )
        # node_or_create keeps kwargs only for new nodes; refresh the view
        # fields on revisits so sector coverage stays correct.
        if observation.heading_sector is not None:
            node.heading_sector = observation.heading_sector
        if pose is not None:
            node.pose = pose
        node = self.graph.add_observation(node)
        if not observation.target_present:
            self.graph.mark_negative(node_id, reason="target not observed in this view")
            if self.negative_memory is not None and observation.heading_sector is not None:
                try:
                    self.negative_memory.add_negative(
                        target_key=self.negative_target_key,
                        heading_sector=observation.heading_sector,
                        reason="当前稳定观察未发现目标",
                        source_event_id=f"explore_{self.session_id}_{node_id}",
                        observation_pose=observation.pose,
                        confidence=0.6,
                    )
                except Exception:
                    pass
        if match.has_candidate:
            self.graph.mark_target_candidate(node_id)
        return info_gain, new_labels, new_relations, new_sector

    def _target_object_id(self, observation: LiveObservation) -> str | None:
        labels = {
            str(item.get("label_zh") or item.get("label") or item.get("name") or "")
            for item in observation.scene_objects
        }
        for object_id, entry in self.semantic_graph.object_map.objects.items():
            if entry.label in labels:
                return object_id
        return None

    def _make_decision_record(
        self,
        *,
        observation: LiveObservation,
        match: SemanticMatch,
        goal: Any,
        scored: ScoredGoal,
        candidates: list[ScoredGoal],
        cycle: int,
    ) -> DecisionRecord:
        decision_id = f"decision_{self.session_id}_{cycle}_{len(self.decision_records) + 1}"
        command = make_motion_command(
            plan_id=f"live_plan_{self.session_id}_{cycle}",
            decision_id=decision_id,
            turn_deg=float(goal.relative_dyaw or 0.0),
            forward_m=float(goal.relative_dx or 0.0),
            reason_zh="；".join(scored.reasons) or goal.semantic_reason or "选择当前可达候选",
            target_place_id=self._current_place_id,
            target_frontier_id=goal.provenance.get("frontier_id") if isinstance(goal.provenance, dict) else None,
            safety_limited=True,
        )
        command_values = command.to_dict()
        if command.forward_m > 0.0:
            backend_config = getattr(self._backend, "config", None)
            segment_limit = float(
                getattr(backend_config, "forward_step_m", command.forward_m)
            )
            maximum = float(
                getattr(backend_config, "max_forward_step_m", command.forward_m)
            )
            executable = min(command.forward_m, maximum)
            segment_limit = max(0.01, min(segment_limit, executable))
            segment_count = max(1, int(math.ceil(executable / segment_limit)))
            command_values["forward_m"] = executable
            command_values["instruction_zh"] = (
                f"前进 {executable:.2f} m（分 {segment_count} 段，"
                f"每段不超过 {segment_limit:.2f} m），每段停止并校验"
            )
            command_values["safety_limited"] = bool(
                executable < command.forward_m or segment_count > 1
            )
            command_values["requested_motion"] = {
                "turn_deg": command.turn_deg,
                "planner_forward_m": command.forward_m,
                "forward_m": executable,
                "segment_limit_m": segment_limit,
                "segment_count": segment_count,
                "segmented": segment_count > 1,
            }
            command = type(command)(**command_values)
        # Rebuild with the stable decision id used by the record.
        command = type(command)(**{**command.to_dict(), "decision_id": decision_id})
        live_plan = self._build_live_navigation_plan(goal, observation)
        navigation_plan = (
            live_plan.to_dict()
            if live_plan is not None
            else {
                "plan_id": command.plan_id,
                "planning_frame": "odom",
                "start_pose": observation.pose,
                "goal": goal.to_dict(),
                "executable": True,
                "planner": "relative_goal_fallback",
            }
        )
        return DecisionRecord(
            decision_id=decision_id,
            session_id=self.session_id,
            task_id=self.task_context.task_id if self.task_context else f"task_{self.target}",
            cycle=cycle,
            timestamp=float(self._now()),
            raw_task_text=self.task_context.raw_text if self.task_context else self.target,
            canonical_target=self.target,
            map_revision=self.semantic_graph.revision,
            current_place_id=self._current_place_id,
            current_pose=observation.pose,
            target_match_level=match.target_match_level,
            selected_long_term_goal=goal.to_dict(),
            candidate_ranking=[item.to_dict() for item in candidates],
            navigation_plan=navigation_plan,
            next_motion_command=command.to_dict(),
            reason_zh=command.reason_zh,
            evidence=[{"type": "visual_observation", "observation_id": observation.bundle_id}],
            negative_evidence=[],
            requested_motion=command.requested_motion,
        )

    def _build_live_navigation_plan(
        self, goal: Any, observation: LiveObservation
    ) -> Any | None:
        """Plan against the current semantic graph when a frontier exists.

        Relative turn goals remain executable primitives for RGB-only robots;
        the shared NavigationPlan is still attached whenever the live graph
        has a concrete frontier for the selected heading.
        """
        sector = getattr(goal, "heading_sector", None)
        if sector is None or self._current_place_id is None:
            return None
        frontier_id = f"F{int(sector) + 1:02d}"
        try:
            return plan_live_graph_path(
                self.semantic_graph.to_dict(),
                current_place_id=self._current_place_id,
                goal={"target_frontier_id": frontier_id, "confidence": 0.8},
                robot_pose=observation.pose,
                target_status="target_candidate" if observation.target_present else "target_not_seen",
            )
        except (KeyError, TypeError, ValueError):
            return None

    def _emit(self, name: str, **details: Any) -> None:
        event: dict[str, Any] = {
            "event": name,
            "state": self._state.value,
            "host_s": round(float(self._now()), 6),
            **details,
        }
        event.setdefault("task_id", self.task_context.task_id if self.task_context else None)
        event.setdefault("executor_id", self.executor_id)
        event.setdefault("worker_generation", self.worker_generation)
        self.events.append(event)
        self._on_event(event)

    def _result(self, finish_reason: str, started: float, planning_cycles: int,
                motion_steps: int, observations: int, replans: int,
                navigation_failures: int, verify_attempts: int,
                semantic_goal_selection_count: int,
                fallback_goal_selection_count: int, reason: str) -> SessionResult:
        duration = max(0.0, float(self._now()) - started)
        summary = {
            "result": finish_reason,
            "target": self.target,
            "duration_s": round(duration, 3),
            "planning_cycles": planning_cycles,
            "motion_steps": motion_steps,
            "observations": observations,
            "unique_nodes": len(self.graph.nodes),
            "unique_places": len(self.semantic_graph.place_graph.places),
            "unique_objects": len(self.semantic_graph.object_map.objects),
            "frontiers_discovered": len(self.semantic_graph.frontiers),
            "map_nodes_total": len(self.semantic_graph.to_dict().get("nodes") or []),
            "replans": replans,
            "navigation_failures": navigation_failures,
            "verify_attempts": verify_attempts,
            "semantic_goal_selection_count": semantic_goal_selection_count,
            "fallback_goal_selection_count": fallback_goal_selection_count,
            "finish_reason": reason,
        }
        return SessionResult(
            result=finish_reason,
            target=self.target,
            duration_s=duration,
            planning_cycles=planning_cycles,
            motion_steps=motion_steps,
            observations=observations,
            unique_nodes=len(self.graph.nodes),
            unique_places=len(self.semantic_graph.place_graph.places),
            unique_objects=len(self.semantic_graph.object_map.objects),
            frontiers_discovered=len(self.semantic_graph.frontiers),
            map_nodes_total=len(self.semantic_graph.to_dict().get("nodes") or []),
            replans=replans,
            navigation_failures=navigation_failures,
            verify_attempts=verify_attempts,
            semantic_goal_selection_count=semantic_goal_selection_count,
            fallback_goal_selection_count=fallback_goal_selection_count,
            finish_reason=reason,
            session_id=self.session_id,
            summary=summary,
        )

    def save_artifacts(self, session_dir: str | Path) -> Path:
        """Persist compatibility ledger plus the authoritative semantic map."""
        run_dir = Path(session_dir) / self.session_id
        run_dir.mkdir(parents=True, exist_ok=True)
        target = run_dir / "exploration_graph.json"
        graph_path = self.graph.save(target)
        (run_dir / "semantic_map.json").write_text(
            json.dumps(self.semantic_graph.to_dict(), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        return graph_path
