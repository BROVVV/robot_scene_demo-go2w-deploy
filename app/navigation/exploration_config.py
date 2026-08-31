"""Typed loaders for exploration policy and the operator-supervised profile.

Conventions follow the rest of the repo: plain ``yaml.safe_load`` from
``configs/`` and dataclasses with explicit defaults so a missing file never
silently leaves the system without a policy.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

DEFAULT_EXPLORATION_CONFIG = "configs/exploration/default.yaml"
DEFAULT_EXPERIMENT_PROFILE = "configs/go2w/high_level_experiment.yaml"


@dataclass
class ExplorationBudget:
    max_search_seconds: float = 600.0
    max_planning_cycles: int = 100
    max_motion_steps: int = 50
    max_replans: int = 100
    max_same_node_visits: int = 2
    max_navigation_failures_per_goal: int = 2
    max_consecutive_no_information_cycles: int = 20
    verify_attempts: int = 3
    negative_memory_ttl_seconds: float = 120.0

    def remaining(self, *, elapsed_sec: float, planning_cycles: int,
                  motion_steps: int) -> bool:
        if elapsed_sec >= self.max_search_seconds:
            return False
        if self.max_planning_cycles > 0 and planning_cycles >= self.max_planning_cycles:
            return False
        if self.max_motion_steps > 0 and motion_steps >= self.max_motion_steps:
            return False
        return True

    def to_dict(self) -> dict[str, Any]:
        return {
            "max_search_seconds": self.max_search_seconds,
            "max_planning_cycles": self.max_planning_cycles,
            "max_motion_steps": self.max_motion_steps,
            "max_replans": self.max_replans,
            "max_same_node_visits": self.max_same_node_visits,
            "max_navigation_failures_per_goal": self.max_navigation_failures_per_goal,
            "max_consecutive_no_information_cycles": self.max_consecutive_no_information_cycles,
            "verify_attempts": self.verify_attempts,
            "negative_memory_ttl_seconds": self.negative_memory_ttl_seconds,
        }


@dataclass
class ScoringWeights:
    semantic_relevance: float = 0.35
    novelty: float = 0.25
    information_gain: float = 0.20
    frontier_bonus: float = 0.10
    continuity_bonus: float = 0.10
    visited_penalty: float = 0.30
    negative_evidence_penalty: float = 0.25
    navigation_failure_penalty: float = 0.35
    estimated_motion_cost: float = 0.15
    oscillation_penalty: float = 0.20

    def to_dict(self) -> dict[str, Any]:
        return {
            "semantic_relevance": self.semantic_relevance,
            "novelty": self.novelty,
            "information_gain": self.information_gain,
            "frontier_bonus": self.frontier_bonus,
            "continuity_bonus": self.continuity_bonus,
            "visited_penalty": self.visited_penalty,
            "negative_evidence_penalty": self.negative_evidence_penalty,
            "navigation_failure_penalty": self.navigation_failure_penalty,
            "estimated_motion_cost": self.estimated_motion_cost,
            "oscillation_penalty": self.oscillation_penalty,
        }


@dataclass
class CandidateConfig:
    heading_sectors: int = 12
    max_candidates: int = 12
    unvisited_sector_bonus: float = 0.30
    anchor_reobserve_radius_m: float = 1.5
    last_known_revisit_priority: float = 0.8
    fallback_turn_deg: float = 30.0
    min_turn_deg: float = 5.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "heading_sectors": self.heading_sectors,
            "max_candidates": self.max_candidates,
            "unvisited_sector_bonus": self.unvisited_sector_bonus,
            "anchor_reobserve_radius_m": self.anchor_reobserve_radius_m,
            "last_known_revisit_priority": self.last_known_revisit_priority,
            "fallback_turn_deg": self.fallback_turn_deg,
            "min_turn_deg": self.min_turn_deg,
        }


@dataclass
class MemoryConfig:
    negative_ttl_seconds: float = 120.0
    node_revisit_ttl_seconds: float = 60.0
    max_same_node_visits: int = 2

    def to_dict(self) -> dict[str, Any]:
        return {
            "negative_ttl_seconds": self.negative_ttl_seconds,
            "node_revisit_ttl_seconds": self.node_revisit_ttl_seconds,
            "max_same_node_visits": self.max_same_node_visits,
        }


@dataclass
class RecoveryConfig:
    replan_after_failure: bool = True
    timeout_retry_count: int = 1
    backend_reconnect_attempts: int = 3
    backend_reconnect_delay_seconds: float = 2.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "replan_after_failure": self.replan_after_failure,
            "timeout_retry_count": self.timeout_retry_count,
            "backend_reconnect_attempts": self.backend_reconnect_attempts,
            "backend_reconnect_delay_seconds": self.backend_reconnect_delay_seconds,
        }


@dataclass
class ExplorationPolicy:
    budget: ExplorationBudget = field(default_factory=ExplorationBudget)
    candidates: CandidateConfig = field(default_factory=CandidateConfig)
    scoring: ScoringWeights = field(default_factory=ScoringWeights)
    memory: MemoryConfig = field(default_factory=MemoryConfig)
    recovery: RecoveryConfig = field(default_factory=RecoveryConfig)
    session_dir: str = "outputs/live_runs"
    jsonl_path: str = "outputs/live_sessions"
    save_graph_every_cycle: bool = True

    def to_dict(self) -> dict[str, Any]:
        return {
            "budget": self.budget.to_dict(),
            "candidates": self.candidates.to_dict(),
            "scoring": self.scoring.to_dict(),
            "memory": self.memory.to_dict(),
            "recovery": self.recovery.to_dict(),
            "session_dir": self.session_dir,
            "jsonl_path": self.jsonl_path,
            "save_graph_every_cycle": self.save_graph_every_cycle,
        }


def load_exploration_policy(
    path: str | Path | None = None,
    *,
    overrides: dict[str, Any] | None = None,
) -> ExplorationPolicy:
    """Load configs/exploration/default.yaml and apply nested overrides."""
    source = Path(path or DEFAULT_EXPLORATION_CONFIG)
    payload: dict[str, Any] = {}
    if source.is_file():
        payload = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    payload = _merge(payload, overrides or {})
    exploration = payload.get("exploration") or {}
    policy = ExplorationPolicy(
        budget=_budget_from(exploration.get("budget") or {}),
        candidates=_candidates_from(exploration.get("candidates") or {}),
        scoring=_scoring_from(exploration.get("scoring") or {}),
        memory=_memory_from(exploration.get("memory") or {}),
        recovery=_recovery_from(exploration.get("recovery") or {}),
        session_dir=str(exploration.get("logging", {}).get("session_dir")
                        or payload.get("session_dir") or "outputs/live_runs"),
        jsonl_path=str(exploration.get("logging", {}).get("jsonl_path")
                       or payload.get("jsonl_path") or "outputs/live_sessions"),
        save_graph_every_cycle=bool(
            exploration.get("logging", {}).get("save_graph_every_cycle", True)
        ),
    )
    return policy


def load_go2w_experiment_profile(
    path: str | Path | None = None,
) -> dict[str, Any]:
    source = Path(path or DEFAULT_EXPERIMENT_PROFILE)
    if not source.is_file():
        return {"profile": "operator_supervised_experiment"}
    payload = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    return payload


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def _budget_from(value: dict[str, Any]) -> ExplorationBudget:
    return ExplorationBudget(
        max_search_seconds=float(value.get("max_search_seconds", 600.0)),
        max_planning_cycles=int(value.get("max_planning_cycles", 100)),
        max_motion_steps=int(value.get("max_motion_steps", 50)),
        max_replans=int(value.get("max_replans", 100)),
        max_same_node_visits=int(value.get("max_same_node_visits", 2)),
        max_navigation_failures_per_goal=int(value.get("max_navigation_failures_per_goal", 2)),
        max_consecutive_no_information_cycles=int(
            value.get("max_consecutive_no_information_cycles", 20)
        ),
        verify_attempts=int(value.get("verify_attempts", 3)),
        negative_memory_ttl_seconds=float(value.get("negative_memory_ttl_seconds", 120.0)),
    )


def _scoring_from(value: dict[str, Any]) -> ScoringWeights:
    return ScoringWeights(
        semantic_relevance=float(value.get("semantic_relevance", 0.35)),
        novelty=float(value.get("novelty", 0.25)),
        information_gain=float(value.get("information_gain", 0.20)),
        frontier_bonus=float(value.get("frontier_bonus", 0.10)),
        continuity_bonus=float(value.get("continuity_bonus", 0.10)),
        visited_penalty=float(value.get("visited_penalty", 0.30)),
        negative_evidence_penalty=float(value.get("negative_evidence_penalty", 0.25)),
        navigation_failure_penalty=float(value.get("navigation_failure_penalty", 0.35)),
        estimated_motion_cost=float(value.get("estimated_motion_cost", 0.15)),
        oscillation_penalty=float(value.get("oscillation_penalty", 0.20)),
    )


def _candidates_from(value: dict[str, Any]) -> CandidateConfig:
    return CandidateConfig(
        heading_sectors=int(value.get("heading_sectors", 12)),
        max_candidates=int(value.get("max_candidates", 12)),
        unvisited_sector_bonus=float(value.get("unvisited_sector_bonus", 0.30)),
        anchor_reobserve_radius_m=float(value.get("anchor_reobserve_radius_m", 1.5)),
        last_known_revisit_priority=float(value.get("last_known_revisit_priority", 0.8)),
        fallback_turn_deg=float(value.get("fallback_turn_deg", 30.0)),
        min_turn_deg=float(value.get("min_turn_deg", 5.0)),
    )


def _memory_from(value: dict[str, Any]) -> MemoryConfig:
    return MemoryConfig(
        negative_ttl_seconds=float(value.get("negative_ttl_seconds", 120.0)),
        node_revisit_ttl_seconds=float(value.get("node_revisit_ttl_seconds", 60.0)),
        max_same_node_visits=int(value.get("max_same_node_visits", 2)),
    )


def _recovery_from(value: dict[str, Any]) -> RecoveryConfig:
    return RecoveryConfig(
        replan_after_failure=bool(value.get("replan_after_failure", True)),
        timeout_retry_count=int(value.get("timeout_retry_count", 1)),
        backend_reconnect_attempts=int(value.get("backend_reconnect_attempts", 3)),
        backend_reconnect_delay_seconds=float(
            value.get("backend_reconnect_delay_seconds", 2.0)
        ),
    )


def _merge(base: dict[str, Any], overlay: dict[str, Any]) -> dict[str, Any]:
    result = dict(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = value
    return result
