"""Frontier-style exploration planning on a video navigation topology,
plus the session-aware live exploration goal scoring (plan section 10)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .exploration_config import ScoringWeights
from .exploration_graph import ExplorationGraph, NodeState
from .models import ExplorationGoal, NavigationWaypoint, Pose2D


def generate_exploration_candidates(
    navigation_map: dict[str, Any],
    target_search_result: dict[str, Any] | None = None,
    max_candidates: int = 8,
) -> list[NavigationWaypoint]:
    nodes = list(navigation_map.get("nodes", []))
    candidates: list[NavigationWaypoint] = []
    for index, node in enumerate(nodes[1:] or nodes):
        pose = Pose2D.from_dict(node.get("pose") or {})
        information_gain = _information_gain(index, len(nodes))
        target_relevance = _target_relevance(node, target_search_result or {})
        path_cost = index / max(len(nodes), 1)
        score = information_gain + target_relevance + 0.2 - path_cost * 0.25
        if score <= 0:
            continue
        candidates.append(
            NavigationWaypoint(
                waypoint_id=f"frontier_{index + 1:02d}",
                pose=pose,
                source_frame_id=node.get("frame_id"),
                semantic_label=f"探索点 {index + 1}",
                waypoint_type="frontier",
                confidence=round(min(score, 1.0), 4),
                provenance={
                    "source": "video_frontier_exploration",
                    "information_gain": round(information_gain, 4),
                    "target_relevance": round(target_relevance, 4),
                    "path_cost": round(path_cost, 4),
                },
            )
        )
    candidates.sort(key=lambda item: item.confidence, reverse=True)
    return candidates[:max_candidates]


@dataclass
class ScoredGoal:
    goal: ExplorationGoal
    score: float
    components: dict[str, float] = field(default_factory=dict)
    reasons: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "goal": self.goal.to_dict(),
            "score": round(self.score, 4),
            "components": {key: round(value, 4) for key, value in self.components.items()},
            "reasons": self.reasons,
        }


def score_exploration_goal(
    goal: ExplorationGoal,
    *,
    graph: ExplorationGraph,
    weights: ScoringWeights | None = None,
    current_yaw_deg: float = 0.0,
) -> ScoredGoal:
    """Score one candidate with the configurable plan-section-10 formula."""
    weights = weights or ScoringWeights()
    components: dict[str, float] = {}
    reasons: list[str] = []

    # ---- positive terms ----------------------------------------------------
    semantic = max(0.0, min(1.0, goal.semantic_relevance))
    components["semantic_relevance"] = semantic
    if goal.semantic_anchor:
        reasons.append(f"语义锚点 {goal.semantic_anchor}")

    node = graph.get_node(goal.target_node_id) if goal.target_node_id else None
    max_visits = max(1, graph_visit_limit(graph))
    visited = node.visited_count if node is not None else 0
    novelty = 1.0 - min(1.0, visited / max_visits)
    if node is not None and node.heading_sector is not None:
        sector_visits = graph.sector_visited_count(node.heading_sector)
        novelty = min(novelty, 1.0 - min(1.0, sector_visits / max(1, max_visits * 2)))
    components["novelty"] = novelty

    information = max(0.0, min(1.0, goal.expected_information_gain))
    components["information_gain"] = information

    frontier = 0.0
    if goal.heading_sector is not None and graph.sector_visited_count(goal.heading_sector) == 0:
        frontier = 1.0
        reasons.append(f"未探索扇区 {goal.heading_sector}")
    components["frontier_bonus"] = frontier

    dyaw = abs(float(goal.relative_dyaw or 0.0))
    continuity = 1.0 - min(1.0, dyaw / 90.0)
    components["continuity_bonus"] = continuity

    # ---- penalties ----------------------------------------------------------
    visited_penalty = min(1.0, visited / max(1, max_visits))
    if node is None and goal.heading_sector is not None:
        visited_penalty = max(
            visited_penalty,
            min(1.0, graph.sector_visited_count(goal.heading_sector) / max(1, max_visits * 2)),
        )
    components["visited_penalty"] = visited_penalty
    if visited_penalty > 0.3:
        reasons.append(f"节点已访问 {visited} 次")

    negative = 0.0
    negative_refs: list[str] = []
    if node is not None:
        negative = min(1.0, node.negative_evidence_count * 0.35)
    prov_negative = float(goal.provenance.get("negative_memory_penalty", 0.0) or 0.0)
    negative = max(negative, min(1.0, prov_negative))
    negative_refs = list(goal.provenance.get("negative_memory_refs") or [])
    components["negative_evidence_penalty"] = negative
    if negative > 0.0:
        reasons.append("存在负证据" + (f"({len(negative_refs)} 条)" if negative_refs else ""))

    failure = 0.0
    if node is not None:
        failure = min(1.0, node.navigation_fail_count * 0.5)
    if goal.heading_sector is not None:
        sector_failures = graph.sector_failure_count(goal.heading_sector)
        failure = max(failure, min(1.0, sector_failures * 0.35))
    components["navigation_failure_penalty"] = failure
    if failure > 0.0:
        reasons.append(f"导航失败 {node.navigation_fail_count if node else 0} 次")

    cost = min(1.0, dyaw / 180.0)
    if goal.goal_type == "RELATIVE_MOVE":
        cost = 0.5
    components["estimated_motion_cost"] = cost

    oscillation = graph.oscillation_penalty(goal.heading_sector, goal.target_node_id)
    components["oscillation_penalty"] = oscillation
    if oscillation > 0.0:
        reasons.append("检测到振荡风险")

    score = (
        weights.semantic_relevance * semantic
        + weights.novelty * novelty
        + weights.information_gain * information
        + weights.frontier_bonus * frontier
        + weights.continuity_bonus * continuity
        - weights.visited_penalty * visited_penalty
        - weights.negative_evidence_penalty * negative
        - weights.navigation_failure_penalty * failure
        - weights.estimated_motion_cost * cost
        - weights.oscillation_penalty * oscillation
    )
    return ScoredGoal(
        goal=goal,
        score=score,
        components={
            **components,
            "score": score,
            "current_yaw_deg": current_yaw_deg,
        },
        reasons=reasons,
    )


def select_exploration_goal(
    candidates: list[ExplorationGoal],
    *,
    graph: ExplorationGraph,
    weights: ScoringWeights | None = None,
    current_yaw_deg: float = 0.0,
    exclude_node_ids: set[str] | None = None,
    exclude_sectors: set[int] | None = None,
) -> ScoredGoal | None:
    """Score all candidates and return the best that is not excluded."""
    exclude_node_ids = exclude_node_ids or set()
    exclude_sectors = exclude_sectors or set()
    scored: list[ScoredGoal] = []
    for goal in candidates:
        if goal.target_node_id in exclude_node_ids:
            continue
        if goal.heading_sector in exclude_sectors:
            continue
        scored.append(
            score_exploration_goal(goal, graph=graph, weights=weights,
                                   current_yaw_deg=current_yaw_deg)
        )
    if not scored:
        return None
    return max(scored, key=lambda item: item.score)


def graph_visit_limit(graph: ExplorationGraph) -> int:
    """Per-node visit cap: 2 by default, visible through node state."""
    return 2


def _information_gain(index: int, total: int) -> float:
    if total <= 1:
        return 0.5
    return 0.35 + 0.5 * (index + 1) / total


def _target_relevance(node: dict[str, Any], result: dict[str, Any]) -> float:
    text = " ".join(str(item) for item in node.get("objects", []))
    profile = result.get("target_profile") or {}
    terms = profile.get("context_terms") or profile.get("detector_terms") or []
    if any(str(term).lower() in text.lower() for term in terms):
        return 0.25
    return 0.0
