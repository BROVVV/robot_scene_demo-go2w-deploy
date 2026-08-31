"""Tests for LongTermGoalSelector and SpatialSearchReasoner."""

from __future__ import annotations

from app.navigation.long_term_goal_selector import (
    MATCH_PARTIAL,
    MATCH_ZERO,
    LongTermGoalSelector,
)
from app.reasoning.semantic_navigation.models import (
    GoalGraph,
    GoalGraphEdge,
    GoalGraphNode,
    GraphMatchResult,
    GraphMatchState,
)
from app.reasoning.semantic_navigation.semantic_prior_provider import RuleSemanticPriorProvider
from app.reasoning.semantic_navigation.spatial_reasoner import SpatialSearchReasoner
from app.spatial.models import (
    INTENT_EXPLORE_FRONTIER,
    INTENT_INSPECT_ANCHOR_REGION,
    FrontierCandidate,
    SemanticRegion,
    SemanticPrior,
)


def _frontiers() -> list[FrontierCandidate]:
    return [
        FrontierCandidate("F1", position=(1.0, 0.0), bearing_deg=0.0, distance_m=1.0,
                          spatial_information_gain=0.8),
        FrontierCandidate("F2", position=(-1.0, 0.0), bearing_deg=180.0, distance_m=1.0,
                          spatial_information_gain=0.3),
    ]


def test_zero_selects_frontier():
    selector = LongTermGoalSelector()
    result = selector.select(match_state=MATCH_ZERO, frontiers=_frontiers())
    assert result is not None
    assert result.intent.intent_type == INTENT_EXPLORE_FRONTIER
    assert result.intent.target_frontier_id == "F1"


def test_psg_prior_affects_ranking():
    selector = LongTermGoalSelector()
    prior = SemanticPrior(frontier_scores={"F2": 1.0})
    result = selector.select(match_state=MATCH_ZERO, frontiers=_frontiers(), psg_prior=prior)
    # F2 has lower spatial gain but very high PSG score, should be selected.
    assert result.intent.target_frontier_id == "F2"


def test_partial_anchor_region_preferred():
    selector = LongTermGoalSelector()
    region = SemanticRegion(
        region_id="r1", anchor_object_id="a1", relation="near", confidence=0.9
    )
    prior = SemanticPrior(region_hypotheses=[region])
    result = selector.select(match_state=MATCH_PARTIAL, frontiers=_frontiers(), psg_prior=prior)
    assert result.intent.intent_type == INTENT_INSPECT_ANCHOR_REGION


def test_spatial_reasoner_maps_graph_match():
    goal_graph = GoalGraph(
        task_id="t", raw_query="饮水机旁边的蓝色垃圾桶", target_node_id="goal_target",
        nodes=[
            GoalGraphNode("goal_target", "蓝色垃圾桶", "target"),
            GoalGraphNode("anchor_1", "饮水机", "anchor"),
        ],
        edges=[GoalGraphEdge("e1", "goal_target", "anchor_1", "near")],
    )
    match = GraphMatchResult(state=GraphMatchState.PARTIAL, score=0.6)
    reasoner = SpatialSearchReasoner()
    result = reasoner.propose(graph_match=match, frontiers=_frontiers())
    assert result is not None
