"""Tests for RuleSemanticPriorProvider separation and negative memory."""

from __future__ import annotations

from app.reasoning.semantic_navigation.models import GoalGraph, GoalGraphEdge, GoalGraphNode
from app.reasoning.semantic_navigation.semantic_prior_provider import RuleSemanticPriorProvider
from app.spatial.spatial_memory import SpatialMemory


def _goal_graph() -> GoalGraph:
    return GoalGraph(
        task_id="t",
        raw_query="饮水机旁边的蓝色垃圾桶",
        target_node_id="goal_target",
        nodes=[
            GoalGraphNode("goal_target", "蓝色垃圾桶", "target"),
            GoalGraphNode("anchor_1", "饮水机", "anchor"),
        ],
        edges=[GoalGraphEdge("e1", "goal_target", "anchor_1", "near")],
    )


def test_provider_creates_region_from_observed_anchor():
    provider = RuleSemanticPriorProvider(confidence=0.7)
    observed = {
        "nodes": [
            {"node_id": "n1", "label_zh": "饮水机", "attributes": {"bearing_deg": 20.0}},
        ]
    }
    prior = provider.predict(_goal_graph(), observed_scene_graph=observed)
    assert len(prior.region_hypotheses) == 1
    region = prior.region_hypotheses[0]
    assert region.anchor_object_id == "n1"
    assert region.relation == "near"
    assert region.state == "PREDICTED"


def test_provider_no_false_positive_without_anchor():
    provider = RuleSemanticPriorProvider()
    prior = provider.predict(_goal_graph(), observed_scene_graph={"nodes": []})
    assert prior.region_hypotheses == []


def test_spatial_memory_blacklists_repeated_negative():
    memory = SpatialMemory()
    for _ in range(3):
        memory.region_negative("r1", place_id="P1")
    assert memory.regions["r1"].state == "REJECTED"
    assert "r1" in memory.blacklist
