"""Explainable lexical/attribute/relationship Goal Graph matcher."""

from __future__ import annotations

import re
from typing import Any

from app.reasoning.semantic_navigation.models import GoalGraph, GraphMatchResult, GraphMatchState


class SemanticNavigationGraphMatcher:
    def __init__(self, partial_threshold: float = 0.30,
                 strong_threshold: float = 0.72) -> None:
        if not 0.0 <= partial_threshold <= strong_threshold <= 1.0:
            raise ValueError("graph match thresholds must satisfy 0 <= partial <= strong <= 1")
        self.partial_threshold = partial_threshold
        self.strong_threshold = strong_threshold

    def match(self, goal_graph: GoalGraph, scene_graph: Any, *,
              target_profile: Any = None) -> GraphMatchResult:
        scene_nodes = list(_field(scene_graph, "nodes", []))
        scene_edges = list(_field(scene_graph, "edges", []))
        node_matches: dict[str, Any] = {}
        attribute_support: list[str] = []
        warnings: list[str] = []
        evidence_refs: list[str] = []
        roles = {node.node_id: node.role for node in goal_graph.nodes}
        weights = _role_weights(goal_graph)
        node_score = 0.0
        for goal_node in goal_graph.nodes:
            candidate, lexical = _best_node(goal_node, scene_nodes)
            if candidate is None or lexical <= 0.0:
                continue
            node_matches[goal_node.node_id] = candidate
            scene_id = str(_field(candidate, "node_id", ""))
            if scene_id:
                evidence_refs.append(scene_id)
            supported = _attribute_support(goal_node.attributes, candidate)
            attribute_support.extend(
                f"{goal_node.node_id}:{item}" for item in supported
            )
            attribute_factor = 1.0
            if goal_node.attributes:
                support_ratio = len(supported) / len(goal_node.attributes)
                attribute_factor = 0.5 + 0.5 * support_ratio
                if not supported:
                    warnings.append(f"attribute_mismatch:{goal_node.node_id}")
            node_score += weights[goal_node.node_id] * lexical * attribute_factor

        matched_relations: list[str] = []
        unmatched_relations: list[str] = []
        explicit_edges = [edge for edge in goal_graph.edges if edge.relation != "context_hint"]
        for edge in goal_graph.edges:
            left = node_matches.get(edge.source_node_id)
            right = node_matches.get(edge.target_node_id)
            descriptor = f"{edge.source_node_id}:{edge.relation}:{edge.target_node_id}"
            if left is not None and right is not None and _relation_present(
                left, right, edge.relation, scene_edges
            ):
                matched_relations.append(descriptor)
            elif edge.relation != "context_hint":
                unmatched_relations.append(descriptor)

        relation_score = (
            len(matched_relations) / max(1, len(explicit_edges)) if explicit_edges else 0.0
        )
        score = min(1.0, 0.85 * node_score + 0.15 * relation_score)
        matched_ids = list(node_matches)
        target_present = goal_graph.target_node_id in node_matches
        if score >= self.strong_threshold and target_present and not unmatched_relations:
            state = GraphMatchState.STRONG
        elif score >= self.partial_threshold or matched_ids:
            state = GraphMatchState.PARTIAL
        else:
            state = GraphMatchState.ZERO
        missing = [node.node_id for node in goal_graph.nodes if node.node_id not in node_matches]
        anchors = [
            str(_field(node_matches[node_id], "node_id", ""))
            for node_id in matched_ids if roles.get(node_id) in {"anchor", "context"}
        ]
        reason = {
            GraphMatchState.ZERO: "当前观察未匹配到目标图中的目标或锚点。",
            GraphMatchState.PARTIAL: "当前观察匹配到部分目标图节点，仍需围绕锚点重观测。",
            GraphMatchState.STRONG: "目标图得到较强视觉支持，但仍须走现有视觉复核链确认目标。",
        }[state]
        return GraphMatchResult(
            state=state,
            score=round(score, 4),
            matched_goal_node_ids=matched_ids,
            missing_goal_node_ids=missing,
            matched_scene_node_ids=[
                str(_field(item, "node_id", "")) for item in node_matches.values()
            ],
            supporting_anchor_scene_node_ids=[item for item in anchors if item],
            unmatched_relations=unmatched_relations,
            target_node_visually_present=target_present,
            reason_zh=reason,
            evidence_refs=list(dict.fromkeys(evidence_refs)),
            warnings=warnings,
            matched_relations=matched_relations,
            attribute_support=attribute_support,
            anchor_support=[item for item in anchors if item],
        )


def _role_weights(goal_graph: GoalGraph) -> dict[str, float]:
    raw = {"target": 0.60, "anchor": 0.30, "context": 0.10}
    totals = sum(raw.get(node.role, 0.05) for node in goal_graph.nodes)
    return {node.node_id: raw.get(node.role, 0.05) / totals for node in goal_graph.nodes}


def _best_node(goal_node: Any, scene_nodes: list[Any]) -> tuple[Any | None, float]:
    terms = [_normalize(goal_node.label), *[_normalize(item) for item in goal_node.aliases]]
    best = None
    best_score = 0.0
    for node in scene_nodes:
        labels = [
            _normalize(str(_field(node, "label", ""))),
            _normalize(str(_field(node, "label_zh", ""))),
        ]
        attrs = _field(node, "attributes", {}) or {}
        labels.extend(_normalize(str(item)) for item in attrs.get("aliases", []))
        score = max((_lexical(left, right) for left in terms for right in labels), default=0.0)
        if score > best_score:
            best, best_score = node, score
    return best, best_score


def _lexical(left: str, right: str) -> float:
    if not left or not right:
        return 0.0
    if left == right:
        return 1.0
    if len(left) >= 2 and (left in right or right in left):
        return 0.82
    left_tokens = set(re.findall(r"[a-z0-9]+|[\u4e00-\u9fff]", left))
    right_tokens = set(re.findall(r"[a-z0-9]+|[\u4e00-\u9fff]", right))
    overlap = len(left_tokens & right_tokens) / max(1, len(left_tokens | right_tokens))
    return 0.65 * overlap if overlap >= 0.5 else 0.0


def _attribute_support(attributes: list[str], scene_node: Any) -> list[str]:
    data = _field(scene_node, "attributes", {}) or {}
    text = _normalize(" ".join([
        str(_field(scene_node, "label", "")), str(_field(scene_node, "label_zh", "")),
        str(data.get("attributes", "")), str(data.get("color", "")),
    ]))
    return [item for item in attributes if _normalize(item) and _normalize(item) in text]


def _relation_present(left: Any, right: Any, relation: str, edges: list[Any]) -> bool:
    left_id = str(_field(left, "node_id", ""))
    right_id = str(_field(right, "node_id", ""))
    aliases = {relation}
    if relation == "near":
        aliases.update({"adjacent_to", "next_to", "beside"})
    if relation == "inside":
        aliases.add("in")
    for edge in edges:
        source = str(_field(edge, "source_node_id", ""))
        target = str(_field(edge, "target_node_id", ""))
        rel = str(_field(edge, "relation", ""))
        if rel in aliases and {(source, target), (target, source)} & {
            (left_id, right_id), (right_id, left_id)
        }:
            return True
    return False


def _field(value: Any, name: str, default: Any) -> Any:
    if isinstance(value, dict):
        return value.get(name, default)
    return getattr(value, name, default)


def _normalize(value: str) -> str:
    return "".join(re.findall(r"[a-z0-9\u4e00-\u9fff]+", value.lower()))
