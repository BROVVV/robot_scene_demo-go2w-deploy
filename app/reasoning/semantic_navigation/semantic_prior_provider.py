"""PSG SemanticPriorProvider: turns GoalGraph + observed anchors into spatial
semantic prior hypotheses (regions and frontier scores).

PSG output is always *predicted*, never observed fact.  It can influence
frontier ranking but must not confirm a target or directly emit motion.
"""

from __future__ import annotations

from typing import Any

from app.reasoning.semantic_navigation.models import GoalGraph
from app.spatial.models import SemanticPrior, SemanticRegion


class RuleSemanticPriorProvider:
    """A deterministic, explainable PSG provider.

    It is a placeholder for future LLM-based PSG; the interface and negative-
    evidence handling are already correct.
    """

    def __init__(self, *, confidence: float = 0.6) -> None:
        self.confidence = float(confidence)

    def predict(
        self,
        goal_graph: GoalGraph | None,
        observed_scene_graph: Any = None,
        spatial_context: Any = None,
        semantic_memory: Any = None,
    ) -> SemanticPrior:
        if goal_graph is None:
            return SemanticPrior(confidence=0.0, provenance={"source": "rule_psg_no_goal"})
        anchor_nodes = [node for node in goal_graph.nodes if node.role in {"anchor", "context"}]
        observed = self._observed_nodes(observed_scene_graph)
        regions: list[SemanticRegion] = []
        frontier_scores: dict[str, float] = {}
        for goal_node in anchor_nodes:
            matches = self._match_observed(goal_node.label, observed)
            if not matches:
                continue
            anchor = matches[0]
            anchor_id = str(anchor.get("node_id") or anchor.get("id") or goal_node.node_id)
            center = self._center(anchor)
            bearing = self._bearing(anchor)
            region = SemanticRegion(
                region_id=f"region_{goal_node.node_id}_{anchor_id}",
                anchor_object_id=anchor_id,
                relation=self._relation_for(goal_graph, goal_node.node_id),
                center=center,
                radius_min_m=0.3,
                radius_max_m=1.5,
                bearing_range_deg=((bearing - 30.0, bearing + 30.0) if bearing is not None else None),
                confidence=self.confidence,
                metric_claim=center is not None,
                source="rule_psg",
                state="PREDICTED",
            )
            regions.append(region)
            # Give a small semantic bonus to frontiers whose bearing is near the
            # anchor bearing when no metric center is available.
            if spatial_context is not None and center is None and bearing is not None:
                frontiers = getattr(spatial_context, "get_frontiers", lambda: [])()
                for frontier in frontiers:
                    if frontier.bearing_deg is None:
                        continue
                    delta = abs((frontier.bearing_deg - bearing + 180.0) % 360.0 - 180.0)
                    if delta < 45.0:
                        frontier_scores[frontier.frontier_id] = max(
                            frontier_scores.get(frontier.frontier_id, 0.0),
                            0.6 * (1.0 - delta / 45.0),
                        )
        return SemanticPrior(
            predicted_nodes=[node.to_dict() for node in goal_graph.nodes],
            predicted_relations=[edge.to_dict() for edge in goal_graph.edges],
            anchor_hypotheses=[region.to_dict() for region in regions],
            region_hypotheses=regions,
            frontier_scores=frontier_scores,
            confidence=self.confidence if regions else 0.2,
            provenance={"source": "rule_semantic_prior_provider"},
        )

    def _observed_nodes(self, scene_graph: Any) -> list[dict[str, Any]]:
        if scene_graph is None:
            return []
        nodes = getattr(scene_graph, "nodes", None)
        if nodes is None and isinstance(scene_graph, dict):
            nodes = scene_graph.get("nodes") or []
        result: list[dict[str, Any]] = []
        for node in nodes or []:
            if isinstance(node, dict):
                result.append(node)
            elif hasattr(node, "to_dict"):
                result.append(node.to_dict())
            else:
                result.append(
                    {
                        "node_id": getattr(node, "node_id", None),
                        "id": getattr(node, "node_id", None),
                        "label": getattr(node, "label", None),
                        "label_zh": getattr(node, "label_zh", None),
                        "attributes": getattr(node, "attributes", {}) or {},
                    }
                )
        return result

    def _match_observed(self, label: str, observed: list[dict[str, Any]]) -> list[dict[str, Any]]:
        key = self._norm(label)
        matches = []
        for node in observed:
            node_label = str(node.get("label_zh") or node.get("label") or node.get("name") or "")
            if key and key == self._norm(node_label):
                matches.append(node)
        return matches

    @staticmethod
    def _center(node: dict[str, Any]) -> tuple[float, float] | None:
        attrs = node.get("attributes") or {}
        if isinstance(attrs.get("map_xyz"), (list, tuple)) and len(attrs["map_xyz"]) >= 2:
            return (float(attrs["map_xyz"][0]), float(attrs["map_xyz"][1]))
        if isinstance(attrs.get("camera_xyz"), (list, tuple)) and len(attrs["camera_xyz"]) >= 2:
            # Camera-local cannot be treated as map; only useful for bearing.
            return None
        return None

    @staticmethod
    def _bearing(node: dict[str, Any]) -> float | None:
        attrs = node.get("attributes") or {}
        value = attrs.get("bearing_deg")
        if isinstance(value, (int, float)):
            return float(value)
        return None

    @staticmethod
    def _relation_for(goal_graph: GoalGraph, node_id: str) -> str:
        for edge in goal_graph.edges:
            if edge.source_node_id == node_id or edge.target_node_id == node_id:
                return edge.relation
        return "near"

    @staticmethod
    def _norm(value: str) -> str:
        return "".join(ch for ch in str(value).lower() if ch.isalnum())
