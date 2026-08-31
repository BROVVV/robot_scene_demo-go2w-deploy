"""Merge observed and predicted video scene graphs while preserving source."""

from __future__ import annotations

from app.video.schemas import PSGLayer, SceneGraph, SceneGraphEdge, SceneGraphNode


class PSGGraphMerger:
    """Build a hybrid graph without letting predictions overwrite observations."""

    def __init__(self, confidence_threshold: float = 0.45) -> None:
        self.confidence_threshold = confidence_threshold

    def merge(
        self,
        observed_graph: SceneGraph,
        psg_layer: PSGLayer,
    ) -> tuple[SceneGraph, dict[str, object]]:
        observed_ids = {node.node_id for node in observed_graph.nodes}
        observed_signatures = {
            (_signature(node.label), node.category)
            for node in observed_graph.nodes
            if node.source == "observed"
        }
        nodes: list[SceneGraphNode] = list(observed_graph.nodes)
        edges: list[SceneGraphEdge] = list(observed_graph.edges)
        dropped_nodes: list[str] = []
        resolved_nodes: list[str] = []
        admitted_predicted_ids: set[str] = set()

        for node in psg_layer.predicted_nodes:
            node.source = "predicted"
            node.can_confirm_target = False
            if node.confidence < self.confidence_threshold:
                dropped_nodes.append(node.node_id)
                continue
            if (_signature(node.label), node.category) in observed_signatures:
                node.attributes["resolved_by_observed"] = True
                resolved_nodes.append(node.node_id)
                continue
            nodes.append(node)
            admitted_predicted_ids.add(node.node_id)

        dropped_edges: list[str] = []
        for edge in psg_layer.predicted_edges:
            edge.source = "predicted"
            edge.attributes["requires_visual_confirmation"] = True
            has_observed_anchor = (
                edge.source_node_id in observed_ids or edge.target_node_id in observed_ids
            )
            endpoints_known = (
                edge.source_node_id in observed_ids
                or edge.source_node_id in admitted_predicted_ids
            ) and (
                edge.target_node_id in observed_ids
                or edge.target_node_id in admitted_predicted_ids
            )
            if not has_observed_anchor or not endpoints_known:
                dropped_edges.append(edge.edge_id)
                continue
            edges.append(edge)

        report = {
            "observed_nodes": len(observed_graph.nodes),
            "observed_edges": len(observed_graph.edges),
            "predicted_nodes_admitted": len(admitted_predicted_ids),
            "predicted_edges_admitted": len(psg_layer.predicted_edges) - len(dropped_edges),
            "dropped_predicted_nodes": dropped_nodes,
            "dropped_predicted_edges": dropped_edges,
            "resolved_predicted_nodes": resolved_nodes,
            "warnings": list(psg_layer.warnings),
        }
        return SceneGraph(nodes=nodes, edges=edges), report


def _signature(value: str) -> str:
    return value.lower().replace("_", " ").strip()
