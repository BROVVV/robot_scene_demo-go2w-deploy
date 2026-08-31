"""Predictive Scene Graph layer for video full-scene maps."""

from __future__ import annotations

from typing import Any

from app.video.schemas import (
    NextBestView,
    PSGHypothesis,
    PSGLayer,
    SceneGraph,
    SceneGraphEdge,
    SceneGraphNode,
)


ALLOWED_PSG_RELATIONS = {
    "may_connect_to",
    "likely_connects_to",
    "likely_room_beyond",
    "likely_corridor_continuation",
    "likely_exit_direction",
    "explore_candidate",
    "may_contain",
    "may_be_near",
}
FORBIDDEN_PSG_RELATIONS = {
    "target_is_at",
    "object_confirmed",
    "safe_to_enter",
    "free_space_confirmed",
    "passable_confirmed",
    "obstacle_cleared",
}
SAFE_ACTIONS = {
    "turn_left_and_observe",
    "turn_right_and_observe",
    "move_forward_short_and_observe",
    "stop_and_reobserve",
    "scan_left_right",
    "approach_anchor_carefully",
}
FORBIDDEN_ACTIONS = {"enter_room", "cross_door", "go_to_target", "confirm_target"}


class VideoPSGPredictor:
    """Generate a conservative prediction layer from observed graph evidence."""

    def __init__(
        self,
        max_predicted_nodes: int = 30,
        confidence_threshold: float = 0.45,
    ) -> None:
        self.max_predicted_nodes = max_predicted_nodes
        self.confidence_threshold = confidence_threshold

    def predict(
        self,
        observed_graph: SceneGraph,
        memory_context: dict[str, Any] | None = None,
    ) -> PSGLayer:
        del memory_context
        layer = PSGLayer()
        predicted_count = 0
        for node in observed_graph.nodes:
            if predicted_count >= self.max_predicted_nodes:
                break
            role = str(node.attributes.get("navigation_role", ""))
            label_text = f"{node.label} {node.label_zh}".lower()
            if role == "passage" or any(term in label_text for term in ["door", "doorway", "门", "通道"]):
                self._add_prediction(
                    layer,
                    anchor=node,
                    label="room",
                    label_zh="房间",
                    relation="may_connect_to",
                    action=_action_from_position(str(node.attributes.get("stable_position_2d", ""))),
                    reason_zh=f"观察到 {node.label_zh}，其后方可能连接新的房间或区域，需要视觉确认。",
                    confidence=max(self.confidence_threshold, min(0.72, node.confidence * 0.8)),
                )
                predicted_count += 1
            elif "corridor" in label_text or "走廊" in label_text:
                self._add_prediction(
                    layer,
                    anchor=node,
                    label="corridor_continuation",
                    label_zh="走廊延伸",
                    relation="likely_corridor_continuation",
                    action="scan_left_right",
                    reason_zh="已观察到走廊区域，走廊两端可能还有可探索方向。",
                    confidence=0.58,
                )
                predicted_count += 1
            elif any(term in label_text for term in ["stairs", "elevator", "exit", "楼梯", "电梯", "出口"]):
                self._add_prediction(
                    layer,
                    anchor=node,
                    label="transition_area",
                    label_zh="楼层/出口过渡区域",
                    relation="likely_connects_to",
                    action="approach_anchor_carefully",
                    reason_zh=f"{node.label_zh} 是强导航地标，附近可能有过渡空间，必须近距离重观测。",
                    confidence=max(self.confidence_threshold, min(0.68, node.confidence * 0.75)),
                )
                predicted_count += 1
        return self._sanitize(layer, observed_graph)

    def _add_prediction(
        self,
        layer: PSGLayer,
        anchor: SceneGraphNode,
        label: str,
        label_zh: str,
        relation: str,
        action: str,
        reason_zh: str,
        confidence: float,
    ) -> None:
        index = len(layer.predicted_nodes) + 1
        node_id = f"pred_{label}_{index:03d}"
        edge_id = f"pred_edge_{index:03d}"
        confidence = _clamp(confidence)
        layer.predicted_nodes.append(
            SceneGraphNode(
                node_id=node_id,
                node_type="predicted_region",
                label="room beyond door" if label == "room" else label,
                label_zh="门后的可能房间" if label == "room" else label_zh,
                category="region",
                source="predicted",
                confidence=confidence,
                evidence_level="predicted_explorable",
                based_on=[anchor.node_id],
                can_confirm_target=False,
                attributes={
                    "reason_zh": reason_zh,
                    "navigation_role": "explore_point",
                    "requires_visual_confirmation": True,
                    "can_confirm_target": False,
                },
            )
        )
        layer.predicted_edges.append(
            SceneGraphEdge(
                edge_id=edge_id,
                source_node_id=anchor.node_id,
                target_node_id=node_id,
                relation=relation,
                source="predicted",
                confidence=confidence,
                evidence_level="predicted_explorable",
                reason=reason_zh,
                attributes={
                    "requires_visual_confirmation": True,
                    "based_on": [anchor.node_id],
                    "navigation_use": "exploration_only",
                },
            )
        )
        layer.hypotheses.append(
            PSGHypothesis(
                hypothesis_id=f"hyp_{index:03d}",
                predicted_node_id=node_id,
                predicted_edge_id=edge_id,
                prediction_zh=reason_zh,
                prediction_en=f"{anchor.label} may indicate an explorable {label}.",
                based_on=[anchor.node_id],
                confidence=confidence,
                suggested_observation=action,
                risk_level="low" if action != "approach_anchor_carefully" else "medium",
                can_confirm_target=False,
            )
        )
        layer.next_best_views.append(
            NextBestView(
                view_id=f"view_{index:03d}",
                anchor_node_id=anchor.node_id,
                target_node_id=node_id,
                action=action,
                reason_zh=reason_zh,
                requires_visual_confirmation=True,
                risk_level="low" if action != "approach_anchor_carefully" else "medium",
                priority=confidence,
            )
        )

    def _sanitize(self, layer: PSGLayer, observed_graph: SceneGraph) -> PSGLayer:
        observed_ids = {node.node_id for node in observed_graph.nodes}
        allowed_node_ids: set[str] = set()
        safe_nodes: list[SceneGraphNode] = []
        for node in layer.predicted_nodes:
            node.source = "predicted"
            node.can_confirm_target = False
            node.evidence_level = "predicted_explorable"
            node.confidence = _clamp(node.confidence)
            node.based_on = [item for item in node.based_on if item in observed_ids]
            if not node.based_on:
                layer.warnings.append(f"dropped_predicted_node_without_observed_basis:{node.node_id}")
                continue
            if node.confidence < self.confidence_threshold:
                layer.warnings.append(f"dropped_low_confidence_predicted_node:{node.node_id}")
                continue
            allowed_node_ids.add(node.node_id)
            safe_nodes.append(node)

        safe_edges: list[SceneGraphEdge] = []
        for edge in layer.predicted_edges:
            edge.source = "predicted"
            edge.evidence_level = "predicted_explorable"
            edge.confidence = _clamp(edge.confidence)
            edge.attributes["requires_visual_confirmation"] = True
            edge.attributes["navigation_use"] = "exploration_only"
            if edge.relation in FORBIDDEN_PSG_RELATIONS or edge.relation not in ALLOWED_PSG_RELATIONS:
                layer.warnings.append(f"dropped_forbidden_predicted_relation:{edge.relation}")
                continue
            connected_observed = edge.source_node_id in observed_ids or edge.target_node_id in observed_ids
            connected_predicted = edge.source_node_id in allowed_node_ids or edge.target_node_id in allowed_node_ids
            if not connected_observed or not connected_predicted:
                layer.warnings.append(f"dropped_unanchored_predicted_edge:{edge.edge_id}")
                continue
            safe_edges.append(edge)

        safe_views = []
        for view in layer.next_best_views:
            if view.action in FORBIDDEN_ACTIONS:
                layer.warnings.append(f"rewrote_forbidden_action:{view.action}")
                view.action = "stop_and_reobserve"
            if view.action not in SAFE_ACTIONS:
                view.action = "stop_and_reobserve"
            view.requires_visual_confirmation = True
            safe_views.append(view)

        layer.predicted_nodes = safe_nodes
        layer.predicted_edges = safe_edges
        layer.hypotheses = [
            item
            for item in layer.hypotheses
            if item.predicted_node_id in allowed_node_ids and not item.can_confirm_target
        ]
        layer.next_best_views = [
            item
            for item in safe_views
            if item.target_node_id is None or item.target_node_id in allowed_node_ids
        ]
        return layer


def _action_from_position(position: str) -> str:
    if "left" in position:
        return "turn_left_and_observe"
    if "right" in position:
        return "turn_right_and_observe"
    if "front" in position or "center" in position:
        return "move_forward_short_and_observe"
    return "scan_left_right"


def _clamp(value: Any) -> float:
    try:
        return max(0.0, min(1.0, float(value)))
    except (TypeError, ValueError):
        return 0.0
