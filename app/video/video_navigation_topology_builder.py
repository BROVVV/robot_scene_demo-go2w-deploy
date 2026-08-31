"""Build navigation topology from observed and predicted video graphs."""

from __future__ import annotations

from typing import Any

from app.video.schemas import NextBestView, SceneGraph, SceneGraphEdge, SceneGraphNode


class VideoNavigationTopologyBuilder:
    """Create a place-centric semantic topology without metric pose."""

    def __init__(self, observed_only: bool = False) -> None:
        self.observed_only = observed_only

    def build(
        self,
        hybrid_graph: SceneGraph,
        next_best_views: list[NextBestView],
    ) -> dict[str, Any]:
        included_ids = {
            node.node_id
            for node in hybrid_graph.nodes
            if self._include_node(node)
        }
        nodes = [
            self._topology_node(node)
            for node in sorted(hybrid_graph.nodes, key=_node_sort_key)
            if node.node_id in included_ids
        ]
        edges = [
            self._topology_edge(edge)
            for edge in hybrid_graph.edges
            if self._include_edge(edge, included_ids)
        ]
        exploration_candidates = self._exploration_candidates(next_best_views, included_ids)
        return {
            "map_type": "video_navigation_topology_without_metric_pose",
            "schema_version": "1.0",
            "nodes": nodes,
            "edges": edges,
            "exploration_candidates": exploration_candidates,
            "next_best_views": [view.to_dict() for view in next_best_views if not self.observed_only],
            "recommended_navigation_actions": self._recommended_actions(exploration_candidates),
            "validation": _validation_report(nodes, edges),
            "limitations": [
                "No odom, SLAM, depth map, or robot pose trace was provided.",
                "Place order is inferred from frame timestamps rather than metric coordinates.",
                "Predicted nodes are exploration candidates and require visual confirmation.",
            ],
        }

    def build_navigation_map(self, topology: dict[str, Any]) -> dict[str, Any]:
        nodes = topology.get("nodes", [])
        places = [
            {
                "place_id": item["node_id"],
                "scene_type": item["label"],
                "description_zh": item.get("label_zh"),
                "start_time": item.get("start_time"),
                "end_time": item.get("end_time"),
                "source": item["source"],
            }
            for item in nodes
            if item.get("node_type") == "place" and item.get("source") == "observed"
        ]
        objects = [
            {
                "object_id": item["node_id"],
                "type": item["label"],
                "label_zh": item.get("label_zh"),
                "primary_place_id": item.get("properties", {}).get("primary_place_id"),
                "position_2d": item.get("properties", {}).get("stable_position_2d"),
                "navigation_role": item.get("node_type"),
                "source": item["source"],
            }
            for item in nodes
            if item.get("node_type") in {"landmark", "passage", "obstacle", "free_space"}
            and item.get("source") == "observed"
        ]
        return {
            "map_type": topology.get("map_type", "video_navigation_topology_without_metric_pose"),
            "places": places,
            "objects": objects,
            "exploration_candidates": topology.get("exploration_candidates", []),
            "recommended_navigation_actions": topology.get("recommended_navigation_actions", []),
            "limitations": topology.get("limitations", []),
        }

    def _include_node(self, node: SceneGraphNode) -> bool:
        if self.observed_only and node.source != "observed":
            return False
        if node.source == "predicted":
            return True
        return _nav_node_type(node) in {"place", "passage", "free_space", "obstacle", "landmark", "object"}

    def _include_edge(self, edge: SceneGraphEdge, included_ids: set[str]) -> bool:
        if self.observed_only and edge.source != "observed":
            return False
        if edge.source_node_id not in included_ids or edge.target_node_id not in included_ids:
            return False
        if edge.source == "predicted":
            return edge.relation in {
                "may_connect_to",
                "likely_connects_to",
                "likely_room_beyond",
                "likely_corridor_continuation",
                "explore_candidate",
            }
        return edge.relation in {
            "temporal_next",
            "connected_to",
            "contains",
            "observed_in",
            "adjacent_to",
            "near",
            "left_of",
            "right_of",
            "in_front_of",
            "behind",
            "blocks",
            "passable_in",
            "through",
        }

    def _topology_node(self, node: SceneGraphNode) -> dict[str, Any]:
        node_type = _nav_node_type(node)
        properties = dict(node.attributes)
        properties["category"] = node.category
        properties["evidence_level"] = node.evidence_level
        properties["based_on"] = list(node.based_on)
        properties["can_confirm_target"] = bool(node.can_confirm_target and node.source == "observed")
        properties["display_in_png"] = _display_in_png(node, node_type)
        if node.source == "predicted":
            properties["requires_visual_confirmation"] = True
            properties["can_confirm_target"] = False
        return {
            "node_id": node.node_id,
            "node_type": node_type,
            "label": node.label,
            "label_zh": node.label_zh,
            "source": node.source,
            "confidence": node.confidence,
            "start_time": node.attributes.get("start_time") or node.attributes.get("first_seen_sec"),
            "end_time": node.attributes.get("end_time") or node.attributes.get("last_seen_sec"),
            "evidence_frames": _evidence_frames(node.based_on),
            "requires_visual_confirmation": node.source == "predicted",
            "can_confirm_target": bool(node.can_confirm_target and node.source == "observed"),
            "properties": properties,
        }

    def _topology_edge(self, edge: SceneGraphEdge) -> dict[str, Any]:
        relation = _nav_relation(edge)
        properties = dict(edge.attributes)
        properties["evidence_level"] = edge.evidence_level
        if edge.reason:
            properties["reason"] = edge.reason
        if edge.source == "predicted":
            properties["requires_visual_confirmation"] = True
            properties.setdefault("navigation_use", "exploration_only")
        return {
            "edge_id": edge.edge_id,
            "from": edge.source_node_id,
            "to": edge.target_node_id,
            "source_node_id": edge.source_node_id,
            "target_node_id": edge.target_node_id,
            "relation": relation,
            "source": edge.source,
            "confidence": edge.confidence,
            "requires_visual_confirmation": edge.source == "predicted",
            "properties": properties,
        }

    def _exploration_candidates(
        self,
        next_best_views: list[NextBestView],
        included_ids: set[str],
    ) -> list[dict[str, Any]]:
        if self.observed_only:
            return []
        candidates = []
        for view in next_best_views:
            if view.target_node_id and view.target_node_id not in included_ids:
                continue
            candidates.append(
                {
                    "candidate_id": view.target_node_id,
                    "anchor": view.anchor_node_id,
                    "action": view.action,
                    "reason_zh": view.reason_zh,
                    "requires_visual_confirmation": True,
                    "risk_level": view.risk_level,
                    "priority": view.priority,
                    "source": "predicted",
                    "navigation_use": "exploration_only",
                }
            )
        return candidates

    def _recommended_actions(self, exploration_candidates: list[dict[str, Any]]) -> list[dict[str, Any]]:
        actions: list[dict[str, Any]] = [
            {
                "action": "stop_and_reobserve",
                "reason_zh": "当前没有 odom/SLAM/深度图，仅输出语义拓扑；执行前应停下并重新视觉确认。",
                "requires_visual_confirmation": True,
            }
        ]
        actions.extend(exploration_candidates[:5])
        return actions


def _nav_node_type(node: SceneGraphNode) -> str:
    if node.source == "predicted":
        label = f"{node.label} {node.label_zh}".lower()
        if "door" in label or "passage" in label or "门" in label or "通道" in label:
            return "predicted_passage"
        return "predicted_region"
    if node.node_type == "place":
        return "place"
    role = str(node.attributes.get("navigation_role", node.node_type))
    if node.node_type in {"passage", "free_space", "obstacle", "landmark"}:
        return node.node_type
    if role in {"passage", "free_space", "obstacle", "landmark"}:
        return role
    return "object"


def _nav_relation(edge: SceneGraphEdge) -> str:
    if edge.source == "predicted":
        if edge.relation in {
            "may_connect_to",
            "likely_connects_to",
            "likely_room_beyond",
            "likely_corridor_continuation",
        }:
            return "may_connect_to"
        return "explore_candidate"
    if edge.relation == "located_in":
        return "contains"
    if edge.relation == "passable_through":
        return "passable_in"
    return edge.relation


def _display_in_png(node: SceneGraphNode, node_type: str) -> bool:
    if node_type in {"place", "passage", "free_space", "obstacle", "predicted_region", "predicted_passage"}:
        return True
    if node_type == "landmark":
        return node.confidence >= 0.45
    return False


def _evidence_frames(based_on: list[str]) -> list[int]:
    frames: list[int] = []
    for item in based_on:
        if item.startswith("frame_"):
            try:
                frames.append(int(item.removeprefix("frame_")))
            except ValueError:
                continue
    return frames


def _node_sort_key(node: SceneGraphNode) -> tuple[int, float, str]:
    order = {
        "place": 0,
        "passage": 1,
        "free_space": 2,
        "obstacle": 3,
        "landmark": 4,
        "object": 5,
        "predicted_region": 6,
        "predicted_passage": 6,
    }
    node_type = _nav_node_type(node)
    start = node.attributes.get("start_time") or node.attributes.get("first_seen_sec") or 0.0
    return (order.get(node_type, 9), float(start), node.node_id)


def _validation_report(nodes: list[dict[str, Any]], edges: list[dict[str, Any]]) -> dict[str, Any]:
    node_ids = {node["node_id"] for node in nodes}
    root_like = [node_id for node_id in node_ids if "root" in str(node_id).lower() or node_id == "video"]
    star_edges = [
        edge
        for edge in edges
        if edge.get("from") in root_like or edge.get("to") in root_like
    ]
    return {
        "has_place_backbone": any(node.get("node_type") == "place" for node in nodes),
        "place_count": sum(1 for node in nodes if node.get("node_type") == "place"),
        "temporal_next_edges": sum(1 for edge in edges if edge.get("relation") == "temporal_next"),
        "root_like_nodes": root_like,
        "root_star_edges": len(star_edges),
        "predicted_nodes_require_confirmation": all(
            bool(node.get("requires_visual_confirmation"))
            for node in nodes
            if str(node.get("source")) == "predicted"
        ),
    }
