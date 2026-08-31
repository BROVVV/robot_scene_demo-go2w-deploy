"""JSON and GraphML writers for video full-scene graphs."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import networkx as nx

from app.video.schemas import SceneGraph


def write_json(payload: Any, path: str | Path) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return output


def write_scene_graph_json(graph: SceneGraph, path: str | Path, **extra: Any) -> Path:
    payload = graph.to_dict()
    payload.update(extra)
    return write_json(payload, path)


def write_scene_graph_graphml(graph: SceneGraph, path: str | Path) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    nx_graph = nx.MultiDiGraph()
    for node in graph.nodes:
        payload = node.to_dict()
        node_id = payload.pop("node_id")
        nx_graph.add_node(node_id, **{key: _graphml_value(value) for key, value in payload.items()})
    for index, edge in enumerate(graph.edges):
        payload = edge.to_dict()
        source = payload.pop("source_node_id")
        target = payload.pop("target_node_id")
        payload.pop("edge_id", None)
        nx_graph.add_edge(
            source,
            target,
            key=str(index),
            **{key: _graphml_value(value) for key, value in payload.items()},
        )
    nx.write_graphml(nx_graph, output)
    return output


def topology_to_scene_graph(topology: dict[str, Any]) -> SceneGraph:
    from app.video.schemas import SceneGraphEdge, SceneGraphNode

    nodes = [
        SceneGraphNode(
            node_id=str(item["node_id"]),
            node_type=str(item.get("node_type", "node")),
            label=str(item.get("label", "node")),
            label_zh=str(item.get("label_zh", item.get("label", "节点"))),
            category=str(item.get("properties", {}).get("category", item.get("node_type", "unknown"))),
            source=str(item.get("source", "observed")),
            confidence=float(item.get("confidence", 0.0)),
            evidence_level=str(item.get("properties", {}).get("evidence_level", "observed_candidate")),
            based_on=[str(value) for value in item.get("properties", {}).get("based_on", [])],
            can_confirm_target=bool(item.get("can_confirm_target", False)),
            attributes={
                **dict(item.get("properties", {})),
                "start_time": item.get("start_time"),
                "end_time": item.get("end_time"),
                "evidence_frames": item.get("evidence_frames", []),
                "requires_visual_confirmation": item.get("requires_visual_confirmation", False),
            },
        )
        for item in topology.get("nodes", [])
    ]
    edges = [
        SceneGraphEdge(
            edge_id=str(item.get("edge_id", f"topo_edge_{index:04d}")),
            source_node_id=str(item.get("from", item.get("source_node_id"))),
            target_node_id=str(item.get("to", item.get("target_node_id"))),
            relation=str(item.get("relation", "connected_to")),
            source=str(item.get("source", "observed")),
            confidence=float(item.get("confidence", 0.0)),
            evidence_level=str(item.get("properties", {}).get("evidence_level", "observed_candidate")),
            reason=item.get("properties", {}).get("reason"),
            attributes={
                **dict(item.get("properties", {})),
                "requires_visual_confirmation": item.get("requires_visual_confirmation", False),
            },
        )
        for index, item in enumerate(topology.get("edges", []), start=1)
    ]
    return SceneGraph(nodes=nodes, edges=edges)


def _graphml_value(value: Any) -> str | int | float | bool:
    if value is None:
        return ""
    if isinstance(value, (str, int, float, bool)):
        return value
    return json.dumps(value, ensure_ascii=False, sort_keys=True)
