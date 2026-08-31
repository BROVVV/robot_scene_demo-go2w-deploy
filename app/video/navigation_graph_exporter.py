"""Export helpers for navigation topology JSON, GraphML, and debug reports."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from app.video.video_graph_io import topology_to_scene_graph, write_json, write_scene_graph_graphml


def write_navigation_topology_json(topology: dict[str, Any], path: str | Path) -> Path:
    return write_json(topology, path)


def write_navigation_topology_graphml(topology: dict[str, Any], path: str | Path) -> Path:
    return write_scene_graph_graphml(topology_to_scene_graph(topology), path)


def write_navigation_topology_debug(topology: dict[str, Any], path: str | Path) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    nodes = topology.get("nodes", [])
    edges = topology.get("edges", [])
    validation = topology.get("validation", {})
    lines = [
        "# Video Navigation Topology Debug",
        "",
        f"- map_type: `{topology.get('map_type')}`",
        f"- nodes: {len(nodes)}",
        f"- edges: {len(edges)}",
        f"- places: {sum(1 for node in nodes if node.get('node_type') == 'place')}",
        f"- predicted_nodes: {sum(1 for node in nodes if node.get('source') == 'predicted')}",
        f"- root_star_edges: {validation.get('root_star_edges', 0)}",
        "",
        "## Place Backbone",
    ]
    places = sorted(
        [node for node in nodes if node.get("node_type") == "place"],
        key=lambda item: (item.get("start_time") or 0.0, item.get("node_id")),
    )
    for place in places:
        lines.append(
            f"- `{place['node_id']}` {place.get('label_zh') or place.get('label')} "
            f"{place.get('start_time')}s -> {place.get('end_time')}s"
        )
    lines.extend(["", "## Predicted Layer"])
    for node in nodes:
        if node.get("source") == "predicted":
            based_on = node.get("properties", {}).get("based_on", [])
            lines.append(
                f"- `{node['node_id']}` {node.get('label_zh') or node.get('label')} "
                f"based_on={based_on} requires_visual_confirmation={node.get('requires_visual_confirmation')}"
            )
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return output


__all__ = [
    "write_navigation_topology_debug",
    "write_navigation_topology_graphml",
    "write_navigation_topology_json",
]
