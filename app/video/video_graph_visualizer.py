"""Navigation-topology PNG renderer with a place-backbone layout."""

from __future__ import annotations

from pathlib import Path
from typing import Any


def render_topology_png(topology: dict[str, Any], path: str | Path) -> Path | None:
    try:
        import matplotlib.pyplot as plt
        import networkx as nx
    except Exception:
        return None

    nodes = [
        item
        for item in topology.get("nodes", [])
        if item.get("properties", {}).get("display_in_png", True)
    ]
    node_by_id = {str(node["node_id"]): node for node in nodes}
    if not node_by_id:
        return None

    graph = nx.DiGraph()
    for node_id in node_by_id:
        graph.add_node(node_id)
    visible_edges = []
    for edge in topology.get("edges", []):
        source = str(edge.get("from") or edge.get("source_node_id"))
        target = str(edge.get("to") or edge.get("target_node_id"))
        if source in node_by_id and target in node_by_id:
            graph.add_edge(source, target, relation=edge.get("relation"), source=edge.get("source"))
            visible_edges.append(edge)

    positions = _layout_positions(nodes, visible_edges)
    labels = {node_id: _node_label(node) for node_id, node in node_by_id.items()}
    colors = [_node_color(node_by_id[node_id]) for node_id in graph.nodes]
    sizes = [_node_size(node_by_id[node_id]) for node_id in graph.nodes]

    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    width = max(9, 2 + 2.4 * max(1, _place_count(nodes)))
    height = max(6, 1.3 * max(1, len(nodes) ** 0.5) + 4)
    plt.figure(figsize=(width, height))
    ax = plt.gca()
    ax.set_facecolor("#f8fafc")

    observed_edges = [
        (str(edge.get("from") or edge.get("source_node_id")), str(edge.get("to") or edge.get("target_node_id")))
        for edge in visible_edges
        if edge.get("source") != "predicted"
    ]
    predicted_edges = [
        (str(edge.get("from") or edge.get("source_node_id")), str(edge.get("to") or edge.get("target_node_id")))
        for edge in visible_edges
        if edge.get("source") == "predicted"
    ]
    nx.draw_networkx_edges(
        graph,
        positions,
        edgelist=observed_edges,
        arrows=True,
        arrowstyle="->",
        width=1.6,
        edge_color="#475569",
        connectionstyle="arc3,rad=0.04",
    )
    nx.draw_networkx_edges(
        graph,
        positions,
        edgelist=predicted_edges,
        arrows=True,
        arrowstyle="->",
        width=1.4,
        style="dashed",
        edge_color="#d97706",
        connectionstyle="arc3,rad=0.12",
    )
    nx.draw_networkx_nodes(
        graph,
        positions,
        node_color=colors,
        node_size=sizes,
        edgecolors="#0f172a",
        linewidths=1.2,
    )
    nx.draw_networkx_labels(graph, positions, labels=labels, font_size=8)
    edge_labels = {
        (
            str(edge.get("from") or edge.get("source_node_id")),
            str(edge.get("to") or edge.get("target_node_id")),
        ): str(edge.get("relation", ""))
        for edge in visible_edges
        if edge.get("relation") in {"temporal_next", "through", "may_connect_to", "passable_in"}
    }
    nx.draw_networkx_edge_labels(
        graph,
        positions,
        edge_labels=edge_labels,
        font_size=7,
        font_color="#334155",
        label_pos=0.55,
    )
    plt.axis("off")
    plt.tight_layout()
    plt.savefig(output, dpi=170)
    plt.close()
    return output


def _layout_positions(
    nodes: list[dict[str, Any]],
    edges: list[dict[str, Any]],
) -> dict[str, tuple[float, float]]:
    positions: dict[str, tuple[float, float]] = {}
    places = sorted(
        [node for node in nodes if node.get("node_type") == "place"],
        key=lambda item: (item.get("start_time") or 0.0, item["node_id"]),
    )
    place_x: dict[str, float] = {}
    for index, place in enumerate(places):
        x = index * 3.2
        place_x[str(place["node_id"])] = x
        positions[str(place["node_id"])] = (x, 0.0)

    child_counts: dict[tuple[str, str], int] = {}
    for edge in edges:
        source = str(edge.get("from") or edge.get("source_node_id"))
        target = str(edge.get("to") or edge.get("target_node_id"))
        node = next((item for item in nodes if str(item["node_id"]) == target), None)
        if not node or source not in place_x or target in positions:
            continue
        node_type = str(node.get("node_type"))
        offset_index = child_counts.get((source, node_type), 0)
        child_counts[(source, node_type)] = offset_index + 1
        x = place_x[source] + (offset_index - 0.5) * 0.65
        y = _child_y(node_type, offset_index)
        positions[target] = (x, y)

    for edge in edges:
        source = str(edge.get("from") or edge.get("source_node_id"))
        target = str(edge.get("to") or edge.get("target_node_id"))
        node = next((item for item in nodes if str(item["node_id"]) == target), None)
        if not node or target in positions or str(node.get("source")) != "predicted":
            continue
        anchor = positions.get(source, (max(place_x.values(), default=0.0), 0.0))
        count = sum(1 for node_id in positions if str(node_id).startswith("pred_"))
        positions[target] = (anchor[0] + 1.6 + 0.4 * count, anchor[1] + 1.9 + 0.25 * count)

    fallback_x = max(place_x.values(), default=0.0) + 1.5
    for index, node in enumerate(nodes):
        node_id = str(node["node_id"])
        if node_id not in positions:
            positions[node_id] = (fallback_x + 0.8 * (index % 4), -2.4 - 0.45 * (index // 4))
    return positions


def _child_y(node_type: str, offset_index: int) -> float:
    if node_type == "passage":
        return 1.25 + 0.35 * offset_index
    if node_type == "free_space":
        return -1.05 - 0.2 * offset_index
    if node_type == "obstacle":
        return -1.75 - 0.25 * offset_index
    if node_type == "landmark":
        return 0.95 if offset_index % 2 == 0 else -0.55
    return -2.3


def _node_label(node: dict[str, Any]) -> str:
    prefix = "PRED" if node.get("source") == "predicted" else node.get("node_type", "node")
    label = node.get("label") or node.get("node_id")
    return f"{prefix}\n{label}"


def _node_color(node: dict[str, Any]) -> str:
    node_type = node.get("node_type")
    if node.get("source") == "predicted":
        return "#fde68a"
    if node_type == "place":
        return "#dbeafe"
    if node_type == "passage":
        return "#bfdbfe"
    if node_type == "free_space":
        return "#bbf7d0"
    if node_type == "obstacle":
        return "#fecaca"
    if node_type == "landmark":
        return "#e9d5ff"
    return "#e5e7eb"


def _node_size(node: dict[str, Any]) -> int:
    if node.get("node_type") == "place":
        return 2400
    if node.get("source") == "predicted":
        return 1800
    return 1500


def _place_count(nodes: list[dict[str, Any]]) -> int:
    return sum(1 for node in nodes if node.get("node_type") == "place")
