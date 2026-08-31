"""Build and export topology graphs from scene analysis results."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import networkx as nx

from app.schemas import SceneAnalysisResult


plt.rcParams["font.sans-serif"] = [
    "Noto Sans CJK SC",
    "Noto Sans CJK JP",
    "Droid Sans Fallback",
    "DejaVu Sans",
]
plt.rcParams["axes.unicode_minus"] = False


def build_topology_graph(result: SceneAnalysisResult) -> nx.DiGraph:
    graph = nx.DiGraph()

    for obj in result.objects:
        graph.add_node(
            obj.id,
            label=obj.name_zh,
            name=obj.name,
            category=obj.category,
            color=obj.color or "",
            confidence=obj.confidence,
        )

    for relation in result.relations:
        graph.add_edge(
            relation.source_id,
            relation.target_id,
            relation_type=relation.relation_type,
            label=relation.description_zh or relation.relation_type,
            estimated_distance_m=(
                relation.estimated_distance_m
                if relation.estimated_distance_m is not None
                else ""
            ),
            confidence=relation.confidence,
        )

    return graph


def export_topology_graph(
    result: SceneAnalysisResult,
    output_dir: str | Path,
    png_filename: str = "topology_graph.png",
    graphml_filename: str = "topology_graph.graphml",
) -> tuple[Path, Path]:
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    graph = build_topology_graph(result)
    png_path = output_path / png_filename
    graphml_path = output_path / graphml_filename

    nx.write_graphml(graph, graphml_path)
    _draw_graph(graph, png_path)

    return png_path, graphml_path


def _draw_graph(graph: nx.DiGraph, output_path: Path) -> None:
    plt.figure(figsize=(10, 7))

    if graph.number_of_nodes() == 0:
        plt.text(0.5, 0.5, "No objects detected", ha="center", va="center")
        plt.axis("off")
        plt.tight_layout()
        plt.savefig(output_path, dpi=160)
        plt.close()
        return

    pos = nx.spring_layout(graph, seed=42)
    node_labels = {
        node_id: data.get("label", node_id)
        for node_id, data in graph.nodes(data=True)
    }
    edge_labels = {
        (source, target): data.get("label") or data.get("relation_type", "")
        for source, target, data in graph.edges(data=True)
    }

    nx.draw_networkx_nodes(
        graph,
        pos,
        node_color="#d9edf7",
        edgecolors="#333333",
        node_size=2200,
    )
    nx.draw_networkx_edges(
        graph,
        pos,
        arrows=True,
        arrowstyle="-|>",
        arrowsize=18,
        edge_color="#555555",
    )
    nx.draw_networkx_labels(graph, pos, labels=node_labels, font_size=10)
    nx.draw_networkx_edge_labels(graph, pos, edge_labels=edge_labels, font_size=8)

    plt.axis("off")
    plt.tight_layout()
    plt.savefig(output_path, dpi=160)
    plt.close()
