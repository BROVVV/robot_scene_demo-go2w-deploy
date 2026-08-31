"""Build and export a semantic memory graph for a sampled video."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import networkx as nx

from app.video.models import FrameAnalysisResult
from app.video.spatial_context import find_nearby_objects


def build_video_memory_graph(
    search_result: dict[str, Any],
    frame_results: list[FrameAnalysisResult],
    output_dir: str | Path,
) -> dict[str, Path]:
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    nodes, edges = _build_graph_payload(search_result, frame_results)
    payload = {"nodes": nodes, "edges": edges}

    json_path = output / "video_memory_graph.json"
    json_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    graph = nx.MultiDiGraph()
    for node in nodes:
        attributes = {key: _graphml_value(value) for key, value in node.items() if key != "id"}
        graph.add_node(node["id"], **attributes)
    for index, edge in enumerate(edges):
        attributes = {
            key: _graphml_value(value)
            for key, value in edge.items()
            if key not in {"source", "target"}
        }
        graph.add_edge(edge["source"], edge["target"], key=str(index), **attributes)

    graphml_path = output / "video_memory_graph.graphml"
    nx.write_graphml(graph, graphml_path)
    return {"video_memory_graph_json": json_path, "video_memory_graph_graphml": graphml_path}


def _build_graph_payload(
    search_result: dict[str, Any],
    frame_results: list[FrameAnalysisResult],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    nodes: list[dict[str, Any]] = []
    edges: list[dict[str, Any]] = []
    video_id = "video_root"
    target_id = "target_query"
    nodes.append(
        {
            "id": video_id,
            "type": "VideoNode",
            **search_result["video_meta"],
        }
    )
    nodes.append(
        {
            "id": target_id,
            "type": "TargetNode",
            "label": search_result["task"]["target"],
            "canonical_label": search_result["task"]["canonical_target"],
            "found": search_result["target_found"],
        }
    )

    for frame in frame_results:
        frame_id = f"frame_{frame.frame_id:06d}"
        nodes.append(
            {
                "id": frame_id,
                "type": "FrameNode",
                "frame_id": frame.frame_id,
                "timestamp_sec": frame.timestamp_sec,
                "image_path": frame.image_path,
                "annotated_frame_path": frame.annotated_frame_path or "",
            }
        )
        edges.append({"source": video_id, "target": frame_id, "type": "video_has_frame"})
        for obj in frame.objects:
            object_id = str(obj["object_id"])
            nodes.append(
                {
                    "id": object_id,
                    "type": "ObjectNode",
                    "label": obj.get("label"),
                    "label_zh": obj.get("label_zh"),
                    "confidence": obj.get("confidence"),
                    "bbox": obj.get("bbox"),
                    "track_id": obj.get("track_id") or "",
                    "is_target_candidate": bool(obj.get("is_target_candidate")),
                }
            )
            edges.append(
                {"source": frame_id, "target": object_id, "type": "frame_observes_object"}
            )
            if obj.get("is_target_candidate"):
                edges.append(
                    {"source": object_id, "target": target_id, "type": "object_matches_target"}
                )
                for nearby in find_nearby_objects(obj, frame.objects):
                    if nearby.get("object_id"):
                        edges.append(
                            {
                                "source": object_id,
                                "target": nearby["object_id"],
                                "type": "object_near_object",
                                "distance_normalized": nearby["distance_normalized"],
                            }
                        )

    for region in search_result.get("candidate_regions", []):
        region_id = region["region_id"]
        nodes.append(
            {
                "id": region_id,
                "type": "RegionNode",
                "timestamp_sec": region["timestamp_sec"],
                "priority": region["priority"],
                "reason": region["reason"],
                "score": region["score"],
            }
        )
        edges.append(
            {
                "source": f"frame_{int(region['frame_id']):06d}",
                "target": region_id,
                "type": "frame_belongs_to_candidate_region",
            }
        )

    best = search_result.get("best_evidence")
    if best:
        direct_observations = [
            item
            for item in search_result.get("timeline", [])
            if item.get("type") == "direct_detection"
        ]
        last_seen = max(
            direct_observations,
            key=lambda item: item["timestamp_sec"],
            default={"frame_id": best["frame_id"]},
        )
        evidence_id = "target_best_evidence"
        nodes.append(
            {
                "id": evidence_id,
                "type": "EvidenceNode",
                "timestamp_sec": best["timestamp_sec"],
                "confidence": best["confidence"],
                "evidence_score": best["evidence_score"],
                "description": best["description"],
            }
        )
        edges.extend(
            [
                {"source": target_id, "target": evidence_id, "type": "target_best_evidence"},
                {
                    "source": evidence_id,
                    "target": f"frame_{int(best['frame_id']):06d}",
                    "type": "evidence_observed_at",
                },
                {
                    "source": target_id,
                    "target": f"frame_{int(last_seen['frame_id']):06d}",
                    "type": "target_last_seen_at",
                },
            ]
        )
    return nodes, edges


def _graphml_value(value: Any) -> str | int | float | bool:
    if value is None:
        return ""
    if isinstance(value, (str, int, float, bool)):
        return value
    return json.dumps(value, ensure_ascii=False, sort_keys=True)
