"""Build a predictive scene graph from scene memories and target evidence."""

from __future__ import annotations

import json
from pathlib import Path
import re
from typing import Any

import networkx as nx

from app.video.video_scene_reasoner import FrameSceneReasoningResult


def build_video_psg(
    target: str,
    frame_results: list[FrameSceneReasoningResult],
    memory_updates: list[dict[str, Any]],
    output_dir: str | Path,
) -> tuple[dict[str, Any], dict[str, Path]]:
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    nodes: list[dict[str, Any]] = [
        {
            "id": "target_query",
            "type": "TargetNode",
            "label": target,
            "found": any(
                item.target_evidence.get("directly_found") for item in frame_results
            ),
        }
    ]
    edges: list[dict[str, Any]] = []
    hypotheses = _collect_hypotheses(frame_results)

    for index, memory in enumerate(memory_updates, start=1):
        place_id = f"place_{index:03d}_{_slug(memory.get('room_type', 'unknown'))}"
        nodes.append(
            {
                "id": place_id,
                "type": "PlaceNode",
                "room_type": memory.get("room_type", "unknown"),
                "time_range": memory.get("time_range", []),
                "summary": memory.get("scene_summary", ""),
            }
        )
        memory_id = str(memory["memory_id"])
        nodes.append(
            {
                "id": memory_id,
                "type": "MemoryNode",
                "importance": memory.get("importance", "low"),
                "summary": memory.get("scene_summary", ""),
            }
        )
        edges.append({"source": memory_id, "target": place_id, "type": "memory_of"})
        target_relation = (
            "target_found_in"
            if memory.get("target_context", {}).get("found")
            else "target_not_found_in"
        )
        edges.append(
            {"source": "target_query", "target": place_id, "type": target_relation}
        )
        for landmark_index, landmark in enumerate(
            memory.get("place_signature", {}).get("stable_landmarks", []), start=1
        ):
            object_id = f"object_{index:03d}_{landmark_index:03d}_{_slug(landmark)}"
            nodes.append(
                {"id": object_id, "type": "ObjectNode", "label": landmark}
            )
            edges.append(
                {"source": object_id, "target": place_id, "type": "observed_in"}
            )
        for region_index, region in enumerate(memory.get("regions", []), start=1):
            region_id = f"region_{index:03d}_{region_index:03d}_{_slug(region.get('name', 'region'))}"
            nodes.append(
                {
                    "id": region_id,
                    "type": "RegionNode",
                    "label": region.get("name"),
                    "status": region.get("status"),
                }
            )
            edges.append(
                {"source": region_id, "target": place_id, "type": "observed_in"}
            )
            if region.get("type") == "traversable_region":
                edges.append(
                    {"source": place_id, "target": region_id, "type": "traversable_to"}
                )

    for index, hypothesis in enumerate(hypotheses, start=1):
        hypothesis_id = hypothesis["hypothesis_id"]
        nodes.append(
            {
                "id": hypothesis_id,
                "type": "HypothesisNode",
                "label": hypothesis["summary"],
                "confidence": hypothesis["confidence"],
            }
        )
        related_place = _related_place(memory_updates, hypothesis.get("frame_id"))
        relation_type = hypothesis["type"]
        edges.append(
            {
                "source": "target_query",
                "target": hypothesis_id,
                "type": relation_type,
            }
        )
        if related_place:
            edges.append(
                {"source": hypothesis_id, "target": related_place, "type": "supports"}
            )
        evidence_id = f"evidence_{index:03d}"
        nodes.append(
            {
                "id": evidence_id,
                "type": "EvidenceNode",
                "label": "；".join(hypothesis.get("supporting_evidence", [])),
            }
        )
        edges.append(
            {"source": evidence_id, "target": hypothesis_id, "type": "supports"}
        )

    payload = {
        "target": target,
        "target_found": nodes[0]["found"],
        "nodes": nodes,
        "edges": edges,
        "hypotheses": hypotheses,
    }
    hypotheses_path = output / "video_hypotheses.json"
    hypotheses_path.write_text(
        json.dumps(
            {
                "target": target,
                "target_found": payload["target_found"],
                "hypotheses": hypotheses,
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    graph = nx.MultiDiGraph()
    for node in nodes:
        graph.add_node(
            node["id"],
            **{
                key: _graphml_value(value)
                for key, value in node.items()
                if key != "id"
            },
        )
    for index, edge in enumerate(edges):
        graph.add_edge(
            edge["source"],
            edge["target"],
            key=str(index),
            **{
                key: _graphml_value(value)
                for key, value in edge.items()
                if key not in {"source", "target"}
            },
        )
    graphml_path = output / "video_predictive_scene_graph.graphml"
    nx.write_graphml(graph, graphml_path)
    json_path = output / "video_predictive_scene_graph.json"
    json_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return payload, {
        "video_predictive_scene_graph": graphml_path,
        "video_predictive_scene_graph_json": json_path,
        "video_hypotheses": hypotheses_path,
    }


def _collect_hypotheses(
    results: list[FrameSceneReasoningResult],
) -> list[dict[str, Any]]:
    hypotheses: list[dict[str, Any]] = []
    seen: set[tuple[str, str]] = set()
    for result in results:
        for item in result.psg_hypotheses:
            relation = str(item.get("relation_type", "supports"))
            summary = str(item.get("hypothesis", "")).strip()
            key = (relation, summary)
            if not summary or key in seen:
                continue
            seen.add(key)
            hypotheses.append(
                {
                    "hypothesis_id": f"hyp_{len(hypotheses) + 1:03d}",
                    "type": relation,
                    "summary": summary,
                    "related_place": result.scene_understanding.get(
                        "room_type", "unknown"
                    ),
                    "frame_id": result.frame_id,
                    "timestamp_sec": result.timestamp_sec,
                    "supporting_evidence": item.get("supporting_evidence", []),
                    "confidence": float(item.get("confidence", 0.5)),
                }
            )
    return hypotheses


def _related_place(
    memories: list[dict[str, Any]], frame_id: int | None
) -> str | None:
    for index, memory in enumerate(memories, start=1):
        if frame_id in memory.get("frame_ids", []):
            return f"place_{index:03d}_{_slug(memory.get('room_type', 'unknown'))}"
    return None


def _slug(value: Any) -> str:
    slug = re.sub(r"[^a-z0-9]+", "_", str(value).lower()).strip("_")
    return slug or "node"


def _graphml_value(value: Any) -> str | int | float | bool:
    if value is None:
        return ""
    if isinstance(value, (str, int, float, bool)):
        return value
    return json.dumps(value, ensure_ascii=False, sort_keys=True)
