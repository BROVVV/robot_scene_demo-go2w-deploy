"""Build a lightweight video navigation map and topology from frame poses."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .models import VideoFramePose


def build_video_navigation_map(
    trajectory: list[VideoFramePose],
    observations: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    observations_by_frame = _observations_by_frame(observations or [])
    nodes = []
    edges = []
    for index, item in enumerate(trajectory):
        node_id = f"place_{index:03d}"
        objects = observations_by_frame.get(item.frame_id, [])
        nodes.append(
            {
                "node_id": node_id,
                "frame_id": item.frame_id,
                "timestamp": item.timestamp_sec,
                "pose": item.pose.to_dict(),
                "scale_status": item.pose.scale_status,
                "place_type": _place_type(objects),
                "objects": objects,
                "traversable": True,
                "properties": {
                    "navigation_role": "start" if index == 0 else "place",
                    "source": "video_trajectory_projection",
                },
            }
        )
        if index > 0:
            previous = nodes[index - 1]
            distance = item.pose.distance_to(trajectory[index - 1].pose)
            edges.append(
                {
                    "from": previous["node_id"],
                    "to": node_id,
                    "distance": distance,
                    "direction": _direction(trajectory[index - 1], item),
                    "traversable": True,
                }
            )
    scale_status = trajectory[0].pose.scale_status if trajectory else "unknown"
    return {
        "metadata": {
            "version": "video_navigation_map_v1",
            "frame_id": "video_map",
            "scale_status": scale_status,
            "map_type": (
                "metric_video_navigation_map"
                if scale_status == "metric"
                else "relative_video_navigation_map"
            ),
        },
        "nodes": nodes,
        "edges": edges,
    }


def build_video_navigation_topology(navigation_map: dict[str, Any]) -> dict[str, Any]:
    return {
        "metadata": {
            **dict(navigation_map.get("metadata") or {}),
            "version": "video_navigation_topology_v1",
        },
        "nodes": navigation_map.get("nodes", []),
        "edges": navigation_map.get("edges", []),
    }


def write_navigation_map_outputs(
    navigation_map: dict[str, Any],
    output_dir: str | Path,
) -> dict[str, Path]:
    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    topology = build_video_navigation_topology(navigation_map)
    paths = {
        "navigation_map": _write_json(navigation_map, directory / "navigation_map.json"),
        "navigation_topology": _write_json(topology, directory / "navigation_topology.json"),
        "navigation_topology_graphml": write_topology_graphml(
            topology,
            directory / "navigation_topology.graphml",
        ),
    }
    return paths


def write_topology_graphml(topology: dict[str, Any], path: str | Path) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<graphml xmlns="http://graphml.graphdrawing.org/xmlns">',
        '<graph edgedefault="undirected">',
    ]
    for node in topology.get("nodes", []):
        lines.append(f'<node id="{node.get("node_id", "")}"/>')
    for index, edge in enumerate(topology.get("edges", [])):
        lines.append(
            f'<edge id="e{index}" source="{edge.get("from", "")}" target="{edge.get("to", "")}"/>'
        )
    lines.extend(["</graph>", "</graphml>"])
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return output


def _write_json(payload: object, path: Path) -> Path:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return path


def _observations_by_frame(observations: list[dict[str, Any]]) -> dict[int, list[str]]:
    values: dict[int, list[str]] = {}
    for item in observations:
        frame_id = item.get("frame_id")
        if frame_id is None:
            continue
        labels = item.get("objects") or item.get("labels") or []
        values[int(frame_id)] = [str(label) for label in labels if str(label).strip()]
    return values


def _place_type(objects: list[str]) -> str:
    joined = " ".join(objects).lower()
    if any(term in joined for term in ("door", "doorway", "entrance", "门")):
        return "transition"
    if any(term in joined for term in ("corridor", "hallway", "走廊")):
        return "corridor"
    return "observed_place"


def _direction(previous: VideoFramePose, current: VideoFramePose) -> str:
    dyaw = current.pose.yaw - previous.pose.yaw
    if dyaw > 0.35:
        return "left"
    if dyaw < -0.35:
        return "right"
    return "forward"
