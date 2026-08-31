"""Graph-based visual path planner for relative or metric video maps."""

from __future__ import annotations

import heapq
from uuid import uuid4
from typing import Any

from .models import NavigationPlan, NavigationWaypoint, Pose2D, path_length


def plan_visual_path(
    navigation_map: dict[str, Any],
    goal: NavigationWaypoint,
    *,
    mode: str = "visual_preview",
    target_status: str = "target_not_seen",
    navigation_strategy: str = "exploration",
) -> NavigationPlan:
    nodes = navigation_map.get("nodes", [])
    if not nodes:
        raise ValueError("navigation_map has no nodes")
    start_node = nodes[0]
    goal_node = _nearest_node(nodes, goal.pose)
    node_path = _dijkstra(nodes, navigation_map.get("edges", []), start_node["node_id"], goal_node["node_id"])
    poses = [Pose2D.from_dict(node["pose"]) for node in node_path]
    if not poses or poses[-1].distance_to(goal.pose) > 1e-6:
        poses.append(goal.pose)
    start_pose = Pose2D.from_dict(start_node["pose"])
    length = path_length(poses)
    scale_status = start_pose.scale_status
    executable = scale_status == "metric" and goal.pose.frame_id == "map"
    reason = (
        "metric map pose is available"
        if executable
        else (
            "No metric scale: visual preview is relative and not directly executable"
            if scale_status != "metric"
            else "No valid map-frame goal pose or map transform"
        )
    )
    waypoints = [
        NavigationWaypoint(
            waypoint_id="start",
            pose=start_pose,
            source_frame_id=start_node.get("frame_id"),
            semantic_label="Video Frame 0 起点",
            waypoint_type="start",
            confidence=1.0,
        )
    ]
    for index, node in enumerate(node_path[1:-1], start=1):
        waypoints.append(
            NavigationWaypoint(
                waypoint_id=f"waypoint_{index:02d}",
                pose=Pose2D.from_dict(node["pose"]),
                source_frame_id=node.get("frame_id"),
                semantic_label=f"视频轨迹点 {index:02d}",
                waypoint_type="trajectory",
                confidence=0.8,
            )
        )
    waypoints.append(goal)
    return NavigationPlan(
        plan_id=f"visual_nav_{uuid4().hex[:10]}",
        mode=mode,
        planning_frame=start_pose.frame_id,
        scale_status=scale_status,
        start_pose=start_pose,
        goal_pose=goal.pose,
        waypoints=waypoints,
        path=poses,
        path_length=length,
        estimated_time_sec=length / 0.45 if scale_status == "metric" else None,
        navigation_strategy=navigation_strategy,
        target_status=target_status,
        confidence=min(goal.confidence, 0.95),
        executable=executable,
        executable_reason=reason,
        provenance={
            "planner": "video_topology_dijkstra",
            "path_source": "trajectory_projection",
            "scale_verified": scale_status == "metric",
        },
    )


def _nearest_node(nodes: list[dict[str, Any]], pose: Pose2D) -> dict[str, Any]:
    return min(nodes, key=lambda node: Pose2D.from_dict(node.get("pose") or {}).distance_to(pose))


def _dijkstra(
    nodes: list[dict[str, Any]],
    edges: list[dict[str, Any]],
    start_id: str,
    goal_id: str,
) -> list[dict[str, Any]]:
    by_id = {node["node_id"]: node for node in nodes}
    graph: dict[str, list[tuple[float, str]]] = {node_id: [] for node_id in by_id}
    for edge in edges:
        if not edge.get("traversable", True):
            continue
        a, b = edge.get("from"), edge.get("to")
        distance = float(edge.get("distance", 1.0) or 1.0)
        if a in graph and b in graph:
            graph[a].append((distance, b))
            graph[b].append((distance, a))
    queue: list[tuple[float, str]] = [(0.0, start_id)]
    previous: dict[str, str | None] = {start_id: None}
    distances = {start_id: 0.0}
    while queue:
        distance, node_id = heapq.heappop(queue)
        if node_id == goal_id:
            break
        if distance > distances.get(node_id, float("inf")):
            continue
        for step_cost, next_id in graph.get(node_id, []):
            new_distance = distance + step_cost
            if new_distance < distances.get(next_id, float("inf")):
                distances[next_id] = new_distance
                previous[next_id] = node_id
                heapq.heappush(queue, (new_distance, next_id))
    if goal_id not in previous:
        return [by_id[start_id], by_id[goal_id]]
    order = []
    current: str | None = goal_id
    while current is not None:
        order.append(by_id[current])
        current = previous[current]
    order.reverse()
    return order
