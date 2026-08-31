"""Fast local graph planner for the live semantic navigation graph."""

from __future__ import annotations

import heapq
import math
from typing import Any
from uuid import uuid4

from app.navigation.models import NavigationPlan, NavigationWaypoint, Pose2D, path_length


def plan_live_graph_path(
    semantic_map: dict[str, Any],
    *,
    current_place_id: str | None = None,
    goal: dict[str, Any] | Any,
    robot_pose: dict[str, Any] | None = None,
    target_status: str = "target_not_seen",
) -> NavigationPlan:
    nodes = list(semantic_map.get("nodes") or [])
    edges = list(semantic_map.get("edges") or [])
    if not nodes:
        raise ValueError("semantic map has no nodes")
    by_id = {str(node.get("node_id")): node for node in nodes if node.get("node_id")}
    start_id = current_place_id or semantic_map.get("current_place_id")
    if start_id not in by_id:
        start_id = _nearest_node_id(by_id, robot_pose or {})
    goal_id = _goal_id(goal, by_id)
    if goal_id not in by_id:
        raise ValueError("live graph goal is not in semantic map")
    path_ids = graph_shortest_path(by_id, edges, str(start_id), str(goal_id))
    poses = [_pose_from_node(by_id[node_id]) for node_id in path_ids]
    if robot_pose:
        start_pose = _pose_from_dict(robot_pose)
        if poses and start_pose.distance_to(poses[0]) > 1e-6:
            poses.insert(0, start_pose)
    else:
        start_pose = poses[0]
    goal_pose = _pose_from_node(by_id[goal_id])
    waypoints = [
        NavigationWaypoint(
            waypoint_id=node_id,
            pose=_pose_from_node(by_id[node_id]),
            semantic_label=str(by_id[node_id].get("label") or node_id),
            waypoint_type=str(by_id[node_id].get("node_type") or "place").lower(),
            confidence=float(by_id[node_id].get("confidence", 0.8) or 0.8),
            provenance={"source": "live_semantic_navigation_graph"},
        )
        for node_id in path_ids
    ]
    length = path_length(poses)
    return NavigationPlan(
        plan_id=f"live_plan_{uuid4().hex[:10]}",
        mode="live_graph",
        planning_frame=start_pose.frame_id,
        scale_status="metric" if _metric_pose(start_pose, goal_pose) else "relative",
        start_pose=start_pose,
        goal_pose=goal_pose,
        waypoints=waypoints,
        path=poses,
        path_length=length,
        estimated_time_sec=length / 0.45 if length is not None else None,
        navigation_strategy="live_graph_dijkstra",
        target_status=target_status,
        confidence=float(getattr(goal, "confidence", 0.8) or (goal.get("confidence", 0.8) if isinstance(goal, dict) else 0.8)),
        executable=True,
        executable_reason="current Place to reachable live graph goal",
        provenance={
            "planner": "live_graph_dijkstra",
            "start_node": start_id,
            "goal_node": goal_id,
            "source": "semantic_navigation_graph",
        },
    )


def graph_shortest_path(
    nodes: dict[str, dict[str, Any]],
    edges: list[dict[str, Any]],
    start_id: str,
    goal_id: str,
) -> list[str]:
    """Dijkstra over live graph edges, skipping failed/unreachable edges."""
    graph: dict[str, list[tuple[float, str]]] = {node_id: [] for node_id in nodes}
    for edge in edges:
        if edge.get("traversable", True) is False:
            continue
        if str(edge.get("relation", "")).upper() in {"OBSERVED_FROM", "NEAR", "LEFT_OF", "RIGHT_OF"}:
            continue
        left, right = str(edge.get("from", "")), str(edge.get("to", ""))
        if left not in graph or right not in graph:
            continue
        cost = float(edge.get("distance", 1.0) or 1.0)
        cost += float(edge.get("failure_count", 0.0) or 0.0) * 2.0
        graph[left].append((cost, right))
        if edge.get("directed") is not True:
            graph[right].append((cost, left))
    queue: list[tuple[float, str]] = [(0.0, start_id)]
    distance = {start_id: 0.0}
    previous: dict[str, str | None] = {start_id: None}
    while queue:
        current, node_id = heapq.heappop(queue)
        if node_id == goal_id:
            break
        if current > distance.get(node_id, float("inf")):
            continue
        for cost, next_id in graph.get(node_id, []):
            candidate = current + cost
            if candidate < distance.get(next_id, float("inf")):
                distance[next_id] = candidate
                previous[next_id] = node_id
                heapq.heappush(queue, (candidate, next_id))
    if goal_id not in previous:
        raise ValueError(f"no reachable live graph path: {start_id} -> {goal_id}")
    result: list[str] = []
    current: str | None = goal_id
    while current is not None:
        result.append(current)
        current = previous[current]
    return list(reversed(result))


def _goal_id(goal: dict[str, Any] | Any, nodes: dict[str, dict[str, Any]]) -> str:
    if isinstance(goal, dict):
        for key in ("goal_id", "target_frontier_id", "target_place_id", "target_object_id", "node_id"):
            if goal.get(key):
                return str(goal[key])
    else:
        for key in ("target_frontier_id", "target_place_id", "target_object_id", "target_node_id", "goal_id"):
            value = getattr(goal, key, None)
            if value:
                return str(value)
    raise ValueError("goal has no live graph node id")


def _pose_from_node(node: dict[str, Any]) -> Pose2D:
    pose = node.get("pose") or {}
    return _pose_from_dict(pose)


def _pose_from_dict(pose: dict[str, Any]) -> Pose2D:
    yaw_value = pose.get("yaw_rad")
    if yaw_value is None:
        yaw_value = pose.get("yaw")
    if yaw_value is None and pose.get("yaw_deg") is not None:
        yaw_value = math.radians(float(pose["yaw_deg"]))
    return Pose2D(
        x=float(pose.get("x", 0.0)),
        y=float(pose.get("y", 0.0)),
        yaw=float(yaw_value or 0.0),
        frame_id=str(pose.get("frame_id", "odom")),
        source=str(pose.get("source", "live_odom")),
        scale_status="metric" if str(pose.get("quality", "")).lower() in {"metric", "metric_rgbd"} else "relative",
    )


def _nearest_node_id(nodes: dict[str, dict[str, Any]], pose: dict[str, Any]) -> str:
    current = _pose_from_dict(pose)
    return min(nodes, key=lambda node_id: current.distance_to(_pose_from_node(nodes[node_id])))


def _metric_pose(start: Pose2D, goal: Pose2D) -> bool:
    return start.scale_status == "metric" and goal.scale_status == "metric"
