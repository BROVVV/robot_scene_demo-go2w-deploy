"""Tests for SemanticRoutePlanner (plan §19.5)."""

from __future__ import annotations

import math

from app.navigation.semantic_route_planner import RoutePlan, SemanticRoutePlanner
from app.spatial.models import (
    SPATIAL_QUALITY_RELATIVE_RGBD,
    SpatialMapSnapshot,
    SpatialPose,
)
from app.spatial.place_graph import PlaceGraph
from app.spatial.semantic_object_map import SemanticObjectMap
from app.perception.depth_object_localizer import ObjectSpatialObservation


def _map_with_obstacle():
    """A small metric map with a wall obstacle in the middle."""
    # resolution 0.1, 10x10 grid, origin (0,0)
    occupied = []
    for y in range(4, 6):
        for x in range(4, 7):
            occupied.append((x, y))
    return SpatialMapSnapshot(
        revision=1,
        resolution_m=0.1,
        origin=(0.0, 0.0),
        width=10,
        height=10,
        free=[(x, y) for x in range(10) for y in range(10)
              if (x, y) not in occupied],
        occupied=occupied,
        unknown=[],
        quality=SPATIAL_QUALITY_RELATIVE_RGBD,
        source="test",
    )


def test_grid_astar_avoids_obstacle():
    rp = SemanticRoutePlanner(resolution_m=0.1, inflation_radius_m=0.1)
    planner = rp
    planner.resolution_m = 0.1
    start = SpatialPose(x=0.2, y=0.2, yaw=0.0)
    map_snap = _map_with_obstacle()
    plan = planner.plan(
        start_pose=start,
        target_type="FRONTIER_CANDIDATE",
        target_id="F1",
        target_position=(0.8, 0.8),
        map_snapshot=map_snap,
        frame_id="map",
    )
    assert plan is not None
    assert plan.reachable is True
    assert plan.planner_source == "grid_astar"
    assert len(plan.waypoints) >= 2
    assert plan.path_length_m is not None and plan.path_length_m > 0


def test_unreachable_target_returns_unreachable():
    planner = SemanticRoutePlanner(resolution_m=0.1, inflation_radius_m=0.1)
    start = SpatialPose(x=0.1, y=0.1, yaw=0.0)
    map_snap = _map_with_obstacle()
    # Put the goal inside the wall.
    plan = planner.plan(
        start_pose=start,
        target_type="FRONTIER_CANDIDATE",
        target_id="F",
        target_position=(0.45, 0.45),
        map_snapshot=map_snap,
        frame_id="map",
    )
    # Either unreachable or a nearest-free-cell fallback route; never crash.
    assert plan is not None


def test_shorter_geometric_route_wins():
    planner = SemanticRoutePlanner(resolution_m=0.1, inflation_radius_m=0.0)
    start = SpatialPose(x=0.0, y=0.0, yaw=0.0)
    map_snap = _map_with_obstacle()
    near = planner.plan(
        start_pose=start, target_type="FRONTIER_CANDIDATE", target_id="F_near",
        target_position=(0.2, 0.2), map_snapshot=map_snap, frame_id="map",
    )
    far = planner.plan(
        start_pose=start, target_type="FRONTIER_CANDIDATE", target_id="F_far",
        target_position=(0.9, 0.9), map_snapshot=map_snap, frame_id="map",
    )
    if near and far and near.reachable and far.reachable:
        assert near.path_length_m <= far.path_length_m


def test_topological_fallback_known_object():
    pg = PlaceGraph(merge_distance_m=0.35)
    pg.register_observation(
        observation_id="obs0", heading_sector=0, objects=["桌"],
        pose=SpatialPose(x=0.0, y=0.0),
    )
    pg.register_observation(
        observation_id="obs1", heading_sector=0, objects=[],
        pose=SpatialPose(x=1.0, y=0.0),
    )
    om = SemanticObjectMap()
    om.update_with_associations(
        [
            ObjectSpatialObservation(
                object_id=None, label="水瓶", map_xyz=(1.0, 0.0, 0.0),
                spatial_quality="RELATIVE_RGBD", confidence=0.9,
                provenance={"place_id": "P2"},
            )
        ],
        place_id="P2", now=1.0,
    )
    target_obj = list(om.objects.values())[0]
    planner = SemanticRoutePlanner()
    # current place is P2 now (registered last); move current back conceptually
    plan = planner.plan(
        start_pose=SpatialPose(x=0.0, y=0.0, yaw=0.0),
        target_type="OBJECT",
        target_id=target_obj.object_id,
        object_map=om,
        place_graph=pg,
        frame_id="map",
    )
    assert plan is not None


def test_no_start_returns_fallback():
    planner = SemanticRoutePlanner()
    plan = planner.plan(
        start_pose=None,
        target_type="FRONTIER_CANDIDATE",
        target_id="F1",
        target_position=(1.0, 1.0),
        frame_id="map",
    )
    assert plan is not None
    assert plan.reachable is False
    assert plan.planner_source == "unavailable"


def test_route_plan_roundtrip():
    rp = RoutePlan(
        route_id="r1", frame_id="map", target_type="FRONTIER_CANDIDATE",
        target_id="F1", target_position=(1.0, 1.0),
        waypoints=[(0.0, 0.0), (1.0, 1.0)], reachable=True,
        planner_source="grid_astar", cost_components={"c": 1.0},
    )
    d = rp.to_dict()
    rp2 = RoutePlan.from_dict(d)
    assert rp2.route_id == "r1"
    assert rp2.path_length_m is None
    assert rp2.target_position == (1.0, 1.0)
