"""Tests for PlaceGraph: rotations stay in one place, relocations create new
places."""

from __future__ import annotations

from app.spatial.models import SpatialPose
from app.spatial.place_graph import PlaceGraph


def test_in_place_rotation_keeps_one_place():
    graph = PlaceGraph()
    pose = SpatialPose(x=0.0, y=0.0, yaw=0.0)
    for i in range(4):
        place_id, created = graph.register_observation(
            observation_id=f"obs_{i}",
            heading_sector=i,
            objects=["door"],
            pose=pose,
        )
        assert place_id == "P1"
        assert created is (i == 0)
    assert len(graph.places) == 1
    assert graph.places["P1"].heading_coverage == {"0": 1, "1": 1, "2": 1, "3": 1}
    assert len(graph.edges) == 0


def test_translation_creates_new_place():
    graph = PlaceGraph()
    graph.register_observation(
        observation_id="obs_0", heading_sector=0, objects=[], pose=SpatialPose(x=0, y=0)
    )
    # 0.5m is beyond the default 0.35m merge radius -> a new Place.
    place_id, created = graph.register_observation(
        observation_id="obs_1", heading_sector=0, objects=[],
        pose=SpatialPose(x=0.5, y=0.0),
    )
    assert created is True
    assert place_id == "P2"
    assert len(graph.places) == 2
    assert len(graph.edges) == 1
    assert graph.edges[0].from_place == "P1"
    assert graph.edges[0].to_place == "P2"


def test_relative_displacement_creates_new_place_without_pose():
    graph = PlaceGraph()
    graph.register_observation(observation_id="obs_0", heading_sector=0, objects=[])
    place_id, created = graph.register_observation(
        observation_id="obs_1", heading_sector=0, objects=[],
        observed_displacement_m=0.20,
    )
    assert created is True
    assert place_id == "P2"


def test_small_displacement_stays_same_place():
    graph = PlaceGraph()
    graph.register_observation(
        observation_id="obs_0", heading_sector=0, objects=[], pose=SpatialPose(x=0, y=0)
    )
    place_id, created = graph.register_observation(
        observation_id="obs_1", heading_sector=0, objects=[],
        pose=SpatialPose(x=0.05, y=0.0),
    )
    assert created is False
    assert place_id == "P1"


def test_revisit_reuses_existing_place_and_adds_edge():
    """Plan §6: returning to an old Place reuses it instead of creating a
    duplicate, and a movement edge is added on the return trip."""
    graph = PlaceGraph()
    graph.register_observation(
        observation_id="obs_0", heading_sector=0, objects=[], pose=SpatialPose(x=0, y=0)
    )
    # Move far away -> P2
    graph.register_observation(
        observation_id="obs_1", heading_sector=0, objects=[],
        pose=SpatialPose(x=1.5, y=0.0),
    )
    assert len(graph.places) == 2
    # Return to P1's location -> reuse P1, not create P3
    place_id, created = graph.register_observation(
        observation_id="obs_2", heading_sector=0, objects=[],
        pose=SpatialPose(x=0.05, y=0.0),
    )
    assert place_id == "P1"
    assert created is False
    assert len(graph.places) == 2
    # P2 -> P1 creates a movement (revisit) edge
    assert any(
        e.from_place == "P2" and e.to_place == "P1"
        for e in graph.edges
    )
    assert graph.places["P1"].revisited is True


def test_place_pose_fusion_running_mean():
    """Plan §6.3: Place pose is fused (running mean) not kept at first pose."""
    graph = PlaceGraph(merge_distance_m=0.35)
    graph.register_observation(
        observation_id="obs_0", heading_sector=0, objects=[],
        pose=SpatialPose(x=0.0, y=0.0, yaw=0.0),
    )
    graph.register_observation(
        observation_id="obs_1", heading_sector=0, objects=[],
        pose=SpatialPose(x=0.1, y=0.0, yaw=10.0),
    )
    place = graph.places["P1"]
    assert place.pose_observation_count == 2
    assert place.pose_mean is not None
    assert abs(place.pose_mean.yaw - 5.0) < 1.0
    assert abs(place.pose_mean.x - 0.05) < 0.02
