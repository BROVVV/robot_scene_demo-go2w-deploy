"""Tests for metric frontier extraction."""

from __future__ import annotations

from app.spatial.frontier_extractor import FrontierExtractor
from app.spatial.models import SpatialMapSnapshot, SpatialPose


def _room_map() -> SpatialMapSnapshot:
    # 20x20 grid, free left half, unknown right half -> boundary at x=9/10.
    free = [(x, y) for x in range(10) for y in range(5, 15)]
    occupied = [(x, y) for x in range(10, 20) for y in range(10, 12)]
    return SpatialMapSnapshot(
        revision=1,
        resolution_m=0.1,
        origin=(0.0, 0.0),
        width=20,
        height=20,
        free=free,
        occupied=occupied,
        unknown=[],
    )


def test_extract_finds_frontier():
    extractor = FrontierExtractor(min_component_size=5)
    robot = SpatialPose(x=0.5, y=1.0, yaw=0.0)
    frontiers = extractor.extract(_room_map(), robot)
    assert len(frontiers) >= 1
    f = frontiers[0]
    assert f.position is not None
    assert f.distance_m is not None
    assert f.bearing_deg is not None
    assert f.spatial_information_gain > 0.0


def test_extract_empty_without_map():
    extractor = FrontierExtractor()
    assert extractor.extract(None) == []


def test_extract_small_component_filtered():
    # Only 3 free cells touching unknown -> below min_component_size 5.
    free = [(0, 0), (1, 0), (2, 0)]
    map_snapshot = SpatialMapSnapshot(
        revision=1, resolution_m=0.1, origin=(0, 0), width=10, height=10,
        free=free, occupied=[], unknown=[],
    )
    frontiers = FrontierExtractor(min_component_size=5).extract(map_snapshot)
    assert frontiers == []
