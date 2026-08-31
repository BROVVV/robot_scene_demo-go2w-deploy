# Copyright 2026 robot_scene_demo maintainers

"""Unit tests for PlainSlamSpatialProvider (plan §11 / §18.4).

Validates the OccupancyGrid/Odometry conversion, the METRIC_LIDAR quality,
the plain_slam provenance chain, frontier extraction from the ray-traced map,
and the fallback path when the mapping pipeline goes stale.
"""

from __future__ import annotations

import math
import time
from types import SimpleNamespace

import pytest

from app.spatial.models import SPATIAL_QUALITY_METRIC_LIDAR
from app.spatial.plain_slam_spatial_provider import (
    SOURCE_MAP,
    PlainSlamSpatialProvider,
)


def make_map_msg(
    *,
    width: int = 20,
    height: int = 20,
    resolution: float = 0.1,
    origin_x: float = -1.0,
    origin_y: float = -1.0,
    free_cells: list[tuple[int, int]] | None = None,
    occupied_cells: list[tuple[int, int]] | None = None,
    frame_id: str = "pslam_odom",
) -> SimpleNamespace:
    data = [-1] * (width * height)  # unknown by default
    for x, y in free_cells or []:
        data[y * width + x] = 0
    for x, y in occupied_cells or []:
        data[y * width + x] = 100
    info = SimpleNamespace(
        width=width,
        height=height,
        resolution=resolution,
        origin=SimpleNamespace(position=SimpleNamespace(x=origin_x, y=origin_y)),
    )
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id),
        info=info,
        data=data,
    )


def make_odom_msg(
    x: float = 0.5,
    y: float = 1.0,
    yaw: float = 0.0,
    frame_id: str = "pslam_odom",
    child_frame_id: str = "base_link_mapping_assist",
) -> SimpleNamespace:
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id, stamp=SimpleNamespace(nanosec=123)),
        child_frame_id=child_frame_id,
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y),
                orientation=SimpleNamespace(
                    w=math.cos(yaw / 2.0),
                    x=0.0,
                    y=0.0,
                    z=math.sin(yaw / 2.0),
                ),
            )
        ),
    )


def make_provider() -> PlainSlamSpatialProvider:
    return PlainSlamSpatialProvider(enable_ros=False)


def corridor_map() -> tuple[list[tuple[int, int]], list[tuple[int, int]]]:
    """2 m x 2 m map: a 1 m wide corridor opening, wall at grid x=10."""
    free = [(x, y) for x in range(1, 10) for y in range(8, 12)]
    occupied = [(10, y) for y in range(8, 12)]
    return free, occupied


def test_map_conversion_and_provenance() -> None:
    provider = make_provider()
    free, occupied = corridor_map()
    provider._on_occupancy_grid(make_map_msg(free_cells=free, occupied_cells=occupied))
    snapshot = provider.get_map()
    assert snapshot is not None
    assert snapshot.source == SOURCE_MAP
    assert snapshot.quality == SPATIAL_QUALITY_METRIC_LIDAR
    assert snapshot.resolution_m == 0.1
    assert snapshot.origin == (-1.0, -1.0)
    assert len(snapshot.free) == len(free)
    assert len(snapshot.occupied) == len(occupied)
    assert snapshot.unknown  # all remaining cells stay unknown
    assert snapshot.provenance["frame_id"] == "pslam_odom"
    assert snapshot.provenance["mapping_mode"] == "mapping_assist"
    assert snapshot.provenance["pandar_extrinsic_status"] == "candidate_unconfirmed"


def test_pose_conversion_and_quality() -> None:
    provider = make_provider()
    provider._on_occupancy_grid(make_map_msg())
    provider._on_odometry(make_odom_msg(x=0.5, y=1.0, yaw=math.pi / 2.0))
    pose = provider.get_pose()
    assert pose is not None
    assert pose.frame_id == "pslam_odom"
    assert pose.source == "plain_slam_pandarxt16_odom"
    assert pose.quality == SPATIAL_QUALITY_METRIC_LIDAR
    assert abs(pose.x - 0.5) < 1e-9
    assert abs(pose.yaw - math.pi / 2.0) < 1e-9
    assert provider.quality() == SPATIAL_QUALITY_METRIC_LIDAR


def test_frontier_extraction_from_ray_traced_map() -> None:
    provider = make_provider()
    free, occupied = corridor_map()
    provider._on_occupancy_grid(make_map_msg(free_cells=free, occupied_cells=occupied))
    provider._on_odometry(make_odom_msg(x=0.5, y=1.0, yaw=0.0))
    frontiers = provider.get_frontiers()
    assert frontiers, "expected at least one frontier from free/unknown boundary"
    # Corridor cells are grid x in 1..9 (world -0.9..-0.1 m); a frontier
    # component centroid must lie inside that corridor band.
    assert any(
        f.position is not None and f.position[0] < 0.0 for f in frontiers
    )


def test_stale_map_falls_back(monkeypatch: pytest.MonkeyPatch) -> None:
    provider = make_provider()
    free, occupied = corridor_map()
    provider._on_occupancy_grid(make_map_msg(free_cells=free, occupied_cells=occupied))
    provider._on_odometry(make_odom_msg())
    assert provider.get_map() is not None
    # Age the map out of the freshness window.
    provider._map_ts = time.monotonic() - provider.map_stale_s - 1.0
    assert provider.get_map() is None
    # Fallback path must not crash and returns the fallback's frontier list.
    frontiers = provider.get_frontiers()
    assert isinstance(frontiers, list)


def test_camera_point_to_spatial_nominal(monkeypatch: pytest.MonkeyPatch) -> None:
    provider = make_provider()
    provider._on_odometry(make_odom_msg(x=1.0, y=1.0, yaw=0.0))
    result = provider.camera_point_to_spatial((0.0, 0.0, 0.5))
    assert result is not None
    assert len(result) == 3
    prov = provider.transform_provenance()
    assert prov["transform_source"] == "nominal_extrinsic"
    assert prov["map_revision"] is None or prov["map_revision"] >= 0


def test_health_reports_mapping_assist_only() -> None:
    provider = make_provider()
    health = provider.health()
    assert health["motion_authorized"] is False
    assert health["safety_authorized"] is False
    assert health["extrinsic_status"] == "candidate_unconfirmed"
    assert health["mapping_mode"] == "mapping_assist"
    assert health["source"] == SOURCE_MAP


def test_revision_increments() -> None:
    provider = make_provider()
    provider._on_occupancy_grid(make_map_msg())
    first = provider.get_map()
    assert first is not None and first.revision == 1
    provider._on_occupancy_grid(make_map_msg())
    second = provider.get_map()
    assert second is not None and second.revision == 2
