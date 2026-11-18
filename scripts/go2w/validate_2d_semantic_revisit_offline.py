#!/usr/bin/env python3
"""Concentrated offline acceptance for the 2-D semantic revisit chain."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from app.perception.depth_object_localizer import ObjectSpatialObservation
from app.manual_web_demo.slam_map_snapshot import load_slam_map_snapshot
from app.spatial.object_display_snapshot import write_object_snapshot
from app.spatial.semantic_object_gate import build_object_snapshot
from app.spatial.object_relation_store import ObjectRelationStore
from app.spatial.occupancy_grid_2d import OccupancyGrid2D
from app.spatial.revisit_visibility import evaluate_revisit_visibility
from app.spatial.semantic_object_map import SemanticObjectMap, SemanticObservationAnchor


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def node(object_id: str, x: float, y: float, *, epoch: str = "epoch-a",
         mobility: str = "semi_static") -> SimpleNamespace:
    return SimpleNamespace(
        object_id=object_id, map_xyz=(x, y, 0.6), mobility_class=mobility,
        topology_eligible=True, lifecycle_state="active", status="CONFIRMED",
        provenance={"map_id": "map-a", "map_epoch": epoch, "map_frame": "map"},
        label="chair", label_zh="椅子", confidence=.9, observation_count=3,
        spatial_quality="METRIC_LIDAR", is_hazard=False,
    )


def observation(source_id: str, label: str, label_zh: str, xyz: tuple[float, float, float],
                stamp: float) -> ObjectSpatialObservation:
    value = ObjectSpatialObservation(
        object_id=source_id, label=label, label_zh=label_zh, category="furniture",
        camera_xyz=(0.0, 0.0, max(.1, xyz[2])), map_xyz=xyz,
        spatial_quality="METRIC_LIDAR", confidence=.9,
        provenance={"map_id": "map-a", "map_epoch": "epoch-a", "map_frame": "map",
                    "timestamp": stamp, "frame_id": source_id},
    )
    value.mobility_class = "semi_static"
    value.mobility_state = "stationary"
    value.topology_eligible = True
    return value


def check_spatial_graph() -> dict[str, int]:
    objects = {
        **{f"a{i}": node(f"a{i}", i * .12, (i % 2) * .12) for i in range(4)},
        **{f"b{i}": node(f"b{i}", 2.55 + i * .12, (i % 2) * .12) for i in range(4)},
    }
    store = ObjectRelationStore()
    result = store.rebuild_spatial_adjacency(
        objects, timestamp=10.0, near_distance_m=3.0,
        near_exit_distance_m=3.4, spatial_neighbor_distance_m=4.5,
        max_near_edges=1, bridge_extra_edges_per_node=2,
    )
    edges = list(store.relations.values())
    cross = [edge for edge in edges if edge.source_object_id[0] != edge.target_object_id[0]]
    require(cross, "dense local neighbours consumed the cross-view bridge")
    require(any(edge.provenance.get("bridge") for edge in cross),
            "cross-view edge was not marked as retained bridge")
    require(all(edge.observation_count == 0 for edge in edges),
            "geometry rebuild fabricated visual observations")

    mismatch = {"x": node("x", 0, 0, epoch="epoch-a"),
                "y": node("y", .5, 0, epoch="epoch-b")}
    rejected = store.rebuild_spatial_adjacency(mismatch, timestamp=11.0)
    require(rejected["selected_count"] == 0, "different map epochs were connected")
    return {"selected": result["selected_count"], "bridges": result["bridge_count"]}


def check_grid_geometry_and_wall() -> dict[str, object]:
    width = height = 8
    data = [0] * (width * height)
    for row in range(height):
        data[row * width + 4] = 100
    grid = OccupancyGrid2D(
        width=width, height=height, resolution=.5,
        origin_x=10.0, origin_y=-3.0, origin_yaw=np.pi / 2,
        data=data, map_epoch="epoch-a",
    )
    world = (9.25, -2.25)  # rotated local (0.75, 0.75) -> cell (1, 1)
    require(grid.world_to_cell(*world) == (1, 1), "rotated non-zero map origin is wrong")
    start = grid_world(grid, .75, 1.75)
    end = grid_world(grid, 3.25, 1.75)
    accepted, reason = grid.line_evidence(start, end)
    require(not accepted and reason == "occupied_cell_between_objects",
            "known wall did not reject a semantic adjacency")
    return {"cell": grid.world_to_cell(*world), "wall_reason": reason}


def grid_world(grid: OccupancyGrid2D, local_x: float, local_y: float) -> tuple[float, float]:
    cosine, sine = np.cos(grid.origin_yaw), np.sin(grid.origin_yaw)
    return (grid.origin_x + cosine * local_x - sine * local_y,
            grid.origin_y + sine * local_x + cosine * local_y)


def check_visibility() -> dict[str, str]:
    depth = np.full((60, 80), 4_000, dtype=np.uint16)
    behind = evaluate_revisit_visibility(
        camera_xyz=(0, 0, -2), depth_image=depth, width=80, height=60,
        fx=60, fy=60, cx=40, cy=30, depth_unit_m=.001,
        localization_valid=True, detector_covered=True,
    )
    require(not behind.counts_as_missing, "turning away counted as object absence")
    clear = evaluate_revisit_visibility(
        camera_xyz=(0, 0, 2), depth_image=depth, width=80, height=60,
        fx=60, fy=60, cx=40, cy=30, depth_unit_m=.001,
        localization_valid=True, detector_covered=True,
    )
    require(clear.counts_as_missing, "clear farther background did not count as absence")
    depth[:, :] = 1_000
    occluded = evaluate_revisit_visibility(
        camera_xyz=(0, 0, 2), depth_image=depth, width=80, height=60,
        fx=60, fy=60, cx=40, cy=30, depth_unit_m=.001,
        localization_valid=True, detector_covered=True,
    )
    require(not occluded.counts_as_missing, "nearer occluder counted as absence")
    return {"turned_away": behind.reason, "clear": clear.reason, "occluded": occluded.reason}


def check_identity_lifecycle_and_epoch() -> dict[str, object]:
    semantic = SemanticObjectMap(merge_distance_m=.6, confirm_min_observations=2)
    first = semantic.update_with_associations(
        [observation("frame-1", "chair", "椅子", (1, 1, .7), 1)], now=1,
        frame_id="frame-1",
    )
    object_id = first.created_ids[0]
    second = semantic.update_with_associations(
        [observation("frame-2", "office chair", "办公椅", (1.08, 1, .7), 2)],
        now=2, frame_id="frame-2",
    )
    require(second.associations[0].persistent_object_id == object_id,
            "chair alias did not reuse the persistent ID")
    require(len(semantic.objects) == 1, "alias association created a duplicate node")
    entry = semantic.objects[object_id]
    for index, timestamp in enumerate((3.0, 4.0, 5.0), start=1):
        semantic.record_missing(
            object_id, observation_id=f"miss-{index}", timestamp=timestamp,
            observer_xy=(0, 0), semi_static_required=3, semi_static_span_s=2,
        )
    require(entry.lifecycle_state == "removed" and not entry.topology_eligible,
            "validated semi-static absence did not remove the current node")

    recovered = semantic.update_with_associations(
        [observation("frame-6", "chair", "椅子", (1.09, 1, .7), 6)],
        now=6, frame_id="frame-6",
    )
    require(recovered.associations[0].persistent_object_id == object_id,
            "positive revisit failed to recover the same ID")
    require(entry.lifecycle_state == "active" and entry.missing_observation_count == 0,
            "positive evidence did not clear absence state")

    old_anchor = SemanticObservationAnchor(
        timestamp=1, camera_xyz=(0, 0, 1), confidence=.9,
        spatial_quality="METRIC_LIDAR", map_xyz_at_insert=(1, 1, .7),
        position_epoch=entry.position_epoch,
    )
    entry.observation_anchors.append(old_anchor)
    require(semantic.start_position_epoch(object_id, (3, 1, .7), timestamp=7),
            "new position epoch was not accepted")
    current_epoch = entry.position_epoch
    current_anchor = SemanticObservationAnchor(
        timestamp=8, camera_xyz=(0, 0, 1), confidence=.9,
        spatial_quality="METRIC_LIDAR", map_xyz_at_insert=(3, 1, .7),
        position_epoch=current_epoch,
    )
    entry.observation_anchors.append(current_anchor)

    def reproject(camera_xyz, timestamp):
        return (99, 99, 99) if timestamp == 1 else (3.2, 1, .7)

    semantic.reproject_observations(reproject)
    require(entry.map_xyz[0] < 4, "old position epoch polluted the current position")

    fixed = node("fixed", 0, 0, mobility="fixed")
    fixed_map = SemanticObjectMap()
    fixed_map.objects["fixed"] = fixed
    # record_missing uses the same fields as SemanticObjectEntry; fill the
    # minimal lifecycle counters for this isolated fixed landmark fixture.
    for name, value in {
        "last_checked_at": 0.0, "missing_last_observation_id": None,
        "missing_observation_count": 0, "missing_first_at": 0.0,
        "missing_viewpoints": [], "provenance": fixed.provenance,
    }.items():
        setattr(fixed, name, value)
    for index in range(4):
        fixed_map.record_missing("fixed", observation_id=f"fixed-{index}",
                                 timestamp=10 + index, observer_xy=(0, 0))
    require(fixed.lifecycle_state != "removed", "fixed landmark removed too early")
    return {"object_id": object_id, "position_epoch": current_epoch,
            "reprojected_x": round(entry.map_xyz[0], 3)}


def check_two_view_snapshot_identity() -> dict[str, object]:
    record = {
        "object_id": "obj_007", "label": "chair", "label_zh": "椅子",
        "map_xyz": [1.25, -0.5, .7], "confidence": .40,
        "confidence_max": .42, "observation_count": 2, "status": "CONFIRMED",
        "mobility_class": "semi_static", "mobility_state": "stationary",
        "topology_eligible": True, "lifecycle_state": "missing_candidate",
        "provenance": {"map_frame": "map", "map_epoch": "epoch-a"},
    }
    objects = build_object_snapshot(
        [record], map_epoch="epoch-a", frame_id="map", graph_revision=9,
        confirmed_min_confidence=.35, confirm_min_observations=2,
        topology_accepted_only=True,
    )
    require(objects["objects"][0]["object_id"] == "obj_007",
            "current lifecycle object was dropped from the 2-D overlay")
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        object_path, map_path = root / "objects.json", root / "map.json"
        write_object_snapshot(object_path, objects)
        map_path.write_text(json.dumps({
            "points": [], "map_kind": "occupancy_grid_2d", "available": True,
            "generated_at": time.time(), "frame_id": "map", "map_epoch": "epoch-a",
            "grid": {"width": 2, "height": 2, "resolution": .5,
                     "origin": {"x": 1, "y": -1, "yaw": .2},
                     "data": [-1, 0, 100, 0]},
        }), encoding="utf-8")
        combined = load_slam_map_snapshot(map_path, object_path)
    require(combined["available"] and combined["web_display_points"] == 4,
            "valid full occupancy grid was treated as an empty point cloud")
    require(combined["object_overlay_status"] == "OK"
            and combined["objects"][0]["object_id"] == "obj_007",
            "map and topology views did not preserve the same object ID")
    return {"object_id": "obj_007", "lifecycle_state": "missing_candidate",
            "grid_cells": combined["web_display_points"]}


def main() -> int:
    report = {
        "spatial_graph": check_spatial_graph(),
        "grid": check_grid_geometry_and_wall(),
        "visibility": check_visibility(),
        "lifecycle": check_identity_lifecycle_and_epoch(),
        "two_view_identity": check_two_view_snapshot_identity(),
    }
    print(json.dumps({"result": "PASS", **report}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

