#!/usr/bin/env python3
"""Read-only live check that map, pose and keyframe graph share one identity."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from app.spatial.nav2_spatial_provider import Nav2SpatialProvider


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--json", required=True)
    args = parser.parse_args()
    if not 0.1 <= args.duration <= 60.0:
        raise ValueError("duration must be in [0.1, 60] seconds")

    provider = Nav2SpatialProvider(enable_ros=True)
    try:
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            grid = provider.get_map()
            graph = provider._keyframe_graph.snapshot()
            pose = provider.get_pose()
            mapping_quality = provider.health().get("mapping_quality") or {}
            quality_compared = "comparable_cells" in (
                mapping_quality.get("world_aligned_change") or {}
            )
            if (grid is not None and graph is not None and pose is not None
                    and quality_compared):
                break
            time.sleep(0.05)
        grid = provider.get_map()
        graph = provider._keyframe_graph.snapshot()
        pose = provider.get_pose()
        provenance = dict(grid.provenance or {}) if grid is not None else None
        identity_matches = bool(
            provenance is not None
            and graph is not None
            and provenance.get("map_id") == graph.map_id
            and provenance.get("map_epoch") == graph.map_epoch
            and provenance.get("graph_revision") == graph.graph_revision
            and provenance.get("sensor_profile_id") == graph.sensor_profile_id
            and graph.sensor_profile_id not in {"", "unknown"}
        )
        mapping_quality = provider.health().get("mapping_quality") or {}
        quality_compared = "comparable_cells" in (
            mapping_quality.get("world_aligned_change") or {}
        )
        anchor_probe_stamp = time.time()
        anchor_probe = None
        if pose is not None:
            anchor_probe = provider.anchor_map_point(
                (float(pose.x) + 1.0, float(pose.y), 0.0),
                anchor_probe_stamp,
            )
        result = {
            "schema_version": "go2w_keyframe_world_binding_probe_v1",
            "read_only": True,
            "motion_sent": False,
            "map_received": grid is not None,
            "pose_received": pose is not None,
            "map_provenance": provenance,
            "keyframe_graph": asdict(graph) if graph is not None else None,
            "health": provider.health(),
            "identity_matches": identity_matches,
            "quality_compared": quality_compared,
            "anchor_probe_stamp": anchor_probe_stamp,
            "anchor_probe": anchor_probe,
            "acquisition_pose_anchor_matches": bool(
                anchor_probe is not None
                and anchor_probe.get("map_id") == graph.map_id
                and anchor_probe.get("map_epoch") == graph.map_epoch
                and anchor_probe.get("anchor_association_method")
                in {"nearest_time", "acquisition_pose_gate"}
            ) if graph is not None else False,
        }
        result["passed"] = bool(
            result["map_received"] and result["pose_received"]
            and identity_matches and quality_compared
            and result["acquisition_pose_anchor_matches"]
        )
    finally:
        provider.close()

    output = Path(args.json)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
