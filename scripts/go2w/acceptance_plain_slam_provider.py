#!/usr/bin/env python3
# Copyright 2026 robot_scene_demo maintainers
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Live PlainSlamSpatialProvider acceptance probe (plan §28).

Reads the REAL /go2w/slam/map_2d + /go2w/slam/odom_base topics through
PlainSlamSpatialProvider and reports map/pose/frontiers/provenance.  This is
the dry-run's spatial-data core, without cameras/LLM/motion.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import rclpy  # noqa: E402

from app.spatial.plain_slam_spatial_provider import (  # noqa: E402
    PlainSlamSpatialProvider,
)


def main() -> int:
    rclpy.init()
    provider = PlainSlamSpatialProvider(
        enable_ros=True,
        map_topic="/go2w/slam/map_2d",
        odom_topic="/go2w/slam/odom_base",
    )
    deadline = time.time() + 15
    while time.time() < deadline and provider.get_map() is None:
        provider.spin_once()
        time.sleep(0.1)
    provider.spin_once()

    snapshot = provider.get_map()
    pose = provider.get_pose()
    frontiers = provider.get_frontiers()

    result = {
        "provider": "plain_slam_pandarxt16",
        "get_map": snapshot is not None,
        "map_source": snapshot.source if snapshot else None,
        "map_revision": snapshot.revision if snapshot else None,
        "map_cells": {
            "free": len(snapshot.free) if snapshot else 0,
            "occupied": len(snapshot.occupied) if snapshot else 0,
            "unknown": len(snapshot.unknown) if snapshot else 0,
        },
        "map_frame_provenance": (
            snapshot.provenance if snapshot else None
        ),
        "get_pose": pose is not None,
        "pose_source": pose.source if pose else None,
        "pose_xy_yaw": [pose.x, pose.y, pose.yaw] if pose else None,
        "quality": provider.quality(),
        "frontier_count": len(frontiers),
        "frontiers": [
            {
                "id": f.frontier_id,
                "position_m": list(f.position) if f.position else None,
                "distance_m": f.distance_m,
                "gain": f.spatial_information_gain,
            }
            for f in frontiers[:6]
        ],
        "transform_provenance": provider.transform_provenance(),
        "health": provider.health(),
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    provider.close()
    rclpy.shutdown()
    return 0 if snapshot is not None else 1


if __name__ == "__main__":
    sys.exit(main())