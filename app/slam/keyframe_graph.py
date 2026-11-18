"""Versioned keyframe graph contract shared by SLAM, mapping and semantics."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import threading
from typing import Any


@dataclass(frozen=True)
class KeyframePose:
    anchor_id: str
    source_stamp: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class KeyframeGraphSnapshot:
    map_id: str
    map_epoch: str
    frame_id: str
    sensor_profile_id: str
    graph_revision: int
    nodes: tuple[KeyframePose, ...]


class KeyframeGraphStore:
    """Thread-safe immutable snapshots from ``/go2w/slam/keyframe_graph``."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._snapshot: KeyframeGraphSnapshot | None = None

    def update_json(self, payload: str | dict[str, Any]) -> bool:
        value = json.loads(payload) if isinstance(payload, str) else payload
        if value.get("schema_version") != "go2w_keyframe_graph_v1":
            return False
        nodes: list[KeyframePose] = []
        for item in value.get("nodes") or []:
            pose = item.get("corrected_pose")
            if not isinstance(pose, (list, tuple)) or len(pose) != 3:
                continue
            numbers = [float(item.get("source_stamp")), *(float(v) for v in pose)]
            if not all(math.isfinite(number) for number in numbers):
                continue
            nodes.append(KeyframePose(
                anchor_id=str(item.get("anchor_id") or f"keyframe_{item.get('unique_id')}"),
                source_stamp=numbers[0], x=numbers[1], y=numbers[2], yaw=numbers[3],
            ))
        snapshot = KeyframeGraphSnapshot(
            map_id=str(value.get("map_id") or ""),
            map_epoch=str(value.get("map_epoch") or ""),
            frame_id=str(value.get("frame_id") or "map"),
            sensor_profile_id=str(value.get("sensor_profile_id") or "unknown"),
            graph_revision=int(value.get("graph_revision") or 0),
            nodes=tuple(nodes),
        )
        with self._lock:
            current = self._snapshot
            if current is not None:
                if (snapshot.map_id, snapshot.map_epoch) == (current.map_id, current.map_epoch):
                    if snapshot.graph_revision < current.graph_revision:
                        return False
            self._snapshot = snapshot
        return True

    def snapshot(self) -> KeyframeGraphSnapshot | None:
        with self._lock:
            return self._snapshot

    def anchor_map_point(
        self, map_xyz: tuple[float, float, float], source_stamp: float,
        *, max_time_delta_s: float = 3.0,
        observer_pose: tuple[float, float, float] | None = None,
        max_observer_distance_m: float = 0.75,
        max_observer_yaw_rad: float = math.radians(20.0),
    ) -> dict[str, Any] | None:
        snapshot = self.snapshot()
        if snapshot is None or not snapshot.nodes:
            return None
        anchor = min(snapshot.nodes, key=lambda node: abs(node.source_stamp - source_stamp))
        delta = abs(anchor.source_stamp - source_stamp)
        association_method = "nearest_time"
        observer_distance = None
        observer_yaw_delta = None
        if delta > max_time_delta_s:
            # A stationary robot can observe for much longer than the SLAM
            # keyframe trigger interval.  In that case timestamp-only matching
            # would incorrectly make every later semantic observation
            # unanchored.  A stale-time fallback is permitted only when the
            # acquisition-time robot pose is independently available in the
            # same corrected map frame and is spatially consistent with a
            # saved keyframe.  This explicitly avoids attaching a long VLM
            # result to the robot's *current* pose.
            if observer_pose is None:
                return None
            ox, oy, oyaw = (float(value) for value in observer_pose)
            if not all(math.isfinite(value) for value in (ox, oy, oyaw)):
                return None

            def spatial_error(node: KeyframePose) -> tuple[float, float]:
                distance = math.hypot(ox - node.x, oy - node.y)
                yaw_delta = abs((oyaw - node.yaw + math.pi) % (2.0 * math.pi) - math.pi)
                return distance, yaw_delta

            anchor = min(snapshot.nodes, key=lambda node: spatial_error(node))
            observer_distance, observer_yaw_delta = spatial_error(anchor)
            if (observer_distance > max_observer_distance_m
                    or observer_yaw_delta > max_observer_yaw_rad):
                return None
            delta = abs(anchor.source_stamp - source_stamp)
            association_method = "acquisition_pose_gate"
        dx, dy = float(map_xyz[0]) - anchor.x, float(map_xyz[1]) - anchor.y
        cosine, sine = math.cos(anchor.yaw), math.sin(anchor.yaw)
        point = (
            cosine * dx + sine * dy,
            -sine * dx + cosine * dy,
            float(map_xyz[2]),
        )
        return {
            "map_id": snapshot.map_id,
            "map_epoch": snapshot.map_epoch,
            "anchor_id": anchor.anchor_id,
            "anchor_graph_revision": snapshot.graph_revision,
            "point_in_anchor": point,
            "anchor_time_delta_s": delta,
            "anchor_association_method": association_method,
            "anchor_observer_distance_m": observer_distance,
            "anchor_observer_yaw_delta_rad": observer_yaw_delta,
            "frame_id": snapshot.frame_id,
            "sensor_profile_id": snapshot.sensor_profile_id,
        }

    def reproject(
        self, anchor_id: str, point_in_anchor: tuple[float, float, float],
        *, map_id: str | None = None, map_epoch: str | None = None,
    ) -> tuple[float, float, float] | None:
        snapshot = self.snapshot()
        if snapshot is None:
            return None
        if map_id and snapshot.map_id != map_id:
            return None
        if map_epoch and snapshot.map_epoch != map_epoch:
            return None
        anchor = next((node for node in snapshot.nodes if node.anchor_id == anchor_id), None)
        if anchor is None:
            return None
        x, y, z = (float(v) for v in point_in_anchor)
        cosine, sine = math.cos(anchor.yaw), math.sin(anchor.yaw)
        return (
            anchor.x + cosine * x - sine * y,
            anchor.y + sine * x + cosine * y,
            z,
        )
