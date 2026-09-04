"""PlaceGraph: spatial observation places, not per-bundle nodes.

A Place is created only when the robot physically relocates enough (metric pose
distance or observed displacement).  In-place rotations only update
``heading_coverage`` on the current Place.

The graph also performs *global* nearest-place association when a pose is
available: returning to an old location reuses the old Place node instead of
creating a synthetic duplicate (loop-closure behaviour).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any

from app.spatial.models import MovementEdge, PlaceNode, SpatialPose
from app.spatial.spatial_transform import circular_mean, weighted_position_mean


@dataclass
class PlaceObservation:
    observation_id: str
    heading_sector: int | None
    timestamp: float
    objects: list[str] = field(default_factory=list)
    rgbd_frame_id: str | None = None
    pose: dict[str, Any] | None = None
    provenance: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "observation_id": self.observation_id,
            "heading_sector": self.heading_sector,
            "timestamp": self.timestamp,
            "objects": self.objects,
            "rgbd_frame_id": self.rgbd_frame_id,
            "pose": self.pose,
            "provenance": self.provenance,
        }


class PlaceGraph:
    def __init__(
        self,
        *,
        merge_distance_m: float = 0.6,
        relocation_min_displacement_m: float = 0.10,
    ) -> None:
        self.merge_distance_m = float(merge_distance_m)
        self.relocation_min_displacement_m = float(relocation_min_displacement_m)
        self.places: dict[str, PlaceNode] = {}
        self.observations: dict[str, PlaceObservation] = {}
        self.edges: list[MovementEdge] = []
        self._current_place_id: str | None = None
        self._sequence = 0
        self._last_edge_signature: tuple[str, str, str] | None = None

    def register_observation(
        self,
        *,
        observation_id: str,
        heading_sector: int | None,
        objects: list[str],
        rgbd_frame_id: str | None = None,
        pose: SpatialPose | None = None,
        observed_displacement_m: float | None = None,
        timestamp: float | None = None,
        target_candidate: bool = False,
    ) -> tuple[str, bool]:
        """Register one observation; returns (place_id, created_new_place)."""
        now = timestamp if timestamp is not None else time.time()
        previous_place_id = self._current_place_id
        if previous_place_id is None:
            place_id = self._new_place(pose=pose)
            created = True
            revisited = False
        else:
            place_id, created, revisited = self._resolve_place(
                pose=pose,
                observed_displacement_m=observed_displacement_m,
            )
        place = self.places[place_id]
        place.observation_ids.append(observation_id)
        place.visit_count += 1
        place.revisited = bool(revisited or (not created and previous_place_id == place_id and place.visit_count > 1))
        if heading_sector is not None:
            place.heading_coverage[str(heading_sector)] = (
                place.heading_coverage.get(str(heading_sector), 0) + 1
            )
        for label in objects:
            if label not in place.observed_object_labels:
                place.observed_object_labels.append(label)
        if target_candidate:
            place.target_candidate = True
        self._update_pose_fusion(place, pose, now)
        self.observations[observation_id] = PlaceObservation(
            observation_id=observation_id,
            heading_sector=heading_sector,
            timestamp=now,
            objects=list(objects),
            rgbd_frame_id=rgbd_frame_id,
            pose=pose.to_dict() if pose else None,
            provenance={
                "place_id": place_id,
                "revisited": revisited,
                "created": created,
            },
        )
        # A movement edge is inserted whenever the resolved place differs from
        # the previous current place (both fresh relocation and revisit).
        if previous_place_id is not None and previous_place_id != place_id:
            self._add_movement_edge(previous_place_id, place_id, observed_displacement_m)
        self._current_place_id = place_id
        return place_id, created

    def attach_objects(self, place_id: str, persistent_object_ids: list[str]) -> None:
        """Attach stable persistent entity ids to a Place (not labels)."""
        place = self.places.get(place_id)
        if place is None:
            return
        for object_id in persistent_object_ids:
            if object_id and object_id not in place.observed_object_ids:
                place.observed_object_ids.append(object_id)

    def _resolve_place(
        self,
        *,
        pose: SpatialPose | None,
        observed_displacement_m: float | None,
    ) -> tuple[str, bool, bool]:
        current = self.places[self._current_place_id]
        # Independent wheel motion is the primary relocation evidence.  A
        # turn-only cycle may contain a badly drifting LIO pose; never let
        # that pose create P2/P3 while the wheels report no translation.
        if observed_displacement_m is not None:
            if observed_displacement_m < self.relocation_min_displacement_m:
                return current.place_id, False, False
            if pose is None:
                new_id = self._new_place(pose=None, from_place=self._current_place_id)
                return new_id, True, False
            nearest_id, distance = self.nearest_place_id(pose)
            if nearest_id is not None and distance <= self.merge_distance_m and nearest_id != current.place_id:
                return nearest_id, False, True
            # When LIO still claims that the robot is at the current place but
            # wheel odometry observed a real translation, create a topological
            # place without trusting that inconsistent metric pose.
            place_pose = None if nearest_id == current.place_id else pose
            new_id = self._new_place(pose=place_pose, from_place=self._current_place_id)
            return new_id, True, False
        if pose is not None:
            # Global nearest-place association: revisiting an old place should
            # reuse that place, not spawn a duplicate.
            nearest_id, distance = self.nearest_place_id(pose)
            if nearest_id is not None and distance <= self.merge_distance_m:
                # Still at the current place: in-place rotation / small motion.
                if self.places[nearest_id] is current:
                    return current.place_id, False, False
                # Loop closure / revisit: reuse an old Place.
                return nearest_id, False, True
            # Significant relocation to a genuinely new area.
            new_id = self._new_place(pose=pose, from_place=self._current_place_id)
            return new_id, True, False
        if observed_displacement_m is not None and observed_displacement_m >= self.relocation_min_displacement_m:
            new_id = self._new_place(pose=None, from_place=self._current_place_id)
            return new_id, True, False
        return self._current_place_id, False, False

    def nearest_place_id(self, pose: SpatialPose, *, max_distance: float | None = None) -> tuple[str | None, float]:
        """Return (place_id, distance) of the nearest Place with a pose."""
        best_id: str | None = None
        best_dist = float("inf")
        for place_id, place in self.places.items():
            if place.pose is None:
                continue
            distance = math.hypot(pose.x - place.pose.x, pose.y - place.pose.y)
            if distance < best_dist:
                best_id = place_id
                best_dist = distance
        if max_distance is not None and best_dist > max_distance:
            return None, best_dist
        return best_id, best_dist

    def _new_place(self, *, pose: SpatialPose | None, from_place: str | None = None) -> str:
        self._sequence += 1
        place_id = f"P{self._sequence}"
        self.places[place_id] = PlaceNode(
            place_id=place_id,
            pose=pose,
            pose_quality=pose.quality if pose else "unavailable",
            pose_mean=None,
            pose_observation_count=0,
            provenance={"created_at": time.time(), "from_place": from_place},
        )
        if from_place is not None and from_place in self.places:
            self._add_movement_edge(from_place, place_id, None)
        return place_id

    def _add_movement_edge(
        self,
        from_place: str,
        to_place: str,
        observed_displacement_m: float | None,
    ) -> None:
        signature = (from_place, to_place, f"{self._current_place_id}")
        # Avoid duplicate edges for the same transition within one observation
        # burst and avoid self-edges.
        if from_place == to_place:
            return
        for edge in self.edges:
            if edge.from_place == from_place and edge.to_place == to_place:
                # Already connected; update displacement with latest observation.
                if observed_displacement_m is not None:
                    edge.observed_displacement_m = observed_displacement_m
                return
        self.edges.append(
            MovementEdge(
                edge_id=f"E{len(self.edges) + 1}",
                from_place=from_place,
                to_place=to_place,
                observed_displacement_m=observed_displacement_m,
                success_count=0,
                failure_count=0,
                blocked_count=0,
                recovery_count=0,
                status="OPEN",
                traversability_score=1.0,
                cost=max(0.1, float(observed_displacement_m or 1.0)),
                provenance={"source": "place_graph_relocation_or_revisit"},
            )
        )
        self._last_edge_signature = signature

    def _find_edge(self, from_place: str, to_place: str) -> MovementEdge | None:
        for edge in self.edges:
            if (
                (edge.from_place == from_place and edge.to_place == to_place)
                or (
                    not edge.provenance.get("directed")
                    and edge.from_place == to_place
                    and edge.to_place == from_place
                )
            ):
                return edge
        return None

    def record_edge_success(
        self,
        from_place: str,
        to_place: str,
        *,
        displacement_m: float | None = None,
        now: float | None = None,
    ) -> None:
        edge = self._find_edge(from_place, to_place)
        if edge is None:
            return
        edge.success_count += 1
        edge.navigation_result = "succeeded"
        edge.last_success_at = now if now is not None else time.time()
        edge.status = "OPEN"
        edge.traversability_score = min(
            1.0, edge.traversability_score + 0.1
        )
        if displacement_m is not None:
            edge.observed_displacement_m = displacement_m
            edge.cost = max(0.1, displacement_m)
        edge.failure_count = max(0, edge.failure_count - 0)
        edge.last_failure_reason = ""

    def record_edge_failure(
        self,
        from_place: str,
        to_place: str,
        *,
        reason: str = "",
        now: float | None = None,
        max_failures: int = 2,
    ) -> None:
        edge = self._find_edge(from_place, to_place)
        if edge is None:
            return
        edge.failure_count += 1
        edge.navigation_result = "failed"
        edge.last_failure_at = now if now is not None else time.time()
        edge.last_failure_reason = reason
        edge.traversability_score = max(
            0.0, edge.traversability_score - 0.35
        )
        edge.cost += 2.0
        if edge.failure_count >= max_failures:
            edge.status = "BLOCKED"
            edge.blocked_count += 1

    def _update_pose_fusion(self, place: PlaceNode, pose: SpatialPose | None, now: float) -> None:
        if pose is None:
            return
        place.pose_observation_count += 1
        place.last_pose_update = now
        place.pose_quality = _quality_merge(place.pose_quality, pose.quality)
        if place.pose_mean is None:
            place.pose_mean = SpatialPose(
                x=pose.x, y=pose.y, yaw=pose.yaw,
                frame_id=pose.frame_id, quality=place.pose_quality,
                source=pose.source, provenance=dict(pose.provenance),
            )
        else:
            prev = place.pose_mean
            count = place.pose_observation_count
            place.pose_mean = SpatialPose(
                x=round(prev.x + (pose.x - prev.x) / count, 4),
                y=round(prev.y + (pose.y - prev.y) / count, 4),
                yaw=round(circular_mean([prev.yaw, pose.yaw]) % 360.0, 4),
                frame_id=pose.frame_id,
                quality=place.pose_quality,
                source=pose.source,
                provenance=dict(pose.provenance),
            )
        # The current pose is the latest observation; the mean is kept for
        # stable Place identity / pose refinement.
        place.pose = pose

    def mark_negative(self, place_id: str | None = None) -> None:
        place_id = place_id or self._current_place_id
        if place_id in self.places:
            self.places[place_id].negative_evidence += 1

    def current_place(self) -> PlaceNode | None:
        if self._current_place_id is None:
            return None
        return self.places.get(self._current_place_id)

    def to_dict(self) -> dict[str, Any]:
        return {
            "places": [place.to_dict() for place in self.places.values()],
            "observations": [obs.to_dict() for obs in self.observations.values()],
            "edges": [edge.to_dict() for edge in self.edges],
            "current_place_id": self._current_place_id,
            "merge_distance_m": self.merge_distance_m,
            "relocation_min_displacement_m": self.relocation_min_displacement_m,
        }


def _quality_merge(left: str, right: str) -> str:
    order = {
        "RGB_ONLY": 0,
        "CAMERA_LOCAL": 1,
        "RELATIVE_RGBD": 2,
        "METRIC_RGBD": 3,
    }
    return left if order.get(left, 0) >= order.get(right, 0) else right
