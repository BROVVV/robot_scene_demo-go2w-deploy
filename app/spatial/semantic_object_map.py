"""SemanticObjectMap: persistent semantic entity map with data association.

It stores only facts from the perception/localizer stack.  PSG predictions are
never inserted here (they live in :class:`SemanticPrior`).

The map performs cross-view entity association using geometry as the primary
signal, semantics as a gate and appearance/place/temporal signals as auxiliary
evidence.  A one-to-one greedy assignment prevents two side-by-side objects of
the same label collapsing into a single entity.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any

from app.perception.depth_object_localizer import ObjectSpatialObservation
from app.spatial.models import (
    SPATIAL_QUALITY_CAMERA_LOCAL,
    SPATIAL_QUALITY_METRIC_RGBD,
    SPATIAL_QUALITY_RELATIVE_RGBD,
)
from app.spatial.spatial_transform import (
    dynamic_merge_distance_m,
    quality_weight,
    weighted_position_mean,
)

ENTITY_TENTATIVE = "TENTATIVE"
ENTITY_CONFIRMED = "CONFIRMED"
ENTITY_STALE = "STALE"
ENTITY_REJECTED = "REJECTED"

ACTION_CREATED = "CREATED"
ACTION_MERGED = "MERGED"
ACTION_REJECTED = "REJECTED"


@dataclass
class SemanticAssociation:
    observation_index: int
    source_object_id: str | None
    persistent_object_id: str
    action: str
    association_score: float
    distance_m: float | None
    reasons: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "observation_index": self.observation_index,
            "source_object_id": self.source_object_id,
            "persistent_object_id": self.persistent_object_id,
            "action": self.action,
            "association_score": round(self.association_score, 4),
            "distance_m": round(self.distance_m, 4) if self.distance_m is not None else None,
            "reasons": self.reasons,
        }


@dataclass
class SemanticMapUpdateResult:
    created_ids: list[str] = field(default_factory=list)
    updated_ids: list[str] = field(default_factory=list)
    associations: list[SemanticAssociation] = field(default_factory=list)
    rejected_pairs: list[dict[str, Any]] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "created_ids": self.created_ids,
            "updated_ids": self.updated_ids,
            "associations": [item.to_dict() for item in self.associations],
            "rejected_pairs": self.rejected_pairs,
        }


@dataclass
class SemanticObjectEntry:
    object_id: str
    label: str
    depth_m: float | None = None
    camera_xyz: tuple[float, float, float] | None = None
    map_xyz: tuple[float, float, float] | None = None
    bearing_deg: float | None = None
    spatial_quality: str = SPATIAL_QUALITY_CAMERA_LOCAL
    confidence: float = 0.0
    observation_count: int = 0
    first_seen: float = 0.0
    last_seen: float = 0.0
    seen_from_places: list[str] = field(default_factory=list)
    negative_evidence: int = 0
    provenance: dict[str, Any] = field(default_factory=dict)
    source_observation_ids: list[str] = field(default_factory=list)
    merge_history: list[dict[str, Any]] = field(default_factory=list)
    association_score: float = 0.0
    status: str = ENTITY_TENTATIVE
    position_mean_xyz: tuple[float, float, float] | None = None
    position_variance_xyz: tuple[float, float, float] | None = None
    map_observation_count: int = 0

    def to_dict(self) -> dict[str, Any]:
        return {
            "object_id": self.object_id,
            "label": self.label,
            "depth_m": self.depth_m,
            "camera_xyz": list(self.camera_xyz) if self.camera_xyz else None,
            "map_xyz": list(self.map_xyz) if self.map_xyz else None,
            "bearing_deg": self.bearing_deg,
            "spatial_quality": self.spatial_quality,
            "confidence": self.confidence,
            "observation_count": self.observation_count,
            "first_seen": self.first_seen,
            "last_seen": self.last_seen,
            "seen_from_places": self.seen_from_places,
            "negative_evidence": self.negative_evidence,
            "provenance": self.provenance,
            "source_observation_ids": self.source_observation_ids,
            "merge_history": self.merge_history,
            "association_score": self.association_score,
            "status": self.status,
            "position_mean_xyz": list(self.position_mean_xyz) if self.position_mean_xyz else None,
            "position_variance_xyz": list(self.position_variance_xyz) if self.position_variance_xyz else None,
            "map_observation_count": self.map_observation_count,
        }


class SemanticObjectMap:
    def __init__(
        self,
        *,
        merge_distance_m: float = 0.4,
        label_similarity: bool = True,
        confirm_min_observations: int = 2,
        stale_after_seconds: float = 120.0,
        geometry_weight: float = 0.60,
        semantic_weight: float = 0.15,
        place_weight: float = 0.10,
        appearance_weight: float = 0.10,
        temporal_weight: float = 0.05,
    ) -> None:
        self.merge_distance_m = float(merge_distance_m)
        self.label_similarity = bool(label_similarity)
        self.confirm_min_observations = max(1, int(confirm_min_observations))
        self.stale_after_seconds = float(stale_after_seconds)
        self.weights = {
            "geometry": float(geometry_weight),
            "semantic": float(semantic_weight),
            "place": float(place_weight),
            "appearance": float(appearance_weight),
            "temporal": float(temporal_weight),
        }
        self.objects: dict[str, SemanticObjectEntry] = {}
        self._next_id = 1
        self._recent_source_ids: dict[str, str] = {}
        self._rejected_pairs_cache: list[dict[str, Any]] = []

    def update(
        self,
        spatial_objects: list[ObjectSpatialObservation],
        *,
        place_id: str | None = None,
        now: float | None = None,
    ) -> list[str]:
        """Backwards-compatible update; returns newly created object ids."""
        result = self.update_with_associations(
            spatial_objects, place_id=place_id, now=now
        )
        return list(result.created_ids)

    def update_with_associations(
        self,
        spatial_objects: list[ObjectSpatialObservation],
        *,
        place_id: str | None = None,
        now: float | None = None,
        frame_id: str | None = None,
    ) -> SemanticMapUpdateResult:
        """Update observed objects with one-to-one data association.

        Returns a rich result with created/updated ids, associations and
        rejected-pair debug information.
        """
        now = now if now is not None else time.time()
        result = SemanticMapUpdateResult()
        self._rejected_pairs_cache = []
        if not spatial_objects:
            return result
        # One-to-one greedy assignment between detections and persistent
        # entities.  Each entity can accept at most one detection per frame,
        # and each detection can merge into at most one entity.
        candidates: list[tuple[float, int, str, SemanticObjectEntry, float]] = []
        for index, obs in enumerate(spatial_objects):
            if place_id:
                obs.provenance = {**obs.provenance, "place_id": place_id}
            if frame_id:
                obs.provenance = {**obs.provenance, "frame_id": frame_id}
            candidate = self._best_candidate(obs, now=now, observation_index=index)
            if candidate is not None:
                entry, score, distance, reasons = candidate
                candidates.append((score, index, entry.object_id, entry, distance))
        result.rejected_pairs.extend(self._rejected_pairs_cache)
        candidates.sort(key=lambda item: item[0], reverse=True)
        assigned_entities: set[str] = set()
        assigned_indices: set[int] = set()
        assignments: dict[int, tuple[str, SemanticObjectEntry, float, list[str]]] = {}
        for score, index, entity_id, entry, distance in candidates:
            if index in assigned_indices:
                continue
            if entity_id in assigned_entities:
                # Another detection claimed this entity this frame.
                result.rejected_pairs.append(
                    {
                        "observation_index": index,
                        "persistent_object_id": entity_id,
                        "decision": "ONE_TO_ONE_REJECTED",
                        "reason": "entity already assigned to another detection in this frame",
                        "association_score": round(score, 4),
                    }
                )
                continue
            assigned_entities.add(entity_id)
            assigned_indices.add(index)
            assignments[index] = (entity_id, entry, distance, [])
        # Reconstruct reasons for assigned items.
        assignments = {
            index: (entity_id, entry, distance, reasons)
            for index, (entity_id, entry, distance, _) in assignments.items()
            for obs in [spatial_objects[index]]
            for reasons in [self._candidate_reasons_for(obs, entry, distance, now)]
        }
        for index, obs in enumerate(spatial_objects):
            if index in assignments:
                entity_id, entry, distance, reasons = assignments[index]
                self._merge(entry, obs, place_id=place_id, now=now,
                            association_score=self._association_score(obs, entry, distance),
                            reasons=reasons)
                result.updated_ids.append(entity_id)
                result.associations.append(
                    SemanticAssociation(
                        observation_index=index,
                        source_object_id=obs.object_id,
                        persistent_object_id=entity_id,
                        action=ACTION_MERGED,
                        association_score=self._association_score(obs, entry, distance),
                        distance_m=distance,
                        reasons=reasons,
                    )
                )
            else:
                object_id = f"obj_{self._next_id:03d}"
                self._next_id += 1
                self.objects[object_id] = SemanticObjectEntry(
                    object_id=object_id,
                    label=obs.label,
                    depth_m=obs.depth_m,
                    camera_xyz=obs.camera_xyz,
                    map_xyz=obs.map_xyz,
                    bearing_deg=obs.bearing_deg,
                    spatial_quality=obs.spatial_quality,
                    confidence=obs.confidence,
                    observation_count=1,
                    first_seen=now,
                    last_seen=now,
                    seen_from_places=[place_id] if place_id else [],
                    provenance=dict(obs.provenance),
                    source_observation_ids=_source_ids(obs),
                    status=ENTITY_TENTATIVE,
                    position_mean_xyz=obs.map_xyz,
                    position_variance_xyz=(0.0, 0.0, 0.0),
                    map_observation_count=1 if obs.map_xyz is not None else 0,
                )
                result.created_ids.append(object_id)
                result.associations.append(
                    SemanticAssociation(
                        observation_index=index,
                        source_object_id=obs.object_id,
                        persistent_object_id=object_id,
                        action=ACTION_CREATED,
                        association_score=0.0,
                        distance_m=None,
                        reasons=["no candidate passed hard gates"],
                    )
                )
        self._reconcile_statuses(now=now)
        return result

    def _best_candidate(
        self,
        obs: ObjectSpatialObservation,
        *,
        now: float,
        observation_index: int | None = None,
    ) -> tuple[SemanticObjectEntry, float, float, list[str]] | None:
        best: SemanticObjectEntry | None = None
        best_score = -1.0
        best_dist: float | None = None
        best_reasons: list[str] = []
        threshold = dynamic_merge_distance_m(
            label=obs.label,
            base=self.merge_distance_m,
            depth_m=obs.depth_m,
            confidence=obs.confidence,
        )
        for entry in self.objects.values():
            dist = self._pair_distance(entry, obs)
            if dist is None or dist > threshold:
                if dist is not None:
                    self._rejected_pairs_cache.append(
                        {
                            "observation_index": observation_index,
                            "persistent_object_id": entry.object_id,
                            "decision": "HARD_GATE_DISTANCE",
                            "distance_m": round(dist, 4),
                            "threshold_m": round(threshold, 4),
                        }
                    )
                continue
            if not self._hard_gate(entry, obs):
                self._rejected_pairs_cache.append(
                    {
                        "observation_index": observation_index,
                        "persistent_object_id": entry.object_id,
                        "decision": "HARD_GATE",
                        "reason": "label/frame incompatibility",
                    }
                )
                continue
            score, reasons = self._association_score_detail(entry, obs, dist)
            if score > best_score:
                best = entry
                best_score = score
                best_dist = dist
                best_reasons = reasons
        if best is None:
            return None
        return best, best_score, best_dist or 0.0, best_reasons

    def _hard_gate(self, entry: SemanticObjectEntry, obs: ObjectSpatialObservation) -> bool:
        # Semantic gate: same label/category by default.
        if self.label_similarity and entry.label != obs.label:
            return False
        # Frame compatibility.  World (map_xyz) coordinates are comparable
        # across frames, so the frame gate only applies to camera-local /
        # RGB_ONLY observations where raw camera coordinates are frame-locked.
        if entry.map_xyz is None or obs.map_xyz is None:
            entry_frame = entry.provenance.get("map_frame") or entry.provenance.get("frame_id")
            obs_frame = obs.provenance.get("map_frame") or obs.provenance.get("frame_id")
            if entry_frame and obs_frame and entry_frame != obs_frame:
                return False
        return True

    def _pair_distance(
        self,
        entry: SemanticObjectEntry,
        obs: ObjectSpatialObservation,
    ) -> float | None:
        if obs.map_xyz is not None and entry.map_xyz is not None:
            return math.dist(obs.map_xyz, entry.map_xyz)
        # Camera-local coordinates only when the frame is explicitly shared.
        if (
            obs.camera_xyz is not None
            and entry.camera_xyz is not None
            and _same_camera_frame(entry, obs)
        ):
            return math.dist(obs.camera_xyz, entry.camera_xyz)
        # Same-place bearing/depth weak evidence.
        if (
            obs.bearing_deg is not None
            and entry.bearing_deg is not None
            and obs.depth_m is not None
            and entry.depth_m is not None
            and _same_place(entry, obs)
        ):
            return math.hypot(
                abs(float(obs.bearing_deg) - float(entry.bearing_deg)) / 90.0,
                abs(float(obs.depth_m) - float(entry.depth_m)),
            )
        return None

    def _association_score(
        self, obs: ObjectSpatialObservation, entry: SemanticObjectEntry, distance: float | None
    ) -> float:
        detail, _ = self._association_score_detail(entry, obs, distance)
        return detail

    def _association_score_detail(
        self, entry: SemanticObjectEntry, obs: ObjectSpatialObservation, distance: float | None
    ) -> tuple[float, list[str]]:
        scores: dict[str, float] = {}
        reasons: list[str] = []
        if distance is not None and (obs.map_xyz is not None or entry.map_xyz is not None):
            # Geometry score normalized by a generous max merge radius.
            max_dist = max(0.5, dynamic_merge_distance_m(
                label=obs.label, base=self.merge_distance_m, depth_m=obs.depth_m,
                confidence=obs.confidence,
            ))
            scores["geometry"] = max(0.0, 1.0 - distance / max_dist)
            reasons.append(f"geometry={scores['geometry']:.2f} dist={distance:.2f}m")
        elif distance is not None:
            scores["geometry"] = 0.5
            reasons.append("camera-local/place-weak geometry")
        else:
            scores["geometry"] = 0.0
        scores["semantic"] = 1.0 if self._semantic_compatible(entry, obs) else 0.0
        if scores["semantic"] > 0:
            reasons.append("semantic=1.0")
        scores["place"] = (
            1.0 if _same_place(entry, obs) else (0.5 if obs.map_xyz is None else 0.0)
        )
        if scores["place"] > 0:
            reasons.append(f"place={scores['place']:.2f}")
        # No heavy appearance model; when appearance is unavailable it is
        # redistributed over the available terms.
        scores["appearance"] = 0.0
        scores["temporal"] = 1.0 if (
            entry.last_seen > 0 and obs.provenance.get("timestamp")
        ) else 0.5
        total_weight = sum(
            self.weights[key] for key in ("geometry", "semantic", "place", "appearance", "temporal")
        )
        score = sum(scores[key] * self.weights[key] for key in scores) / max(total_weight, 1e-9)
        return max(0.0, min(1.0, score)), reasons

    def _semantic_compatible(self, entry: SemanticObjectEntry, obs: ObjectSpatialObservation) -> bool:
        if not self.label_similarity:
            return True
        return entry.label == obs.label

    def _rejected_pair_debug(
        self, obs: ObjectSpatialObservation, index: int, distance: float | None
    ) -> list[dict[str, Any]]:
        return list(getattr(self, "_rejected_pairs_cache", []))

    def _candidate_reasons_for(
        self,
        obs: ObjectSpatialObservation,
        entry: SemanticObjectEntry,
        distance: float,
        now: float,
    ) -> list[str]:
        score, reasons = self._association_score_detail(entry, obs, distance)
        return reasons

    def _merge(
        self,
        entry: SemanticObjectEntry,
        obs: ObjectSpatialObservation,
        *,
        place_id: str | None,
        now: float,
        association_score: float,
        reasons: list[str] | None = None,
    ) -> None:
        entry.observation_count += 1
        entry.last_seen = now
        entry.association_score = max(entry.association_score, association_score)
        source_id = _source_id(obs)
        if source_id and source_id not in entry.source_observation_ids:
            entry.source_observation_ids.append(source_id)
        entry.merge_history.append(
            {
                "timestamp": now,
                "source_observation_id": source_id,
                "association_score": association_score,
                "association_reasons": reasons or [],
                "action": ACTION_MERGED,
            }
        )
        # Position fusion: never blindly overwrite; use quality-weighted mean.
        if obs.map_xyz is not None:
            prev_mean = entry.position_mean_xyz or entry.map_xyz
            prev_weight = quality_weight(entry.spatial_quality)
            new_weight = quality_weight(obs.spatial_quality)
            if new_weight > 0:
                if prev_mean is not None and prev_weight > 0 and entry.map_observation_count > 0:
                    fused = weighted_position_mean(
                        [prev_mean, obs.map_xyz],
                        [prev_weight * entry.map_observation_count, new_weight],
                    )
                else:
                    fused = obs.map_xyz
                entry.position_mean_xyz = fused
                entry.map_xyz = fused
                entry.map_observation_count += 1
                # Keep a simple variance approximation from the recent samples.
                if entry.map_observation_count >= 2:
                    from app.spatial.spatial_transform import position_variance

                    entry.position_variance_xyz = position_variance(
                        [prev_mean or fused, obs.map_xyz], fused
                    )
            else:
                # CAMERA_LOCAL / RGB_ONLY does not participate in global fusion.
                pass
        else:
            if entry.position_mean_xyz is None and obs.camera_xyz is not None:
                entry.position_mean_xyz = None
        # Camera-local fields still update for the current view.
        if obs.camera_xyz is not None:
            entry.camera_xyz = obs.camera_xyz
        if obs.depth_m is not None:
            entry.depth_m = obs.depth_m
        if obs.bearing_deg is not None:
            entry.bearing_deg = obs.bearing_deg
        # Never downgrade spatial quality.
        if _quality_rank(obs.spatial_quality) > _quality_rank(entry.spatial_quality):
            entry.spatial_quality = obs.spatial_quality
        entry.confidence = max(entry.confidence, obs.confidence)
        if place_id and place_id not in entry.seen_from_places:
            entry.seen_from_places.append(place_id)

    def _reconcile_statuses(self, *, now: float) -> None:
        for entry in self.objects.values():
            if entry.status in {ENTITY_REJECTED}:
                continue
            if entry.last_seen and now - entry.last_seen > self.stale_after_seconds:
                entry.status = ENTITY_STALE
                continue
            if entry.observation_count >= self.confirm_min_observations and (
                entry.map_observation_count >= 2 or entry.observation_count >= 3
            ):
                entry.status = ENTITY_CONFIRMED
            else:
                entry.status = ENTITY_TENTATIVE

    def mark_negative(self, label: str, *, place_id: str | None = None) -> None:
        for entry in self.objects.values():
            if entry.label == label:
                entry.negative_evidence += 1

    def observed_scene_graph(self) -> dict[str, Any]:
        return {
            "nodes": [entry.to_dict() for entry in self.objects.values()],
            "edges": [],
        }

    def summary_stats(self) -> dict[str, Any]:
        confirmed = sum(1 for e in self.objects.values() if e.status == ENTITY_CONFIRMED)
        tentative = sum(1 for e in self.objects.values() if e.status == ENTITY_TENTATIVE)
        merges = sum(len(e.merge_history) for e in self.objects.values())
        return {
            "unique_objects": len(self.objects),
            "confirmed_objects": confirmed,
            "tentative_objects": tentative,
            "entity_merges": merges,
            "entity_creations": self._next_id - 1,
        }

    def to_dict(self) -> dict[str, Any]:
        return {
            "objects": [entry.to_dict() for entry in self.objects.values()],
            "merge_distance_m": self.merge_distance_m,
        }


def _quality_rank(value: str) -> int:
    order = {
        "RGB_ONLY": 0,
        "CAMERA_LOCAL": 1,
        "RELATIVE_RGBD": 2,
        "METRIC_RGBD": 3,
    }
    return order.get(value, 0)


def _source_id(obs: ObjectSpatialObservation) -> str | None:
    value = obs.provenance.get("observation_id") or obs.provenance.get("frame_id")
    return str(value) if value else None


def _source_ids(obs: ObjectSpatialObservation) -> list[str]:
    value = _source_id(obs)
    return [value] if value else []


def _same_camera_frame(entry: SemanticObjectEntry, obs: ObjectSpatialObservation) -> bool:
    left = entry.provenance.get("frame_id")
    right = obs.provenance.get("frame_id")
    return bool(left and right and left == right)


def _same_place(entry: SemanticObjectEntry, obs: ObjectSpatialObservation) -> bool:
    left = entry.provenance.get("place_id")
    right = obs.provenance.get("place_id")
    return bool(left and right and left == right)