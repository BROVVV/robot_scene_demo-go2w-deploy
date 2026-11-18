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
from typing import Any, Callable

from app.detectors.vocabulary import LABEL_ZH_FALLBACK
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

LIFECYCLE_ACTIVE = "active"
LIFECYCLE_STALE = "stale"
LIFECYCLE_MISSING_CANDIDATE = "missing_candidate"
LIFECYCLE_REMOVED = "removed"

ACTION_CREATED = "CREATED"
ACTION_MERGED = "MERGED"
ACTION_REJECTED = "REJECTED"
ACTION_DUPLICATE_IGNORED = "DUPLICATE_IGNORED"


def valid_confidence(value: Any) -> float | None:
    """把分数收敛成 [0,1] 内的有限数；非法值返回 None。

    返回 None 表示「这次观测的分数不可用」，调用方应当把它**当作没有这次
    观测**来处理置信度聚合 —— 但实体本身照建、位置照合，因为这是分数坏了，
    不是几何坏了。NaN 会污染一切算术（累计平均一旦吃进 NaN 就永远是 NaN），
    所以必须在这里拦掉。
    """
    try:
        score = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(score):
        return None
    if score < 0.0 or score > 1.0:
        return None
    return score


@dataclass
class SemanticObservationAnchor:
    """可在 loop closure 后重投影的最小原始观测。"""

    timestamp: float
    camera_xyz: tuple[float, float, float]
    confidence: float
    spatial_quality: str
    map_xyz_at_insert: tuple[float, float, float]
    observation_id: str | None = None
    source_frame_id: str | None = None
    image_stamp: float | None = None
    timestamp_quality: str = "unknown"
    source_sensor: str = "rgbd"
    map_id: str | None = None
    map_epoch: str | None = None
    anchor_id: str | None = None
    anchor_graph_revision: int | None = None
    point_in_anchor: tuple[float, float, float] | None = None
    position_covariance: tuple[float, ...] | None = None
    depth_method: str = "unknown"
    extrinsics_id: str | None = None
    transform_valid: bool = False
    position_epoch: int = 0

    def to_dict(self) -> dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "camera_xyz": list(self.camera_xyz),
            "confidence": self.confidence,
            "spatial_quality": self.spatial_quality,
            "map_xyz_at_insert": list(self.map_xyz_at_insert),
            "observation_id": self.observation_id,
            "source_frame_id": self.source_frame_id,
            "image_stamp": self.image_stamp,
            "timestamp_quality": self.timestamp_quality,
            "source_sensor": self.source_sensor,
            "map_id": self.map_id,
            "map_epoch": self.map_epoch,
            "anchor_id": self.anchor_id,
            "anchor_graph_revision": self.anchor_graph_revision,
            "point_in_anchor": (
                list(self.point_in_anchor) if self.point_in_anchor is not None else None
            ),
            "position_covariance": (
                list(self.position_covariance)
                if self.position_covariance is not None else None
            ),
            "depth_method": self.depth_method,
            "extrinsics_id": self.extrinsics_id,
            "transform_valid": self.transform_valid,
            "position_epoch": self.position_epoch,
        }


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
    mobility_class: str | None = None
    mobility_state: str = "pending"
    mobility_reason: str = "待复核"
    mobility_confidence: float = 0.0
    topology_eligible: bool | None = None
    is_hazard: bool = False
    hazard_rule_id: str | None = None
    hazard_label: str | None = None
    hazard_event_id: str | None = None
    hazard_location_mode: str | None = None
    hazard_position_status: str | None = None
    # `label` 是实体关联键与调试字段，`label_zh` 是显示字段。两者可以不同：
    # 未知英文的 label_zh 会兜底成「未分类物体」，而 label 保持原文，
    # 免得几个互不相干的未知类别并成一个实体（计划书 §3）。
    label_zh: str = ""
    category: str = ""
    depth_m: float | None = None
    camera_xyz: tuple[float, float, float] | None = None
    map_xyz: tuple[float, float, float] | None = None
    bearing_deg: float | None = None
    spatial_quality: str = SPATIAL_QUALITY_CAMERA_LOCAL
    # 聚合置信度（累计平均，见 `_merge`）。**不再等于历史最大值** ——
    # 一次偶然高分不该让实体永久可信（计划书 §2.3 第 4 条）。
    confidence: float = 0.0
    # 历史最高分，只用于审计/调试。
    confidence_max: float = 0.0
    # 参与聚合的有效观测数。与 `observation_count` 不同：后者统计合并次数，
    # 其中可能夹着分数非法（NaN/越界）因而被聚合拒绝的那些。
    confidence_observation_count: int = 0
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
    box_size: tuple[float, float, float] | None = None
    box_size_frame: str | None = None
    map_observation_count: int = 0
    observation_anchors: list[SemanticObservationAnchor] = field(default_factory=list)
    lifecycle_state: str = LIFECYCLE_ACTIVE
    last_checked_at: float = 0.0
    missing_observation_count: int = 0
    missing_first_at: float = 0.0
    missing_last_observation_id: str | None = None
    missing_viewpoints: list[tuple[float, float]] = field(default_factory=list)
    position_epoch: int = 0
    position_changed_at: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "object_id": self.object_id,
            "mobility_class": self.mobility_class,
            "mobility_state": self.mobility_state,
            "mobility_reason": self.mobility_reason,
            "mobility_confidence": self.mobility_confidence,
            "topology_eligible": self.topology_eligible,
            "is_hazard": self.is_hazard,
            "hazard_rule_id": self.hazard_rule_id,
            "hazard_label": self.hazard_label,
            "hazard_event_id": self.hazard_event_id,
            "hazard_location_mode": self.hazard_location_mode,
            "hazard_position_status": self.hazard_position_status,
            "label": self.label,
            "label_zh": self.label_zh,
            "category": self.category,
            "depth_m": self.depth_m,
            "camera_xyz": list(self.camera_xyz) if self.camera_xyz else None,
            "map_xyz": list(self.map_xyz) if self.map_xyz else None,
            "bearing_deg": self.bearing_deg,
            "spatial_quality": self.spatial_quality,
            "confidence": self.confidence,
            "confidence_max": self.confidence_max,
            "confidence_observation_count": self.confidence_observation_count,
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
            "box_size": list(self.box_size) if self.box_size else None,
            "box_size_frame": self.box_size_frame,
            "map_observation_count": self.map_observation_count,
            "observation_anchors": [item.to_dict() for item in self.observation_anchors],
            "lifecycle_state": self.lifecycle_state,
            "last_checked_at": self.last_checked_at,
            "missing_observation_count": self.missing_observation_count,
            "missing_first_at": self.missing_first_at,
            "missing_last_observation_id": self.missing_last_observation_id,
            "missing_viewpoints": [list(item) for item in self.missing_viewpoints],
            "position_epoch": self.position_epoch,
            "position_changed_at": self.position_changed_at,
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
        self.revision = 0
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
            for entry, score, distance, reasons in self._candidate_options(
                obs, now=now, observation_index=index
            ):
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
                source_id = _source_id(obs)
                if source_id and source_id in entry.source_observation_ids:
                    # Replaying the exact same RGB-D frame is useful for
                    # idempotent recovery, but it is not a new observation.
                    # Do not refresh freshness, confidence, position, anchors
                    # or lifecycle counters from duplicate evidence.
                    result.associations.append(
                        SemanticAssociation(
                            observation_index=index,
                            source_object_id=obs.object_id,
                            persistent_object_id=entity_id,
                            action=ACTION_DUPLICATE_IGNORED,
                            association_score=self._association_score(
                                obs, entry, distance
                            ),
                            distance_m=distance,
                            reasons=[*reasons, "duplicate source observation ignored"],
                        )
                    )
                    continue
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
                # 首帧的分数也要过同一道有限性/范围门：非法分数当作"没有这次
                # 观测"，实体置信度为 0 且计数为 0，后一次合法观测才成为基准。
                initial = valid_confidence(obs.confidence)
                self.objects[object_id] = SemanticObjectEntry(
                    object_id=object_id,
                    mobility_class=getattr(obs, "mobility_class", None),
                    mobility_state=getattr(obs, "mobility_state", "pending"),
                    mobility_reason=getattr(obs, "mobility_reason", "待复核"),
                    mobility_confidence=getattr(obs, "mobility_confidence", 0.0),
                    topology_eligible=getattr(obs, "topology_eligible", None),
                    is_hazard=bool(getattr(obs, "is_hazard", False)),
                    hazard_rule_id=getattr(obs, "hazard_rule_id", None),
                    hazard_label=getattr(obs, "hazard_label", None),
                    hazard_event_id=getattr(obs, "hazard_event_id", None),
                    hazard_location_mode=getattr(obs, "hazard_location_mode", None),
                    hazard_position_status=getattr(obs, "hazard_position_status", None),
                    label=obs.label,
                    label_zh=getattr(obs, "label_zh", "") or "",
                    category=getattr(obs, "category", "") or "",
                    depth_m=obs.depth_m,
                    camera_xyz=obs.camera_xyz,
                    map_xyz=obs.map_xyz,
                    bearing_deg=obs.bearing_deg,
                    spatial_quality=obs.spatial_quality,
                    confidence=initial if initial is not None else 0.0,
                    confidence_max=initial if initial is not None else 0.0,
                    confidence_observation_count=1 if initial is not None else 0,
                    observation_count=1,
                    first_seen=now,
                    last_seen=now,
                    seen_from_places=[place_id] if place_id else [],
                    provenance=dict(obs.provenance),
                    source_observation_ids=_source_ids(obs),
                    status=ENTITY_TENTATIVE,
                    position_mean_xyz=obs.map_xyz,
                    # One observation has unknown repeatability.  Zero would
                    # incorrectly claim perfect certainty.
                    position_variance_xyz=None,
                    box_size=obs.box_size_camera or obs.box_size_map,
                    box_size_frame=(
                        "camera_optical" if obs.box_size_camera is not None
                        else (str((obs.provenance or {}).get("map_frame") or "map")
                              if obs.box_size_map is not None else None)
                    ),
                    map_observation_count=1 if obs.map_xyz is not None else 0,
                    observation_anchors=_observation_anchors(obs),
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
        if result.created_ids or result.updated_ids:
            self.revision += 1
        return result

    def _best_candidate(
        self,
        obs: ObjectSpatialObservation,
        *,
        now: float,
        observation_index: int | None = None,
    ) -> tuple[SemanticObjectEntry, float, float, list[str]] | None:
        candidates = self._candidate_options(
            obs, now=now, observation_index=observation_index
        )
        return candidates[0] if candidates else None

    def _candidate_options(
        self,
        obs: ObjectSpatialObservation,
        *,
        now: float,
        observation_index: int | None = None,
    ) -> list[tuple[SemanticObjectEntry, float, float, list[str]]]:
        """Return every viable candidate, ordered for one-to-one fallback."""
        options: list[tuple[SemanticObjectEntry, float, float, list[str]]] = []
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
            options.append((entry, score, dist, reasons))
        options.sort(key=lambda item: (-item[1], item[2], item[0].object_id))
        return options

    def _hard_gate(self, entry: SemanticObjectEntry, obs: ObjectSpatialObservation) -> bool:
        # Semantic gate: same label/category by default.
        if self.label_similarity and not _labels_compatible(entry, obs):
            return False
        # map_xyz 只有在同一世界坐标系里才可比较；pslam_map 与退化路径的
        # pslam_odom 即使数值接近，也不能融合成同一个实体。
        if entry.map_xyz is not None and obs.map_xyz is not None:
            entry_map_frame = entry.provenance.get("map_frame")
            obs_map_frame = obs.provenance.get("map_frame")
            if entry_map_frame and obs_map_frame and entry_map_frame != obs_map_frame:
                return False
            for identity_key in ("map_id", "map_epoch"):
                old_identity = str(entry.provenance.get(identity_key) or "")
                new_identity = str((obs.provenance or {}).get(identity_key) or "")
                if old_identity and new_identity and old_identity != new_identity:
                    return False
        else:
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
        return _labels_compatible(entry, obs)

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
        entry.lifecycle_state = LIFECYCLE_ACTIVE
        entry.last_checked_at = now
        entry.missing_observation_count = 0
        entry.missing_first_at = 0.0
        entry.missing_last_observation_id = None
        entry.missing_viewpoints.clear()
        for name in ("mobility_class", "mobility_state", "mobility_reason",
                     "mobility_confidence", "topology_eligible", "is_hazard",
                     "hazard_rule_id", "hazard_label", "hazard_event_id",
                     "hazard_location_mode", "hazard_position_status"):
            if hasattr(obs, name):
                setattr(entry, name, getattr(obs, name))
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
                if (entry.mobility_class == "semi_static" and prev_mean is not None
                        and entry.map_observation_count > 0):
                    # A movable but currently stationary object follows recent
                    # evidence.  Unlimited historical averaging would leave a
                    # moved chair midway between its old and new positions.
                    fused = tuple(
                        0.7 * float(old) + 0.3 * float(new)
                        for old, new in zip(prev_mean, obs.map_xyz)
                    )
                elif prev_mean is not None and prev_weight > 0 and entry.map_observation_count > 0:
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
                anchor = _observation_anchor(obs)
                if anchor is not None:
                    anchor.position_epoch = entry.position_epoch
                    entry.observation_anchors.append(anchor)
                    entry.observation_anchors = entry.observation_anchors[-32:]
            else:
                # CAMERA_LOCAL / RGB_ONLY does not participate in global fusion.
                pass
        else:
            if entry.position_mean_xyz is None and obs.camera_xyz is not None:
                entry.position_mean_xyz = None
        # Box size EMA（计划书 §11.2 / §25：new = 0.7*old + 0.3*observed）。
        observed_size = obs.box_size_camera or obs.box_size_map
        if observed_size is not None:
            observed_frame = (
                "camera_optical" if obs.box_size_camera is not None
                else str((obs.provenance or {}).get("map_frame") or "map")
            )
            # Never average axis-aligned extents expressed in different axes.
            if entry.box_size_frame not in {None, observed_frame}:
                observed_size = None
            elif entry.box_size_frame is None:
                entry.box_size_frame = observed_frame
        if observed_size is not None:
            if entry.box_size is None:
                entry.box_size = tuple(float(v) for v in observed_size)
            else:
                entry.box_size = tuple(
                    0.7 * float(old) + 0.3 * float(new)
                    for old, new in zip(entry.box_size, observed_size)
                )
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
        # 置信度取累计平均，不再取历史最大值（计划书 §6 阶段 E）。
        # 老写法 `max(entry.confidence, obs.confidence)` 让一次偶然高分永久
        # 成为实体置信度，之后再多低分重复观测也拉不下来 —— 2026-09-17 那轮
        # 41 个持久 ID 里 30 个低于 0.35，实体置信度却照样虚高。
        # 显示门限另有一套组合判据（快照层），平均值只是它的输入之一。
        score = valid_confidence(obs.confidence)
        if score is not None:
            count = entry.confidence_observation_count
            entry.confidence = (entry.confidence * count + score) / (count + 1)
            entry.confidence_observation_count = count + 1
            entry.confidence_max = max(entry.confidence_max, score)
        # 中文显示名/类别只在**新值明确有效**时更新：空串和「未分类物体」
        # 是兜底值，不是知识，拿它顶掉已有的「音箱」就是信息损失。
        new_zh = str(getattr(obs, "label_zh", "") or "").strip()
        if new_zh and new_zh != LABEL_ZH_FALLBACK:
            entry.label_zh = new_zh
        new_category = str(getattr(obs, "category", "") or "").strip()
        if new_category and new_category != "unknown":
            entry.category = new_category
        if place_id and place_id not in entry.seen_from_places:
            entry.seen_from_places.append(place_id)

    def reproject_observations(
        self,
        transform_lookup: Callable[
            [tuple[float, float, float], float],
            tuple[float, float, float] | None,
        ],
        *,
        anchor_lookup: Callable[..., tuple[float, float, float] | None] | None = None,
    ) -> dict[str, int]:
        """重投影原始观测并重算实体位置，persistent object ID 保持不变。"""
        reprojected_objects = 0
        reprojected_observations = 0
        for entry in self.objects.values():
            if entry.lifecycle_state == LIFECYCLE_REMOVED:
                continue
            points: list[tuple[float, float, float]] = []
            weights: list[float] = []
            for anchor in entry.observation_anchors:
                if anchor.position_epoch != entry.position_epoch:
                    continue
                mapped = None
                if (
                    callable(anchor_lookup)
                    and anchor.anchor_id is not None
                    and anchor.point_in_anchor is not None
                ):
                    mapped = anchor_lookup(
                        anchor.anchor_id,
                        anchor.point_in_anchor,
                        map_id=anchor.map_id,
                        map_epoch=anchor.map_epoch,
                    )
                if mapped is None:
                    mapped = transform_lookup(anchor.camera_xyz, anchor.timestamp)
                if mapped is None:
                    anchor.transform_valid = False
                    continue
                anchor.transform_valid = True
                anchor.map_xyz_at_insert = tuple(float(v) for v in mapped)
                points.append(anchor.map_xyz_at_insert)
                weights.append(
                    quality_weight(anchor.spatial_quality)
                    * max(0.05, float(anchor.confidence))
                )
            if not points:
                continue
            fused = weighted_position_mean(points, weights)
            entry.map_xyz = fused
            entry.position_mean_xyz = fused
            entry.map_observation_count = len(points)
            if len(points) >= 2:
                from app.spatial.spatial_transform import position_variance

                entry.position_variance_xyz = position_variance(points, fused)
            else:
                entry.position_variance_xyz = None
            entry.provenance = {
                **entry.provenance,
                "semantic_reprojected": True,
            }
            reprojected_objects += 1
            reprojected_observations += len(points)
        if reprojected_observations:
            self.revision += 1
        return {
            "objects": reprojected_objects,
            "observations": reprojected_observations,
        }

    def _reconcile_statuses(self, *, now: float) -> None:
        for entry in self.objects.values():
            if entry.status in {ENTITY_REJECTED} or entry.lifecycle_state == LIFECYCLE_REMOVED:
                continue
            if (entry.mobility_class != "fixed" and entry.last_seen
                    and now - entry.last_seen > self.stale_after_seconds):
                entry.status = ENTITY_STALE
                if entry.lifecycle_state == LIFECYCLE_ACTIVE:
                    entry.lifecycle_state = LIFECYCLE_STALE
                continue
            if entry.observation_count >= self.confirm_min_observations and (
                entry.map_observation_count >= 2 or entry.observation_count >= 3
            ):
                entry.status = ENTITY_CONFIRMED
            else:
                entry.status = ENTITY_TENTATIVE

    def record_missing(
        self,
        object_id: str,
        *,
        observation_id: str,
        timestamp: float,
        observer_xy: tuple[float, float] | None = None,
        semi_static_required: int = 3,
        semi_static_span_s: float = 2.0,
        fixed_required: int = 5,
        fixed_min_viewpoints: int = 2,
    ) -> str | None:
        """Record one validated, ID-specific absence observation."""
        entry = self.objects.get(str(object_id))
        if entry is None or entry.lifecycle_state == LIFECYCLE_REMOVED:
            return None
        if observation_id and entry.missing_last_observation_id == observation_id:
            return entry.lifecycle_state
        entry.last_checked_at = float(timestamp)
        entry.missing_last_observation_id = str(observation_id or "") or None
        if entry.missing_observation_count == 0:
            entry.missing_first_at = float(timestamp)
        entry.missing_observation_count += 1
        if observer_xy is not None:
            point = (float(observer_xy[0]), float(observer_xy[1]))
            if not any(math.dist(point, old) < 0.5 for old in entry.missing_viewpoints):
                entry.missing_viewpoints.append(point)
        entry.lifecycle_state = LIFECYCLE_MISSING_CANDIDATE
        span = float(timestamp) - entry.missing_first_at
        if entry.mobility_class == "fixed":
            remove = (
                entry.missing_observation_count >= max(1, int(fixed_required))
                and len(entry.missing_viewpoints) >= max(1, int(fixed_min_viewpoints))
            )
        else:
            remove = (
                entry.missing_observation_count >= max(1, int(semi_static_required))
                and span >= max(0.0, float(semi_static_span_s))
            )
        if remove:
            entry.lifecycle_state = LIFECYCLE_REMOVED
            entry.topology_eligible = False
            entry.provenance = {
                **entry.provenance,
                "lifecycle_reason": "validated_absence_at_last_known_position",
                "removed_at": float(timestamp),
            }
        self.revision += 1
        return entry.lifecycle_state

    def start_position_epoch(
        self,
        object_id: str,
        new_position: tuple[float, float, float],
        *,
        timestamp: float,
        reason: str = "confirmed_position_change",
    ) -> bool:
        """Move a durable identity to a new stable segment without averaging history."""
        entry = self.objects.get(str(object_id))
        if entry is None or _finite_position(new_position) is None:
            return False
        position = tuple(float(value) for value in new_position)
        entry.position_epoch += 1
        entry.position_changed_at = float(timestamp)
        entry.map_xyz = position
        entry.position_mean_xyz = position
        entry.position_variance_xyz = None
        entry.map_observation_count = 1
        entry.lifecycle_state = LIFECYCLE_ACTIVE
        entry.topology_eligible = entry.mobility_class in {"fixed", "semi_static"}
        entry.provenance = {
            **entry.provenance,
            "position_change_reason": str(reason),
            "position_epoch": entry.position_epoch,
        }
        self.revision += 1
        return True

    def mark_negative(self, label: str, *, place_id: str | None = None) -> None:
        changed = False
        for entry in self.objects.values():
            if entry.label == label:
                entry.negative_evidence += 1
                changed = True
        if changed:
            self.revision += 1

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
            "schema_version": "semantic_object_map_v2",
            "semantic_revision": self.revision,
            "objects": [entry.to_dict() for entry in self.objects.values()],
            "merge_distance_m": self.merge_distance_m,
        }


def _quality_rank(value: str) -> int:
    order = {
        "RGB_ONLY": 0,
        "CAMERA_LOCAL": 1,
        "RELATIVE_RGBD": 2,
        "METRIC_RGBD": 3,
        # 2026-09-09 P2 修复：plain_slam provider 返回的等级是 METRIC_LIDAR，
        # 之前不在表里 → order.get(..., 0) 让它的 rank 变成 0，
        # 反而比 CAMERA_LOCAL(1) 还低，于是语义实体的 spatial_quality
        # 永远无法从 CAMERA_LOCAL 升级（"只升不降"逻辑被卡死）。
        "METRIC_LIDAR": 4,
    }
    return order.get(value, 0)


_LABEL_EQUIVALENCE = (
    frozenset({"chair", "office chair", "椅子", "办公椅"}),
    frozenset({"desk", "table", "桌子", "办公桌"}),
    frozenset({"box", "cardboard box", "carton", "箱子", "纸箱"}),
    frozenset({"bin", "trash bin", "trash can", "waste bin", "garbage bin", "垃圾桶"}),
)


def _normalized_label(value: Any) -> str:
    return " ".join(str(value or "").strip().casefold().split())


def _labels_compatible(entry: SemanticObjectEntry, obs: ObjectSpatialObservation) -> bool:
    left = {_normalized_label(entry.label), _normalized_label(entry.label_zh)} - {""}
    right = {
        _normalized_label(getattr(obs, "label", "")),
        _normalized_label(getattr(obs, "label_zh", "")),
    } - {""}
    if left.intersection(right):
        return True
    return any(left.intersection(group) and right.intersection(group)
               for group in _LABEL_EQUIVALENCE)


def _finite_position(value: Any) -> tuple[float, float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        return None
    try:
        result = tuple(float(value[index]) for index in range(3))
    except (TypeError, ValueError):
        return None
    return result if all(math.isfinite(part) for part in result) else None


def _observation_anchor(
    obs: ObjectSpatialObservation,
) -> SemanticObservationAnchor | None:
    provenance = obs.provenance or {}
    timestamp = provenance.get("timestamp")
    if timestamp is None or obs.camera_xyz is None or obs.map_xyz is None:
        return None
    point_in_anchor = provenance.get("point_in_anchor")
    covariance = provenance.get("position_covariance")
    return SemanticObservationAnchor(
        timestamp=float(timestamp),
        camera_xyz=tuple(float(v) for v in obs.camera_xyz),
        confidence=float(obs.confidence),
        spatial_quality=str(obs.spatial_quality),
        map_xyz_at_insert=tuple(float(v) for v in obs.map_xyz),
        observation_id=_source_id(obs),
        source_frame_id=str(provenance.get("source_frame") or provenance.get("frame_id") or "") or None,
        image_stamp=float(timestamp),
        timestamp_quality=str(provenance.get("timestamp_quality") or "unknown"),
        source_sensor=str(provenance.get("source_sensor") or "rgbd"),
        map_id=str(provenance.get("map_id") or "") or None,
        map_epoch=str(provenance.get("map_epoch") or "") or None,
        anchor_id=str(provenance.get("anchor_id") or "") or None,
        anchor_graph_revision=(
            int(provenance["anchor_graph_revision"])
            if provenance.get("anchor_graph_revision") is not None else None
        ),
        point_in_anchor=(
            tuple(float(v) for v in point_in_anchor)
            if isinstance(point_in_anchor, (list, tuple)) and len(point_in_anchor) == 3
            else None
        ),
        position_covariance=(
            tuple(float(v) for v in covariance)
            if isinstance(covariance, (list, tuple)) else None
        ),
        depth_method=str(provenance.get("depth_method") or provenance.get("depth_source") or "unknown"),
        extrinsics_id=str(provenance.get("extrinsics_id") or provenance.get("sensor_profile_id") or "") or None,
        transform_valid=bool(
            provenance.get("transform_valid", provenance.get("anchor_status") == "anchored")
        ),
    )


def _observation_anchors(
    obs: ObjectSpatialObservation,
) -> list[SemanticObservationAnchor]:
    anchor = _observation_anchor(obs)
    return [anchor] if anchor is not None else []


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
