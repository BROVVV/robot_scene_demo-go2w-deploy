"""Spatial exploration data models (SemanticNavigation V2 / RGB-D spatial stack).

These models are platform-independent and JSON-safe.  They intentionally
separate observed facts (SemanticObjectMap / PlaceGraph) from predictions
(SemanticPrior / PSG hypotheses).
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

SPATIAL_QUALITY_RGB_ONLY = "RGB_ONLY"
SPATIAL_QUALITY_CAMERA_LOCAL = "CAMERA_LOCAL"
SPATIAL_QUALITY_RELATIVE_RGBD = "RELATIVE_RGBD"
SPATIAL_QUALITY_METRIC_RGBD = "METRIC_RGBD"
SPATIAL_QUALITY_METRIC_LIDAR = "METRIC_LIDAR"

# 计划书 §9.3 / 不变量 3：不同 frame 的坐标严禁裸混。provider 收到与自身
# map frame 不一致的 pose 时抛出该异常（调用方必须 transform 或降级）。
SPATIAL_QUALITY_NO_GLOBAL_POSE = "NO_GLOBAL_SPATIAL_POSE"


class SpatialFrameMismatch(ValueError):
    """A pose/map frame does not match the SpatialProvider's world frame."""

    def __init__(self, pose_frame: str, map_frame: str, detail: str = "") -> None:
        self.pose_frame = str(pose_frame)
        self.map_frame = str(map_frame)
        message = (
            f"SPATIAL_FRAME_MISMATCH: pose_frame={pose_frame} "
            f"map_frame={map_frame}"
        )
        if detail:
            message += f" ({detail})"
        super().__init__(message)

# High-level spatial exploration intents (plan §62)
INTENT_EXPLORE_FRONTIER = "EXPLORE_FRONTIER"
INTENT_INSPECT_ANCHOR_REGION = "INSPECT_ANCHOR_REGION"
INTENT_APPROACH_TARGET = "APPROACH_TARGET"
INTENT_VERIFY_TARGET = "VERIFY_TARGET"
INTENT_REVISIT_PLACE = "REVISIT_PLACE"


@dataclass
class SpatialPose:
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "odom"
    quality: str = SPATIAL_QUALITY_RELATIVE_RGBD
    source: str = "rgbd_odometry"
    provenance: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "SpatialPose":
        return cls(
            x=float(value.get("x", 0.0)),
            y=float(value.get("y", 0.0)),
            yaw=float(value.get("yaw", value.get("yaw_rad", 0.0))),
            frame_id=str(value.get("frame_id", "odom")),
            quality=str(value.get("quality", SPATIAL_QUALITY_RELATIVE_RGBD)),
            source=str(value.get("source", "rgbd_odometry")),
            provenance=dict(value.get("provenance") or {}),
        )


@dataclass
class SpatialMapSnapshot:
    revision: int
    resolution_m: float = 0.05
    origin: tuple[float, float] = (0.0, 0.0)
    width: int = 0
    height: int = 0
    free: list[tuple[int, int]] = field(default_factory=list)
    occupied: list[tuple[int, int]] = field(default_factory=list)
    unknown: list[tuple[int, int]] = field(default_factory=list)
    quality: str = SPATIAL_QUALITY_RELATIVE_RGBD
    source: str = "lightweight_depth_bev"
    provenance: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        # Keep JSON compact: coordinates are int tuples already.
        return value


@dataclass
class FrontierCandidate:
    frontier_id: str
    position: tuple[float, float] | None = None
    frame: str = "odom"
    bearing_deg: float | None = None
    distance_m: float | None = None
    size_score: float = 0.0
    spatial_information_gain: float = 0.0
    reachable: bool = True
    nearby_semantics: list[str] = field(default_factory=list)
    provenance: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["position"] = list(self.position) if self.position else None
        return value

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "FrontierCandidate":
        pos = value.get("position")
        return cls(
            frontier_id=str(value.get("frontier_id") or ""),
            position=tuple(pos) if pos else None,
            frame=str(value.get("frame", "odom")),
            bearing_deg=value.get("bearing_deg"),
            distance_m=value.get("distance_m"),
            size_score=float(value.get("size_score", 0.0)),
            spatial_information_gain=float(value.get("spatial_information_gain", 0.0)),
            reachable=bool(value.get("reachable", True)),
            nearby_semantics=list(value.get("nearby_semantics") or []),
            provenance=dict(value.get("provenance") or {}),
        )


@dataclass
class PlaceNode:
    place_id: str
    pose: SpatialPose | None = None
    pose_quality: str = SPATIAL_QUALITY_RELATIVE_RGBD
    observation_ids: list[str] = field(default_factory=list)
    heading_coverage: dict[str, int] = field(default_factory=dict)
    observed_object_ids: list[str] = field(default_factory=list)
    observed_object_labels: list[str] = field(default_factory=list)
    semantic_interest: float = 0.0
    visit_count: int = 0
    negative_evidence: int = 0
    target_candidate: bool = False
    target_confirmed: bool = False
    revisited: bool = False
    pose_observation_count: int = 0
    pose_mean: SpatialPose | None = None
    last_pose_update: float | None = None
    scene_type: str = ""
    memory_summary: str = ""
    frontier_ids: list[str] = field(default_factory=list)
    provenance: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["pose"] = self.pose.to_dict() if self.pose else None
        value["pose_mean"] = self.pose_mean.to_dict() if self.pose_mean else None
        return value


@dataclass
class MovementEdge:
    edge_id: str
    from_place: str
    to_place: str
    requested_goal: dict[str, Any] = field(default_factory=dict)
    executed_local_actions: list[dict[str, Any]] = field(default_factory=list)
    observed_displacement_m: float | None = None
    observed_yaw_delta_deg: float | None = None
    navigation_result: str = "succeeded"
    success_count: int = 0
    failure_count: int = 0
    blocked_count: int = 0
    recovery_count: int = 0
    last_success_at: float | None = None
    last_failure_at: float | None = None
    status: str = "OPEN"
    last_failure_reason: str = ""
    traversability_score: float = 1.0
    cost: float = 1.0
    provenance: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class SemanticRegion:
    region_id: str
    anchor_object_id: str
    relation: str = "near"
    center: tuple[float, float] | None = None
    radius_min_m: float | None = None
    radius_max_m: float | None = None
    bearing_range_deg: tuple[float, float] | None = None
    confidence: float = 0.5
    metric_claim: bool = False
    source: str = "psg"
    negative_count: int = 0
    state: str = "PREDICTED"

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["center"] = list(self.center) if self.center else None
        value["bearing_range_deg"] = list(self.bearing_range_deg) if self.bearing_range_deg else None
        return value


@dataclass
class SemanticPrior:
    predicted_nodes: list[dict[str, Any]] = field(default_factory=list)
    predicted_relations: list[dict[str, Any]] = field(default_factory=list)
    anchor_hypotheses: list[dict[str, Any]] = field(default_factory=list)
    region_hypotheses: list[SemanticRegion] = field(default_factory=list)
    frontier_scores: dict[str, float] = field(default_factory=dict)
    confidence: float = 0.5
    provenance: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "predicted_nodes": self.predicted_nodes,
            "predicted_relations": self.predicted_relations,
            "anchor_hypotheses": self.anchor_hypotheses,
            "region_hypotheses": [item.to_dict() for item in self.region_hypotheses],
            "frontier_scores": self.frontier_scores,
            "confidence": self.confidence,
            "provenance": self.provenance,
        }


@dataclass
class ExplorationIntent:
    intent_id: str
    intent_type: str
    target_frontier_id: str | None = None
    target_place_id: str | None = None
    target_region: dict[str, Any] | None = None
    target_object_id: str | None = None
    preferred_position: tuple[float, float] | None = None
    preferred_bearing_deg: float | None = None
    semantic_reason: str = ""
    semantic_score: float = 0.0
    psg_score: float = 0.0
    spatial_gain: float = 0.0
    travel_cost: float = 0.0
    provenance: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["preferred_position"] = list(self.preferred_position) if self.preferred_position else None
        return value

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "ExplorationIntent":
        pos = value.get("preferred_position")
        return cls(
            intent_id=str(value.get("intent_id") or ""),
            intent_type=str(value.get("intent_type") or ""),
            target_frontier_id=value.get("target_frontier_id"),
            target_place_id=value.get("target_place_id"),
            target_region=value.get("target_region"),
            target_object_id=value.get("target_object_id"),
            preferred_position=tuple(pos) if pos else None,
            preferred_bearing_deg=value.get("preferred_bearing_deg"),
            semantic_reason=str(value.get("semantic_reason") or ""),
            semantic_score=float(value.get("semantic_score", 0.0)),
            psg_score=float(value.get("psg_score", 0.0)),
            spatial_gain=float(value.get("spatial_gain", 0.0)),
            travel_cost=float(value.get("travel_cost", 0.0)),
            provenance=dict(value.get("provenance") or {}),
        )
