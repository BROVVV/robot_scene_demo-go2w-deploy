"""Dataclasses for video full-scene semantic mapping."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


def to_plain(value: Any) -> Any:
    if hasattr(value, "to_dict"):
        return value.to_dict()
    if isinstance(value, list):
        return [to_plain(item) for item in value]
    if isinstance(value, dict):
        return {key: to_plain(item) for key, item in value.items()}
    return value


@dataclass
class FrameObject:
    frame_object_id: str
    label: str
    label_zh: str
    category: str
    bbox: list[float] | None
    mask_area_ratio: float | None
    confidence: float
    position_2d: str
    attributes: list[str] = field(default_factory=list)
    navigation_role: str = "unknown"
    is_obstacle: bool = False
    is_landmark: bool = False
    evidence_type: str = "visual"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class FrameRelation:
    subject_label: str
    object_label: str
    relation: str
    confidence: float = 0.5
    subject_id: str | None = None
    object_id: str | None = None
    description_zh: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class FreeSpaceRegion:
    position_2d: str
    description_zh: str
    confidence: float = 0.5
    frame_region_id: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class HazardRegion:
    type: str
    position_2d: str
    description_zh: str
    confidence: float = 0.5
    frame_region_id: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class FrameObservation:
    frame_id: int
    timestamp_sec: float
    frame_path: str
    scene_type: str | None
    summary_zh: str
    objects: list[FrameObject] = field(default_factory=list)
    relations: list[FrameRelation] = field(default_factory=list)
    free_space: list[FreeSpaceRegion] = field(default_factory=list)
    hazards: list[HazardRegion] = field(default_factory=list)
    raw_model_output: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return to_plain(asdict(self))


@dataclass
class ObjectTrack:
    track_id: str
    label: str
    label_zh: str
    category: str
    navigation_role: str
    first_seen_sec: float
    last_seen_sec: float
    seen_frame_ids: list[int]
    best_frame_id: int | None
    best_frame_path: str | None
    stable_position_2d: str
    representative_bbox: list[float] | None
    confidence: float
    attributes: list[str] = field(default_factory=list)
    nearby_track_ids: list[str] = field(default_factory=list)
    source: str = "observed"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class SceneGraphNode:
    node_id: str
    node_type: str
    label: str
    label_zh: str
    category: str
    source: str
    confidence: float
    evidence_level: str
    based_on: list[str] = field(default_factory=list)
    can_confirm_target: bool = False
    attributes: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class SceneGraphEdge:
    edge_id: str
    source_node_id: str
    target_node_id: str
    relation: str
    source: str
    confidence: float
    evidence_level: str
    reason: str | None = None
    attributes: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class PSGHypothesis:
    hypothesis_id: str
    predicted_node_id: str | None
    predicted_edge_id: str | None
    prediction_zh: str
    prediction_en: str
    based_on: list[str]
    confidence: float
    suggested_observation: str
    risk_level: str
    can_confirm_target: bool = False

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class NextBestView:
    view_id: str
    anchor_node_id: str
    target_node_id: str | None
    action: str
    reason_zh: str
    requires_visual_confirmation: bool
    risk_level: str
    priority: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class SceneGraph:
    nodes: list[SceneGraphNode] = field(default_factory=list)
    edges: list[SceneGraphEdge] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "nodes": [node.to_dict() for node in self.nodes],
            "edges": [edge.to_dict() for edge in self.edges],
        }


@dataclass
class PSGLayer:
    predicted_nodes: list[SceneGraphNode] = field(default_factory=list)
    predicted_edges: list[SceneGraphEdge] = field(default_factory=list)
    hypotheses: list[PSGHypothesis] = field(default_factory=list)
    next_best_views: list[NextBestView] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "predicted_nodes": [node.to_dict() for node in self.predicted_nodes],
            "predicted_edges": [edge.to_dict() for edge in self.predicted_edges],
            "hypotheses": [item.to_dict() for item in self.hypotheses],
            "next_best_views": [item.to_dict() for item in self.next_best_views],
            "warnings": list(self.warnings),
        }
