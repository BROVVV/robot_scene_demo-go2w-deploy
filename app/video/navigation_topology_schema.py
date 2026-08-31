"""Schema helpers for video navigation topology maps."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


PLACE_NODE = "place"
PASSAGE_NODE = "passage"
FREE_SPACE_NODE = "free_space"
OBSTACLE_NODE = "obstacle"
LANDMARK_NODE = "landmark"
PREDICTED_REGION_NODE = "predicted_region"
PREDICTED_PASSAGE_NODE = "predicted_passage"
ROBOT_POSE_NODE = "robot_pose"

NAVIGATION_NODE_TYPES = {
    PLACE_NODE,
    PASSAGE_NODE,
    FREE_SPACE_NODE,
    OBSTACLE_NODE,
    LANDMARK_NODE,
    PREDICTED_REGION_NODE,
    PREDICTED_PASSAGE_NODE,
    ROBOT_POSE_NODE,
}

NAVIGATION_EDGE_TYPES = {
    "temporal_next",
    "connected_to",
    "contains",
    "observed_in",
    "adjacent_to",
    "near",
    "left_of",
    "right_of",
    "in_front_of",
    "behind",
    "blocks",
    "passable_in",
    "through",
    "may_connect_to",
    "explore_candidate",
}


@dataclass
class PlaceSegment:
    place_id: str
    start_time: float
    end_time: float
    start_frame: int
    end_frame: int
    scene_type: str
    dominant_labels: list[str] = field(default_factory=list)
    passage_candidates: list[str] = field(default_factory=list)
    free_space_hint: str | None = None
    split_reason: str = "initial_segment"
    confidence: float = 0.7
    evidence_frames: list[int] = field(default_factory=list)
    object_track_ids: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class NavigationNode:
    node_id: str
    node_type: str
    label: str
    label_zh: str
    source: str
    confidence: float
    start_time: float | None = None
    end_time: float | None = None
    evidence_frames: list[int] = field(default_factory=list)
    properties: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class NavigationEdge:
    edge_id: str
    from_node: str
    to_node: str
    relation: str
    source: str
    confidence: float
    properties: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "edge_id": self.edge_id,
            "from": self.from_node,
            "to": self.to_node,
            "relation": self.relation,
            "source": self.source,
            "confidence": self.confidence,
            "properties": dict(self.properties),
        }
