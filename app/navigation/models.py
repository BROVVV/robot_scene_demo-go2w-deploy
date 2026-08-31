"""Shared models for video-to-navigation planning and live exploration."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Any


SCALE_METRIC = "metric"
SCALE_RELATIVE = "relative"
SCALE_UNKNOWN = "unknown"


# Goal types of the platform-independent high-level action language.
GOAL_REOBSERVE = "REOBSERVE"
GOAL_ROTATE_VIEW = "ROTATE_VIEW"
GOAL_RELATIVE_MOVE = "RELATIVE_MOVE"
GOAL_NAVIGATE_POSE = "NAVIGATE_POSE"
GOAL_INSPECT_ANCHOR = "INSPECT_ANCHOR"
GOAL_REVISIT_NODE = "REVISIT_NODE"
GOAL_STOP = "STOP"
SUPPORTED_GOAL_TYPES = {
    GOAL_REOBSERVE,
    GOAL_ROTATE_VIEW,
    GOAL_RELATIVE_MOVE,
    GOAL_NAVIGATE_POSE,
    GOAL_INSPECT_ANCHOR,
    GOAL_REVISIT_NODE,
    GOAL_STOP,
}


@dataclass
class ExplorationGoal:
    """Platform-independent high-level exploration action.

    Relative fields (relative_dx / relative_dy / relative_dyaw) are understood
    by relative backends such as the current Go2-W experimental backend; the
    position/yaw + frame fields are used by metric backends.  Only fields that
    match the backend's ``RobotCapabilities`` may be consumed.
    """

    goal_id: str
    goal_type: str

    target_node_id: str | None = None
    position: tuple[float, float] | None = None
    yaw: float | None = None
    frame: str = "odom"

    relative_dx: float | None = None
    relative_dy: float | None = None
    relative_dyaw: float | None = None

    semantic_anchor: str | None = None
    semantic_reason: str = ""
    heading_sector: int | None = None

    expected_information_gain: float = 0.0
    semantic_relevance: float = 0.0
    novelty_score: float = 0.0
    estimated_cost: float = 0.0

    provenance: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.goal_type not in SUPPORTED_GOAL_TYPES:
            raise ValueError(f"unsupported exploration goal type: {self.goal_type}")
        if self.position is not None and len(self.position) != 2:
            raise ValueError("position must be a (x, y) pair")
        if not math.isfinite(float(self.expected_information_gain)):
            raise ValueError("expected_information_gain must be finite")

    def to_dict(self) -> dict[str, Any]:
        return {
            "goal_id": self.goal_id,
            "goal_type": self.goal_type,
            "target_node_id": self.target_node_id,
            "position": list(self.position) if self.position else None,
            "yaw": self.yaw,
            "frame": self.frame,
            "relative_dx": self.relative_dx,
            "relative_dy": self.relative_dy,
            "relative_dyaw": self.relative_dyaw,
            "semantic_anchor": self.semantic_anchor,
            "semantic_reason": self.semantic_reason,
            "heading_sector": self.heading_sector,
            "expected_information_gain": self.expected_information_gain,
            "semantic_relevance": self.semantic_relevance,
            "novelty_score": self.novelty_score,
            "estimated_cost": self.estimated_cost,
            "provenance": self.provenance,
        }

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "ExplorationGoal":
        position = value.get("position")
        return cls(
            goal_id=str(value.get("goal_id") or "goal"),
            goal_type=str(value.get("goal_type") or GOAL_REOBSERVE),
            target_node_id=value.get("target_node_id"),
            position=tuple(position) if position else None,
            yaw=value.get("yaw"),
            frame=str(value.get("frame") or "odom"),
            relative_dx=value.get("relative_dx"),
            relative_dy=value.get("relative_dy"),
            relative_dyaw=value.get("relative_dyaw"),
            semantic_anchor=value.get("semantic_anchor"),
            semantic_reason=str(value.get("semantic_reason") or ""),
            heading_sector=value.get("heading_sector"),
            expected_information_gain=float(value.get("expected_information_gain", 0.0)),
            semantic_relevance=float(value.get("semantic_relevance", 0.0)),
            novelty_score=float(value.get("novelty_score", 0.0)),
            estimated_cost=float(value.get("estimated_cost", 0.0)),
            provenance=dict(value.get("provenance") or {}),
        )


@dataclass
class LiveObservation:
    """Perception payload handed from the live semantic observer to the
    AutonomousExplorer.  Detector details never leak into the explorer."""

    bundle_id: str
    timestamp: float
    image_ref: str | None = None

    # RGB-D atomic frame fields (D435 primary camera)
    depth_ref: str | None = None
    rgbd_frame_id: str | None = None
    intrinsics: dict[str, Any] | None = None
    depth_scale: float | None = None
    depth_aligned_to_rgb: bool = True
    spatial_quality: str = "RGB_ONLY"
    camera_xyz: list[float] | None = None
    map_xyz: list[float] | None = None

    detections: list[dict[str, Any]] = field(default_factory=list)
    scene_graph: dict[str, Any] | None = None
    scene_objects: list[dict[str, Any]] = field(default_factory=list)
    scene_relations: list[dict[str, Any]] = field(default_factory=list)
    target_match: dict[str, Any] | None = None

    pose: dict[str, Any] | None = None
    heading_sector: int | None = None
    sensor_health: dict[str, Any] = field(default_factory=dict)

    provenance: dict[str, Any] = field(default_factory=dict)

    @property
    def object_labels(self) -> list[str]:
        labels: list[str] = []
        for item in self.scene_objects:
            label = str(
                item.get("label_zh") or item.get("label") or item.get("name") or ""
            ).strip()
            if label and label not in labels:
                labels.append(label)
        return labels

    @property
    def target_present(self) -> bool:
        return bool((self.target_match or {}).get("target_present"))

    def to_dict(self) -> dict[str, Any]:
        return {
            "bundle_id": self.bundle_id,
            "timestamp": self.timestamp,
            "image_ref": self.image_ref,
            "depth_ref": self.depth_ref,
            "rgbd_frame_id": self.rgbd_frame_id,
            "intrinsics": self.intrinsics,
            "depth_scale": self.depth_scale,
            "depth_aligned_to_rgb": self.depth_aligned_to_rgb,
            "spatial_quality": self.spatial_quality,
            "camera_xyz": self.camera_xyz,
            "map_xyz": self.map_xyz,
            "detections": self.detections,
            "scene_graph": self.scene_graph,
            "scene_objects": self.scene_objects,
            "scene_relations": self.scene_relations,
            "target_match": self.target_match,
            "pose": self.pose,
            "heading_sector": self.heading_sector,
            "sensor_health": self.sensor_health,
            "provenance": self.provenance,
        }

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "LiveObservation":
        return cls(
            bundle_id=str(value.get("bundle_id") or "bundle"),
            timestamp=float(value.get("timestamp", 0.0)),
            image_ref=value.get("image_ref"),
            depth_ref=value.get("depth_ref"),
            rgbd_frame_id=value.get("rgbd_frame_id"),
            intrinsics=value.get("intrinsics"),
            depth_scale=value.get("depth_scale"),
            depth_aligned_to_rgb=bool(value.get("depth_aligned_to_rgb", True)),
            spatial_quality=str(value.get("spatial_quality") or "RGB_ONLY"),
            camera_xyz=value.get("camera_xyz"),
            map_xyz=value.get("map_xyz"),
            detections=list(value.get("detections") or []),
            scene_graph=value.get("scene_graph"),
            scene_objects=list(value.get("scene_objects") or []),
            scene_relations=list(value.get("scene_relations") or []),
            target_match=value.get("target_match"),
            pose=value.get("pose"),
            heading_sector=value.get("heading_sector"),
            sensor_health=dict(value.get("sensor_health") or {}),
            provenance=dict(value.get("provenance") or {}),
        )


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "video_map"
    source: str = "video_visual_odometry"
    scale_status: str = SCALE_RELATIVE
    provenance: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not all(math.isfinite(float(v)) for v in (self.x, self.y, self.yaw)):
            raise ValueError("Pose2D coordinates must be finite")
        if self.scale_status not in {SCALE_METRIC, SCALE_RELATIVE, SCALE_UNKNOWN}:
            raise ValueError(f"Unsupported scale_status: {self.scale_status}")

    def distance_to(self, other: "Pose2D") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "Pose2D":
        return cls(
            x=float(value.get("x", 0.0)),
            y=float(value.get("y", 0.0)),
            yaw=float(value.get("yaw", value.get("yaw_rad", 0.0))),
            frame_id=str(value.get("frame_id", "video_map")),
            source=str(value.get("source", "video_visual_odometry")),
            scale_status=str(value.get("scale_status", SCALE_RELATIVE)),
            provenance=dict(value.get("provenance") or {}),
        )


@dataclass
class VideoFramePose:
    frame_id: int
    timestamp_sec: float
    pose: Pose2D
    confidence: float = 1.0
    tracking_status: str = "tracked"

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["pose"] = self.pose.to_dict()
        return value


@dataclass
class NavigationWaypoint:
    waypoint_id: str
    pose: Pose2D
    source_frame_id: int | None = None
    semantic_label: str = ""
    waypoint_type: str = "trajectory"
    confidence: float = 1.0
    provenance: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["pose"] = self.pose.to_dict()
        return value


@dataclass
class NavigationPlan:
    plan_id: str
    mode: str
    planning_frame: str
    scale_status: str
    start_pose: Pose2D
    goal_pose: Pose2D | None
    waypoints: list[NavigationWaypoint]
    path: list[Pose2D]
    path_length: float | None
    estimated_time_sec: float | None
    navigation_strategy: str
    target_status: str
    confidence: float
    executable: bool
    executable_reason: str
    provenance: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "plan_id": self.plan_id,
            "mode": self.mode,
            "planning_frame": self.planning_frame,
            "scale_status": self.scale_status,
            "start_pose": self.start_pose.to_dict(),
            "goal_pose": self.goal_pose.to_dict() if self.goal_pose else None,
            "waypoints": [item.to_dict() for item in self.waypoints],
            "path": [item.to_dict() for item in self.path],
            "path_length": self.path_length,
            "estimated_time_sec": self.estimated_time_sec,
            "navigation_strategy": self.navigation_strategy,
            "target_status": self.target_status,
            "confidence": self.confidence,
            "executable": self.executable,
            "executable_reason": self.executable_reason,
            "provenance": self.provenance,
        }


def path_length(path: list[Pose2D]) -> float:
    return sum(a.distance_to(b) for a, b in zip(path, path[1:]))
