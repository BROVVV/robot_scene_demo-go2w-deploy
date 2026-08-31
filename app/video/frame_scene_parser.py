"""Normalize per-frame analysis into full-scene observations."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from app.video.frame_analyzer import FrameAnalyzer
from app.video.models import FrameAnalysisResult, VideoFrame
from app.video.schemas import (
    FrameObject,
    FrameObservation,
    FrameRelation,
    FreeSpaceRegion,
    HazardRegion,
)


FULL_SCENE_TARGET = "full scene semantic mapping"

NAVIGATION_LANDMARK_TERMS = {
    "door",
    "doorway",
    "door frame",
    "room entrance",
    "corridor",
    "hallway",
    "stairs",
    "elevator",
    "exit sign",
    "window",
    "wall",
    "corner",
    "门",
    "门口",
    "走廊",
    "楼梯",
    "电梯",
    "出口",
    "窗户",
    "墙",
}
OBSTACLE_TERMS = {
    "chair",
    "table",
    "box",
    "trash bin",
    "person",
    "cabinet",
    "shelf",
    "bag",
    "backpack",
    "sofa",
    "desk",
    "椅子",
    "桌子",
    "箱子",
    "垃圾桶",
    "人",
    "柜子",
    "架子",
    "包",
    "沙发",
}
FREE_SPACE_TERMS = {
    "floor",
    "path",
    "passage",
    "walkable area",
    "open space",
    "地面",
    "通道",
    "可通行区域",
    "开放空间",
}
PASSAGE_TERMS = {"door", "doorway", "room entrance", "passage", "corridor", "hallway", "门", "门口", "通道", "走廊"}


class FrameSceneParser:
    """Parse sampled video frames into structured full-scene observations."""

    def __init__(
        self,
        detector: str,
        output_dir: str | Path,
        annotate: bool = True,
        mock_path: str | Path = "examples/mock_scene_result.json",
    ) -> None:
        self.detector = detector
        self.analyzer = FrameAnalyzer(
            detector=detector,
            target=FULL_SCENE_TARGET,
            output_dir=output_dir,
            annotate=annotate and detector != "mock",
            mock_path=mock_path,
        )

    def parse(self, frame: VideoFrame) -> FrameObservation:
        return observation_from_frame_analysis(self.analyzer.analyze(frame))


def observation_from_frame_analysis(frame: FrameAnalysisResult) -> FrameObservation:
    objects = [_normalize_object(frame, item) for item in frame.objects]
    source_object_ids = {
        str(item.get("source_object_id")): str(item.get("object_id"))
        for item in frame.objects
        if item.get("source_object_id") and item.get("object_id")
    }
    free_space = [
        FreeSpaceRegion(
            frame_region_id=f"frame_{frame.frame_id:06d}_free_{index:03d}",
            position_2d=obj.position_2d,
            description_zh=f"{obj.label_zh} 可作为候选通行区域。",
            confidence=obj.confidence,
        )
        for index, obj in enumerate(objects, start=1)
        if obj.navigation_role == "free_space"
    ]
    hazards = [
        HazardRegion(
            frame_region_id=f"frame_{frame.frame_id:06d}_hazard_{index:03d}",
            type="obstacle",
            position_2d=obj.position_2d,
            description_zh=f"{obj.label_zh} 可能阻挡通行。",
            confidence=obj.confidence,
        )
        for index, obj in enumerate(objects, start=1)
        if obj.is_obstacle
    ]
    relations = [
        _normalize_relation(frame, item, source_object_ids) for item in frame.relations
    ]
    return FrameObservation(
        frame_id=frame.frame_id,
        timestamp_sec=frame.timestamp_sec,
        frame_path=frame.image_path,
        scene_type=_infer_scene_type(frame.scene_summary, objects),
        summary_zh=frame.scene_summary or "当前帧包含若干可见物体和空间结构。",
        objects=objects,
        relations=relations,
        free_space=free_space,
        hazards=hazards,
        raw_model_output=frame.to_dict(),
    )


def _normalize_object(frame: FrameAnalysisResult, obj: dict[str, Any]) -> FrameObject:
    label = str(obj.get("label") or "object").strip() or "object"
    label_zh = str(obj.get("label_zh") or label).strip() or label
    category = _normalize_category(label, label_zh, str(obj.get("category") or "unknown"))
    role = _navigation_role(label, label_zh, category)
    bbox = obj.get("bbox")
    evidence_type = "bbox" if bbox else "visual"
    attributes = [str(item) for item in (obj.get("attributes") or [])]
    color = str(obj.get("color") or "").strip().lower()
    if color and color not in attributes:
        attributes.append(color)
    return FrameObject(
        frame_object_id=str(obj.get("object_id") or f"frame_{frame.frame_id:06d}_{label}"),
        label=label,
        label_zh=label_zh,
        category=category,
        bbox=[float(value) for value in bbox] if isinstance(bbox, list) else None,
        mask_area_ratio=obj.get("mask_area_ratio"),
        confidence=_clamp(obj.get("final_score") or obj.get("confidence") or 0.0),
        position_2d=str(obj.get("image_position") or obj.get("position_2d") or "unknown"),
        attributes=attributes,
        navigation_role=role,
        is_obstacle=role == "obstacle",
        is_landmark=role in {"landmark", "passage", "room_anchor"},
        evidence_type=evidence_type,
    )


def _normalize_relation(
    frame: FrameAnalysisResult,
    relation: dict[str, Any],
    source_object_ids: dict[str, str] | None = None,
) -> FrameRelation:
    source_object_ids = source_object_ids or {}
    subject_id = relation.get("source_id") or relation.get("subject_id")
    object_id = relation.get("target_id") or relation.get("object_id")
    return FrameRelation(
        subject_id=source_object_ids.get(str(subject_id), subject_id),
        object_id=source_object_ids.get(str(object_id), object_id),
        subject_label=str(relation.get("subject_label") or relation.get("source_id") or "unknown"),
        object_label=str(relation.get("object_label") or relation.get("target_id") or "unknown"),
        relation=str(relation.get("relation") or relation.get("relation_type") or "near"),
        confidence=_clamp(relation.get("confidence") or 0.5),
        description_zh=relation.get("description_zh"),
    )


def _normalize_category(label: str, label_zh: str, category: str) -> str:
    text = f"{label} {label_zh}".lower()
    if any(term in text for term in FREE_SPACE_TERMS):
        return "free_space"
    if any(term in text for term in NAVIGATION_LANDMARK_TERMS):
        return "structure"
    if any(term in text for term in OBSTACLE_TERMS):
        return "obstacle"
    if category in {"object", "structure", "region", "free_space", "obstacle", "person", "sign", "unknown"}:
        return category
    return "object"


def _navigation_role(label: str, label_zh: str, category: str) -> str:
    text = f"{label} {label_zh}".lower()
    if any(term in text for term in FREE_SPACE_TERMS) or category == "free_space":
        return "free_space"
    if any(term in text for term in PASSAGE_TERMS):
        return "passage"
    if any(term in text for term in OBSTACLE_TERMS) or category in {"obstacle", "person"}:
        return "obstacle"
    if any(term in text for term in NAVIGATION_LANDMARK_TERMS) or category in {"structure", "sign"}:
        return "landmark"
    if category == "region":
        return "room_anchor"
    return "ordinary_object"


def _infer_scene_type(summary: str, objects: list[FrameObject]) -> str:
    text = " ".join([summary, *(obj.label for obj in objects), *(obj.label_zh for obj in objects)]).lower()
    if "corridor" in text or "hallway" in text or "走廊" in text:
        return "corridor"
    if "office" in text or "办公室" in text:
        return "office"
    if "living" in text or "客厅" in text:
        return "living_room"
    if "outdoor" in text or "室外" in text:
        return "outdoor"
    if "room" in text or "房间" in text:
        return "room"
    return "unknown"


def _clamp(value: Any) -> float:
    try:
        return max(0.0, min(1.0, float(value)))
    except (TypeError, ValueError):
        return 0.0
