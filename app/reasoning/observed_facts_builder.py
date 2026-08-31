"""Convert visual scene results into compact, observation-only facts."""

from __future__ import annotations

from app.schemas import (
    EvidenceSource,
    ObservedAnchor,
    ObservedSceneFacts,
    SceneAnalysisResult,
    SceneObject,
)


STABLE_ANCHOR_TERMS = {
    "door",
    "wall",
    "floor",
    "desk",
    "table",
    "cabinet",
    "sofa",
    "bed",
    "window",
    "stairs",
    "staircase",
    "corridor",
    "hallway",
    "sign",
    "门",
    "墙",
    "地面",
    "桌",
    "柜",
    "沙发",
    "床",
    "窗",
    "楼梯",
    "走廊",
    "标识",
    "指示牌",
}


def build_observed_scene_facts(
    scene: SceneAnalysisResult,
    target_text: str,
    *,
    searched_regions: list[str] | None = None,
) -> ObservedSceneFacts:
    anchors = [
        ObservedAnchor(
            object_id=obj.id,
            label_zh=obj.name_zh,
            label_en=obj.name,
            bbox=[
                obj.bbox_2d.x1,
                obj.bbox_2d.y1,
                obj.bbox_2d.x2,
                obj.bbox_2d.y2,
            ],
            image_region=infer_image_region(obj),
            confidence=obj.final_score or obj.confidence,
            stable=is_stable_anchor(obj),
            source=infer_object_source(obj),
        )
        for obj in scene.objects
        if obj.visible
    ]
    target_observed = bool(
        scene.target_decision.is_present
        and scene.target_decision.matched_object_ids
    )
    target_evidence = (
        [
            scene.target_decision.match_reason_zh,
            *[
                f"视觉目标对象：{object_id}"
                for object_id in scene.target_decision.matched_object_ids
            ],
        ]
        if target_observed
        else []
    )
    negative_evidence = []
    if not target_observed:
        negative_evidence.append(
            scene.target_decision.match_reason_zh or "当前画面没有视觉确认目标。"
        )
    if searched_regions:
        negative_evidence.extend(f"已观察但未确认目标：{item}" for item in searched_regions)

    return ObservedSceneFacts(
        scene_summary_zh=scene.scene_summary_zh,
        target_text=target_text,
        target_profile=scene.target_profile,
        room_type_guess=_infer_room_type(scene),
        visible_anchors=anchors,
        visible_relations=[
            relation.model_dump(mode="json") for relation in scene.relations
        ],
        target_observed=target_observed,
        target_evidence=target_evidence,
        negative_evidence=negative_evidence,
        searched_regions=searched_regions or [],
    )


def infer_image_region(obj: SceneObject) -> str:
    bbox = obj.bbox_2d
    center_x = (bbox.x1 + bbox.x2) / 2
    center_y = (bbox.y1 + bbox.y2) / 2
    horizontal = "left" if center_x < 0.34 else "right" if center_x > 0.66 else "center"
    vertical = "upper" if center_y < 0.38 else "lower" if center_y > 0.68 else "middle"
    depth = (
        "foreground"
        if obj.position.vertical == "front"
        else "background"
        if obj.position.vertical == "back"
        else "midground"
    )
    return f"{horizontal}/{vertical}/{depth}"


def is_stable_anchor(obj: SceneObject) -> bool:
    text = " ".join([obj.name, obj.name_zh, obj.category, *obj.attributes]).lower()
    return any(term in text for term in STABLE_ANCHOR_TERMS)


def infer_object_source(obj: SceneObject) -> EvidenceSource:
    if obj.crop_verify:
        return EvidenceSource.CROP_VERIFY
    if obj.mask_area_ratio is not None:
        return EvidenceSource.SAM2_MASK
    if obj.detector_score is not None:
        return EvidenceSource.GROUNDED_SAM
    return EvidenceSource.LLM_VISION


def _infer_room_type(scene: SceneAnalysisResult) -> str | None:
    names = " ".join(
        f"{obj.name} {obj.name_zh} {obj.category}" for obj in scene.objects
    ).lower()
    if any(term in names for term in ["desk", "keyboard", "monitor", "办公桌", "键盘"]):
        return "office"
    if any(term in names for term in ["bed", "床"]):
        return "bedroom"
    if any(term in names for term in ["sofa", "沙发"]):
        return "living_room"
    if any(term in names for term in ["corridor", "hallway", "走廊", "doorplate"]):
        return "corridor"
    return None
