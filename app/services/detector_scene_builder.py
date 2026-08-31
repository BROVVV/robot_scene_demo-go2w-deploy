"""Build scene analysis results from local detector outputs."""

from __future__ import annotations

import math

from app.detectors.base import DetectedObject
from app.schemas import (
    BoundingBox2D,
    Position,
    RoutePlan,
    RouteStep,
    SceneAnalysisResult,
    SceneObject,
    TargetDecision,
    TopologyGraph,
)
from app.services.relation_enricher import enrich_scene_relations


def build_scene_from_detections(
    detections: list[DetectedObject],
    target_text: str,
) -> SceneAnalysisResult:
    objects = [
        _scene_object_from_detection(index, detection)
        for index, detection in enumerate(detections, start=1)
    ]
    draft = SceneAnalysisResult(
        scene_summary_zh=f"本地检测器识别到 {len(objects)} 个候选物体。",
        objects=objects,
        relations=[],
        topology=TopologyGraph(),
        target_decision=TargetDecision(
            target_text=target_text,
            is_present=False,
            matched_object_ids=[],
            match_reason_zh="尚未进行目标匹配。",
            confidence=0.0,
        ),
        route_plan=RoutePlan(
            route_type="explore_likely_location",
            summary_zh="请先完成目标匹配。",
            steps=[
                RouteStep(
                    step_id=1,
                    action="stop",
                    distance_m=None,
                    turn_angle_deg=None,
                    description_zh="停止并重新观察",
                )
            ],
            safety_notes_zh=["本地检测器结果，仅用于 Demo"],
        ),
    )
    enriched = enrich_scene_relations(draft)
    target_decision = _match_target(enriched.objects, target_text)
    route_plan = _build_route_plan(enriched.objects, target_decision)
    summary = (
        f"本地 Grounding DINO/SAM2 检测到 {len(enriched.objects)} 个物体，"
        f"补全 {len(enriched.relations)} 条空间关系。"
    )
    return enriched.model_copy(
        update={
            "scene_summary_zh": summary,
            "target_decision": target_decision,
            "route_plan": route_plan,
        }
    )


def _scene_object_from_detection(index: int, detection: DetectedObject) -> SceneObject:
    x1, y1, x2, y2 = detection.bbox_2d
    center_x = (x1 + x2) / 2
    center_y = (y1 + y2) / 2
    area = max(0.001, (x2 - x1) * (y2 - y1))
    attributes = list(detection.attributes)
    attributes.append(f"detection_source={detection.source}")
    if detection.source_prompt_term:
        attributes.append(f"source_prompt_term={detection.source_prompt_term}")
    if detection.mask_area_ratio is not None:
        attributes.append(f"mask_area_ratio={detection.mask_area_ratio:.4f}")

    return SceneObject(
        id=f"obj_{index:03d}",
        name=detection.label,
        name_zh=detection.label_zh,
        category=detection.category,
        color=detection.color,
        attributes=attributes,
        visible=True,
        position=Position(
            horizontal=_horizontal(center_x),
            vertical=_vertical(center_y),
            relative_to_robot=_relative_to_robot(center_x, center_y),
            estimated_distance_m=_estimate_distance(area, center_y),
        ),
        bbox_2d=BoundingBox2D(x1=x1, y1=y1, x2=x2, y2=y2),
        confidence=detection.score,
        detector_score=detection.score,
        text_score=detection.text_score,
        mask_area_ratio=detection.mask_area_ratio,
    )


def _match_target(objects: list[SceneObject], target_text: str) -> TargetDecision:
    target = target_text.lower()
    matched: list[SceneObject] = []

    if "黄衣服" in target_text or "yellow" in target:
        clothing = [
            obj
            for obj in objects
            if obj.category == "clothing"
            and ((obj.color == "yellow") or ("yellow" in obj.name.lower()) or ("黄" in obj.name_zh))
        ]
        chairs = [obj for obj in objects if "chair" in obj.name.lower() or "椅" in obj.name_zh]
        if clothing and chairs:
            cloth = clothing[0]
            chair = min(chairs, key=lambda obj: _object_distance(obj, cloth))
            matched = [chair, cloth]
            return TargetDecision(
                target_text=target_text,
                is_present=True,
                matched_object_ids=[obj.id for obj in matched],
                match_reason_zh=f"检测到{cloth.name_zh}靠近{chair.name_zh}，符合目标描述。",
                confidence=min(chair.confidence, cloth.confidence),
            )

    if ("绿色" in target_text or "green" in target) and ("底座" in target_text or "base" in target):
        green_objects = [
            obj
            for obj in objects
            if (obj.color == "green") or ("green" in obj.name.lower()) or ("绿" in obj.name_zh)
        ]
        bases = [
            obj
            for obj in objects
            if (obj.color == "black" and ("base" in obj.name.lower() or "底座" in obj.name_zh))
            or ("black base" in obj.name.lower())
            or ("黑色底座" in obj.name_zh)
        ]
        if green_objects and bases:
            green_obj = green_objects[0]
            base = min(bases, key=lambda obj: _object_distance(obj, green_obj))
            return TargetDecision(
                target_text=target_text,
                is_present=True,
                matched_object_ids=[green_obj.id, base.id],
                match_reason_zh=f"检测到{green_obj.name_zh}与{base.name_zh}重叠或相邻，符合目标描述。",
                confidence=min(green_obj.confidence, base.confidence),
            )

    scored = [
        (obj, _target_score(obj, target_text))
        for obj in objects
    ]
    scored = [(obj, score) for obj, score in scored if score > 0]
    scored.sort(key=lambda item: item[1], reverse=True)
    matched = [obj for obj, _ in scored[:2]]
    if matched:
        return TargetDecision(
            target_text=target_text,
            is_present=True,
            matched_object_ids=[obj.id for obj in matched],
            match_reason_zh="检测结果中存在与目标描述匹配的物体。",
            confidence=max(obj.confidence for obj in matched),
        )

    return TargetDecision(
        target_text=target_text,
        is_present=False,
        matched_object_ids=[],
        match_reason_zh="本地检测器未发现与目标描述明确匹配的物体。",
        confidence=0.35,
    )


def _build_route_plan(
    objects: list[SceneObject],
    target_decision: TargetDecision,
) -> RoutePlan:
    if not target_decision.is_present or not target_decision.matched_object_ids:
        return RoutePlan(
            route_type="explore_likely_location",
            summary_zh="当前未确认目标，建议缓慢前进后重新观察。",
            steps=[
                RouteStep(
                    step_id=1,
                    action="move_forward",
                    distance_m=0.5,
                    turn_angle_deg=None,
                    description_zh="缓慢向前走 0.5 米",
                ),
                RouteStep(
                    step_id=2,
                    action="stop",
                    distance_m=None,
                    turn_angle_deg=None,
                    description_zh="停止并重新观察",
                ),
            ],
            safety_notes_zh=["本地检测器估计路线，仅用于 Demo"],
        )

    object_by_id = {obj.id: obj for obj in objects}
    target = object_by_id[target_decision.matched_object_ids[0]]
    steps: list[RouteStep] = []
    if target.position.horizontal == "left":
        steps.append(RouteStep(step_id=1, action="turn_left", distance_m=None, turn_angle_deg=20, description_zh="左转 20 度"))
    elif target.position.horizontal == "right":
        steps.append(RouteStep(step_id=1, action="turn_right", distance_m=None, turn_angle_deg=20, description_zh="右转 20 度"))

    distance = target.position.estimated_distance_m or 1.0
    forward = max(0.3, min(1.2, distance * 0.7))
    steps.append(
        RouteStep(
            step_id=len(steps) + 1,
            action="move_forward",
            distance_m=round(forward, 2),
            turn_angle_deg=None,
            description_zh=f"向前走 {forward:.1f} 米",
        )
    )
    steps.append(
        RouteStep(
            step_id=len(steps) + 1,
            action="stop",
            distance_m=None,
            turn_angle_deg=None,
            description_zh="停止并重新观察",
        )
    )
    return RoutePlan(
        route_type="approach_visible_target",
        summary_zh=f"目标位于{target.position.relative_to_robot}方向，建议保守靠近。",
        steps=steps,
        safety_notes_zh=["本地检测器估计路线，仅用于 Demo"],
    )


def _target_score(obj: SceneObject, target_text: str) -> int:
    score = 0
    haystack = " ".join([obj.name, obj.name_zh, obj.category, obj.color or "", *obj.attributes]).lower()
    for token in [target_text, target_text.lower()]:
        if token and token in haystack:
            score += 3
    zh_rules = {
        "椅": ["chair", "椅"],
        "衣": ["clothing", "衣", "coat", "jacket", "shirt"],
        "桌": ["desk", "table", "桌"],
        "手机": ["phone", "手机"],
        "物体": ["object", "item", "物体"],
        "绿色": ["green", "绿"],
        "黑色": ["black", "黑"],
        "底座": ["base", "底座"],
        "显示器": ["monitor", "显示器"],
        "主机": ["computer case", "computer", "主机"],
        "箱": ["box", "箱"],
        "鞋": ["shoe", "鞋"],
        "杯": ["cup", "杯"],
        "瓶": ["bottle", "瓶"],
        "灭火器": ["fire extinguisher", "extinguisher", "灭火器"],
        "消防器材": ["fire extinguisher", "fire equipment", "消防"],
    }
    for key, values in zh_rules.items():
        if key in target_text and any(value in haystack for value in values):
            score += 2
    if "黄" in target_text and ("yellow" in haystack or "黄" in haystack):
        score += 2
    return score


def _object_distance(left: SceneObject, right: SceneObject) -> float:
    left_center = ((left.bbox_2d.x1 + left.bbox_2d.x2) / 2, (left.bbox_2d.y1 + left.bbox_2d.y2) / 2)
    right_center = ((right.bbox_2d.x1 + right.bbox_2d.x2) / 2, (right.bbox_2d.y1 + right.bbox_2d.y2) / 2)
    return math.dist(left_center, right_center)


def _horizontal(center_x: float) -> str:
    if center_x < 0.33:
        return "left"
    if center_x > 0.66:
        return "right"
    return "center"


def _vertical(center_y: float) -> str:
    if center_y > 0.66:
        return "front"
    if center_y < 0.33:
        return "back"
    return "middle"


def _relative_to_robot(center_x: float, center_y: float) -> str:
    vertical = _vertical(center_y)
    horizontal = _horizontal(center_x)
    if horizontal == "center":
        return vertical
    return f"{vertical}-{horizontal}"


def _estimate_distance(area: float, center_y: float) -> float:
    area_term = 1.0 / max(0.2, math.sqrt(area) * 4)
    vertical_term = 1.4 - center_y
    return round(max(0.3, min(5.0, area_term + vertical_term)), 2)
