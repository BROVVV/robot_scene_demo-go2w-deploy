"""Image-space context utilities for target evidence."""

from __future__ import annotations

import math
from typing import Any


IGNORED_REFERENCE_LABELS = {"unknown", "object", "thing", "stuff", "background", "物体", "背景"}


def get_image_position(bbox: list[float] | tuple[float, ...]) -> str:
    x1, y1, x2, y2 = _normalized_bbox(bbox)
    center_x = (x1 + x2) / 2
    center_y = (y1 + y2) / 2
    horizontal = "left" if center_x < 1 / 3 else "right" if center_x >= 2 / 3 else "center"
    vertical = "upper" if center_y < 1 / 3 else "lower" if center_y >= 2 / 3 else "middle"
    return f"{vertical}_{horizontal}"


def relative_direction_hint(image_position: str) -> str:
    mapping = {
        "lower_center": "front_low",
        "lower_left": "left_front_low",
        "lower_right": "right_front_low",
        "middle_center": "front",
        "middle_left": "left_front",
        "middle_right": "right_front",
    }
    if image_position.startswith("upper_"):
        return "far_or_high_area"
    return mapping.get(image_position, "unknown")


def find_nearby_objects(
    target_object: dict[str, Any],
    objects: list[dict[str, Any]],
    limit: int = 3,
) -> list[dict[str, Any]]:
    target_center = bbox_center(target_object.get("bbox", [0, 0, 1, 1]))
    target_labels = {
        _normalize_reference_label(target_object.get("label")),
        _normalize_reference_label(target_object.get("label_zh")),
    }
    target_labels.discard("")
    nearby = []
    for obj in objects:
        if obj is target_object or obj.get("object_id") == target_object.get("object_id"):
            continue
        if obj.get("is_target_candidate"):
            continue
        object_labels = {
            _normalize_reference_label(obj.get("label")),
            _normalize_reference_label(obj.get("label_zh")),
        }
        object_labels.discard("")
        if target_labels & object_labels:
            continue
        label = str(obj.get("label_zh") or obj.get("label") or "").strip()
        if label.lower() in IGNORED_REFERENCE_LABELS or not label:
            continue
        distance = math.dist(target_center, bbox_center(obj.get("bbox", [0, 0, 1, 1])))
        nearby.append(
            {
                "label": obj.get("label"),
                "label_zh": obj.get("label_zh"),
                "distance_normalized": round(distance, 4),
                "relation_hint": "near" if distance <= 0.35 else "near_background",
                "object_id": obj.get("object_id"),
            }
        )
    nearby.sort(key=lambda item: item["distance_normalized"])
    return nearby[:limit]


def describe_target(
    target_label: str,
    image_position: str,
    nearby_objects: list[dict[str, Any]],
) -> str:
    references = [
        str(item.get("label_zh") or item.get("label"))
        for item in nearby_objects
        if item.get("label_zh") or item.get("label")
    ]
    suffix = f"，附近参照物有{'、'.join(references)}" if references else "，附近没有稳定参照物"
    return f"{target_label}位于画面 {image_position}{suffix}。"


def bbox_center(bbox: list[float] | tuple[float, ...]) -> tuple[float, float]:
    x1, y1, x2, y2 = _normalized_bbox(bbox)
    return ((x1 + x2) / 2, (y1 + y2) / 2)


def bbox_area(bbox: list[float] | tuple[float, ...]) -> float:
    x1, y1, x2, y2 = _normalized_bbox(bbox)
    return max(0.0, x2 - x1) * max(0.0, y2 - y1)


def bbox_iou(
    left: list[float] | tuple[float, ...],
    right: list[float] | tuple[float, ...],
) -> float:
    ax1, ay1, ax2, ay2 = _normalized_bbox(left)
    bx1, by1, bx2, by2 = _normalized_bbox(right)
    intersection = max(0.0, min(ax2, bx2) - max(ax1, bx1)) * max(
        0.0, min(ay2, by2) - max(ay1, by1)
    )
    union = bbox_area(left) + bbox_area(right) - intersection
    return 0.0 if union <= 0 else intersection / union


def _normalized_bbox(bbox: list[float] | tuple[float, ...]) -> tuple[float, float, float, float]:
    if len(bbox) != 4:
        return (0.0, 0.0, 1.0, 1.0)
    values = tuple(max(0.0, min(1.0, float(value))) for value in bbox)
    x1, y1, x2, y2 = values
    return min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)


def _normalize_reference_label(value: Any) -> str:
    return " ".join(str(value or "").lower().replace("_", " ").split())
