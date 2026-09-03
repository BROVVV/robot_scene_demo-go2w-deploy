"""Deterministic same-frame geometric relations for the semantic topology.

The VLM is allowed to add semantic relations, but it is not required for
basic left/right, above/below, depth ordering, and proximity.  This extractor
only emits relations whose two endpoints are present in the same frame.  The
ObjectRelationStore later maps those frame ids to persistent ``obj_*`` ids.
"""

from __future__ import annotations

import math
from typing import Any


def _id(item: dict[str, Any], index: int) -> str:
    value = item.get("frame_object_id") or item.get("id") or item.get("object_id")
    return str(value or f"geometric_obj_{index:03d}").strip()


def _label(item: dict[str, Any]) -> str:
    return str(item.get("label_zh") or item.get("label") or item.get("name") or "object").strip()


def _bbox_center(item: dict[str, Any]) -> tuple[float, float] | None:
    value = item.get("bbox_2d") or item.get("bbox")
    if isinstance(value, dict):
        value = [value.get(key) for key in ("x1", "y1", "x2", "y2")]
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        return None
    try:
        x1, y1, x2, y2 = (float(part) for part in value)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(part) for part in (x1, y1, x2, y2)) or x2 <= x1 or y2 <= y1:
        return None
    return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)


def _xyz(item: dict[str, Any]) -> tuple[float, float, float] | None:
    value = item.get("camera_xyz") or item.get("map_xyz")
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        return None
    try:
        result = tuple(float(value[index]) for index in range(3))
    except (TypeError, ValueError):
        return None
    return result if all(math.isfinite(part) for part in result) else None


def _depth(item: dict[str, Any], xyz: tuple[float, float, float] | None) -> float | None:
    value = item.get("depth_m") or item.get("estimated_distance_m")
    try:
        value = float(value) if value is not None else (float(xyz[2]) if xyz else None)
    except (TypeError, ValueError):
        return None
    return value if value is not None and math.isfinite(value) and value > 0.0 else None


def _relation(
    source: dict[str, Any], target: dict[str, Any], relation: str, confidence: float,
    *, provenance: str, description: str,
) -> dict[str, Any]:
    return {
        "subject_id": source["_frame_id"],
        "object_id": target["_frame_id"],
        "subject_label": source["_label"],
        "object_label": target["_label"],
        "relation": relation,
        "confidence": max(0.0, min(1.0, float(confidence))),
        "description_zh": description,
        "provenance": provenance,
    }


def extract_geometric_relations(
    objects: list[dict[str, Any]],
    *,
    min_horizontal_separation: float = 0.10,
    min_vertical_separation: float = 0.10,
    min_depth_separation_m: float = 0.25,
    near_distance_m: float = 0.90,
) -> list[dict[str, Any]]:
    """Generate conservative, frame-bound geometric relations.

    Image coordinates use the usual top-left origin, so a smaller center-y is
    ``above``.  Relation confidence is lowered for tiny separations; no
    relation is emitted for ambiguous overlap.
    """
    prepared: list[dict[str, Any]] = []
    seen: set[str] = set()
    for index, raw in enumerate(objects or [], start=1):
        if not isinstance(raw, dict):
            continue
        item = dict(raw)
        item["_frame_id"] = _id(item, index)
        if not item["_frame_id"] or item["_frame_id"] in seen:
            continue
        seen.add(item["_frame_id"])
        item["_label"] = _label(item)
        prepared.append(item)

    result: list[dict[str, Any]] = []
    for left_index, source in enumerate(prepared):
        for target in prepared[left_index + 1:]:
            source_conf = float(source.get("confidence", source.get("score", 0.5)) or 0.5)
            target_conf = float(target.get("confidence", target.get("score", 0.5)) or 0.5)
            base_conf = max(0.45, min(0.98, math.sqrt(max(0.0, source_conf * target_conf))))
            source_center = _bbox_center(source)
            target_center = _bbox_center(target)
            if source_center and target_center:
                dx = target_center[0] - source_center[0]
                dy = target_center[1] - source_center[1]
                if abs(dx) >= min_horizontal_separation:
                    if dx > 0:
                        result.append(_relation(source, target, "left_of", base_conf,
                                                provenance="geometric_same_frame_bbox",
                                                description="同一帧二维框显示该物体在左侧"))
                    else:
                        result.append(_relation(target, source, "left_of", base_conf,
                                                provenance="geometric_same_frame_bbox",
                                                description="同一帧二维框显示该物体在左侧"))
                if abs(dy) >= min_vertical_separation:
                    if dy > 0:
                        result.append(_relation(source, target, "above", base_conf,
                                                provenance="geometric_same_frame_bbox",
                                                description="同一帧二维框显示该物体在上方"))
                    else:
                        result.append(_relation(target, source, "above", base_conf,
                                                provenance="geometric_same_frame_bbox",
                                                description="同一帧二维框显示该物体在上方"))

            source_xyz = _xyz(source)
            target_xyz = _xyz(target)
            if source_xyz and target_xyz:
                distance = math.dist(source_xyz, target_xyz)
                if distance <= near_distance_m:
                    confidence = min(0.95, base_conf + 0.05)
                    result.append(_relation(source, target, "near", confidence,
                                            provenance="geometric_same_frame_rgbd",
                                            description=f"同一帧 RGB-D 距离约 {distance:.2f} 米"))
                source_depth = _depth(source, source_xyz)
                target_depth = _depth(target, target_xyz)
                if source_depth is not None and target_depth is not None:
                    depth_delta = target_depth - source_depth
                    if abs(depth_delta) >= min_depth_separation_m:
                        if depth_delta > 0:
                            result.append(_relation(source, target, "in_front_of", base_conf,
                                                    provenance="geometric_same_frame_depth",
                                                    description="同一帧深度显示该物体更靠前"))
                        else:
                            result.append(_relation(target, source, "in_front_of", base_conf,
                                                    provenance="geometric_same_frame_depth",
                                                    description="同一帧深度显示该物体更靠前"))
    return result


build_geometric_relations = extract_geometric_relations
