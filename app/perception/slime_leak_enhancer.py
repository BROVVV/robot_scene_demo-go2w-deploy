"""Lightweight visual cue for brightly coloured simulated liquid leaks.

The open-vocabulary detector remains the primary semantic detector.  Slime is
amorphous, however, so GroundingDINO can miss it even when the colour patch is
obvious to a person.  This module adds a deterministic fallback for the demo:
find compact high-saturation green regions and emit a mask-backed leak
candidate.  A detected pipe is used as context when available.

This is deliberately a *simulation cue*, not a general industrial leak
detector.  The emitted label is distinct so hazard rules cannot accidentally
treat an ordinary ``pipe`` detection as a confirmed leak.
"""
from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from app.detectors.base import DetectedObject


DEFAULT_HSV_RANGES = (
    # OpenCV HSV uses H in [0, 179].  Keep the production default to green:
    # the live room has a saturated yellow wall that open-vocabulary models
    # occasionally call a spill, so an orange/yellow range is unsafe here.
    {"name": "fluorescent_green", "lower": [32, 105, 70], "upper": [92, 255, 255]},
)


def enhance_slime_leak_detections(
    image_path: str,
    detections: Iterable[DetectedObject],
    config: dict[str, Any] | None,
    *,
    context_detections: Iterable[DetectedObject] = (),
    tracked_pipe_contexts: Iterable[dict[str, Any]] = (),
    display_language: str = "zh",
) -> list[DetectedObject]:
    """Append visually verified slime candidates to detector results.

    Existing open-vocabulary leak boxes are retained.  If one overlaps a
    colour component, it is relabelled as the verified simulation cue so the
    hazard rule has one safe trigger path.  Otherwise a new candidate is
    created from the colour mask.  Returned boxes are normalized to [0, 1].
    """
    items = list(detections)
    settings = config or {}
    if not settings.get("enabled", False):
        return items

    try:
        import cv2
    except ImportError:
        return items

    path = Path(image_path)
    if not path.is_file():
        return items
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None or image.ndim != 3:
        return items
    height, width = image.shape[:2]
    if height <= 0 or width <= 0:
        return items

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    combined = np.zeros((height, width), dtype=np.uint8)
    configured_ranges = settings.get("hsv_ranges") or DEFAULT_HSV_RANGES
    for row in configured_ranges:
        try:
            lower = np.asarray(row["lower"], dtype=np.uint8)
            upper = np.asarray(row["upper"], dtype=np.uint8)
        except (KeyError, TypeError, ValueError):
            continue
        if lower.shape != (3,) or upper.shape != (3,):
            continue
        combined = cv2.bitwise_or(combined, cv2.inRange(hsv, lower, upper))

    # Do not run a fixed 5x5 opening over every cue.  It erases the thin/far
    # streaks for which this fallback exists.  A small close joins compression
    # gaps; connected-component filtering below removes isolated noise.  Large
    # components may still opt into the configured opening kernel.
    small_kernel = _odd_kernel(settings.get("small_morphology_kernel_px", 3), 3)
    if small_kernel > 1:
        kernel = np.ones((small_kernel, small_kernel), dtype=np.uint8)
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)

    component_count, component_map, stats, _ = cv2.connectedComponentsWithStats(
        combined, connectivity=8
    )
    image_area = float(width * height)
    min_area_ratio = float(settings.get("min_area_ratio", 0.0008))
    min_roi_area_ratio = float(settings.get("min_roi_area_ratio", 0.0015))
    min_valid_pixels = max(3, int(settings.get("min_valid_pixels", 12)))
    max_area_ratio = float(settings.get("max_area_ratio", 0.10))
    min_standalone_area_ratio = float(
        settings.get("min_standalone_area_ratio", max(0.0015, min_area_ratio))
    )
    min_dimension_px = max(2, int(settings.get("min_dimension_px", 6)))
    max_candidates = max(1, int(settings.get("max_candidates", 3)))
    pipe_margin = float(settings.get("pipe_context_margin_ratio", 0.08))
    require_pipe_when_visible = bool(
        settings.get("require_pipe_context_when_visible", True)
    )
    require_semantic_context = bool(
        settings.get("require_semantic_context", True)
    )
    require_pipe_context = bool(settings.get("require_pipe_context", True))

    pipe_terms = {
        str(value).strip().casefold()
        for value in settings.get(
            "pipe_labels", ["pipe", "pipeline", "industrial pipe", "管道"]
        )
        if str(value).strip()
    }
    leak_terms = {
        str(value).strip().casefold()
        for value in settings.get(
            "leak_prompt_labels", ["green slime", "slime", "liquid leak", "fluid spill"]
        )
        if str(value).strip()
    }
    pipe_contexts: list[dict[str, Any]] = []
    roi_expand_px = max(1, int(settings.get("pipe_roi_expand_px", 6)))
    roi_expand_ratio = max(0.0, float(settings.get("pipe_roi_expand_width_ratio", 0.20)))
    context_only = [
        item for item in context_detections
        if not any(item is persisted for persisted in items)
    ]
    pipe_sources = [
        (index, item) for index, item in enumerate(items)
    ] + [
        (None, item) for item in context_only
    ]
    for index, item in pipe_sources:
        if not _item_terms(item).intersection(pipe_terms):
            continue
        box = _normalized_box(item.bbox_2d)
        if box is None:
            continue
        surface = _decode_item_mask(item, height, width)
        search_surface = None
        if surface is not None and int(surface.sum()) >= min_valid_pixels:
            ys, xs = np.nonzero(surface)
            apparent_width = min(max(1, int(xs.max() - xs.min() + 1)),
                                 max(1, int(ys.max() - ys.min() + 1)))
            expand = min(roi_expand_px, max(1, int(round(apparent_width * roi_expand_ratio))))
            kernel = np.ones((2 * expand + 1, 2 * expand + 1), dtype=np.uint8)
            search_surface = cv2.dilate(surface.astype(np.uint8), kernel).astype(bool)
        pipe_contexts.append({
            "index": index,
            "box": box,
            "surface": surface,
            "search_surface": search_surface,
            "tracked": False,
        })
    for tracked in tracked_pipe_contexts:
        surface = tracked.get("surface")
        if not isinstance(surface, np.ndarray) or surface.shape != (height, width):
            continue
        if int(surface.sum()) < min_valid_pixels:
            continue
        ys, xs = np.nonzero(surface)
        box = [xs.min() / width, ys.min() / height,
               (xs.max() + 1) / width, (ys.max() + 1) / height]
        apparent_width = min(max(1, int(xs.max() - xs.min() + 1)),
                             max(1, int(ys.max() - ys.min() + 1)))
        expand = min(roi_expand_px, max(1, int(round(apparent_width * roi_expand_ratio))))
        kernel = np.ones((2 * expand + 1, 2 * expand + 1), dtype=np.uint8)
        pipe_contexts.append({
            "index": None,
            "box": box,
            "surface": surface,
            "search_surface": cv2.dilate(surface.astype(np.uint8), kernel).astype(bool),
            "tracked": True,
            "tracking_quality": float(tracked.get("tracking_quality") or 0.0),
        })
    leak_boxes = [
        _normalized_box(item.bbox_2d)
        for item in items
        if _item_terms(item).intersection(leak_terms)
    ]
    leak_boxes = [box for box in leak_boxes if box is not None]

    candidates: list[dict[str, Any]] = []
    for component in range(1, component_count):
        x, y, box_width, box_height, area = (
            int(value) for value in stats[component]
        )
        area_ratio = area / image_area
        if area < min_valid_pixels or area_ratio > max_area_ratio:
            continue
        if max(box_width, box_height) < min_dimension_px:
            continue
        box = [
            x / width,
            y / height,
            (x + box_width) / width,
            (y + box_height) / height,
        ]
        component_mask = component_map == component
        best_context: dict[str, Any] | None = None
        best_surface_fraction = 0.0
        for context in pipe_contexts:
            search_surface = context["search_surface"]
            surface_fraction = (
                float(np.count_nonzero(component_mask & search_surface)) / float(area)
                if search_surface is not None else 0.0
            )
            bbox_near = _boxes_near(box, context["box"], pipe_margin)
            rank = (surface_fraction > 0.0, surface_fraction, bbox_near)
            current_rank = (
                best_surface_fraction > 0.0,
                best_surface_fraction,
                bool(best_context and best_context.get("bbox_near")),
            )
            if rank > current_rank:
                best_context = {**context, "bbox_near": bbox_near}
                best_surface_fraction = surface_fraction
        surface_match = best_surface_fraction >= float(
            settings.get("min_pipe_surface_overlap", 0.25)
        )
        bbox_near = bool(best_context and best_context.get("bbox_near"))
        grounded_leak = any(
            _intersection_fraction(box, leak_box) >= 0.20
            for leak_box in leak_boxes
        )
        if pipe_contexts and require_pipe_when_visible and not (surface_match or bbox_near):
            continue
        roi_ratio = 0.0
        if best_context and best_context.get("search_surface") is not None:
            roi_area = int(np.count_nonzero(best_context["search_surface"]))
            if roi_area:
                roi_ratio = area / float(roi_area)
        if area_ratio < min_area_ratio and not (
            surface_match and roi_ratio >= min_roi_area_ratio
        ):
            continue
        if not (surface_match or bbox_near or grounded_leak) \
                and area_ratio < min_standalone_area_ratio:
            continue

        context_quality = (
            "tracked_pipe_mask" if surface_match and best_context and best_context.get("tracked")
            else "pipe_surface_mask" if surface_match
            else "pipe_bbox_weak" if bbox_near
            else "semantic_prompt_weak" if grounded_leak
            else "colour_only"
        )
        # Only a measured surface mask can promote a colour cue to confirmed
        # visual evidence.  A rectangular pipe box is retained as a suspected
        # cue and can trigger a focused pipe re-check, but is not equivalent.
        confirmed = surface_match
        if confirmed and not _has_local_colour_contrast(
            image, component_mask, best_context.get("surface") if best_context else None,
            float(settings.get("min_local_colour_contrast", 18.0)),
        ):
            confirmed = False
            context_quality = "pipe_surface_low_contrast"
        if require_pipe_context and not (surface_match or bbox_near):
            confirmed = False
        if require_semantic_context and not (surface_match or bbox_near or grounded_leak):
            confirmed = False
        candidates.append({
            "component": component,
            "area_ratio": area_ratio,
            "roi_area_ratio": roi_ratio,
            "box": box,
            "component_mask": component_mask,
            "grounded_leak": grounded_leak,
            "confirmed": confirmed,
            "context_quality": context_quality,
            "context": best_context,
            "surface_fraction": best_surface_fraction,
        })

    candidates.sort(
        key=lambda row: (
            row["confirmed"], row["surface_fraction"], row["grounded_leak"],
            row["roi_area_ratio"], row["area_ratio"],
        ),
        reverse=True,
    )
    output = list(items)
    host_counts: dict[int, int] = {}
    selected = []
    for row in candidates:
        context = row.get("context")
        host_index = int(context["index"]) if context and context.get("index") is not None else -1
        if host_counts.get(host_index, 0) >= int(settings.get("max_candidates_per_pipe", 2)):
            continue
        selected.append(row)
        host_counts[host_index] = host_counts.get(host_index, 0) + 1
        if len(selected) >= max_candidates:
            break
    for row in selected:
        area_ratio = float(row["area_ratio"])
        box = row["box"]
        confirmed = bool(row["confirmed"])
        context = row.get("context")
        existing_index = _overlapping_leak_detection(output, box, leak_terms)
        attributes = [
            "visual_cue=high_saturation_slime_colour",
            f"semantic_context={row['context_quality']}",
            f"colour_area_ratio={area_ratio:.6f}",
            f"roi_area_ratio={float(row['roi_area_ratio']):.6f}",
            f"pipe_surface_overlap={float(row['surface_fraction']):.4f}",
            f"evidence_state={'confirmed_demo_candidate' if confirmed else 'suspected'}",
        ]
        component_mask = row["component_mask"]
        mask_rle = _encode_mask_runs(component_mask)
        host_index = int(context["index"]) if context and context.get("index") is not None else None
        context_rle = (
            _encode_mask_runs(context["surface"])
            if context and context.get("surface") is not None else None
        )
        score = min(
            0.94,
            0.58 + (0.24 if confirmed else 0.0)
            + (0.06 if row["grounded_leak"] else 0.0) + min(0.06, area_ratio),
        )
        review_only = row["context_quality"] == "colour_only"
        label = (
            "simulated liquid leak" if confirmed
            else "fluorescent colour review region" if review_only
            else "simulated liquid leak candidate"
        )
        label_zh = (
            "模拟泄漏危险信号" if confirmed
            else "指定颜色复核区域" if review_only
            else "疑似模拟泄漏信号"
        )
        if str(display_language).strip().casefold() == "en":
            label_zh = label
        category = "review" if review_only else "hazard"
        if existing_index is not None:
            prior = output[existing_index]
            output[existing_index] = replace(
                prior,
                label=label,
                label_zh=label_zh,
                category=category,
                color="fluorescent",
                attributes=list(dict.fromkeys([*prior.attributes, *attributes])),
                mask_area_ratio=area_ratio,
                mask_rle=mask_rle,
                mask_size=(height, width),
                mask_source="hsv_slime_mask",
                context_mask_rle=context_rle,
                context_mask_size=(height, width) if context_rle else None,
                host_detection_index=host_index,
                context_quality=row["context_quality"],
                hazard_state="confirmed_demo_candidate" if confirmed else "suspected",
                source="grounded_sam+hsv_slime_cue",
                source_prompt_term=label,
                score=max(float(prior.score), score),
            )
            continue
        output.append(
            DetectedObject(
                label=label,
                label_zh=label_zh,
                category=category,
                bbox_2d=tuple(box),
                score=score,
                color="fluorescent",
                attributes=attributes,
                mask_area_ratio=area_ratio,
                mask_rle=mask_rle,
                mask_size=(height, width),
                mask_source="hsv_slime_mask",
                context_mask_rle=context_rle,
                context_mask_size=(height, width) if context_rle else None,
                host_detection_index=host_index,
                context_quality=row["context_quality"],
                hazard_state="confirmed_demo_candidate" if confirmed else "suspected",
                source="hsv_slime_cue",
                source_prompt_term=label,
            )
        )
    return output


def select_low_confidence_pipe_contexts(
    detections: Iterable[DetectedObject],
    config: dict[str, Any] | None,
    *,
    persistent_min_confidence: float,
) -> list[DetectedObject]:
    """Keep narrow pipe-mask evidence for leak review without persisting it.

    Prompt-free YOLOE has a large fixed vocabulary and can label a visible
    round outlet as ``pipe bowl`` at a score below the ordinary semantic-map
    threshold.  Throwing that segmentation mask away also removes the only
    measured surface on which the fluorescent demo cue can be verified.

    This selector is intentionally stricter than normal detection: the label
    must be an explicit pipe alias, the score must clear a dedicated floor,
    the box must stay local, and a real segmentation mask is mandatory.  The
    returned detections are context-only and must never be inserted into the
    persistent object map by the caller.
    """
    settings = config or {}
    pipe_terms = {
        str(value).strip().casefold()
        for value in settings.get(
            "pipe_labels", ["pipe", "pipeline", "industrial pipe", "管道"]
        )
        if str(value).strip()
    }
    minimum = max(0.0, float(settings.get("min_pipe_context_confidence", 0.10)))
    maximum_area = min(
        1.0, max(0.0, float(settings.get("max_pipe_context_bbox_area_ratio", 0.25)))
    )
    selected: list[DetectedObject] = []
    for item in detections:
        score = float(getattr(item, "score", 0.0) or 0.0)
        if score >= float(persistent_min_confidence) or score < minimum:
            continue
        if not _item_terms(item).intersection(pipe_terms):
            continue
        box = _normalized_box(getattr(item, "bbox_2d", None))
        if box is None or (box[2] - box[0]) * (box[3] - box[1]) > maximum_area:
            continue
        if not getattr(item, "mask_rle", None) or not getattr(item, "mask_size", None):
            continue
        selected.append(item)
    return selected


def _item_terms(item: DetectedObject) -> set[str]:
    return {
        str(value).strip().casefold()
        for value in (item.label, item.label_zh, item.source_prompt_term)
        if value
    }


def _normalized_box(value: Any) -> list[float] | None:
    try:
        x1, y1, x2, y2 = (float(part) for part in value)
    except (TypeError, ValueError):
        return None
    if not all(np.isfinite([x1, y1, x2, y2])) or x2 <= x1 or y2 <= y1:
        return None
    return [x1, y1, x2, y2]


def _odd_kernel(value: Any, default: int) -> int:
    try:
        size = max(1, int(value))
    except (TypeError, ValueError):
        size = default
    return size if size % 2 else size + 1


def _decode_item_mask(item: DetectedObject, height: int, width: int) -> np.ndarray | None:
    size = getattr(item, "mask_size", None)
    runs = getattr(item, "mask_rle", None)
    if not runs or not size or tuple(int(v) for v in size) != (height, width):
        return None
    mask = np.zeros((height, width), dtype=bool)
    try:
        for row, start, length in runs:
            row, start, length = int(row), int(start), int(length)
            if 0 <= row < height and length > 0:
                mask[row, max(0, start):min(width, start + length)] = True
    except (TypeError, ValueError):
        return None
    return mask


def _has_local_colour_contrast(
    image: np.ndarray,
    cue_mask: np.ndarray,
    pipe_mask: np.ndarray | None,
    threshold: float,
) -> bool:
    if pipe_mask is None or threshold <= 0.0:
        return True
    reference = pipe_mask & ~cue_mask
    if int(reference.sum()) < 8 or int(cue_mask.sum()) < 3:
        return False
    cue = image[cue_mask].astype(np.float32)
    host = image[reference].astype(np.float32)
    # Median BGR distance is deliberately simple and deterministic.  It
    # rejects an already-green painted host without pretending to identify a
    # material type.
    return float(np.linalg.norm(np.median(cue, axis=0) - np.median(host, axis=0))) >= threshold


def _boxes_near(left: list[float], right: list[float], margin: float) -> bool:
    expanded = [
        max(0.0, right[0] - margin),
        max(0.0, right[1] - margin),
        min(1.0, right[2] + margin),
        min(1.0, right[3] + margin),
    ]
    return not (
        left[2] < expanded[0]
        or left[0] > expanded[2]
        or left[3] < expanded[1]
        or left[1] > expanded[3]
    )


def _overlapping_leak_detection(
    detections: list[DetectedObject], box: list[float], leak_terms: set[str]
) -> int | None:
    best_index: int | None = None
    best_overlap = 0.0
    for index, item in enumerate(detections):
        if not _item_terms(item).intersection(leak_terms):
            continue
        candidate = _normalized_box(item.bbox_2d)
        if candidate is None:
            continue
        overlap = _iou(candidate, box)
        if overlap >= 0.20 and overlap > best_overlap:
            best_index, best_overlap = index, overlap
    return best_index


def _iou(left: list[float], right: list[float]) -> float:
    x1, y1 = max(left[0], right[0]), max(left[1], right[1])
    x2, y2 = min(left[2], right[2]), min(left[3], right[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    left_area = max(0.0, left[2] - left[0]) * max(0.0, left[3] - left[1])
    right_area = max(0.0, right[2] - right[0]) * max(0.0, right[3] - right[1])
    union = left_area + right_area - intersection
    return intersection / union if union > 0.0 else 0.0


def _intersection_fraction(left: list[float], right: list[float]) -> float:
    """Fraction of the colour component covered by a semantic box."""
    x1, y1 = max(left[0], right[0]), max(left[1], right[1])
    x2, y2 = min(left[2], right[2]), min(left[3], right[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    left_area = max(0.0, left[2] - left[0]) * max(0.0, left[3] - left[1])
    return intersection / left_area if left_area > 0.0 else 0.0


def _encode_mask_runs(mask: np.ndarray) -> list[list[int]]:
    runs: list[list[int]] = []
    for row in range(mask.shape[0]):
        xs = np.flatnonzero(mask[row])
        if xs.size == 0:
            continue
        starts = xs[np.r_[True, np.diff(xs) > 1]]
        ends = xs[np.r_[np.diff(xs) > 1, True]]
        runs.extend(
            [int(row), int(start), int(end - start + 1)]
            for start, end in zip(starts, ends)
        )
    return runs
