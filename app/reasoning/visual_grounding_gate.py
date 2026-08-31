"""Keep LLM hypotheses separate from visual target confirmation."""

from __future__ import annotations

from app.schemas import (
    Actionability,
    LLMSearchHypothesis,
    NodeObservationStatus,
    SceneAnalysisResult,
    SceneObject,
)


def apply_visual_grounding_gate(
    hypotheses: list[LLMSearchHypothesis],
    scene: SceneAnalysisResult,
) -> list[LLMSearchHypothesis]:
    visual_target_found = bool(
        scene.target_decision.is_present
        and scene.target_decision.matched_object_ids
    )
    confirmed_ids = set(scene.target_decision.matched_object_ids)
    confirmed_objects = [
        item for item in scene.objects if item.id in confirmed_ids
    ]
    preferred_hypothesis_id: str | None = None
    if visual_target_found and hypotheses:
        for hypothesis in hypotheses:
            if confirmed_ids & set(hypothesis.supporting_visible_anchor_ids):
                preferred_hypothesis_id = hypothesis.hypothesis_id
                break
        if preferred_hypothesis_id is None and confirmed_objects:
            confirmed_regions = {
                item.position.horizontal for item in confirmed_objects
            }
            for hypothesis in hypotheses:
                hint = (hypothesis.image_region_hint or "").lower()
                if any(region in hint for region in confirmed_regions):
                    preferred_hypothesis_id = hypothesis.hypothesis_id
                    break
        if preferred_hypothesis_id is None:
            preferred_hypothesis_id = max(
                hypotheses,
                key=lambda item: item.confidence,
            ).hypothesis_id
    result: list[LLMSearchHypothesis] = []
    for hypothesis in hypotheses:
        anchor_match = bool(
            confirmed_ids & set(hypothesis.supporting_visible_anchor_ids)
        )
        if visual_target_found and (
            anchor_match
            or hypothesis.candidate_region_type == "observed_target"
            or hypothesis.hypothesis_id == preferred_hypothesis_id
        ):
            result.append(
                hypothesis.model_copy(
                    update={
                        "status": NodeObservationStatus.OBSERVED,
                        "should_not_mark_found": False,
                        "confidence": max(
                            hypothesis.confidence,
                            scene.target_decision.confidence,
                        ),
                    }
                )
            )
        elif hypothesis.actionability in {
            Actionability.NEEDS_HUMAN,
            Actionability.UNSAFE_OR_IMPOSSIBLE,
        }:
            result.append(
                hypothesis.model_copy(
                    update={
                        "status": NodeObservationStatus.UNREACHABLE,
                        "should_not_mark_found": True,
                    }
                )
            )
        else:
            result.append(
                hypothesis.model_copy(
                    update={
                        "status": NodeObservationStatus.INFERRED,
                        "should_not_mark_found": True,
                    }
                )
            )
    return result


def collect_dynamic_detector_prompts(
    hypotheses: list[LLMSearchHypothesis],
    max_terms: int = 12,
) -> list[str]:
    terms: list[str] = []
    seen: set[str] = set()
    for hypothesis in sorted(
        hypotheses, key=lambda item: item.confidence, reverse=True
    ):
        for value in hypothesis.suggested_detector_prompts_en:
            normalized = " ".join(str(value).strip().lower().split())
            if not normalized or normalized in seen:
                continue
            seen.add(normalized)
            terms.append(normalized)
            if len(terms) >= max_terms:
                return terms
    return terms


def merge_visual_retry_scene(
    base_scene: SceneAnalysisResult,
    retry_scene: SceneAnalysisResult,
) -> tuple[SceneAnalysisResult, dict]:
    existing = list(base_scene.objects)
    added: list[SceneObject] = []
    id_map: dict[str, str] = {}
    for retry_object in retry_scene.objects:
        duplicate = _best_overlap(retry_object, existing + added)
        if duplicate is not None:
            id_map[retry_object.id] = duplicate.id
            continue
        new_id = f"retry_obj_{len(added) + 1:03d}"
        id_map[retry_object.id] = new_id
        added.append(
            retry_object.model_copy(
                update={
                    "id": new_id,
                    "attributes": [
                        *retry_object.attributes,
                        "dynamic_visual_retry=true",
                    ],
                }
            )
        )

    matched_ids = [
        id_map[item]
        for item in retry_scene.target_decision.matched_object_ids
        if item in id_map
    ]
    upgraded = bool(
        retry_scene.target_decision.is_present and matched_ids
    )
    decision = base_scene.target_decision
    if upgraded:
        decision = decision.model_copy(
            update={
                "is_present": True,
                "matched_object_ids": matched_ids,
                "match_reason_zh": (
                    "LLM 动态检测词触发二次视觉复核后确认目标。"
                ),
                "confidence": retry_scene.target_decision.confidence,
            }
        )
    merged = base_scene.model_copy(
        update={
            "objects": [*existing, *added],
            "target_decision": decision,
        }
    )
    return merged, {
        "attempted": True,
        "upgraded_target": upgraded,
        "retry_object_count": len(retry_scene.objects),
        "added_object_ids": [item.id for item in added],
        "matched_object_ids": matched_ids,
    }


def _best_overlap(
    candidate: SceneObject,
    objects: list[SceneObject],
) -> SceneObject | None:
    best: SceneObject | None = None
    best_iou = 0.0
    for item in objects:
        score = _iou(candidate, item)
        if score > best_iou:
            best_iou = score
            best = item
    return best if best_iou >= 0.65 else None


def _iou(left: SceneObject, right: SceneObject) -> float:
    x1 = max(left.bbox_2d.x1, right.bbox_2d.x1)
    y1 = max(left.bbox_2d.y1, right.bbox_2d.y1)
    x2 = min(left.bbox_2d.x2, right.bbox_2d.x2)
    y2 = min(left.bbox_2d.y2, right.bbox_2d.y2)
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    left_area = (
        max(0.0, left.bbox_2d.x2 - left.bbox_2d.x1)
        * max(0.0, left.bbox_2d.y2 - left.bbox_2d.y1)
    )
    right_area = (
        max(0.0, right.bbox_2d.x2 - right.bbox_2d.x1)
        * max(0.0, right.bbox_2d.y2 - right.bbox_2d.y1)
    )
    union = left_area + right_area - intersection
    return intersection / union if union > 0 else 0.0
