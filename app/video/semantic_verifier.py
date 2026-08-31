"""Candidate-crop verification for sampled video frames."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from app.config import Settings, get_settings
from app.detectors.crop_verifier import CropVerifier
from app.video.models import FrameAnalysisResult
from app.video.target_profile import TargetProfile
from app.video.target_search import normalize_label, target_label_match_score
from app.vision.crop_utils import save_candidate_crop
from app.vision.schema import CandidateObject
from app.vision.score_fusion import apply_score_fusion


def verify_video_candidates(
    frame_results: list[FrameAnalysisResult],
    target: str,
    target_profile: TargetProfile,
    output_dir: str | Path,
    settings: Settings | None = None,
    every_n_frames: int | None = None,
    max_candidates: int | None = None,
) -> dict[str, Any]:
    config = settings or get_settings()
    verifier = CropVerifier(config)
    if not verifier.available:
        return {"enabled": False, "attempted": 0, "reason": "missing_api_key"}
    interval = max(1, every_n_frames or config.video_verify_every_n_frames)
    limit = max_candidates or config.crop_verify_max_candidates
    output = Path(output_dir)
    results: list[dict[str, Any]] = []
    attempted = 0
    failures = 0

    for frame_order, frame in enumerate(frame_results):
        frame_candidates = [
            obj
            for obj in frame.objects
            if target_label_match_score(
                target,
                str(obj.get("label", "")),
                str(obj.get("label_zh", "")),
                target_profile=target_profile,
            )
            > 0
        ]
        if not frame_candidates:
            continue
        frame_candidates.sort(
            key=lambda item: float(item.get("confidence", 0.0)),
            reverse=True,
        )
        if frame_order % interval != 0 and frame_order != 0:
            continue
        verified_bboxes = []
        frame_positive = False
        frame_best_score = 0.0
        frame_reasons = []
        for obj in frame_candidates:
            if attempted >= limit:
                break
            attempted += 1
            candidate = CandidateObject(
                object_id=str(obj.get("object_id")),
                label=str(obj.get("label") or "object"),
                label_zh=str(obj.get("label_zh") or ""),
                bbox=list(obj.get("bbox") or []),
                score=float(obj.get("confidence", 0.0)),
                detector_score=float(obj.get("confidence", 0.0)),
                source=str(frame.metadata.get("detector") or "video"),
                frame_index=frame.frame_id,
                timestamp_sec=frame.timestamp_sec,
                attributes=list(obj.get("attributes") or []),
            )
            crop_path = (
                output
                / "video_crops"
                / f"video_f{frame.frame_id:06d}_{candidate.object_id}.jpg"
            )
            try:
                verification_bbox, context_objects = _verification_region(
                    obj, frame.objects, target_profile
                )
                saved, _ = save_candidate_crop(
                    frame.image_path,
                    verification_bbox,
                    crop_path,
                    config.crop_verify_expand_ratio,
                )
                candidate.crop_path = str(saved)
                verify_result = verifier.verify_crop(
                    saved,
                    target_profile,
                    candidate.label,
                    context={
                        "frame_id": frame.frame_id,
                        "timestamp_sec": frame.timestamp_sec,
                        "candidate_bbox": candidate.bbox,
                        "verification_bbox": verification_bbox,
                        "context_objects": context_objects,
                    },
                )
            except (OSError, ValueError) as exc:
                verify_result = {
                    "verification_failed": True,
                    "failure_reason": f"{type(exc).__name__}: {exc}",
                    "is_target": False,
                }
            if verify_result.get("verification_failed"):
                failures += 1
            apply_score_fusion(candidate, verify_result, config)
            obj.update(
                {
                    "crop_path": candidate.crop_path,
                    "crop_verify": verify_result,
                    "final_score": candidate.final_score,
                    "decision": candidate.decision,
                    "rejection_reason": candidate.rejection_reason,
                    "explanation": candidate.explanation,
                }
            )
            results.append(candidate.to_dict())
            if candidate.decision == "confirmed":
                frame_positive = True
                verified_bboxes.append(candidate.bbox)
            frame_best_score = max(frame_best_score, float(candidate.final_score or 0.0))
            if verify_result.get("evidence"):
                frame_reasons.extend(verify_result["evidence"])
        frame.metadata["semantic_verification"] = {
            "attempted": True,
            "is_present": frame_positive,
            "confidence": frame_best_score,
            "reason": "；".join(str(item) for item in frame_reasons[:3]),
            "matched_bboxes": verified_bboxes,
        }
        if attempted >= limit:
            break

    return {
        "enabled": True,
        "attempted": attempted,
        "failures": failures,
        "verify_every_n_frames": interval,
        "results": results,
    }


def _verification_region(
    candidate: dict[str, Any],
    frame_objects: list[dict[str, Any]],
    target_profile: TargetProfile,
) -> tuple[list[float], list[dict[str, Any]]]:
    """Include the nearest explicit relation anchor without weakening the gate."""
    bbox = _valid_bbox(candidate.get("bbox"))
    if bbox is None:
        return list(candidate.get("bbox") or []), []
    if not target_profile.relation_constraints:
        return bbox, []
    context_terms = {
        normalize_label(item)
        for item in target_profile.context_terms()
        if normalize_label(item)
    }
    if not context_terms:
        return bbox, []
    matches: list[tuple[float, dict[str, Any], list[float]]] = []
    candidate_center = _bbox_center(bbox)
    for obj in frame_objects:
        if obj is candidate or obj.get("object_id") == candidate.get("object_id"):
            continue
        labels = {
            normalize_label(str(obj.get("label") or "")),
            normalize_label(str(obj.get("label_zh") or "")),
        }
        labels.discard("")
        if not any(
            term == label or term in label or label in term
            for term in context_terms
            for label in labels
        ):
            continue
        context_bbox = _valid_bbox(obj.get("bbox"))
        if context_bbox is None:
            continue
        center = _bbox_center(context_bbox)
        distance = (center[0] - candidate_center[0]) ** 2 + (
            center[1] - candidate_center[1]
        ) ** 2
        matches.append((distance, obj, context_bbox))
    if not matches:
        return bbox, []
    _, anchor, anchor_bbox = min(matches, key=lambda item: item[0])
    union = [
        min(bbox[0], anchor_bbox[0]),
        min(bbox[1], anchor_bbox[1]),
        max(bbox[2], anchor_bbox[2]),
        max(bbox[3], anchor_bbox[3]),
    ]
    return union, [
        {
            "object_id": anchor.get("object_id"),
            "label": anchor.get("label"),
            "label_zh": anchor.get("label_zh"),
            "bbox": anchor_bbox,
        }
    ]


def _valid_bbox(value: Any) -> list[float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        return None
    try:
        bbox = [float(item) for item in value]
    except (TypeError, ValueError):
        return None
    if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
        return None
    return bbox


def _bbox_center(bbox: list[float]) -> tuple[float, float]:
    return ((bbox[0] + bbox[2]) / 2.0, (bbox[1] + bbox[3]) / 2.0)


def verify_grounded_candidates(
    frame_results: list[FrameAnalysisResult],
    target: str,
    target_profile: TargetProfile,
    max_verify_frames: int = 6,
) -> dict[str, Any]:
    """Legacy wrapper retained for callers outside the main pipeline."""
    return verify_video_candidates(
        frame_results,
        target,
        target_profile,
        output_dir="outputs",
        max_candidates=max_verify_frames,
    )
