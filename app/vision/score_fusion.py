"""Fuse detector, semantic, attribute and context evidence."""

from __future__ import annotations

from typing import Any

from app.config import Settings, get_settings
from app.vision.schema import CandidateObject


def fuse_score(
    candidate: CandidateObject,
    verify_result: dict[str, Any] | None = None,
    settings: Settings | None = None,
) -> float:
    config = settings or get_settings()
    detector = _score(candidate.detector_score, candidate.score)
    if not verify_result:
        return round(detector, 4)
    vlm = _score(verify_result.get("target_match_score"), 0.0)
    attribute = _score(verify_result.get("attribute_score"), 0.0)
    context = _score(verify_result.get("context_score"), 0.0)
    weighted = (
        detector * config.fusion_weight_detector
        + vlm * config.fusion_weight_vlm
        + attribute * config.fusion_weight_attribute
        + context * config.fusion_weight_context
    )
    weight_total = (
        config.fusion_weight_detector
        + config.fusion_weight_vlm
        + config.fusion_weight_attribute
        + config.fusion_weight_context
    )
    if weight_total <= 0:
        return round(detector, 4)
    return round(max(0.0, min(1.0, weighted / weight_total)), 4)


def decide_candidate(
    final_score: float,
    verify_result: dict[str, Any] | None,
    settings: Settings | None = None,
) -> tuple[str, str | None]:
    config = settings or get_settings()
    if verify_result and verify_result.get("verification_failed"):
        return "candidate", "verification_failed"
    if (
        verify_result
        and bool(verify_result.get("is_target"))
        and final_score >= config.final_target_score_threshold
    ):
        return "confirmed", None
    if final_score >= config.final_candidate_score_threshold:
        reason = None
        if verify_result and not verify_result.get("is_target"):
            reason = str(verify_result.get("rejection_reason") or "semantic_verifier_not_confirmed")
        return "candidate", reason
    return (
        "rejected",
        str((verify_result or {}).get("rejection_reason") or "fused_score_below_threshold"),
    )


def build_explanation(
    candidate: CandidateObject,
    verify_result: dict[str, Any] | None,
) -> str:
    parts = [
        f"detector={candidate.source}",
        f"detector_score={_score(candidate.detector_score, candidate.score):.2f}",
    ]
    if verify_result:
        parts.append(
            f"crop_target_score={_score(verify_result.get('target_match_score'), 0.0):.2f}"
        )
        evidence = verify_result.get("evidence") or []
        if evidence:
            parts.append("evidence=" + "；".join(str(item) for item in evidence[:3]))
        if verify_result.get("rejection_reason"):
            parts.append("rejection=" + str(verify_result["rejection_reason"]))
    return "; ".join(parts)


def apply_score_fusion(
    candidate: CandidateObject,
    verify_result: dict[str, Any] | None = None,
    settings: Settings | None = None,
) -> CandidateObject:
    candidate.crop_verify = verify_result
    candidate.final_score = fuse_score(candidate, verify_result, settings)
    candidate.decision, candidate.rejection_reason = decide_candidate(
        candidate.final_score, verify_result, settings
    )
    candidate.explanation = build_explanation(candidate, verify_result)
    return candidate


def _score(value: Any, default: float) -> float:
    try:
        return max(0.0, min(1.0, float(value)))
    except (TypeError, ValueError):
        return max(0.0, min(1.0, float(default)))
