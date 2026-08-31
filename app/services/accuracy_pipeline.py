"""High-recall candidate crop verification and explainable score fusion."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from app.config import Settings
from app.detectors.crop_verifier import CropVerifier
from app.schemas import SceneAnalysisResult
from app.reasoning.target_profile import TargetProfile
from app.vision.crop_utils import save_candidate_crop
from app.vision.schema import CandidateObject
from app.vision.score_fusion import apply_score_fusion


def enhance_image_result(
    result: SceneAnalysisResult,
    image_path: str | Path,
    profile: TargetProfile,
    settings: Settings,
    output_dir: str | Path,
    enable_crop_verify: bool | None = None,
    verifier: CropVerifier | None = None,
) -> tuple[SceneAnalysisResult, dict[str, Path]]:
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    crop_enabled = (
        settings.enable_crop_verify
        if enable_crop_verify is None
        else enable_crop_verify
    )
    candidates = _scene_candidates(result)
    raw_candidates = [candidate.to_dict() for candidate in candidates]
    target_ids = set(result.target_decision.matched_object_ids)
    verifier = verifier or CropVerifier(settings)
    verify_results: list[dict[str, Any]] = []

    ranked = sorted(
        candidates,
        key=lambda item: float(item.detector_score or item.score),
        reverse=True,
    )
    verify_ids = {
        item.object_id
        for item in ranked[: max(0, settings.crop_verify_max_candidates)]
    }
    for candidate in candidates:
        verify_result: dict[str, Any] | None = None
        should_crop = settings.crop_verify_save_crops or (
            crop_enabled and verifier.available
        )
        if should_crop and candidate.bbox:
            crop_dir = output / "crops"
            crop_path = crop_dir / f"image_candidate_{candidate.object_id}.jpg"
            try:
                saved, _ = save_candidate_crop(
                    image_path,
                    candidate.bbox,
                    crop_path,
                    settings.crop_verify_expand_ratio,
                )
                candidate.crop_path = str(saved)
            except (OSError, ValueError) as exc:
                verify_result = {
                    "verification_failed": True,
                    "failure_reason": f"{type(exc).__name__}: {exc}",
                    "is_target": False,
                }
        if (
            verify_result is None
            and crop_enabled
            and verifier.available
            and candidate.object_id in verify_ids
            and candidate.crop_path
        ):
            verify_result = verifier.verify_crop(
                candidate.crop_path,
                profile,
                candidate.label,
                context={"full_image_target_match": candidate.object_id in target_ids},
            )
            verify_results.append(
                {"object_id": candidate.object_id, **verify_result}
            )
        apply_score_fusion(candidate, verify_result, settings)
        if not verify_result and candidate.object_id in target_ids:
            candidate.decision = "candidate"
            candidate.explanation = (
                (candidate.explanation or "")
                + "; full-image analyzer matched this object; crop verification unavailable"
            ).strip("; ")

    object_updates = {candidate.object_id: candidate for candidate in candidates}
    updated_objects = []
    for obj in result.objects:
        candidate = object_updates[obj.id]
        updated_objects.append(
            obj.model_copy(
                update={
                    "detector_score": candidate.detector_score,
                    "mask_area_ratio": candidate.mask_area_ratio,
                    "crop_path": candidate.crop_path,
                    "crop_verify": candidate.crop_verify,
                    "final_score": candidate.final_score,
                    "decision": candidate.decision,
                    "rejection_reason": candidate.rejection_reason,
                    "explanation": candidate.explanation,
                }
            )
        )

    confirmed_ids = [
        candidate.object_id
        for candidate in candidates
        if candidate.decision == "confirmed"
    ]
    updated_decision = result.target_decision
    if crop_enabled and verifier.available:
        updated_decision = result.target_decision.model_copy(
            update={
                "is_present": bool(confirmed_ids),
                "matched_object_ids": confirmed_ids,
                "match_reason_zh": (
                    "候选区域复核与多源置信度融合确认目标。"
                    if confirmed_ids
                    else "候选区域复核后尚无达到确认阈值的目标。"
                ),
                "confidence": max(
                    (
                        float(candidate.final_score or 0.0)
                        for candidate in candidates
                        if candidate.object_id in confirmed_ids
                    ),
                    default=0.0,
                ),
            }
        )
    summary = {
        "num_raw_candidates": len(candidates),
        "num_verified": len(verify_results),
        "num_confirmed": sum(item.decision == "confirmed" for item in candidates),
        "num_rejected": sum(item.decision == "rejected" for item in candidates),
    }
    config_payload = detection_config_payload(settings, crop_enabled)
    enhanced = result.model_copy(
        update={
            "objects": updated_objects,
            "target_decision": updated_decision,
            "target_profile": profile.to_dict(),
            "detection_config": config_payload,
            "candidate_summary": summary,
            "candidate_objects": [candidate.to_dict() for candidate in candidates],
        }
    )

    paths = {
        "target_profile": _write_json(profile.to_dict(), output / "target_profile.json"),
        "candidate_objects": _write_json(
            raw_candidates, output / "candidate_objects.json"
        ),
        "crop_verify_results": _write_json(
            verify_results, output / "crop_verify_results.json"
        ),
        "fused_objects": _write_json(
            [candidate.to_dict() for candidate in candidates],
            output / "fused_objects.json",
        ),
    }
    paths["detection_debug_report"] = _write_debug_report(
        profile, config_payload, summary, candidates, output / "detection_debug_report.md"
    )
    return enhanced, paths


def detection_config_payload(
    settings: Settings,
    crop_enabled: bool | None = None,
) -> dict[str, Any]:
    return {
        "image_max_side": settings.image_max_side,
        "image_detail": settings.image_detail,
        "enable_target_profile": settings.enable_target_profile,
        "enable_gdino_high_recall": settings.enable_gdino_high_recall,
        "grounding_dino_box_threshold": settings.grounding_dino_box_threshold,
        "grounding_dino_text_threshold": settings.grounding_dino_text_threshold,
        "max_detected_objects": settings.max_detected_objects,
        "enable_crop_verify": (
            settings.enable_crop_verify if crop_enabled is None else crop_enabled
        ),
        "crop_verify_expand_ratio": settings.crop_verify_expand_ratio,
        "enable_score_fusion": settings.enable_score_fusion,
        "final_target_score_threshold": settings.final_target_score_threshold,
        "final_candidate_score_threshold": settings.final_candidate_score_threshold,
        "grounding_prompt_llm_expansion_enabled": (
            settings.grounding_prompt_llm_expansion_enabled
        ),
        "grounding_prompt_require_non_empty": settings.grounding_prompt_require_non_empty,
        "grounding_prompt_retry_on_empty": settings.grounding_prompt_retry_on_empty,
        "grounding_prompt_debug_output": settings.grounding_prompt_debug_output,
        "grounding_prompt_retry_debug_output": settings.grounding_prompt_retry_debug_output,
    }


def _scene_candidates(result: SceneAnalysisResult) -> list[CandidateObject]:
    candidates = []
    for obj in result.objects:
        mask_area_ratio = obj.mask_area_ratio
        if mask_area_ratio is None:
            for attribute in obj.attributes:
                if str(attribute).startswith("mask_area_ratio="):
                    try:
                        mask_area_ratio = float(str(attribute).split("=", 1)[1])
                    except ValueError:
                        pass
        source = "llm"
        source_prompt_term = None
        for attribute in obj.attributes:
            if str(attribute).startswith("detection_source="):
                source = str(attribute).split("=", 1)[1]
            if str(attribute).startswith("source_prompt_term="):
                source_prompt_term = str(attribute).split("=", 1)[1]
        candidates.append(
            CandidateObject(
                object_id=obj.id,
                label=obj.name,
                label_zh=obj.name_zh,
                bbox=[
                    obj.bbox_2d.x1,
                    obj.bbox_2d.y1,
                    obj.bbox_2d.x2,
                    obj.bbox_2d.y2,
                ],
                score=obj.confidence,
                detector_score=obj.detector_score or obj.confidence,
                mask_area_ratio=mask_area_ratio,
                source=source,
                source_prompt_term=source_prompt_term,
                attributes=list(obj.attributes),
            )
        )
    return candidates


def _write_json(payload: Any, path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return path


def _write_debug_report(
    profile: TargetProfile,
    config: dict[str, Any],
    summary: dict[str, int],
    candidates: list[CandidateObject],
    path: Path,
) -> Path:
    prompt_plan = _read_optional_json(config.get("grounding_prompt_debug_output"))
    retry_plan = _read_optional_json(config.get("grounding_prompt_retry_debug_output"))
    rows = [
        "# Detection Debug Report",
        "",
        "## Task",
        "",
        f"- raw_task: {profile.raw_query}",
        f"- target: {profile.canonical_name_zh}",
        f"- target_category: {prompt_plan.get('target_category', profile.target_type) if prompt_plan else profile.target_type}",
        "",
        "## Grounding Prompt Plan",
        "",
    ]
    if prompt_plan:
        rows.extend(
            [
                f"- grounding_strategy: {prompt_plan.get('grounding_strategy', 'unknown')}",
                f"- prompt_source: {prompt_plan.get('prompt_source', 'unknown')}",
                f"- prompt_valid: {prompt_plan.get('is_valid_for_grounding_dino', False)}",
                f"- requires_proxy_objects: {prompt_plan.get('requires_proxy_objects', False)}",
                f"- requires_scene_confirmation: {prompt_plan.get('requires_scene_confirmation', False)}",
                f"- requires_state_verification: {prompt_plan.get('requires_state_verification', False)}",
                "",
                "### Direct Terms",
                "",
                *_markdown_terms(prompt_plan.get("direct_terms_en") or []),
                "",
                "### Proxy Object Terms",
                "",
                *_markdown_terms(prompt_plan.get("proxy_object_terms_en") or []),
                "",
                "### Context Anchor Terms",
                "",
                *_markdown_terms(prompt_plan.get("context_anchor_terms_en") or []),
                "",
                "### Final GroundingDINO Prompt",
                "",
                str(prompt_plan.get("grounding_prompt") or ""),
                "",
            ]
        )
    else:
        rows.extend(
            [
                f"- prompt_valid: {bool(profile.grounding_prompt)}",
                f"- prompt_source: {profile.resolver_source}",
                "",
                "### Final GroundingDINO Prompt",
                "",
                profile.grounding_prompt,
                "",
            ]
        )
    if retry_plan:
        rows.extend(
            [
                "## Retry Prompt Expansion",
                "",
                f"- retry_count: {retry_plan.get('retry_count', 0)}",
                f"- retry_prompt: {retry_plan.get('grounding_prompt', '')}",
                f"- reason_zh: {retry_plan.get('prompt_reason_zh', '')}",
                "",
            ]
        )
    rows.extend(
        [
            "## Detection Summary",
            "",
            json.dumps(summary, ensure_ascii=False, indent=2),
            "",
            f"- config: {json.dumps(config, ensure_ascii=False)}",
            "",
            "| object | label | detector | final | decision | reason |",
            "|---|---|---:|---:|---|---|",
        ]
    )
    for item in candidates:
        rows.append(
            f"| {item.object_id} | {item.label} | "
            f"{float(item.detector_score or 0.0):.3f} | "
            f"{float(item.final_score or 0.0):.3f} | {item.decision} | "
            f"{item.rejection_reason or ''} |"
        )
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")
    return path


def _read_optional_json(path_value: Any) -> dict[str, Any] | None:
    if not path_value:
        return None
    path = Path(str(path_value))
    if not path.is_file():
        return None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None
    return payload if isinstance(payload, dict) else None


def _markdown_terms(terms: list[Any]) -> list[str]:
    if not terms:
        return ["- (none)"]
    return [f"- {term}" for term in terms]
