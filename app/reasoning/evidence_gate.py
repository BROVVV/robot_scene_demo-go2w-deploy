"""Visual evidence gate for target confirmation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from app.config import Settings, get_settings
from app.schemas import SceneAnalysisResult


@dataclass(frozen=True)
class EvidenceGateConfig:
    enabled: bool = True
    require_visual_evidence: bool = True
    require_bbox: bool = True
    require_crop_verify: bool = True
    crop_verify_requested: bool = False
    crop_verify_available: bool = True
    require_mask: bool = False
    min_score: float = 0.72

    @classmethod
    def from_settings(cls, settings: Settings | None = None) -> "EvidenceGateConfig":
        config = settings or get_settings()
        crop_verify_requested = config.target_confirmation_require_crop_verify
        crop_verify_available = bool(config.siliconflow_api_key) and config.enable_crop_verify
        return cls(
            enabled=config.evidence_gating_enabled,
            require_visual_evidence=config.target_confirmation_require_visual_evidence,
            require_bbox=config.target_confirmation_require_bbox,
            require_crop_verify=crop_verify_requested and crop_verify_available,
            crop_verify_requested=crop_verify_requested,
            crop_verify_available=crop_verify_available,
            require_mask=config.target_confirmation_require_mask,
            min_score=config.target_confirmation_min_score,
        )


def gate_scene_target(
    scene: SceneAnalysisResult,
    settings: Settings | None = None,
) -> tuple[SceneAnalysisResult, dict[str, Any]]:
    config = EvidenceGateConfig.from_settings(settings)
    report = evaluate_scene_target(scene, config)
    if not config.enabled or report["target_found"] == scene.target_decision.is_present:
        return scene, report
    decision = scene.target_decision.model_copy(
        update={
            "is_present": report["target_found"],
            "matched_object_ids": (
                scene.target_decision.matched_object_ids
                if report["target_found"]
                else []
            ),
            "match_reason_zh": report["reason_zh"],
            "confidence": report["score"],
        }
    )
    route = scene.route_plan
    if not report["target_found"] and route.route_type == "approach_visible_target":
        route = route.model_copy(
            update={
                "route_type": "explore_likely_location",
                "summary_zh": "目标尚未通过视觉证据门控，继续搜索并重观测候选区域。",
            }
        )
    return scene.model_copy(update={"target_decision": decision, "route_plan": route}), report


def evaluate_scene_target(
    scene: SceneAnalysisResult,
    config: EvidenceGateConfig | None = None,
) -> dict[str, Any]:
    gate = config or EvidenceGateConfig.from_settings()
    if not gate.enabled:
        return {
            "target": scene.target_decision.target_text,
            "target_status": (
                "visual_confirmed" if scene.target_decision.is_present else "not_observed"
            ),
            "target_found": scene.target_decision.is_present,
            "score": scene.target_decision.confidence,
            "reason_zh": "视觉证据门控已关闭。",
            "passed_rules": ["EVIDENCE_GATING_DISABLED"],
            "blocking_rules": [],
            "candidates": [],
        }
    matched_ids = set(scene.target_decision.matched_object_ids)
    candidates = [
        _candidate_from_object(obj, scene)
        for obj in scene.objects
        if obj.id in matched_ids or obj.decision in {"confirmed", "candidate"}
    ]
    if not candidates:
        status = "llm_hypothesis_only" if scene.target_decision.is_present else "not_observed"
        return {
            "target": scene.target_decision.target_text,
            "target_status": status,
            "target_found": False,
            "score": 0.0,
            "reason_zh": "没有可追溯 bbox/crop/frame 视觉候选，不能确认目标。",
            "passed_rules": [],
            "blocking_rules": ["TARGET_CONFIRMATION_REQUIRE_VISUAL_EVIDENCE"],
            "candidates": [],
        }
    reports = [evaluate_candidate(candidate, gate) for candidate in candidates]
    best = max(reports, key=lambda item: item["score"])
    found = any(item["target_found"] for item in reports)
    if found:
        confirmed = max(
            [item for item in reports if item["target_found"]],
            key=lambda item: item["score"],
        )
        return {
            **confirmed,
            "target": scene.target_decision.target_text,
            "candidates": reports,
        }
    return {
        **best,
        "target": scene.target_decision.target_text,
        "target_status": "visual_candidate",
        "target_found": False,
        "reason_zh": "当前存在候选视觉区域，但尚未满足目标确认门控。",
        "candidates": reports,
    }


def evaluate_candidate(candidate: dict[str, Any], config: EvidenceGateConfig) -> dict[str, Any]:
    passed: list[str] = []
    blocked: list[str] = []
    if candidate.get("source") == "llm_commonsense":
        blocked.append("LLM_COMMONSENSE_CANNOT_CONFIRM")
    if config.require_visual_evidence and not candidate.get("has_visual_evidence"):
        blocked.append("TARGET_CONFIRMATION_REQUIRE_VISUAL_EVIDENCE")
    else:
        passed.append("TARGET_CONFIRMATION_REQUIRE_VISUAL_EVIDENCE")
    if config.require_bbox and not candidate.get("bbox"):
        blocked.append("TARGET_CONFIRMATION_REQUIRE_BBOX")
    else:
        passed.append("TARGET_CONFIRMATION_REQUIRE_BBOX")
    if config.require_crop_verify and candidate.get("crop_verify_score") is None:
        blocked.append("TARGET_CONFIRMATION_REQUIRE_CROP_VERIFY")
    else:
        passed.append(_crop_verify_rule_name(candidate, config))
    if config.require_mask and candidate.get("mask_area_ratio") is None:
        blocked.append("TARGET_CONFIRMATION_REQUIRE_MASK")
    elif config.require_mask:
        passed.append("TARGET_CONFIRMATION_REQUIRE_MASK")
    score = _candidate_score(candidate)
    if score < config.min_score:
        blocked.append("TARGET_CONFIRMATION_MIN_SCORE")
    else:
        passed.append("TARGET_CONFIRMATION_MIN_SCORE")
    found = not blocked
    return {
        "candidate_id": candidate.get("candidate_id"),
        "target_status": "visual_confirmed" if found else "visual_candidate",
        "target_found": found,
        "score": score,
        "reason_zh": (
            "候选区域具有 bbox/crop/frame 等视觉证据，并通过门控。"
            if found
            else "候选区域证据不足，不能确认已找到目标。"
        ),
        "passed_rules": passed,
        "blocking_rules": blocked,
        "evidence": candidate,
    }


def _candidate_from_object(obj: Any, scene: SceneAnalysisResult) -> dict[str, Any]:
    crop_score = None
    if obj.crop_verify:
        crop_score = obj.crop_verify.get("target_match_score")
        if crop_score is None and obj.crop_verify.get("is_target") is True:
            crop_score = obj.final_score or obj.confidence
    bbox = [obj.bbox_2d.x1, obj.bbox_2d.y1, obj.bbox_2d.x2, obj.bbox_2d.y2]
    return {
        "candidate_id": obj.id,
        "label": obj.name,
        "label_zh": obj.name_zh,
        "source": "visual_detector",
        "has_visual_evidence": True,
        "frame_id": "single_image",
        "image_path": None,
        "crop_path": obj.crop_path,
        "bbox": bbox,
        "mask_area_ratio": obj.mask_area_ratio,
        "detector_score": obj.detector_score or obj.confidence,
        "crop_verify_score": crop_score,
        "fused_score": obj.final_score,
        "source_detector": scene.detection_config or {},
    }


def _candidate_score(candidate: dict[str, Any]) -> float:
    for key in ("fused_score", "crop_verify_score", "detector_score"):
        value = candidate.get(key)
        if value is not None:
            try:
                return max(0.0, min(1.0, float(value)))
            except (TypeError, ValueError):
                continue
    return 0.0


def _crop_verify_rule_name(
    candidate: dict[str, Any],
    config: EvidenceGateConfig,
) -> str:
    if candidate.get("crop_verify_score") is not None:
        return "TARGET_CONFIRMATION_REQUIRE_CROP_VERIFY"
    if config.crop_verify_requested and not config.crop_verify_available:
        return "TARGET_CONFIRMATION_CROP_VERIFY_UNAVAILABLE"
    if not config.crop_verify_requested:
        return "TARGET_CONFIRMATION_CROP_VERIFY_NOT_REQUIRED"
    return "TARGET_CONFIRMATION_REQUIRE_CROP_VERIFY"
