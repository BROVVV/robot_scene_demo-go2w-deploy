"""Target matching, evidence scoring, and candidate-region reasoning."""

from __future__ import annotations

import re
from typing import Any

from app.video.models import FrameAnalysisResult, VideoMetadata
from app.video.object_tracker import track_observation_counts
from app.video.spatial_context import (
    bbox_area,
    bbox_center,
    describe_target,
    find_nearby_objects,
    get_image_position,
    relative_direction_hint,
)
from app.video.target_context_rules import TARGET_CONTEXT_RULES
from app.video.target_profile import TargetProfile, TargetProfileResolver


TARGET_SYNONYMS = {
    "手机": ["手机", "phone", "mobile phone", "cell phone", "iphone", "smartphone"],
    "电视": [
        "电视",
        "电视机",
        "television",
        "tv",
        "tv screen",
        "monitor",
        "display",
        "flat screen",
        "large black screen",
        "screen-like rectangle",
    ],
    "水杯": ["水杯", "杯子", "cup", "mug", "bottle"],
    "充电器": ["充电器", "charger", "adapter", "charging cable", "cable"],
    "钥匙": ["钥匙", "key", "keys", "keychain"],
    "鞋子": ["鞋", "鞋子", "shoe", "shoes", "sneaker", "sneakers"],
    "厕所": [
        "厕所",
        "洗手间",
        "卫生间",
        "盥洗室",
        "公厕",
        "toilet",
        "restroom",
        "bathroom",
        "washroom",
        "lavatory",
        "wc",
        "toilet sign",
        "restroom sign",
        "bathroom sign",
    ],
}


def normalize_label(label: str) -> str:
    return " ".join(
        re.sub(r"[^a-z0-9\u4e00-\u9fff]+", " ", label.lower()).split()
    )


def canonical_target(target: str) -> str:
    normalized = normalize_label(target)
    for canonical, synonyms in TARGET_SYNONYMS.items():
        for item in sorted(synonyms, key=len, reverse=True):
            term = normalize_label(item)
            if normalized == term:
                return canonical
            if _contains_target_term(normalized, term):
                return canonical
    return TargetProfileResolver().resolve(
        target,
        use_llm=False,
    ).canonical_name_zh


def target_label_match_score(
    target: str,
    label: str,
    label_zh: str = "",
    target_profile: TargetProfile | None = None,
) -> float:
    profile = target_profile or TargetProfileResolver().resolve(target, use_llm=False)
    target_key = canonical_target(target)
    target_terms = {
        normalize_label(item)
        for item in [
            target,
            target_key,
            *TARGET_SYNONYMS.get(target_key, []),
            *profile.direct_terms(),
        ]
        if item
    }
    object_terms = {normalize_label(label), normalize_label(label_zh)}
    if target_terms & object_terms:
        return 1.0
    if any(
        target_term and object_term and target_term in object_term
        for target_term in target_terms
        for object_term in object_terms
    ):
        return 0.8
    return 0.0


def search_target_in_video(
    target: str,
    video_meta: VideoMetadata | dict[str, Any],
    frame_results: list[FrameAnalysisResult],
    tracks: list[dict[str, Any]],
    detector: str = "llm",
    enable_knowledge: bool = False,
    target_profile: TargetProfile | None = None,
    require_confirmed_tracks: bool = False,
) -> dict[str, Any]:
    meta = video_meta.to_dict() if isinstance(video_meta, VideoMetadata) else dict(video_meta)
    profile = target_profile or TargetProfileResolver().resolve(target, use_llm=False)
    track_counts = track_observation_counts(tracks)
    track_by_id = {str(track.get("track_id")): track for track in tracks}
    direct_candidates: list[dict[str, Any]] = []
    timeline: list[dict[str, Any]] = []

    for frame in frame_results:
        semantic_verification = frame.metadata.get("semantic_verification")
        target_decision = (
            semantic_verification
            if detector == "grounded_sam" and semantic_verification is not None
            else frame.metadata.get("target_decision") or {}
        )
        decision_matches = set(target_decision.get("matched_object_ids") or [])
        decision_present = bool(target_decision.get("is_present"))
        semantic_frame_confirmed = (
            detector == "grounded_sam"
            and semantic_verification is not None
            and decision_present
        )
        verification_failed = bool(
            semantic_verification
            and semantic_verification.get("attempted")
            and str(semantic_verification.get("reason", "")).startswith(
                "verification_failed:"
            )
        )
        verified_bboxes = (
            semantic_verification.get("matched_bboxes", [])
            if semantic_verification
            else []
        )
        for obj in frame.objects:
            label_match_score = target_label_match_score(
                target,
                str(obj.get("label", "")),
                str(obj.get("label_zh", "")),
                target_profile=profile,
            )
            match_score = label_match_score
            matched_by_frame_reasoner = (
                detector != "grounded_sam"
                and
                decision_present
                and str(obj.get("source_object_id")) in decision_matches
            )
            if matched_by_frame_reasoner:
                match_score = max(match_score, 0.5)
            if match_score <= 0:
                continue
            constrained_grounded_candidate = (
                detector == "grounded_sam"
                and bool(
                    profile.attributes
                    or profile.relation_constraints
                )
                and semantic_verification is not None
            )
            if constrained_grounded_candidate and not decision_present:
                if not (
                    verification_failed
                    and label_match_score >= 0.8
                    and float(obj.get("confidence", 0.0)) >= 0.45
                ):
                    continue
            verification_alignment = _verification_alignment(
                obj.get("bbox", []),
                verified_bboxes,
            )
            if (
                constrained_grounded_candidate
                and decision_present
                and verified_bboxes
                and verification_alignment < 0.5
            ):
                continue
            observation_count = track_counts[str(obj.get("track_id"))]
            track = track_by_id.get(str(obj.get("track_id")), {})
            if require_confirmed_tracks and track.get("decision") != "confirmed":
                continue
            if (
                detector == "grounded_sam"
                and not matched_by_frame_reasoner
                and float(obj.get("confidence", 0.0)) < 0.35
                and observation_count < 2
            ):
                continue
            obj["is_target_candidate"] = True
            area_score = min(1.0, bbox_area(obj.get("bbox", [])) / 0.15)
            stability = min(1.0, observation_count / 3.0)
            score = (
                0.45 * float(obj.get("confidence", 0.0))
                + 0.30 * match_score
                + 0.10 * area_score
                + 0.10 * stability
                + 0.05 * verification_alignment
            )
            candidate = {
                "frame": frame,
                "object": obj,
                "score": round(score, 4),
                "match_score": match_score,
                "label_match_score": label_match_score,
                "verification_alignment": round(verification_alignment, 4),
                "evidence_source": (
                    "open_vocabulary_label_and_semantic_verifier"
                    if semantic_frame_confirmed
                    else "open_vocabulary_label_and_frame_target_decision"
                    if matched_by_frame_reasoner and label_match_score > 0
                    else "frame_target_decision"
                    if matched_by_frame_reasoner
                    else "open_vocabulary_label"
                ),
                "match_reason": (
                    target_decision.get("match_reason_zh")
                    or target_decision.get("reason", "")
                ),
                "track_final_score": track.get("final_score"),
            }
            direct_candidates.append(candidate)
            timeline.append(
                {
                    "timestamp_sec": frame.timestamp_sec,
                    "frame_id": frame.frame_id,
                    "type": "direct_detection",
                    "confidence": obj.get("confidence", 0.0),
                    "score": round(score, 4),
                    "label": obj.get("label"),
                    "track_id": obj.get("track_id"),
                    "description": f"检测到与“{target}”匹配的 {obj.get('label_zh') or obj.get('label')}。",
                    "evidence_source": candidate["evidence_source"],
                }
            )

    direct_candidates.sort(key=lambda item: item["score"], reverse=True)
    candidate_regions = _candidate_regions(
        target,
        frame_results,
        direct_candidates,
        enable_knowledge,
        profile,
    )
    timeline.extend(_region_timeline(candidate_regions))
    timeline.sort(key=lambda item: (item["timestamp_sec"], item["type"]))

    best_evidence = (
        _best_evidence(target, direct_candidates[0])
        if direct_candidates
        else None
    )
    found = best_evidence is not None
    suggestion = _search_suggestion(
        target,
        best_evidence,
        candidate_regions,
        profile,
    )
    result: dict[str, Any] = {
        "task": {
            "target": target,
            "canonical_target": profile.canonical_name_zh,
            "resolved_target": profile.canonical_name_zh,
            "video_path": meta.get("video_path"),
            "detector": detector,
            "enable_knowledge": enable_knowledge,
        },
        "video_meta": meta,
        "target_found": found,
        "best_evidence": best_evidence,
        "candidate_regions": candidate_regions,
        "timeline": timeline,
        "navigation_interpretation": {
            "can_generate_real_navigation": False,
            "reason": (
                "当前视频没有 odom / SLAM 位姿，因此不能可靠生成从机器狗"
                "当前位置到目标的真实导航路线。"
            ),
            "suggestion": suggestion,
        },
        "target_profile": profile.to_dict(),
    }
    if not found:
        result["reason"] = (
            "target_not_observed_but_contextual_clues_exist"
            if candidate_regions
            else "target_not_observed_and_no_strong_contextual_clue"
        )
    return result


def _best_evidence(target: str, candidate: dict[str, Any]) -> dict[str, Any]:
    frame: FrameAnalysisResult = candidate["frame"]
    obj = candidate["object"]
    nearby = find_nearby_objects(obj, frame.objects)
    position = get_image_position(obj.get("bbox", []))
    return {
        "timestamp_sec": frame.timestamp_sec,
        "frame_id": frame.frame_id,
        "frame_path": frame.image_path,
        "annotated_frame_path": frame.annotated_frame_path,
        "object_id": obj.get("object_id"),
        "track_id": obj.get("track_id"),
        "label": obj.get("label"),
        "label_zh": obj.get("label_zh"),
        "confidence": obj.get("confidence"),
        "evidence_score": candidate["score"],
        "bbox": obj.get("bbox"),
        "mask_area_ratio": obj.get("mask_area_ratio"),
        "image_position": position,
        "relative_direction_hint": relative_direction_hint(position),
        "nearby_objects": nearby,
        "description": describe_target(
            str(obj.get("label_zh") or target),
            position,
            nearby,
        ),
        "evidence_source": candidate.get("evidence_source"),
        "match_reason": candidate.get("match_reason") or "",
    }


def _candidate_regions(
    target: str,
    frame_results: list[FrameAnalysisResult],
    direct_candidates: list[dict[str, Any]],
    enable_knowledge: bool,
    target_profile: TargetProfile,
) -> list[dict[str, Any]]:
    regions = []
    used_frames: set[int] = set()
    for candidate in direct_candidates[:5]:
        frame: FrameAnalysisResult = candidate["frame"]
        if frame.frame_id in used_frames:
            continue
        used_frames.add(frame.frame_id)
        nearby = find_nearby_objects(candidate["object"], frame.objects)
        regions.append(
            {
                "region_id": f"region_frame_{frame.frame_id:06d}_direct",
                "timestamp_sec": frame.timestamp_sec,
                "frame_id": frame.frame_id,
                "priority": "high" if candidate["score"] >= 0.65 else "medium",
                "reason": f"直接检测到与“{target}”匹配的目标。",
                "nearby_objects": [
                    item.get("label_zh") or item.get("label") for item in nearby
                ],
                "frame_path": frame.image_path,
                "score": candidate["score"],
                "source": "direct_detection",
            }
        )

    rule = TARGET_CONTEXT_RULES.get(canonical_target(target), {})
    context_terms = [
        *(target_profile.context_terms() if enable_knowledge else []),
        *(rule.get("likely_objects", []) if enable_knowledge else []),
    ]
    if not context_terms:
        return regions
    normalized_context_terms = [normalize_label(item) for item in context_terms]
    contextual = []
    for frame in frame_results:
        labels = [
            str(obj.get("label_zh") or obj.get("label") or "")
            for obj in frame.objects
        ]
        matched = [
            label
            for label in labels
            if any(
                term in normalize_label(label) or normalize_label(label) in term
                for term in normalized_context_terms
                if term and normalize_label(label)
            )
        ]
        if not matched:
            continue
        hit_count = len(set(matched))
        contextual.append((hit_count, frame, list(dict.fromkeys(matched))))

    contextual.sort(key=lambda item: (-item[0], item[1].timestamp_sec))
    for hit_count, frame, matched in contextual:
        if frame.frame_id in used_frames or len(regions) >= 8:
            continue
        score = min(0.6, 0.25 + 0.1 * hit_count)
        regions.append(
            {
                "region_id": f"region_frame_{frame.frame_id:06d}_context",
                "timestamp_sec": frame.timestamp_sec,
                "frame_id": frame.frame_id,
                "priority": "medium" if hit_count >= 2 else "low",
                "reason": f"画面出现与“{target}”相关的上下文物体：{'、'.join(matched)}。",
                "nearby_objects": matched,
                "likely_regions": (
                    (target_profile.likely_regions_zh or rule.get("likely_regions", []))
                    if enable_knowledge
                    else []
                ),
                "frame_path": frame.image_path,
                "score": round(score, 3),
                "source": "context_rule",
            }
        )
    return regions


def _region_timeline(regions: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [
        {
            "timestamp_sec": item["timestamp_sec"],
            "frame_id": item["frame_id"],
            "type": "candidate_region",
            "confidence": item["score"],
            "score": item["score"],
            "label": None,
            "track_id": None,
            "description": item["reason"],
        }
        for item in regions
        if item["source"] == "context_rule"
    ]


def _search_suggestion(
    target: str,
    best_evidence: dict[str, Any] | None,
    regions: list[dict[str, Any]],
    target_profile: TargetProfile,
) -> str:
    if best_evidence:
        references = [
            item.get("label_zh") or item.get("label")
            for item in best_evidence["nearby_objects"]
        ]
        reference_text = f"（参照物：{'、'.join(references)}）" if references else ""
        return (
            f"建议回到视频第 {best_evidence['timestamp_sec']:.2f} 秒附近的视觉区域"
            f"{reference_text}，在 {best_evidence['relative_direction_hint']} 方向继续确认。"
        )
    rule = TARGET_CONTEXT_RULES.get(canonical_target(target), {})
    if regions and target_profile.search_hint_zh:
        return target_profile.search_hint_zh
    if regions and rule.get("search_hint"):
        return rule["search_hint"]
    if regions:
        return "优先回到候选区域对应的时间片和视觉场景继续搜索。"
    return "视频中没有直接目标或足够强的上下文线索，建议补充覆盖更多区域的视频。"


def _contains_target_term(text: str, term: str) -> bool:
    if not term:
        return False
    if any("\u4e00" <= character <= "\u9fff" for character in term):
        return term in text
    return f" {term} " in f" {text} "


def _verification_alignment(
    bbox: list[float] | tuple[float, ...],
    verified_bboxes: list[list[float]],
) -> float:
    if not verified_bboxes:
        return 0.0
    center = bbox_center(bbox)
    distances = [
        ((center[0] - other[0]) ** 2 + (center[1] - other[1]) ** 2) ** 0.5
        for other in (bbox_center(item) for item in verified_bboxes)
    ]
    return max(0.0, 1.0 - min(distances) / 0.5)
