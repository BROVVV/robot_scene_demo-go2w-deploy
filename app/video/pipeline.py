"""End-to-end orchestration for semantic video target search."""

from __future__ import annotations

import json
import hashlib
from dataclasses import replace
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Callable
from uuid import uuid4

from app.config import get_settings
from app.memory.observation_memory_store import ObservationMemoryStore
from app.reasoning.dynamic_detector_prompts import build_dynamic_detector_prompts
from app.reasoning.evidence_gate import EvidenceGateConfig, evaluate_candidate
from app.reasoning.llm_prior_generator import LLMPriorGenerator, LLMPriorInput
from app.reasoning.prior_usage_auditor import (
    build_prior_usage_report,
    write_prior_usage_report,
)
from app.memory.memory_retriever import MemoryRetriever
from app.memory.video_memory_store import VideoMemoryStore
from app.video.frame_analyzer import FrameAnalyzer, highlight_target_candidates
from app.video.models import FrameAnalysisResult
from app.video.object_tracker import track_objects
from app.video.spatial_context import (
    describe_target,
    find_nearby_objects,
    get_image_position,
    relative_direction_hint,
)
from app.video.target_search import search_target_in_video
from app.video.target_profile import TargetProfileResolver
from app.video.semantic_verifier import verify_video_candidates
from app.video.video_memory_builder import VideoMemoryBuilder
from app.video.video_memory_graph import build_video_memory_graph
from app.video.video_psg_builder import build_video_psg
from app.video.video_reader import read_and_sample_video
from app.video.video_scene_reasoner import VideoSceneReasoner
from app.video.video_search_summarizer import (
    build_navigation_trace,
    enrich_search_result,
    write_video_reasoning_report,
)


ProgressCallback = Callable[[int, int, str], None]


def run_video_search(
    video_path: str | Path,
    target: str,
    detector: str = "llm",
    sample_fps: float | None = None,
    max_frames: int | None = None,
    enable_knowledge: bool = False,
    output_dir: str | Path = "outputs",
    annotate: bool = True,
    enable_video_memory: bool | None = None,
    memory_store_path: str | Path | None = None,
    progress_callback: ProgressCallback | None = None,
    enable_tracking: bool | None = None,
    enable_crop_verify: bool | None = None,
    verify_every_n_frames: int | None = None,
    track_iou_threshold: float | None = None,
    target_confirm_min_frames: int | None = None,
    target_confirm_score: float | None = None,
    enable_llm_prior: bool | None = None,
    enable_observation_memory: bool | None = None,
    enable_evidence_gating: bool | None = None,
    disable_handwritten_priors: bool = False,
    disable_static_kb: bool = False,
    prior_audit: bool = False,
    include_runtime_artifacts: bool = False,
) -> tuple[dict[str, Any], dict[str, Path]]:
    if not target.strip():
        raise ValueError("Target must not be empty.")
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    settings = get_settings()
    if enable_knowledge:
        enable_llm_prior = True if enable_llm_prior is None else enable_llm_prior
        enable_observation_memory = (
            True if enable_observation_memory is None else enable_observation_memory
        )
        enable_evidence_gating = (
            True if enable_evidence_gating is None else enable_evidence_gating
        )
        disable_handwritten_priors = True
        disable_static_kb = True
    settings = replace(
        settings,
        llm_commonsense_prior_enabled=(
            settings.llm_commonsense_prior_enabled
            if enable_llm_prior is None
            else enable_llm_prior
        ),
        observation_memory_enabled=(
            settings.observation_memory_enabled
            if enable_observation_memory is None
            else enable_observation_memory
        ),
        evidence_gating_enabled=(
            settings.evidence_gating_enabled
            if enable_evidence_gating is None
            else enable_evidence_gating
        ),
        handwritten_object_priors_enabled=(
            False
            if disable_handwritten_priors
            else settings.handwritten_object_priors_enabled
        ),
        handwritten_location_priors_enabled=(
            False
            if disable_handwritten_priors
            else settings.handwritten_location_priors_enabled
        ),
        handwritten_room_priors_enabled=(
            False if disable_handwritten_priors else settings.handwritten_room_priors_enabled
        ),
        allow_handcrafted_search_rules=(
            False if disable_handwritten_priors else settings.allow_handcrafted_search_rules
        ),
        static_knowledge_base_enabled=(
            False if disable_static_kb else settings.static_knowledge_base_enabled
        ),
        prior_usage_audit_enabled=prior_audit or settings.prior_usage_audit_enabled,
    )
    sample_fps = settings.video_sample_fps if sample_fps is None else sample_fps
    max_frames = settings.video_max_frames if max_frames is None else max_frames
    tracking_enabled = (
        settings.video_enable_tracking
        if enable_tracking is None
        else enable_tracking
    )
    crop_verify_enabled = (
        settings.enable_crop_verify
        if enable_crop_verify is None
        else enable_crop_verify
    )
    scene_memory_enabled = (
        settings.video_enable_scene_memory
        if enable_video_memory is None
        else enable_video_memory
    )
    configured_store_path = Path(
        memory_store_path or settings.video_memory_store_path
    )
    video_id = _build_video_id(video_path)
    memory_store = VideoMemoryStore(
        configured_store_path,
        dedup_similarity=settings.video_memory_dedup_similarity,
    )
    retrieved_memory_context: dict[str, Any] = {
        "positive_memories": [],
        "negative_memories": [],
        "target_prior": {},
    }
    observation_store = ObservationMemoryStore(settings=settings)
    observation_memory_summaries = (
        observation_store.retrieve(target.strip(), settings.observation_memory_retrieval_top_k)
        if settings.observation_memory_enabled
        else []
    )
    if scene_memory_enabled and settings.video_enable_memory_retrieval:
        retrieved_memory_context = MemoryRetriever(memory_store).retrieve(
            target.strip(),
            top_k=settings.video_memory_retrieval_top_k,
        )
    target_profile = TargetProfileResolver().resolve(
        target.strip(),
        use_llm=detector in {"llm", "grounded_sam"},
    )
    _write_json(target_profile.to_dict(), output / "video_target_profile.json")
    metadata, frames = read_and_sample_video(
        video_path,
        sample_fps=sample_fps,
        max_frames=max_frames,
        output_dir=output / "video_frames",
    )

    analyzer = FrameAnalyzer(
        detector=detector,
        target=target.strip(),
        target_profile=target_profile,
        output_dir=output,
        annotate=annotate,
    )
    frame_results: list[FrameAnalysisResult] = []
    frame_errors: list[dict[str, Any]] = []
    total = len(frames)
    for index, frame in enumerate(frames, start=1):
        if progress_callback:
            progress_callback(index - 1, total, f"正在分析 {frame.timestamp_sec:.2f}s")
        try:
            frame_results.append(analyzer.analyze(frame))
        except Exception as exc:
            frame_errors.append(
                {
                    "frame_id": frame.frame_id,
                    "timestamp_sec": frame.timestamp_sec,
                    "image_path": str(frame.image_path),
                    "error": str(exc),
                }
            )
        if progress_callback:
            progress_callback(index, total, f"已完成 {index}/{total} 帧")

    if not frame_results:
        details = frame_errors[0]["error"] if frame_errors else "unknown error"
        raise RuntimeError(f"All sampled frames failed analysis. First error: {details}")

    semantic_verification = {"enabled": False, "attempted": 0}
    if crop_verify_enabled and detector in {"grounded_sam", "llm"}:
        semantic_verification = verify_video_candidates(
            frame_results,
            target.strip(),
            target_profile,
            output,
            settings=settings,
            every_n_frames=verify_every_n_frames,
        )

    tracks = (
        track_objects(
            frame_results,
            iou_threshold=(
                track_iou_threshold
                if track_iou_threshold is not None
                else settings.video_track_iou_threshold
            ),
            max_missing_frames=settings.video_track_max_missing_frames,
            min_hits=settings.video_track_min_hits,
            confirm_min_frames=(
                target_confirm_min_frames
                if target_confirm_min_frames is not None
                else settings.video_target_confirm_min_frames
            ),
            confirm_score=(
                target_confirm_score
                if target_confirm_score is not None
                else settings.video_target_confirm_score
            ),
        )
        if tracking_enabled
        else track_objects(
            frame_results,
            iou_threshold=1.1,
            max_missing_frames=0,
            min_hits=1,
            confirm_min_frames=1,
            confirm_score=0.0,
        )
    )
    print(
        "[VideoTracking] "
        f"tracks={len(tracks)} "
        f"confirmed={sum(track.get('decision') == 'confirmed' for track in tracks)} "
        f"crop_verified={semantic_verification.get('attempted', 0)}"
    )
    search_result = search_target_in_video(
        target=target.strip(),
        video_meta=metadata,
        frame_results=frame_results,
        tracks=tracks,
        detector=detector,
        enable_knowledge=(
            enable_knowledge and settings.allow_handcrafted_search_rules
        ),
        target_profile=target_profile,
        require_confirmed_tracks=(
            tracking_enabled and settings.video_enable_track_level_voting
        ),
    )
    for frame_result in frame_results:
        highlight_target_candidates(frame_result, target.strip())
    search_result["processing"] = {
        "successful_frames": len(frame_results),
        "failed_frames": len(frame_errors),
        "frame_errors": frame_errors,
        "semantic_verification": semantic_verification,
    }
    search_result["target_profile"] = target_profile.to_dict()

    paths: dict[str, Path] = {}
    llm_prior = LLMPriorGenerator(settings=settings).generate(
        LLMPriorInput(
            target=target.strip(),
            scene_summary=_video_scene_summary(frame_results),
            observed_objects=_video_observed_objects(frame_results),
            observed_relations=[],
            observation_memory_summaries=observation_memory_summaries,
            robot_capabilities={"platform": "quadruped"},
            language=settings.llm_prior_output_language,
        )
    )
    paths["video_llm_generated_priors"] = _write_json(
        llm_prior,
        output / "video_llm_generated_priors.json",
    )
    dynamic_prompts = build_dynamic_detector_prompts(
        target.strip(),
        llm_prior,
        _video_scene_summary(frame_results),
        max_prompts=settings.llm_prior_max_detector_prompts,
    )
    paths["video_dynamic_detector_prompts"] = _write_json(
        dynamic_prompts,
        output / "video_dynamic_detector_prompts.json",
    )
    gate_report = _gate_video_search_result(search_result, frame_results, settings)
    search_result["evidence_gating"] = gate_report
    if settings.evidence_gating_enabled:
        if gate_report.get("target_found"):
            search_result["target_found"] = True
            if gate_report.get("best_evidence"):
                search_result["best_evidence"] = gate_report["best_evidence"]
        else:
            search_result["target_found"] = False
            search_result["reason"] = "target_candidate_failed_evidence_gate"
    paths["video_evidence_gating_report"] = _write_json(
        gate_report,
        output / "video_evidence_gating_report.json",
    )
    observation_updates = _video_observation_updates(
        target.strip(),
        search_result,
        gate_report,
    )
    written_observations = []
    observation_error = None
    if settings.observation_memory_enabled and observation_updates:
        try:
            written_observations = observation_store.append_many(observation_updates)
        except ValueError as exc:
            observation_error = str(exc)
    paths["video_observation_memory_updates"] = _write_json(
        {
            "enabled": settings.observation_memory_enabled,
            "memory_store_path": settings.observation_memory_store_path,
            "retrieved_count": len(observation_memory_summaries),
            "update_count": len(observation_updates),
            "written_count": len(written_observations),
            "error": observation_error,
            "memory_updates": observation_updates,
        },
        output / "video_observation_memory_updates.json",
    )
    audit = build_prior_usage_report(
        settings=settings,
        llm_prior=llm_prior,
        dynamic_prompts=dynamic_prompts,
        evidence_report=gate_report,
        observation_memory_used=bool(observation_memory_summaries or written_observations),
        static_kb_used=settings.static_knowledge_base_enabled,
        handcrafted_priors_used=(
            settings.handwritten_object_priors_enabled
            or settings.handwritten_location_priors_enabled
            or settings.handwritten_room_priors_enabled
            or settings.allow_handcrafted_search_rules
        ),
    )
    paths["video_prior_usage_report"] = write_prior_usage_report(
        audit,
        output / "video_prior_usage_report.json",
    )
    paths["video_target_profile"] = output / "video_target_profile.json"
    paths["video_target_search"] = _write_json(
        search_result, output / "video_target_search.json"
    )
    paths["video_target_timeline"] = _write_json(
        search_result["timeline"], output / "video_target_timeline.json"
    )
    paths["video_object_tracks"] = _write_json(
        tracks, output / "video_object_tracks.json"
    )
    paths["video_track_summary"] = _write_json(
        {
            "tracking_enabled": tracking_enabled,
            "track_count": len(tracks),
            "confirmed_count": sum(
                track.get("decision") == "confirmed" for track in tracks
            ),
            "tracks": tracks,
        },
        output / "video_track_summary.json",
    )
    paths["video_crop_verify_results"] = _write_json(
        semantic_verification.get("results", []),
        output / "video_crop_verify_results.json",
    )
    paths["video_tracking_debug_report"] = _write_tracking_report(
        tracks,
        semantic_verification,
        output / "video_tracking_debug_report.md",
    )
    paths["video_candidate_regions"] = _write_json(
        search_result["candidate_regions"], output / "video_candidate_regions.json"
    )
    paths.update(build_video_memory_graph(search_result, frame_results, output))

    if scene_memory_enabled:
        reasoner = VideoSceneReasoner(
            target=target.strip(),
            target_profile=target_profile,
            enable_negative_evidence=settings.video_enable_negative_evidence,
            always_write_memory=settings.video_always_write_memory,
        )
        scene_results = []
        previous_summary: str | None = None
        for frame_result in frame_results:
            scene_result = reasoner.reason(
                video_id=video_id,
                frame=frame_result,
                memory_context=retrieved_memory_context,
                previous_frame_summary=previous_summary,
            )
            scene_results.append(scene_result)
            previous_summary = scene_result.scene_understanding.get("scene_summary")
            reasoning_path = (
                output
                / "video_scene_results"
                / f"frame_{frame_result.frame_id:06d}_reasoning.json"
            )
            _write_json(scene_result.to_dict(), reasoning_path)

        memory_updates = VideoMemoryBuilder(
            always_write=settings.video_always_write_memory,
            min_importance=settings.video_min_memory_importance,
            max_entries=settings.video_max_memory_entries_per_video,
        ).build(scene_results)
        memory_written_count = memory_store.append_many(memory_updates)
        paths["video_memory_updates"] = _write_json(
            {
                "video_id": video_id,
                "target": target.strip(),
                "memory_update_count": len(memory_updates),
                "memory_written_count": memory_written_count,
                "memory_updates": memory_updates,
            },
            output / "video_memory_updates.json",
        )
        paths["video_spatial_memory_snapshot"] = _write_json(
            {
                "memory_store_path": str(configured_store_path),
                "memory_count": len(memory_store.load_all()),
                "memories": memory_store.load_all(),
            },
            output / "video_spatial_memory_snapshot.json",
        )
        psg_result: dict[str, Any] = {"hypotheses": []}
        if settings.video_enable_video_psg:
            psg_result, psg_paths = build_video_psg(
                target=target.strip(),
                frame_results=scene_results,
                memory_updates=memory_updates,
                output_dir=output,
            )
            paths.update(psg_paths)
        navigation_trace = build_navigation_trace(
            video_id, target.strip(), scene_results
        )
        paths["video_navigation_trace"] = _write_json(
            navigation_trace, output / "video_navigation_trace.json"
        )
        search_result = enrich_search_result(
            search_result=search_result,
            frame_results=scene_results,
            memory_updates=memory_updates,
            psg_result=psg_result,
            memory_written_count=memory_written_count,
            memory_store_path=str(configured_store_path),
            retrieved_memory_context=retrieved_memory_context,
        )
        paths["video_target_search"] = _write_json(
            search_result, output / "video_target_search.json"
        )
        if settings.video_enable_reasoning_report:
            report_files = [
                str(path)
                for path in [
                    output / "video_target_search.json",
                    output / "video_memory_updates.json",
                    output / "video_predictive_scene_graph.graphml",
                    output / "video_hypotheses.json",
                    output / "video_navigation_trace.json",
                    configured_store_path,
                ]
            ]
            paths["video_reasoning_report"] = write_video_reasoning_report(
                search_result,
                output / "video_reasoning_report.md",
                output_files=report_files,
            )
    else:
        from app.video.video_report import write_video_report

        paths["video_reasoning_report"] = write_video_report(
            search_result, output / "video_reasoning_report.md"
        )
    if include_runtime_artifacts:
        search_result["_runtime_artifacts"] = {
            "frame_results": frame_results,
            "object_tracks": tracks,
            "target_profile": target_profile,
            "settings": settings,
        }
    return search_result, paths


def _write_json(payload: Any, path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return path


def _build_video_id(video_path: str | Path) -> str:
    path = Path(video_path)
    digest = hashlib.sha1(str(path.expanduser().resolve()).encode("utf-8")).hexdigest()[:8]
    stem = "".join(
        character if character.isalnum() else "_"
        for character in path.stem
    ).strip("_")
    return f"{stem or 'video'}_{digest}"


def _video_scene_summary(frame_results: list[FrameAnalysisResult]) -> str:
    summaries = [frame.scene_summary for frame in frame_results[:8] if frame.scene_summary]
    return " | ".join(summaries)


def _video_observed_objects(frame_results: list[FrameAnalysisResult]) -> list[dict[str, Any]]:
    objects: list[dict[str, Any]] = []
    for frame in frame_results[:20]:
        for obj in frame.objects[:20]:
            objects.append(
                {
                    "object_id": obj.get("object_id"),
                    "label": obj.get("label"),
                    "label_zh": obj.get("label_zh"),
                    "bbox": obj.get("bbox"),
                    "frame_id": frame.frame_id,
                    "source": "video_frame_observation",
                    "confidence": obj.get("confidence"),
                }
            )
    return objects


def _gate_video_search_result(
    search_result: dict[str, Any],
    frame_results: list[FrameAnalysisResult],
    settings: Any,
) -> dict[str, Any]:
    config = EvidenceGateConfig.from_settings(settings)
    candidate_pairs = _video_gate_candidates(search_result, frame_results)
    if not candidate_pairs:
        return {
            "target": search_result.get("task", {}).get("target"),
            "target_status": "not_observed",
            "target_found": False,
            "score": 0.0,
            "reason_zh": "视频中没有当前视觉候选。",
            "passed_rules": [],
            "blocking_rules": ["TARGET_CONFIRMATION_REQUIRE_VISUAL_EVIDENCE"],
            "candidates": [],
        }
    reports = [
        evaluate_candidate(
            _candidate_from_video_object(search_result, frame, obj),
            config,
        )
        for frame, obj in candidate_pairs
    ]
    found = any(item["target_found"] for item in reports)
    report = (
        max(
            [item for item in reports if item["target_found"]],
            key=lambda item: item["score"],
        )
        if found
        else max(reports, key=lambda item: item["score"])
    )
    best_evidence = (
        _best_video_evidence_from_report(
            report,
            search_result.get("task", {}).get("target") or "",
            frame_results,
        )
        if found
        else None
    )
    return {
        **report,
        "target": search_result.get("task", {}).get("target"),
        "reason_zh": (
            "视频中至少一个目标候选通过 bbox/crop/frame 视觉证据门控。"
            if found
            else "视频目标候选均未满足当前视觉证据门控。"
        ),
        "candidates": reports,
        "best_evidence": best_evidence,
    }


def evaluate_video_search_evidence(
    search_result: dict[str, Any],
    frame_results: list[FrameAnalysisResult],
    settings: Any | None = None,
) -> dict[str, Any]:
    """Public reuse point for live and prerecorded evidence gating."""
    return _gate_video_search_result(
        search_result,
        frame_results,
        settings or get_settings(),
    )


def _video_gate_candidates(
    search_result: dict[str, Any],
    frame_results: list[FrameAnalysisResult],
) -> list[tuple[FrameAnalysisResult, dict[str, Any]]]:
    candidates = []
    for frame in frame_results:
        for obj in frame.objects:
            if obj.get("is_target_candidate"):
                candidates.append((frame, obj))
    if candidates:
        return candidates

    best = search_result.get("best_evidence")
    best_id = best.get("object_id") if isinstance(best, dict) else None
    if best_id is None:
        return []
    for frame in frame_results:
        for obj in frame.objects:
            if obj.get("object_id") == best_id:
                return [(frame, obj)]
    return []


def _candidate_from_video_object(
    search_result: dict[str, Any],
    frame: FrameAnalysisResult,
    obj: dict[str, Any],
) -> dict[str, Any]:
    crop_verify = obj.get("crop_verify") or {}
    crop_score = crop_verify.get("target_match_score")
    if crop_score is None and crop_verify.get("is_target") is True:
        crop_score = obj.get("final_score") or obj.get("confidence")
    return {
        "candidate_id": obj.get("object_id"),
        "label": obj.get("label"),
        "label_zh": obj.get("label_zh"),
        "source": "visual_detector",
        "has_visual_evidence": True,
        "frame_id": frame.frame_id,
        "image_path": frame.image_path,
        "crop_path": obj.get("crop_path"),
        "bbox": obj.get("bbox"),
        "mask_area_ratio": obj.get("mask_area_ratio"),
        "detector_score": obj.get("detector_score") or obj.get("confidence"),
        "crop_verify_score": crop_score,
        "fused_score": obj.get("final_score"),
        "source_detector": search_result.get("task", {}).get("detector"),
        "timestamp_sec": frame.timestamp_sec,
        "annotated_frame_path": frame.annotated_frame_path,
    }


def _best_video_evidence_from_report(
    report: dict[str, Any],
    target: str,
    frame_results: list[FrameAnalysisResult],
) -> dict[str, Any] | None:
    evidence = report.get("evidence") or {}
    object_id = evidence.get("candidate_id")
    if object_id is None:
        return None
    for frame in frame_results:
        for obj in frame.objects:
            if obj.get("object_id") != object_id:
                continue
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
                "evidence_score": report.get("score"),
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
                "evidence_source": "video_evidence_gate",
                "match_reason": report.get("reason_zh") or "",
            }
    return None


def _video_observation_updates(
    target: str,
    search_result: dict[str, Any],
    gate_report: dict[str, Any],
) -> list[dict[str, Any]]:
    updates = []
    for candidate_report in gate_report.get("candidates") or []:
        evidence = candidate_report.get("evidence") or {}
        if not evidence.get("bbox"):
            continue
        updates.append(
            {
                "memory_id": f"mem_{uuid4().hex[:12]}",
                "memory_type": "object_observation",
                "label": evidence.get("label_zh") or evidence.get("label") or target,
                "target_related": True,
                "evidence": {
                    "frame_id": evidence.get("frame_id"),
                    "image_path": evidence.get("image_path"),
                    "crop_path": evidence.get("crop_path"),
                    "bbox": evidence.get("bbox"),
                    "mask_area_ratio": evidence.get("mask_area_ratio"),
                    "detector_score": evidence.get("detector_score"),
                    "crop_verify_score": evidence.get("crop_verify_score"),
                    "source_detector": evidence.get("source_detector"),
                },
                "spatial_context": {
                    "near": [
                        item.get("label_zh") or item.get("label")
                        for item in (search_result.get("best_evidence") or {}).get(
                            "nearby_objects",
                            [],
                        )
                    ],
                    "on": [],
                    "under": [],
                },
                "confirmed_by_user": False,
                "confirmed_by_visual_gate": candidate_report.get("target_found") is True,
                "created_at": datetime.now(UTC).isoformat(),
            }
        )
    return updates


def _write_tracking_report(
    tracks: list[dict[str, Any]],
    verification: dict[str, Any],
    path: Path,
) -> Path:
    lines = [
        "# Video tracking debug report",
        "",
        f"- tracks: {len(tracks)}",
        f"- confirmed: {sum(track.get('decision') == 'confirmed' for track in tracks)}",
        f"- crop verification attempted: {verification.get('attempted', 0)}",
        "",
        "| track | label | frames | score | decision |",
        "|---|---|---:|---:|---|",
    ]
    for track in tracks:
        lines.append(
            f"| {track.get('track_id')} | {track.get('label')} | "
            f"{track.get('frame_count', 0)} | {float(track.get('final_score', 0.0)):.3f} | "
            f"{track.get('decision')} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path
