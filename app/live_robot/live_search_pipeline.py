"""Live-frame orchestration reusing the established video perception stack."""

from __future__ import annotations

import json
import shutil
from pathlib import Path
from typing import Any

from app.config import get_settings
from app.live_robot.frame_bundle_reader import FrameBundle
from app.live_robot.search_state_machine import (
    SearchStateMachine,
    SensorSnapshot,
    VisualEvidence,
)
from app.reasoning.semantic_navigation.goal_graph_builder import GoalGraphBuilder
from app.reasoning.semantic_navigation.graph_matcher import SemanticNavigationGraphMatcher
from app.reasoning.semantic_navigation.models import GraphMatchState, SearchReasoningContext
from app.reasoning.semantic_navigation.router import SemanticSearchController
from app.reasoning.semantic_navigation.semantic_memory import SemanticSearchMemory
from app.reasoning.semantic_navigation.auxiliary_hints import build_psg_auxiliary_hints
from app.memory.observation_memory_store import ObservationMemoryStore
from app.video.frame_analyzer import FrameAnalyzer
from app.video.frame_scene_parser import observation_from_frame_analysis
from app.video.models import VideoFrame, VideoMetadata
from app.video.navigation_graph_exporter import (
    write_navigation_topology_graphml,
    write_navigation_topology_json,
)
from app.video.object_tracker import VideoObjectTracker, track_objects
from app.video.observed_scene_graph_builder import ObservedSceneGraphBuilder
from app.video.pipeline import evaluate_video_search_evidence
from app.video.semantic_verifier import verify_video_candidates
from app.reasoning.target_profile import TargetProfileResolver
from app.video.target_search import search_target_in_video
from app.video.video_graph_io import write_json, write_scene_graph_graphml, write_scene_graph_json
from app.video.video_navigation_topology_builder import VideoNavigationTopologyBuilder
from app.video.video_target_state import apply_target_state


def run_live_bundle_search(
    bundles: list[FrameBundle],
    *,
    target: str,
    detector: str,
    output_dir: str | Path,
    search_mode: str = "observe_only",
    annotate: bool = True,
    enable_crop_verify: bool = True,
    use_llm_profile: bool = True,
    semantic_reasoning: bool = False,
    search_reasoner: str = "legacy",
    search_reasoner_mode: str = "shadow",
) -> dict[str, Any]:
    if not bundles:
        raise ValueError("at least one complete frame bundle is required")
    if not target.strip():
        raise ValueError("target must not be empty")
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    latest_health = dict(bundles[-1].payload["sensor_health"])
    _write_session_contract(
        output,
        target=target,
        detector=detector,
        search_mode=search_mode,
        health=latest_health,
    )
    machine = SearchStateMachine(mode=search_mode, motion_allowed=False)
    machine.start()
    snapshot = sensor_snapshot_from_health(latest_health)
    return _run_after_sensor_snapshot(
        bundles,
        target=target,
        detector=detector,
        output=output,
        search_mode=search_mode,
        annotate=annotate,
        enable_crop_verify=enable_crop_verify,
        latest_health=latest_health,
        machine=machine,
        snapshot=snapshot,
        use_llm_profile=use_llm_profile,
        semantic_reasoning=semantic_reasoning,
        search_reasoner=search_reasoner,
        search_reasoner_mode=search_reasoner_mode,
    )


def sensor_snapshot_from_health(latest_health: dict[str, Any]) -> SensorSnapshot:
    """Keep intrinsic, extrinsic, and fusion readiness semantically separate."""
    return SensorSnapshot(
        camera_fresh=bool(latest_health.get("camera")),
        lidar_fresh=bool(latest_health.get("lidar")),
        robot_stationary=True,
        tf_ready=bool(latest_health.get("tf")),
        extrinsics_ready=bool(latest_health.get("rgb_lidar_extrinsics")),
        lio_fresh=bool(latest_health.get("lio")),
    )


def _run_after_sensor_snapshot(
    bundles: list[FrameBundle],
    *,
    target: str,
    detector: str,
    output: Path,
    search_mode: str,
    annotate: bool,
    enable_crop_verify: bool,
    latest_health: dict[str, Any],
    machine: SearchStateMachine,
    snapshot: SensorSnapshot,
    use_llm_profile: bool = True,
    semantic_reasoning: bool = False,
    search_reasoner: str = "legacy",
    search_reasoner_mode: str = "shadow",
) -> dict[str, Any]:
    machine.sensors(snapshot)
    if machine.state.value == "WAIT_FOR_SENSORS":
        return _write_blocked_session(
            output, target, detector, latest_health, machine, "camera_or_lidar_not_fresh"
        )

    # The ROS bridge prunes old spool bundles while remote inference is running.
    # Preserve the accepted sensor evidence before any potentially slow API call.
    frames = _video_frames(bundles, snapshot_dir=output / "input_frames")
    machine.observation_elapsed(machine.stop_settle_seconds)
    machine.scene_understood()
    profile = TargetProfileResolver().resolve(
        target.strip(),
        use_llm=bool(use_llm_profile) and detector in {"llm", "grounded_sam"},
    )
    write_json(profile.to_dict(), output / "target_profile.json")
    analyzer = FrameAnalyzer(
        detector=detector,
        target=target.strip(),
        target_profile=profile,
        output_dir=output,
        annotate=annotate,
        scene_dir_name="frame_scene_results",
        annotated_dir_name="annotated_frames",
    )
    frame_results = [analyzer.analyze(frame) for frame in frames]
    settings = get_settings()
    crop_report: dict[str, Any] = {"enabled": False, "attempted": 0}
    if enable_crop_verify and detector in {"llm", "grounded_sam"}:
        crop_report = verify_video_candidates(
            frame_results, target.strip(), profile, output, settings=settings
        )
    tracks = track_objects(
        frame_results,
        iou_threshold=settings.video_track_iou_threshold,
        max_missing_frames=settings.video_track_max_missing_frames,
        min_hits=settings.video_track_min_hits,
        confirm_min_frames=settings.video_target_confirm_min_frames,
        confirm_score=settings.video_target_confirm_score,
    )
    metadata = VideoMetadata(
        video_path="live_frame_bundle",
        fps=0.0,
        duration_sec=frames[-1].timestamp_sec,
        frame_count=len(frames),
        width=frames[-1].width,
        height=frames[-1].height,
        sampled_keyframes=len(frames),
    )
    search = search_target_in_video(
        target=target.strip(),
        video_meta=metadata,
        frame_results=frame_results,
        tracks=tracks,
        detector=detector,
        enable_knowledge=False,
        target_profile=profile,
        require_confirmed_tracks=True,
    )
    observations = [observation_from_frame_analysis(item) for item in frame_results]
    graph_tracks = VideoObjectTracker().build_tracks(observations)
    scene_graph = ObservedSceneGraphBuilder().build(observations, graph_tracks)
    gate = evaluate_video_search_evidence(search, frame_results, settings)
    gate = enforce_relation_evidence_gate(gate, profile, scene_graph)
    search["evidence_gating"] = gate
    search["target_found"] = bool(gate.get("target_found"))
    apply_target_state(search)
    machine.detection(bool(search.get("best_evidence")))
    if machine.state.value == "VERIFY_TARGET":
        evidence = _visual_evidence(search, gate, tracks, frame_results)
        machine.verify(evidence)
        if machine.state.value == "LOCALIZE_TARGET":
            machine.localization(False)
            machine.finish_confirmed()
        else:
            machine.next_view_unavailable()
    else:
        machine.next_view_unavailable()
    search["spatial_status"] = machine.spatial_state
    search["navigation_observation_goal"] = None
    search["navigation_goal_block_reason"] = "target_has_no_validated_3d_pose"
    search["runtime_source"] = "live_frame_bundle"
    search["detector_runtime_is_mock"] = detector == "mock"

    if semantic_reasoning and search_reasoner in {"semantic_navigation", "hybrid"}:
        controller = SemanticSearchController(
            profile,
            backend=search_reasoner,
            partial_threshold=settings.live_search_graph_match_partial_threshold,
            strong_threshold=settings.live_search_graph_match_strong_threshold,
        )
        semantic_memory = SemanticSearchMemory(
            default_ttl_sec=settings.live_search_negative_memory_ttl_seconds,
            observation_store=(
                ObservationMemoryStore(settings=settings)
                if settings.live_search_reasoner_use_observation_memory
                else None
            ),
        )
        psg_auxiliary = build_psg_auxiliary_hints(
            scene_graph,
            enabled=settings.live_search_reasoner_use_psg,
            max_predicted_nodes=settings.video_psg_max_predicted_nodes,
            confidence_threshold=settings.video_psg_confidence_threshold,
        )
        context = SearchReasoningContext(
            target_profile=profile,
            goal_graph=controller.goal_graph,
            scene_graph=scene_graph,
            observation_memory=semantic_memory.retrieve_long_term(
                profile.canonical_name_zh
            ),
            negative_memory=semantic_memory,
            auxiliary_hints=list(psg_auxiliary.get("hints") or []),
            auxiliary_status={
                "psg": psg_auxiliary.get("status") or {},
                "llm_situated_prior": {
                    "enabled": settings.live_search_reasoner_use_llm_situated_prior,
                    "available": False,
                    "reason": "no_precomputed_prior_in_observe_only_snapshot",
                    "used_for_target_confirmation": False,
                },
            },
            safety_context={
                "mode": "observe_only",
                "forward_allowed": False,
                "reasoner_mode": search_reasoner_mode,
            },
        )
        directive = controller.propose(context)
        write_json(controller.goal_graph.to_dict(), output / "goal_graph.json")
        write_json(
            context.graph_match.to_dict() if context.graph_match else {},
            output / "semantic_navigation_match.json",
        )
        write_json(directive.to_dict(), output / "search_directive.json")
        write_json(
            {
                "hints": context.auxiliary_hints,
                "status": context.auxiliary_status,
                "can_confirm_target": False,
            },
            output / "semantic_navigation_auxiliary_hints.json",
        )
    topology = VideoNavigationTopologyBuilder(observed_only=True).build(scene_graph, [])
    candidates = [
        item for item in search.get("timeline", []) if item.get("type") == "direct_detection"
    ]
    timeline = [
        {
            "frame_id": item.get("frame_id"),
            "timestamp": item.get("timestamp_sec"),
            "target_status": "target_candidate",
            "best_label": item.get("label"),
            "best_score": item.get("score"),
        }
        for item in search.get("timeline", [])
    ]
    write_json(_json_safe(search), output / "target_search.json")
    write_json(timeline, output / "target_timeline.json")
    write_json(candidates, output / "target_candidates.json")
    write_json(tracks, output / "object_tracks.json")
    write_json(_track_summary(tracks), output / "track_summary.json")
    write_json(crop_report, output / "crop_verify_results.json")
    write_json(gate, output / "evidence_gating_report.json")
    write_json([item.to_dict() for item in observations], output / "frame_observations.json")
    write_scene_graph_json(scene_graph, output / "scene_graph.json")
    write_scene_graph_graphml(scene_graph, output / "scene_graph.graphml")
    write_navigation_topology_json(topology, output / "navigation_topology.json")
    write_navigation_topology_graphml(topology, output / "navigation_topology.graphml")
    write_json(machine.trace, output / "search_trace.json")
    write_json(latest_health, output / "sensor_health.json")
    (output / "safety_events.jsonl").write_text("", encoding="utf-8")
    _write_report(output, search, machine, len(frame_results))
    (output / "final_report.md").write_text(
        (output / "report.md").read_text(encoding="utf-8"), encoding="utf-8"
    )
    # Existing video-compatible aliases keep downstream readers working.
    write_json(profile.to_dict(), output / "video_target_profile.json")
    write_json(_json_safe(search), output / "video_target_search.json")
    write_json(gate, output / "video_evidence_gating_report.json")
    return {
        "status": search.get("target_status"),
        "spatial_status": machine.spatial_state,
        "state": machine.state.value,
        "output_dir": str(output),
        "target_found": bool(search.get("target_found")),
        "semantic_reasoning": bool(semantic_reasoning),
        "search_reasoner": search_reasoner,
        "search_reasoner_mode": search_reasoner_mode,
    }


def _video_frames(
    bundles: list[FrameBundle], *, snapshot_dir: Path | None = None
) -> list[VideoFrame]:
    ordered = sorted(bundles, key=lambda item: int(item.payload["image_receive_time_ns"]))
    origin = int(ordered[0].payload["image_receive_time_ns"])
    if snapshot_dir is not None:
        snapshot_dir.mkdir(parents=True, exist_ok=True)
    frames: list[VideoFrame] = []
    for item in ordered:
        image_path = item.image_path
        if snapshot_dir is not None:
            suffix = image_path.suffix or ".jpg"
            destination = snapshot_dir / f"frame_{item.frame_id:012d}{suffix}"
            temporary = destination.with_suffix(f"{destination.suffix}.tmp")
            try:
                shutil.copyfile(image_path, temporary)
                if temporary.stat().st_size == 0:
                    raise OSError(f"copied frame is empty: {image_path}")
                temporary.replace(destination)
            except OSError as exc:
                temporary.unlink(missing_ok=True)
                raise RuntimeError(
                    f"failed to snapshot live frame {item.frame_id} before inference: "
                    f"{image_path}"
                ) from exc
            image_path = destination
        frames.append(
            VideoFrame(
                frame_id=int(item.payload["frame_id"]),
                timestamp_sec=(int(item.payload["image_receive_time_ns"]) - origin) / 1e9,
                image_path=image_path,
                width=int(item.payload["camera_info"]["width"]),
                height=int(item.payload["camera_info"]["height"]),
            )
        )
    return frames


def _visual_evidence(search, gate, tracks, frame_results) -> VisualEvidence:
    best = search.get("best_evidence") or {}
    best_track = next(
        (item for item in tracks if item.get("track_id") == best.get("track_id")), {}
    )
    candidate = next(
        (
            obj
            for frame in frame_results
            for obj in frame.objects
            if obj.get("object_id") == best.get("object_id")
        ),
        {},
    )
    crop = candidate.get("crop_verify") or {}
    return VisualEvidence(
        bbox=bool(best.get("bbox")),
        mask=best.get("mask_area_ratio") is not None,
        crop_verify=bool(crop.get("is_target")),
        track_vote=best_track.get("decision") == "confirmed",
        evidence_gate=bool(gate.get("target_found")),
        frame_available=bool(best.get("frame_path")),
        source="visual_detector",
    )


def enforce_relation_evidence_gate(gate, profile, scene_graph):
    """Require observed graph support for every explicit relation target.

    Crop verification establishes the candidate's visual identity. For a
    request such as "the blue bin next to the water cooler", confirmation
    additionally needs the observed SceneGraph relation; reasoner output alone
    is never used as confirming evidence.
    """

    if not getattr(profile, "relation_constraints", None):
        return gate
    goal_graph = GoalGraphBuilder().build(profile)
    match = SemanticNavigationGraphMatcher().match(
        goal_graph,
        scene_graph,
        target_profile=profile,
    )
    relation_ok = bool(
        match.state == GraphMatchState.STRONG
        and match.matched_relations
        and not match.unmatched_relations
    )
    result = {
        **gate,
        "relation_evidence": match.to_dict(),
    }
    passed = list(result.get("passed_rules") or [])
    blocked = list(result.get("blocking_rules") or [])
    rule = "TARGET_CONFIRMATION_REQUIRE_RELATION_EVIDENCE"
    if relation_ok:
        if rule not in passed:
            passed.append(rule)
    else:
        result["target_found"] = False
        result["best_evidence"] = None
        if rule not in blocked:
            blocked.append(rule)
        result["reason_zh"] = "目标主体有视觉候选，但显式空间关系缺少强观察证据。"
    result["passed_rules"] = passed
    result["blocking_rules"] = blocked
    return result


def _track_summary(tracks: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "track_count": len(tracks),
        "confirmed_count": sum(item.get("decision") == "confirmed" for item in tracks),
        "candidate_count": sum(item.get("decision") == "candidate" for item in tracks),
        "rejected_count": sum(item.get("decision") == "rejected" for item in tracks),
    }


def _write_blocked_session(output, target, detector, health, machine, reason):
    profile = TargetProfileResolver().resolve(target, use_llm=False).to_dict()
    payload = {
        "target": target,
        "target_status": "target_not_seen",
        "spatial_status": "target_2d_only",
        "target_found": False,
        "blocked_reason": reason,
        "detector": detector,
    }
    empty_graph = {"schema_version": "1.0", "nodes": [], "edges": []}
    for name, value in {
        "target_profile.json": profile,
        "target_search.json": payload,
        "target_timeline.json": [],
        "target_candidates.json": [],
        "object_tracks.json": [],
        "track_summary.json": _track_summary([]),
        "crop_verify_results.json": {"enabled": False, "attempted": 0},
        "evidence_gating_report.json": {"target_found": False, "blocking_rules": [reason]},
        "frame_observations.json": [],
        "scene_graph.json": empty_graph,
        "navigation_topology.json": {**empty_graph, "map_type": "unavailable"},
        "search_trace.json": machine.trace,
        "sensor_health.json": health,
    }.items():
        write_json(value, output / name)
    (output / "scene_graph.graphml").write_text(
        '<?xml version="1.0" encoding="UTF-8"?><graphml xmlns="http://graphml.graphdrawing.org/xmlns"><graph edgedefault="directed"/></graphml>\n',
        encoding="utf-8",
    )
    (output / "navigation_topology.graphml").write_text(
        (output / "scene_graph.graphml").read_text(encoding="utf-8"), encoding="utf-8"
    )
    (output / "safety_events.jsonl").write_text(
        json.dumps({"event": "sensor_gate_closed", "reason": reason}) + "\n",
        encoding="utf-8",
    )
    _write_report(output, payload, machine, 0)
    (output / "final_report.md").write_text(
        (output / "report.md").read_text(encoding="utf-8"), encoding="utf-8"
    )
    return {
        "status": "blocked_wait_for_sensors",
        "state": machine.state.value,
        "output_dir": str(output),
        "target_found": False,
    }


def _write_report(output: Path, search: dict, machine, frames: int) -> None:
    lines = [
        "# Live Robot Search Report",
        "",
        f"- target: {search.get('task', {}).get('target', search.get('target'))}",
        f"- target_status: {search.get('target_status')}",
        f"- spatial_status: {search.get('spatial_status', machine.spatial_state)}",
        f"- final_state: {machine.state.value}",
        f"- analyzed_frames: {frames}",
        "- motion_commands_sent: false",
        "- target_center_used_as_navigation_goal: false",
    ]
    (output / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_session_contract(output, *, target, detector, search_mode, health):
    blockers = []
    if not health.get("camera"):
        blockers.append("camera_not_fresh")
    if not health.get("lidar"):
        blockers.append("lidar_not_fresh")
    if search_mode == "step_search":
        blockers.append("motion_execution_disabled")
    if search_mode in {"nav2_plan_only", "nav2_execute"}:
        blockers.append("navigation_capability_gate_not_attached")
    task = {
        "schema_version": "1.0",
        "task": target,
        "requested_mode": search_mode,
        "runtime_source": "live_frame_bundle",
    }
    write_json(task, output / "task.json")
    write_json(
        {
            "schema_version": "1.0",
            "status": "not_run",
            "reason": "live_sensor_gate_precedes_optional_llm_task_parsing",
            "raw_task": target,
        },
        output / "parsed_task.json",
    )
    write_json(
        {
            "schema_version": "1.0",
            "mode": search_mode,
            "allowed": not blockers,
            "blocking_conditions": blockers,
            "sensor_health": health,
            "motion_authorized": False,
        },
        output / "capability_gate_result.json",
    )
    write_json(
        {
            "schema_version": "1.0",
            "status": "not_generated",
            "reason": "generated only after the live sensor gate passes",
            "detector": detector,
        },
        output / "grounding_prompt_plan.json",
    )
    write_json(
        {
            "schema_version": "1.0",
            "visual_only": True,
            "requires_provenance": True,
            "write_attempted": False,
        },
        output / "memory_provenance.json",
    )
    (output / "motion_commands.jsonl").write_text("", encoding="utf-8")
    (output / "nav2_requests.jsonl").write_text("", encoding="utf-8")
    (output / "sensor_health.jsonl").write_text(
        json.dumps(health, ensure_ascii=False) + "\n", encoding="utf-8"
    )


def _json_safe(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if hasattr(value, "to_dict"):
        return _json_safe(value.to_dict())
    return value
