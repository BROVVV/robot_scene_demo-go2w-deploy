"""Primary video target-search pipeline with optional scene-map assistance."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from app.config import get_settings
from app.video.navigation_graph_exporter import (
    write_navigation_topology_debug,
    write_navigation_topology_graphml,
    write_navigation_topology_json,
)
from app.video.pipeline import run_video_search
from app.video.scene_map_search_ranker import (
    annotate_topology_for_target_search,
    rank_places_for_target_search,
)
from app.video.scene_mapping_auxiliary import run_scene_mapping_auxiliary
from app.video.target_navigation_decision_builder import build_target_navigation_decision
from app.video.video_graph_io import write_json
from app.video.video_graph_visualizer import render_topology_png
from app.video.video_target_report_builder import write_video_target_search_report
from app.video.video_target_state import apply_target_state
from app.navigation.navigation_planning_pipeline import run_video_navigation_planning


def run_video_target_search_pipeline(
    video_path: str,
    target: str,
    detector: str,
    config: Any,
    enable_tracking: bool = True,
    enable_crop_verify: bool = True,
    enable_evidence_gating: bool = True,
    enable_scene_mapping: bool = False,
    enable_navigation_topology: bool = False,
    use_scene_map_for_search: bool = False,
) -> dict[str, Any]:
    """Run target search first, then optional scene-map/topology assistance."""

    if not target.strip():
        raise ValueError("target_search mode requires --target")

    settings = get_settings()
    output_dir = Path(getattr(config, "output_dir", None) or settings.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    search_result, paths = run_video_search(
        video_path=video_path,
        target=target,
        detector=detector,
        sample_fps=getattr(config, "sample_fps", None),
        max_frames=getattr(config, "max_frames", None),
        enable_knowledge=getattr(config, "enable_knowledge", False),
        enable_video_memory=(
            True if getattr(config, "enable_video_memory", False) else None
        ),
        output_dir=output_dir,
        annotate=not bool(getattr(config, "no_annotate", False)),
        enable_tracking=enable_tracking,
        enable_crop_verify=enable_crop_verify,
        verify_every_n_frames=getattr(config, "verify_every_n_frames", None),
        track_iou_threshold=getattr(config, "track_iou_threshold", None),
        target_confirm_min_frames=getattr(config, "target_confirm_min_frames", None),
        target_confirm_score=getattr(config, "target_confirm_score", None),
        enable_llm_prior=getattr(config, "enable_llm_prior", None),
        enable_observation_memory=getattr(config, "enable_observation_memory", None),
        enable_evidence_gating=enable_evidence_gating,
        disable_handwritten_priors=getattr(config, "disable_handwritten_priors", False),
        disable_static_kb=getattr(config, "disable_static_kb", False),
        prior_audit=getattr(config, "prior_audit", False),
        include_runtime_artifacts=True,
    )
    runtime = search_result.pop("_runtime_artifacts", {})
    target_profile = runtime.get("target_profile") or search_result.get("target_profile", {})
    target_profile_dict = (
        target_profile.to_dict() if hasattr(target_profile, "to_dict") else dict(target_profile)
    )
    ranked_places: list[dict[str, Any]] | None = None
    scene_map_result: dict[str, Any] | None = None
    navigation_topology: dict[str, Any] | None = None

    scene_mapping_enabled = bool(enable_scene_mapping or enable_navigation_topology)
    if scene_mapping_enabled:
        aux_config = _aux_config(config, output_dir, enable_navigation_topology)
        scene_map_result = run_scene_mapping_auxiliary(
            video_path=str(video_path),
            frame_results=runtime.get("frame_results", []),
            object_tracks=runtime.get("object_tracks", []),
            target_profile=target_profile,
            config=aux_config,
        )
        paths.update(
            {
                key: Path(value)
                for key, value in scene_map_result.get("output_files", {}).items()
            }
        )
        navigation_topology = scene_map_result.get("navigation_topology")

    apply_target_state(search_result)
    if use_scene_map_for_search and navigation_topology:
        ranked_places = rank_places_for_target_search(
            navigation_topology=navigation_topology,
            target_profile=target_profile_dict,
            target_search_result=search_result,
            config=config,
        )
        search_result["topology_assisted_search"] = {
            "enabled": True,
            "ranked_places": ranked_places,
        }
        search_result["target_status_before_topology_assist"] = search_result.get(
            "target_status"
        )
        apply_target_state(
            search_result,
            ranked_places=ranked_places,
            high_score_threshold=float(
                getattr(
                    config,
                    "search_ranker_high_score_threshold",
                    getattr(settings, "video_search_ranker_high_score_threshold", 0.70),
                )
            ),
        )
        navigation_topology = annotate_topology_for_target_search(
            navigation_topology,
            ranked_places,
            search_result,
        )
        paths.update(_rewrite_topology_outputs(navigation_topology, output_dir))
        paths["video_topology_search_ranking"] = write_json(
            {
                "target": target,
                "ranked_places": ranked_places,
                "context_only_can_confirm": False,
            },
            output_dir / "video_topology_search_ranking.json",
        )
    else:
        search_result["topology_assisted_search"] = {
            "enabled": False,
            "ranked_places": [],
        }

    navigation_decision = build_target_navigation_decision(
        target_search_result=search_result,
        navigation_topology=navigation_topology,
        ranked_places=ranked_places,
        target_profile=target_profile_dict,
        config=config,
    )
    search_result.update(
        {
            "target": target,
            "target_profile": target_profile_dict,
            "scene_mapping_enabled": scene_mapping_enabled,
            "navigation_topology_enabled": bool(enable_navigation_topology),
            "navigation_decision": navigation_decision,
        }
    )
    if scene_map_result is not None:
        search_result["scene_map_result"] = _scene_map_summary(scene_map_result)

    candidates = _target_candidates(search_result)
    paths["video_target_candidates"] = write_json(
        candidates,
        output_dir / "video_target_candidates.json",
    )
    paths["video_navigation_trace"] = write_json(
        _target_navigation_trace(target, search_result, navigation_decision),
        output_dir / "video_navigation_trace.json",
    )
    paths["video_target_timeline"] = write_json(
        _target_timeline(search_result),
        output_dir / "video_target_timeline.json",
    )
    paths["video_target_search"] = write_json(
        _json_safe_result(search_result),
        output_dir / "video_target_search.json",
    )
    if _video_navigation_enabled(config, settings):
        video_navigation = run_video_navigation_planning(
            video_path=video_path,
            target_search_result=_json_safe_result(search_result),
            output_root=output_dir,
            mode=str(
                getattr(config, "video_navigation_mode", None)
                or settings.video_navigation_mode
            ),
            pose_backend=str(
                getattr(config, "video_pose_backend", None)
                or settings.video_pose_backend
            ),
            rgbd_path=getattr(config, "depth_dir", None),
            calibration=_video_navigation_calibration(config, settings),
            force_exploration=bool(getattr(config, "force_exploration", False)),
            max_frames=int(
                getattr(
                    config,
                    "video_navigation_max_frames",
                    settings.video_navigation_max_frames,
                )
            ),
            frame_sample_interval=int(
                getattr(
                    config,
                    "video_navigation_frame_sample_interval",
                    settings.video_navigation_frame_sample_interval,
                )
            ),
            observation_distance=float(
                getattr(
                    config,
                    "video_navigation_target_observation_distance",
                    settings.video_navigation_target_observation_distance,
                )
            ),
            exploration_max_candidates=int(
                getattr(
                    config,
                    "video_navigation_exploration_max_candidates",
                    settings.video_navigation_exploration_max_candidates,
                )
            ),
        )
        search_result["video_navigation"] = video_navigation
        paths.update(
            {
                f"video_navigation_{key}": Path(value)
                for key, value in video_navigation.get("output_files", {}).items()
            }
        )
        paths["video_target_search"] = write_json(
            _json_safe_result(search_result),
            output_dir / "video_target_search.json",
        )
    paths["video_reasoning_report"] = write_video_target_search_report(
        _json_safe_result(search_result),
        output_dir / "video_reasoning_report.md",
        output_files=paths,
    )
    search_result["output_files"] = {key: str(path) for key, path in paths.items()}
    return _json_safe_result(search_result)


def _video_navigation_enabled(config: Any, settings: Any) -> bool:
    value = getattr(config, "enable_video_navigation", None)
    if value is None:
        value = getattr(settings, "video_navigation_enabled", True)
    auto_plan = getattr(config, "video_navigation_auto_plan", None)
    if auto_plan is None:
        auto_plan = getattr(settings, "video_navigation_auto_plan", True)
    return bool(value and auto_plan)


def _video_navigation_calibration(config: Any, settings: Any) -> dict[str, Any] | None:
    selected_mode = str(
        getattr(config, "video_navigation_mode", None) or settings.video_navigation_mode
    )
    calibration: dict[str, Any] = {
        "scale_verified": selected_mode in {"metric_preview", "plan_only", "execute"}
    }
    transform_path = getattr(config, "video_map_transform_json", None)
    if transform_path:
        transform_payload = json.loads(Path(transform_path).read_text(encoding="utf-8"))
        calibration["T_map_video_map"] = (
            transform_payload.get("T_map_video_map")
            if isinstance(transform_payload, dict)
            else transform_payload
        )
        calibration["scale_verified"] = True
    return calibration if calibration["scale_verified"] or getattr(config, "depth_dir", None) else None


def _aux_config(config: Any, output_dir: Path, enable_navigation_topology: bool) -> Any:
    return SimpleNamespace(
        output_dir=str(output_dir),
        enable_video_psg=getattr(config, "enable_video_psg", None),
        enable_navigation_topology=enable_navigation_topology,
        psg_max_predicted_nodes=getattr(config, "psg_max_predicted_nodes", None),
        psg_confidence_threshold=getattr(config, "psg_confidence_threshold", None),
        topology_observed_only=getattr(config, "topology_observed_only", None),
    )


def _rewrite_topology_outputs(
    navigation_topology: dict[str, Any],
    output_dir: Path,
) -> dict[str, Path]:
    paths = {
        "video_navigation_topology": write_navigation_topology_json(
            navigation_topology,
            output_dir / "video_navigation_topology.json",
        ),
        "video_navigation_topology_graphml": write_navigation_topology_graphml(
            navigation_topology,
            output_dir / "video_navigation_topology.graphml",
        ),
        "video_navigation_topology_debug": write_navigation_topology_debug(
            navigation_topology,
            output_dir / "video_navigation_topology_debug.md",
        ),
    }
    rendered = render_topology_png(
        navigation_topology,
        output_dir / "video_navigation_topology.png",
    )
    if rendered:
        paths["video_navigation_topology_png"] = rendered
    return paths


def _target_candidates(search_result: dict[str, Any]) -> list[dict[str, Any]]:
    candidates = [
        item
        for item in search_result.get("timeline", [])
        if item.get("type") == "direct_detection"
    ]
    best = search_result.get("best_evidence")
    if best and not candidates:
        candidates.append(
            {
                "frame_id": best.get("frame_id"),
                "timestamp_sec": best.get("timestamp_sec"),
                "label": best.get("label"),
                "score": best.get("evidence_score") or best.get("confidence"),
                "bbox": best.get("bbox"),
                "evidence_status": "visual_candidate",
            }
        )
    return candidates


def _target_timeline(search_result: dict[str, Any]) -> list[dict[str, Any]]:
    status = search_result.get("target_status")
    timeline = []
    for item in search_result.get("timeline", []):
        timeline.append(
            {
                "frame_id": item.get("frame_id"),
                "timestamp": item.get("timestamp_sec"),
                "target_status": (
                    "target_candidate"
                    if item.get("type") == "direct_detection"
                    else status
                ),
                "num_candidates": 1 if item.get("type") == "direct_detection" else 0,
                "best_label": item.get("label"),
                "best_score": item.get("score"),
                "scene_place_id": item.get("scene_place_id"),
            }
        )
    if not timeline:
        timeline.append(
            {
                "frame_id": None,
                "timestamp": None,
                "target_status": status,
                "num_candidates": 0,
                "scene_place_id": None,
            }
        )
    return timeline


def _target_navigation_trace(
    target: str,
    search_result: dict[str, Any],
    decision: dict[str, Any],
) -> dict[str, Any]:
    step = {
        "step": 1,
        "place_id": decision.get("next_place_id"),
        "action": decision.get("next_action"),
        "reason": decision.get("reason"),
        "requires_visual_confirmation": decision.get("requires_visual_confirmation", True),
    }
    return {
        "target": target,
        "target_status": search_result.get("target_status"),
        "trace": [step],
    }


def _scene_map_summary(scene_map_result: dict[str, Any]) -> dict[str, Any]:
    return {
        "place_segments": scene_map_result.get("place_segments", []),
        "object_tracks": scene_map_result.get("object_tracks", []),
        "psg_layer": scene_map_result.get("psg_layer", {}),
        "navigation_topology": scene_map_result.get("navigation_topology"),
        "output_files": scene_map_result.get("output_files", {}),
    }


def _json_safe_result(search_result: dict[str, Any]) -> dict[str, Any]:
    return {
        key: value
        for key, value in search_result.items()
        if not key.startswith("_")
    }
