"""End-to-end Video-to-Navigation Planning pipeline."""

from __future__ import annotations

from datetime import UTC, datetime
from pathlib import Path
from typing import Any
from uuid import uuid4

from .candidate_goal_generator import generate_candidate_goals
from .exploration_planner import generate_exploration_candidates
from .models import NavigationWaypoint, Pose2D
from .nav2_adapter import adapt_visual_plan_to_nav2_goal
from .navigation_instruction_generator import generate_navigation_instructions
from .navigation_mode import normalize_video_navigation_mode
from .navigation_result_store import write_json, write_webui_manifest
from .semantic_goal_localizer import (
    TARGET_CANDIDATE,
    TARGET_LOST_AFTER_SEEN,
    TARGET_NOT_SEEN,
    TARGET_UNCONFIRMED_BUT_LIKELY_AREA_FOUND,
    TARGET_VISUAL_CONFIRMED,
    localize_semantic_goal,
)
from .target_pose_generator import generate_observation_goal
from .video_navigation_map import build_video_navigation_map, write_navigation_map_outputs
from .video_pose_estimator import (
    estimate_video_trajectory,
    render_trajectory_plot,
    write_trajectory_csv,
)
from .visual_path_planner import plan_visual_path


def run_video_navigation_planning(
    *,
    video_path: str | Path,
    target_search_result: dict[str, Any],
    output_root: str | Path = "outputs",
    mode: str = "visual_preview",
    pose_backend: str = "auto",
    rgbd_path: str | Path | None = None,
    calibration: dict[str, Any] | None = None,
    force_exploration: bool = False,
    max_frames: int = 300,
    frame_sample_interval: int = 5,
    observation_distance: float = 1.5,
    exploration_max_candidates: int = 8,
) -> dict[str, Any]:
    selected_mode = normalize_video_navigation_mode(mode)
    request_id = f"video_nav_{datetime.now(UTC):%Y%m%d_%H%M%S}_{uuid4().hex[:6]}"
    output_dir = Path(output_root) / "video_navigation" / request_id
    output_dir.mkdir(parents=True, exist_ok=True)

    trajectory = estimate_video_trajectory(
        video_path,
        backend=pose_backend,
        rgbd_path=rgbd_path,
        calibration=calibration,
        max_frames=max_frames,
        frame_sample_interval=frame_sample_interval,
    )
    navigation_map = build_video_navigation_map(trajectory, _observations_from_result(target_search_result))
    localization = localize_semantic_goal(target_search_result, trajectory, navigation_map)
    target_status = "target_not_seen" if force_exploration else localization["target_status"]
    goal = _select_goal(
        target_search_result,
        localization,
        trajectory,
        navigation_map,
        target_status,
        observation_distance,
        exploration_max_candidates,
    )
    strategy = _strategy_for_status(target_status, goal)
    plan = plan_visual_path(
        navigation_map,
        goal,
        mode=selected_mode.value,
        target_status=target_status,
        navigation_strategy=strategy,
    )
    instructions = generate_navigation_instructions(plan)
    nav2_result = adapt_visual_plan_to_nav2_goal(
        plan,
        transform=_map_transform_from_calibration(calibration),
    )
    paths = _write_outputs(
        output_dir=output_dir,
        request_id=request_id,
        video_path=video_path,
        target_search_result=target_search_result,
        trajectory=trajectory,
        navigation_map=navigation_map,
        localization=localization,
        plan=plan,
        instructions=instructions,
        nav2_result=nav2_result.to_dict(),
    )
    summary = {
        "request_id": request_id,
        "mode": selected_mode.value,
        "target_status": target_status,
        "navigation_strategy": strategy,
        "scale_status": plan.scale_status,
        "executable": plan.executable,
        "executable_reason": plan.executable_reason,
        "nav2_allowed": nav2_result.allowed,
        "nav2_reason": nav2_result.reason,
    }
    paths["webui_manifest"] = write_webui_manifest(output_dir, paths, summary)
    return {
        "request_id": request_id,
        "output_dir": str(output_dir),
        "summary": summary,
        "trajectory": [item.to_dict() for item in trajectory],
        "navigation_map": navigation_map,
        "target_localization": localization,
        "visual_navigation_plan": plan.to_dict(),
        "navigation_instructions": instructions,
        "nav2_adapter": nav2_result.to_dict(),
        "output_files": {key: str(path) for key, path in paths.items()},
    }


def _select_goal(
    target_search_result: dict[str, Any],
    localization: dict[str, Any],
    trajectory,
    navigation_map: dict[str, Any],
    target_status: str,
    observation_distance: float,
    exploration_max_candidates: int,
) -> NavigationWaypoint:
    if target_status == TARGET_VISUAL_CONFIRMED:
        goal = generate_observation_goal(localization, observation_distance)
        if goal is not None:
            return goal
    if target_status in {TARGET_CANDIDATE, TARGET_UNCONFIRMED_BUT_LIKELY_AREA_FOUND, TARGET_LOST_AFTER_SEEN}:
        goal = generate_observation_goal(localization, observation_distance)
        if goal is not None:
            return goal
        candidates = generate_candidate_goals(target_search_result, trajectory)
        if candidates:
            return candidates[0]
    candidates = generate_exploration_candidates(
        navigation_map,
        target_search_result,
        max_candidates=exploration_max_candidates,
    )
    if candidates:
        return candidates[0]
    start = Pose2D.from_dict(navigation_map["nodes"][0]["pose"])
    return NavigationWaypoint(
        waypoint_id="frontier_00",
        pose=start,
        source_frame_id=navigation_map["nodes"][0].get("frame_id"),
        semantic_label="当前视频起点重新观察",
        waypoint_type="frontier",
        confidence=0.2,
    )


def _strategy_for_status(target_status: str, goal: NavigationWaypoint) -> str:
    if goal.waypoint_type in {"frontier", "exploration"} or target_status == TARGET_NOT_SEEN:
        return "exploration"
    if target_status == TARGET_LOST_AFTER_SEEN:
        return "last_known_reobserve"
    if target_status in {TARGET_CANDIDATE, TARGET_UNCONFIRMED_BUT_LIKELY_AREA_FOUND}:
        return "candidate_navigation"
    return "target_navigation"


def _write_outputs(
    *,
    output_dir: Path,
    request_id: str,
    video_path: str | Path,
    target_search_result: dict[str, Any],
    trajectory,
    navigation_map: dict[str, Any],
    localization: dict[str, Any],
    plan,
    instructions: list[dict[str, Any]],
    nav2_result: dict[str, Any],
) -> dict[str, Path]:
    paths = {
        "request": write_json(
            {
                "request_id": request_id,
                "video_path": str(video_path),
                "target": target_search_result.get("target") or target_search_result.get("task", {}).get("target"),
            },
            output_dir / "request.json",
        ),
        "video_metadata": write_json(target_search_result.get("video_meta", {}), output_dir / "video_metadata.json"),
        "trajectory": write_json([item.to_dict() for item in trajectory], output_dir / "trajectory.json"),
        "trajectory_csv": write_trajectory_csv(trajectory, output_dir / "trajectory.csv"),
        "target_localization": write_json(localization, output_dir / "target_localization.json"),
        "visual_navigation_plan": write_json(plan.to_dict(), output_dir / "visual_navigation_plan.json"),
        "visual_navigation_path": write_json([pose.to_dict() for pose in plan.path], output_dir / "visual_navigation_path.json"),
        "navigation_instructions": write_json(instructions, output_dir / "navigation_instructions.json"),
        "navigation_report": write_json(
            {
                "summary": plan.to_dict(),
                "instructions": instructions,
                "nav2_adapter": nav2_result,
            },
            output_dir / "navigation_report.json",
        ),
        "nav2_plan": write_json(
            {
                "status": "ready" if nav2_result.get("allowed") else "not_requested",
                "reason": nav2_result.get("reason") or "visual_preview_only",
            },
            output_dir / "nav2_plan.json",
        ),
        "nav2_request": write_json(nav2_result, output_dir / "nav2_request.json"),
        "nav2_global_path": write_json(
            {
                "status": "not_requested",
                "reason": "Nav2 ComputePathToPose is only requested through explicit plan_only/execute mode",
            },
            output_dir / "nav2_global_path.json",
        ),
    }
    map_paths = write_navigation_map_outputs(navigation_map, output_dir)
    paths.update(map_paths)
    plot = render_trajectory_plot(trajectory, output_dir / "trajectory_plot.png")
    if plot:
        paths["trajectory_plot"] = plot
    exploration = [
        waypoint.to_dict()
        for waypoint in generate_exploration_candidates(navigation_map, target_search_result)
    ]
    paths["exploration_candidates"] = write_json(exploration, output_dir / "exploration_candidates.json")
    candidates = [
        waypoint.to_dict()
        for waypoint in generate_candidate_goals(target_search_result, trajectory)
    ]
    paths["candidate_goals"] = write_json(candidates, output_dir / "candidate_goals.json")
    return paths


def _observations_from_result(result: dict[str, Any]) -> list[dict[str, Any]]:
    timeline = result.get("timeline") or []
    observations: list[dict[str, Any]] = []
    for item in timeline:
        frame_id = item.get("frame_id")
        label = item.get("label") or item.get("description")
        if frame_id is not None and label:
            observations.append({"frame_id": frame_id, "objects": [label]})
    return observations


def _map_transform_from_calibration(calibration: dict[str, Any] | None):
    if not calibration:
        return None
    return calibration.get("T_map_video_map") or calibration.get("map_transform")
