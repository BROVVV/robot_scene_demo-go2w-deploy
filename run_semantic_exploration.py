#!/usr/bin/env python3
"""High-level autonomous semantic exploration entry point (plan section 23).

Runs the platform-independent AutonomousExplorer against a chosen backend:

* ``--backend go2w_experimental`` (default): real Go2-W via the audited
  ``/go2w/motion`` executor from ``run_autonomous_loop.py``; requires
  ``--operator-supervised-experiment`` (operator with remote present) and the
  perception / odometry / motion stacks running.
* ``--backend mock`` / ``--backend mock_metric``: offline E2E with scripted
  vision and simulated motion (no robot needed).
* ``--replay <session.jsonl>``: deterministic replay of a previous session's
  observations with the mock backend.

Outputs: session JSONL events, ``outputs/live_runs/<session_id>/``
(exploration_graph.json + summary.json).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Callable

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from app.config import get_settings
from app.detectors.siliconflow_vision_worker import quick_target_present, quick_target_state
from app.live_robot.autonomous_explorer import (
    AutonomousExplorer,
    PerceptionFailure,
    SemanticMatch,
    VerificationOutcome,
)
from app.live_robot.experiment_readiness import compute_experiment_readiness
from app.live_robot.mock_observation_scene import (
    MockObservationScene,
    scenario_anchor_then_target,
    scenario_no_target,
    scenario_target_appears_after,
)
from app.live_robot.async_semantic_observer import AsyncSemanticObservationManager, compute_scene_signature
from app.live_robot.latency_profiler import LatencyProfiler
from app.live_robot.verify_cache import VerificationCache, VerificationCacheEntry
from app.live_robot.semantic_observer import (
    SEMANTIC_STATUS_FRESH_FULL,
    SEMANTIC_STATUS_FRESH_QUICK,
    SEMANTIC_STATUS_PENDING,
    SEMANTIC_STATUS_UNAVAILABLE,
    LiveSemanticObserver,
    SemanticObservation,
    _from_payload,
    semantic_observation_to_live,
    semantic_payload_from_quick_target_absence,
)
from app.memory.observation_memory_store import ObservationMemoryStore
from app.navigation.backend_factory import create_backend
from app.navigation.topology_route_executor import (
    TopologyRouteExecutor,
    TopologyRoutePlanner,
)
from app.reasoning.llm_prior_generator import LLMPriorGenerator, LLMPriorInput
from app.navigation.exploration_config import (
    load_exploration_policy,
    load_go2w_experiment_profile,
)
from app.navigation.exploration_graph import ExplorationGraph
from app.navigation.local_scan import LocalScanState, select_local_scan_goal
from app.navigation.models import (
    GOAL_ROTATE_VIEW,
    ExplorationGoal,
    LiveObservation,
)
from app.reasoning.semantic_navigation.models import SearchReasoningContext
from app.reasoning.semantic_navigation.router import SemanticSearchController
from app.reasoning.semantic_navigation.semantic_memory import SemanticSearchMemory
from app.reasoning.target_profile import TargetProfileResolver
from app.spatial.models import SpatialFrameMismatch
from app.perception.target_state import TARGET_PRESENT
from app.spatial.geometric_relation_extractor import extract_geometric_relations
from app.spatial.spatial_pose_validator import MotionEvidence, SpatialPoseValidator


def _navigation_sector(*, yaw_rad: float, sectors: int) -> int:
    """把当前真实 yaw 映射到 0..sectors-1 的导航 heading sector。

    计划书 §6.2 / 不变量 2：导航朝向来自运动位姿事实，与语义状态无关。
    """
    sectors = max(1, int(sectors))
    sector_deg = 360.0 / sectors
    return int(round(math.degrees(float(yaw_rad)) / sector_deg)) % sectors


def _requested_dyaw_deg(step: str) -> float:
    """l30 -> +30, r30 -> -30, f/b -> 0（运动请求的标称转角）。"""
    if not step:
        return 0.0
    if step.startswith("l"):
        try:
            return float(step[1:])
        except ValueError:
            return 0.0
    if step.startswith("r"):
        try:
            return -float(step[1:])
        except ValueError:
            return 0.0
    return 0.0


def _requested_forward_m(step: str) -> float:
    """Parse the local primitive's signed forward/backward request."""
    if not step or step[0] not in {"f", "b"}:
        return 0.0
    try:
        distance = float(step[1:])
    except (TypeError, ValueError):
        return 0.0
    return distance if step[0] == "f" else -distance


def _wrap_pi(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def _classify_perception_error(exc: Exception) -> str:
    """计划书 §8.2：把底层异常映射成结构化错误码（Quick/RGB-D 感知链）。"""
    code = str(getattr(exc, "code", "") or "").strip()
    if code:
        return code
    text = f"{type(exc).__name__}: {exc}".lower()
    if "timed out" in text or "timeout" in text:
        return "QUICK_VLM_TIMEOUT"
    if "rgbd" in text or "depth" in text:
        if "stale" in text or "unavailable" in text:
            return "RGBD_TIMEOUT"
        return "DEPTH_ERROR"
    if "stale" in text:
        return "FRAME_STALE"
    if "tf" in text or "transform" in text:
        return "TF_UNAVAILABLE"
    return "UNKNOWN_PERCEPTION_ERROR"


def _motion_action_server_count() -> tuple[int | None, str]:
    """Count /go2w/motion servers so duplicate action owners fail closed."""
    try:
        completed = subprocess.run(
            ["ros2", "action", "info", "/go2w/motion"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=5.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return None, f"action graph probe failed: {type(exc).__name__}: {exc}"
    output = (completed.stdout or "") + (completed.stderr or "")
    match = re.search(r"Action servers:\s*(\d+)", output)
    if completed.returncode != 0 or match is None:
        return None, output.strip()[-500:] or "action graph count unavailable"
    return int(match.group(1)), output.strip()[-500:]


def _motion_action_server_process_count() -> tuple[int | None, list[int]]:
    """Count real server executables; duplicate ROS node names can collapse."""
    pids: list[int] = []
    try:
        proc_root = Path("/proc")
        for entry in proc_root.iterdir():
            if not entry.name.isdigit():
                continue
            try:
                argv0 = (entry / "cmdline").read_bytes().split(b"\0", 1)[0]
            except (OSError, PermissionError):
                continue
            if Path(os.fsdecode(argv0)).name == "go2w_motion_action_server":
                pids.append(int(entry.name))
    except OSError:
        return None, []
    return len(pids), sorted(pids)


def _topic_publisher_count(topic: str) -> tuple[int | None, str]:
    """Return the ROS publisher count for a safety-critical topic."""

    try:
        completed = subprocess.run(
            ["ros2", "topic", "info", topic, "-v"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=5.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return None, f"topic graph probe failed: {type(exc).__name__}: {exc}"
    output = (completed.stdout or "") + (completed.stderr or "")
    match = re.search(r"Publisher count:\s*(\d+)", output)
    if completed.returncode != 0 or match is None:
        return None, output.strip()[-500:] or "publisher count unavailable"
    return int(match.group(1)), output.strip()[-500:]


def _wheel_odom_process_count() -> tuple[int | None, list[int]]:
    """Count the single project-authoritative odom process.

    The current Go2-W rig has no usable low-state wheel-odom executable; its
    fused motion odom is published by ``sport_odom_bridge.py`` from
    ``/lf/sportmodestate``.  Treat that bridge and the legacy executable as
    equivalent authorities, while still rejecting duplicates.
    """

    pids: list[int] = []
    try:
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            try:
                argv = [
                    os.fsdecode(item)
                    for item in (entry / "cmdline").read_bytes().split(b"\0")
                    if item
                ]
            except (OSError, PermissionError):
                continue
            if (
                any(Path(item).name == "go2w_wheel_odom" for item in argv)
                or any("sport_odom_bridge.py" in item for item in argv)
            ):
                pids.append(int(entry.name))
    except OSError:
        return None, []
    return len(pids), sorted(pids)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Operator-supervised high-level autonomous semantic exploration"
    )
    parser.add_argument("--target", required=True, help="自然语言搜索目标，如 饮水机旁边的蓝色垃圾桶")
    parser.add_argument("--task-context-json", default="")
    parser.add_argument("--session-id", default="")
    parser.add_argument("--executor-id", default="")
    parser.add_argument("--worker-generation", type=int, default=None)
    parser.add_argument(
        "--backend", choices=("go2w_experimental", "mock", "mock_metric"),
        default="go2w_experimental",
        help="robot backend: real Go2-W / offline mocks",
    )
    parser.add_argument(
        "--reasoner", choices=("legacy", "semantic", "semantic_navigation", "hybrid"),
        default="semantic_navigation",
    )
    parser.add_argument(
        "--operator-supervised-experiment", action="store_true",
        help="operator present with remote; enables the experiment profile "
             "(authorizes <=30deg turns without a rotation lease; all other "
             "safety gates stay active)",
    )
    parser.add_argument(
        "--operator-authorized-rotation", action="store_true",
        help="legacy alias of --operator-supervised-experiment for the motion gate",
    )
    parser.add_argument("--max-seconds", type=float, default=600.0)
    parser.add_argument("--max-planning-cycles", type=int, default=100)
    parser.add_argument("--max-motion-steps", type=int, default=50)
    parser.add_argument("--finish-on-visual-confirmation", action="store_true",
                        default=True,
                        help="2D visual confirmation + relation evidence + verify PASS "
                             "ends the task (no metric 3D required)")
    parser.add_argument("--no-finish-on-visual-confirmation", action="store_true")
    parser.add_argument("--turn-only", action="store_true",
                        help="reject forward steps at the motion executor")
    parser.add_argument("--dry-run-motion", action="store_true",
                        help="run the full real pipeline (camera/LLM/SemanticNavigation/"
                             "planner) but never send motion commands "
                             "(REOBSERVE-only backend)")
    parser.add_argument("--output", default="outputs/live_sessions/semantic_exploration.jsonl")
    parser.add_argument("--session-dir", default="outputs/live_runs")
    parser.add_argument("--record-video", default="",
                        help="record camera stream to MP4 (go2w backend only)")
    parser.add_argument("--dry-run", action="store_true",
                        help="offline run with scripted vision (alias of --backend mock)")
    parser.add_argument("--replay", default="",
                        help="replay observations from a previous session JSONL")
    parser.add_argument("--mock-scenario",
                        choices=("target_appears_after_n", "no_target", "anchor_then_target",
                                 "semantic_topology", "green_bin"),
                        default="anchor_then_target")
    parser.add_argument("--mock-target-after", type=int, default=3,
                        help="observations before the target appears (mock scenario)")
    parser.add_argument("--mock-confirm-after-seen", type=int, default=1)
    parser.add_argument("--verify-min-confidence", type=float, default=0.6)
    parser.add_argument("--detector", choices=("llm", "grounded_sam"), default="llm")
    parser.add_argument("--llm-model", default="")
    parser.add_argument("--spool-root", default="runtime/go2w/spool")
    parser.add_argument("--rgbd-source", action="store_true",
                        help="use the D435 atomic RGB-D HTTP source as the primary "
                             "camera for observation (color+depth attached to LiveObservation)")
    parser.add_argument("--rgbd-base-url", default="http://192.168.123.18:8080")
    parser.add_argument("--spatial-v2", action="store_true",
                        help="enable SemanticNavigation V2 spatial exploration loop: PlaceGraph, "
                             "frontier selection, LongTermGoalSelector and LocalGoalExecutor")
    parser.add_argument("--spatial-provider",
                        choices=("camera", "rtabmap", "plain_slam"),
                        default="camera",
                        help="metric spatial provider for the V2 loop: "
                             "camera (default, D435 BEV/camera-local), "
                             "rtabmap (/rtabmap/map + /rtabmap/odom), "
                             "plain_slam (/go2w/slam/map_2d + /go2w/slam/odom_base)")
    parser.add_argument("--rtabmap", action="store_true",
                        help="legacy alias of --spatial-provider rtabmap: use RTAB-Map "
                             "ROS2 topics (/rtabmap/map, /rtabmap/odom) as the "
                             "SpatialProvider; requires the D435 RGB-D bridge "
                             "and rtabmap_slam to be running")
    parser.add_argument("--plain-slam", action="store_true",
                        help="legacy alias of --spatial-provider plain_slam: use the "
                             "plain_slam PandarXT-16 mapping pipeline "
                             "(/go2w/slam/map_2d, /go2w/slam/odom_base)")
    parser.add_argument("--max-local-rotations", type=int, default=3,
                        help="bounded LOCAL_SCAN rotation quota per Place (plan §57)")
    parser.add_argument("--odom-topic", default="/go2w/odom/fused")
    parser.add_argument("--max-radius", type=float, default=0.0,
                        help="search radius limit in metres; 0 = unlimited")
    parser.add_argument("--target-score-min", type=float, default=0.15)
    parser.add_argument("--reach-area-ratio", type=float, default=0.15)
    parser.add_argument("--max-turn-deg", type=float, default=30.0)
    parser.add_argument("--forward-step-m", type=float, default=1.5)
    parser.add_argument("--semantic-allow-forward", action="store_true")
    parser.add_argument("--allow-degraded", action="store_true",
                        help="continue when non-critical readiness checks degrade")
    parser.add_argument("--exploration-config", default="configs/exploration/default.yaml")
    parser.add_argument("--profile-config", default="configs/go2w/high_level_experiment.yaml")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.reasoner == "semantic":
        args.reasoner = "semantic_navigation"
    if args.no_finish_on_visual_confirmation:
        args.finish_on_visual_confirmation = False
    if args.dry_run and args.backend == "go2w_experimental":
        args.backend = "mock"
    # Compatible aliases: --rtabmap / --plain-slam map onto --spatial-provider.
    if args.rtabmap:
        args.spatial_provider = "rtabmap"
    if args.plain_slam:
        args.spatial_provider = "plain_slam"
    if args.replay:
        return run_replay(args)
    if args.backend == "go2w_experimental":
        return run_go2w(args)
    return _run_offline(args)


# ---------------------------------------------------------------------------
# shared artifacts
# ---------------------------------------------------------------------------


def _write_session_artifacts(args, explorer: AutonomousExplorer,
                             result, events: list[dict[str, Any]]) -> Path:
    session_id = explorer.session_id
    run_dir = Path(args.session_dir) / session_id
    run_dir.mkdir(parents=True, exist_ok=True)
    graph_path = explorer.save_artifacts(Path(args.session_dir))
    summary_path = run_dir / "summary.json"
    summary_path.write_text(
        json.dumps(result.to_dict(), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    if explorer.task_context is not None:
        (run_dir / "task.json").write_text(
            json.dumps(explorer.task_context.to_dict(), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    (run_dir / "decisions.jsonl").write_text(
        "".join(json.dumps(item.to_dict(), ensure_ascii=False) + "\n" for item in explorer.decision_records),
        encoding="utf-8",
    )
    (run_dir / "semantic_map.json").write_text(
        json.dumps(explorer.semantic_graph.to_dict(), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    (run_dir / "final_state.json").write_text(
        json.dumps(
            {
                "schema_version": "search_final_state_v1",
                "session_id": session_id,
                "state": explorer.state,
                "result": result.to_dict(),
                "task": explorer.task_context.to_dict() if explorer.task_context else {},
                "current_place_id": explorer.semantic_graph.current_place_id,
                "map_revision": explorer.semantic_graph.revision,
                "last_decision": (
                    explorer.decision_records[-1].to_dict()
                    if explorer.decision_records else None
                ),
            },
            ensure_ascii=False,
            indent=2,
        ) + "\n",
        encoding="utf-8",
    )
    jsonl_path = Path(args.output)
    jsonl_path.parent.mkdir(parents=True, exist_ok=True)
    with jsonl_path.open("a", encoding="utf-8") as handle:
        for event in events:
            handle.write(json.dumps(event, ensure_ascii=False) + "\n")
        handle.write(
            json.dumps(
                {"event": "session_summary", "session_id": session_id,
                 **result.to_dict()},
                ensure_ascii=False,
            )
            + "\n"
        )
    print(json.dumps(result.to_dict(), ensure_ascii=False))
    print(f"session_dir={run_dir}")
    print(f"graph={graph_path}")
    print(f"jsonl={jsonl_path}")
    return run_dir


def _build_offline_components(args):
    if args.mock_scenario == "semantic_topology":
        from app.live_robot.mock_observation_scene import scenario_semantic_topology

        return scenario_semantic_topology()
    if args.mock_scenario == "green_bin":
        from app.live_robot.mock_observation_scene import scenario_green_bin

        return scenario_green_bin()
    if args.mock_scenario == "target_appears_after_n":
        scene = scenario_target_appears_after(max(1, args.mock_target_after))
    elif args.mock_scenario == "no_target":
        scene = scenario_no_target()
    else:
        scene = scenario_anchor_then_target()
    scene.confirm_after_seen = max(1, args.mock_confirm_after_seen)
    return scene


# Explorer events that carry map meaning; the live WebUI worker augments
# these with the full exploration-graph snapshot for the SVG map.
_MAP_RELEVANT_EVENTS = ("observation", "memory_update", "navigation_result")


def _run_offline(args, event_hook=None) -> int:
    policy = load_exploration_policy(args.exploration_config, overrides={
        "exploration": {
            "budget": {
                "max_search_seconds": args.max_seconds,
                "max_planning_cycles": args.max_planning_cycles,
                "max_motion_steps": args.max_motion_steps,
            },
        },
    })
    scene = _build_offline_components(args)
    backend_kind = "mock_metric" if args.backend == "mock_metric" else "mock"
    backend = create_backend(backend_kind)
    task_context = _task_context_from_args(args)
    graph = ExplorationGraph(session_id=args.session_id or time.strftime("explore_offline_%Y%m%d_%H%M%S"))
    holder: dict[str, Any] = {"explorer": None}

    def on_event(event: dict[str, Any]) -> None:
        if event_hook is not None:
            explorer = holder.get("explorer")
            if explorer is not None and event.get("event") in _MAP_RELEVANT_EVENTS:
                event = {**event, "graph": explorer.graph.to_dict()}
            event = event_hook(event, holder)
        if event is None:
            return
        explorer = holder.get("explorer")
        if explorer is not None:
            explorer.events.append(event)

    explorer = AutonomousExplorer(
        target=task_context.canonical_target,
        task_context=task_context,
        observer=scene.observer(),
        matcher=scene.matcher(),
        verifier=scene.verifier(),
        backend=backend,
        policy=policy,
        graph=graph,
        negative_target_key=task_context.canonical_target,
        finish_on_visual_confirmation=args.finish_on_visual_confirmation,
        turn_only=args.turn_only,
        session_id=graph.session_id,
        executor_id=args.executor_id or None,
        worker_generation=args.worker_generation,
        on_event=on_event,
    )
    holder["explorer"] = explorer
    result = explorer.run()
    _write_session_artifacts(args, explorer, result, explorer.events)
    return 0 if result.result == "TARGET_FOUND" else 3


def run_replay(args, event_hook=None) -> int:
    """Deterministic replay of a previous session's observation events."""
    jsonl_path = Path(args.replay)
    if not jsonl_path.is_file():
        print(json.dumps({"status": "failed", "error": f"replay file missing: {jsonl_path}"},
                         ensure_ascii=False))
        return 2
    observations: list[LiveObservation] = []
    for line in jsonl_path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            continue
        if event.get("event") == "observation":
            observation = LiveObservation.from_dict(event)
            observation.target_match = {
                "target_present": bool(event.get("target_present", False)),
                "target_state": str(
                    event.get("target_state")
                    or (event.get("target_match") or {}).get("target_state")
                    or ("PRESENT" if event.get("target_present") else "ABSENT")
                ),
                "score": (
                    float((event.get("target_match") or {}).get("score", 0.0))
                    if isinstance(event.get("target_match"), dict) else 0.0
                ),
            }
            observations.append(observation)
    if not observations:
        print(json.dumps({"status": "failed", "error": "no observation events in replay file"},
                         ensure_ascii=False))
        return 2
    policy = load_exploration_policy(args.exploration_config)
    backend = create_backend("mock")
    index = [0]

    def observe() -> LiveObservation:
        obs = observations[min(index[0], len(observations) - 1)]
        index[0] += 1
        return obs

    def matcher(observation: LiveObservation) -> SemanticMatch:
        return SemanticMatch(
            has_candidate=bool(observation.target_present),
            target_match=observation.target_match,
            target_score=float((observation.target_match or {}).get("score", 0.0)),
            target_match_level="candidate" if observation.target_present else "none",
            target_state=observation.target_state,
            provenance={"source": "replay"},
        )

    def verifier(observation: LiveObservation,
                 match: SemanticMatch) -> VerificationOutcome:
        return VerificationOutcome(
            confirmed=bool(observation.target_present),
            attempts=1,
            reason_zh="replay verify",
        )

    graph = ExplorationGraph(session_id=f"replay_{time.strftime('%Y%m%d_%H%M%S')}")
    holder: dict[str, Any] = {"explorer": None}

    def on_event(event: dict[str, Any]) -> None:
        if event_hook is not None:
            explorer = holder.get("explorer")
            if explorer is not None and event.get("event") in _MAP_RELEVANT_EVENTS:
                event = {**event, "graph": explorer.graph.to_dict()}
            event = event_hook(event, holder)
        if event is None:
            return
        explorer = holder.get("explorer")
        if explorer is not None:
            explorer.events.append(event)

    explorer = AutonomousExplorer(
        target=args.target,
        observer=observe,
        matcher=matcher,
        verifier=verifier,
        backend=backend,
        policy=policy,
        graph=graph,
        negative_target_key=args.target,
        finish_on_visual_confirmation=args.finish_on_visual_confirmation,
        turn_only=args.turn_only,
        session_id=graph.session_id,
        on_event=on_event,
    )
    holder["explorer"] = explorer
    result = explorer.run()
    _write_session_artifacts(args, explorer, result, explorer.events)
    return 0 if result.result == "TARGET_FOUND" else 3


# ---------------------------------------------------------------------------
# real Go2-W
# ---------------------------------------------------------------------------


def run_go2w(args, event_hook=None) -> int:
    if not (args.operator_supervised_experiment or args.operator_authorized_rotation):
        if args.dry_run_motion:
            # WebUI dry-run (plan book §60): the full real perception /
            # reasoning pipeline runs but no motion command is ever sent, so
            # operator rotation authorization is not required.
            print(json.dumps({"status": "dry_run_motion", "note": "no motion commands will be sent"},
                             ensure_ascii=False))
        else:
            print(
                json.dumps(
                    {
                        "status": "blocked",
                        "reason": (
                            "go2w_experimental backend requires "
                            "--operator-supervised-experiment (operator with remote "
                            "present); turns are blocked without operator authorization "
                            "because rotation clearance is fail-closed"
                        ),
                    },
                    ensure_ascii=False,
                )
            )
            return 3
    try:
        import rclpy
    except ImportError as exc:
        print(json.dumps({"status": "failed", "error": f"rclpy unavailable: {exc}"}))
        return 2
    from run_autonomous_loop import AutonomousLoop, PROMPT_MAP  # noqa: E402
    vision_model = str(args.llm_model or get_settings().vision_model)

    # Load the repository-owned operator-supervised profile at the real
    # backend boundary.  This does not invoke Stage-2/Nav2 readiness gates.
    experiment_profile = load_go2w_experiment_profile(args.profile_config)
    profile_name = str(
        experiment_profile.get("profile") or "operator_supervised_experiment"
    )
    profile_limits = experiment_profile.get("limits") or {}
    profile_motion = experiment_profile.get("motion_primitives") or {}
    profile_execution = experiment_profile.get("execution") or {}
    max_turn_deg = min(
        abs(float(args.max_turn_deg)),
        abs(float(profile_limits.get("max_turn_deg", args.max_turn_deg))),
        30.0,
    )
    forward_step_m = min(
        max(0.0, float(args.forward_step_m)),
        max(0.0, float(profile_limits.get("max_forward_step_m", 1.5))),
        1.5,
    )

    policy = load_exploration_policy(args.exploration_config, overrides={
        "exploration": {
            "budget": {
                "max_search_seconds": args.max_seconds,
                "max_planning_cycles": args.max_planning_cycles,
                "max_motion_steps": args.max_motion_steps,
            },
            # Use the profile-clamped value so candidate generation cannot
            # propose a turn larger than the operator-supervised contract.
            "candidates": {"fallback_turn_deg": max_turn_deg},
        },
    })
    rclpy.init()
    output_path = str(Path(args.output))
    node = AutonomousLoop(
        pattern=["f"], output=output_path, forward_vx=0.12,
        forward_seconds=2.0, max_yaw_rate=0.15, min_clearance_m=0.30,
        mode="state_machine_search", max_seconds=args.max_seconds,
        wander_front_go_m=0.45, wander_turn_deg=max_turn_deg,
        max_radius_m=args.max_radius, scan_turn_deg=30.0, scan_span=3,
        pre_scan_turns=0, record_video=args.record_video,
        video_fps=15.0, video_scale=0.4, scan360_steps=8,
        scan360_turn_deg=45.0, odom_topic=args.odom_topic,
    )
    node._target = args.target
    node._detector = args.detector
    node._llm_model = vision_model
    node._spool_root = args.spool_root
    node._target_score_min = args.target_score_min
    node._align_threshold = 0.08
    node._align_yaw_max_deg = 25.0
    node._reach_area_ratio = args.reach_area_ratio
    node._semantic_reasoning = True
    node._search_reasoner = args.reasoner
    node._search_reasoner_mode = "active"
    node._semantic_allow_forward = bool(args.semantic_allow_forward)
    node._front_half_plane_only = False
    node._turn_only = bool(args.turn_only)
    node._max_motion_steps = 0
    node._min_rotation_clearance_m = 0.0
    node._operator_authorized_rotation = bool(
        args.operator_supervised_experiment or args.operator_authorized_rotation
    )
    node._rotation_lease_path = ""
    node._rotation_lease_error = ""
    node._experiment_profile_name = profile_name
    node._experiment_motion_primitives = {
        str(name): bool(enabled) for name, enabled in profile_motion.items()
    }
    node._experiment_max_turn_deg = max_turn_deg
    node._experiment_forward_step_m = forward_step_m
    # A long semantic move is executed as independently stopped and verified
    # chunks.  Each chunk re-enters the LiDAR/mode/arm safety gates.
    node._experiment_forward_segment_m = min(0.30, forward_step_m)
    node._forward_duration_scale = max(
        0.5,
        min(
            2.0,
            float(profile_execution.get("forward_command_duration_scale", 1.0)),
        ),
    )

    try:
        return _run_go2w_explorer(args, node, policy, PROMPT_MAP, event_hook)
    except Exception as exc:
        node.get_logger().error(f"semantic exploration failed: {exc}")
        try:
            node._emergency_stop()
        except Exception:
            pass
        try:
            node._arm(False)
        except Exception:
            pass
        return 4
    finally:
        # Disable background Full Semantic submissions before closing the
        # event stream they use.  In-flight calls are daemon threads and may
        # finish later, but they no longer schedule follow-up work.
        try:
            manager = getattr(node, "_semantic_manager", None)
            if manager is not None:
                manager.close()
        except Exception:
            pass
        try:
            node._output.close()
        except Exception:
            pass
        if args.record_video and node._video is not None:
            try:
                node._video.stop()
            except Exception:
                pass
        # rclpy Node.destroy_node() does not own all ActionClient entities on
        # every supported ROS 2 build.  Destroy the client first so its late
        # goal/result handles cannot touch an already-invalid context during
        # shutdown.  The semantic manager is daemon-threaded, but disable it
        # before the ROS context goes away so no late event is emitted into a
        # closed output stream.
        try:
            node._client.destroy()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _run_go2w_explorer(args, node, policy, prompt_map, event_hook=None) -> int:
    import rclpy  # noqa: F401  (used by the observer closure; rclpy.init already done)
    settings = get_settings()
    env = node._load_detector_env()
    # `vision_model` is used by the nested ``analyze_semantic`` observer.  It
    # is a local variable of the caller ``run_go2w`` and is never passed into
    # this function, so re-derive it here (fixes NameError in the worker).
    vision_model = str(args.llm_model or settings.vision_model or "Qwen/Qwen3-VL-8B-Instruct")
    # Keep proxy variables in the child env; the robot/developer host may
    # require the local forward proxy for external SiliconFlow API access.
    # Ensure the detector subprocess uses a real interpreter on this machine
    # instead of the author's hard-coded path.
    if not env.get("SILICONFLOW_PYTHON") and not env.get("GROUNDED_SAM_PYTHON"):
        for candidate in (
            os.environ.get("GO2W_CONDA_PYTHON"),
            str(PROJECT_ROOT / ".runtime_venv/bin/python"),
            str(PROJECT_ROOT / ".venv/bin/python"),
            sys.executable,
        ):
            if candidate and Path(candidate).is_file():
                env["SILICONFLOW_PYTHON"] = candidate
                break
    spool_root = args.spool_root
    state: dict[str, Any] = {
        "image_path": None,
        "semantic": None,
        "common_sense_pending": False,
        "common_sense_updated_ts": 0.0,
    }
    # 计划书 §5.1/§14：RGB-D 帧缓存（有界）与运动后新帧门控状态。
    rgbd_frame_cache: dict[str, Any] = {}
    motion_state: dict[str, Any] = {
        "motion_end_timestamp": 0.0,
        "last_frame_id_before_motion": None,
        "last_yaw_before": None,
        "pending_motion_evidence": None,
    }

    def _last_success_age_s() -> float | None:
        ts = state.get("last_success_ts")
        if ts is None:
            return None
        return max(0.0, time.time() - float(ts))
    operator_authorized = bool(
        args.operator_supervised_experiment or args.operator_authorized_rotation
    )

    # ---- pre-run sensor waits (mirrors the audited runner) ------------------
    if not node._wait_for(lambda: node._sport is not None and node._odom is not None,
                          10.0, "sport/odom"):
        print(json.dumps({"status": "failed", "error": "sport/odom unavailable"},
                         ensure_ascii=False))
        return 2
    if not node._wait_for(
        lambda: node._lidar_fresh is True and node._clearance is not None,
        5.0, "fresh LiDAR clearance",
    ):
        print(json.dumps({"status": "failed", "error": "LiDAR clearance unavailable"},
                         ensure_ascii=False))
        return 2
    node._motion_origin = node._odom_snapshot()
    ready, reason = node._safety_ok()
    if not ready:
        node._write({"event": "pre_arm_safety_reject", "reason": reason,
                     "host_s": node._host_s()})
        print(json.dumps({"status": "blocked", "reason": reason}, ensure_ascii=False))
        return 2

    # ---- automatic experiment readiness ------------------------------------
    readiness = _probe_readiness(args, node)
    print(json.dumps(readiness.to_dict(), ensure_ascii=False))
    node._write({"event": "experiment_readiness", "host_s": node._host_s(),
                 **readiness.to_dict()})
    if not readiness.ready and not args.allow_degraded:
        print(json.dumps({"status": "blocked", "reason": readiness.reason},
                         ensure_ascii=False))
        return 3

    # ---- semantic stack -----------------------------------------------------
    semantic_memory = SemanticSearchMemory(
        default_ttl_sec=policy.budget.negative_memory_ttl_seconds,
        observation_store=(
            ObservationMemoryStore(settings=settings)
            if settings.live_search_reasoner_use_observation_memory else None
        ),
    )
    llm_prior_generator = LLMPriorGenerator(settings=settings)
    profile = TargetProfileResolver().resolve(args.target, use_llm=False)
    controller = SemanticSearchController(
        profile,
        backend=args.reasoner,
        partial_threshold=settings.live_search_graph_match_partial_threshold,
        strong_threshold=settings.live_search_graph_match_strong_threshold,
    )
    prompt = prompt_map.get(args.target.strip(),
                            f"{args.target.strip()}. {args.target.strip()} object")

    def analyze_semantic(
        image_path: object,
        _profile: object,
        *,
        request_id: str | None = None,
        frame_id: str | None = None,
        robot_pose: dict[str, Any] | None = None,
    ) -> dict:
        if not isinstance(image_path, str):
            raise RuntimeError("semantic observation has no stable image")
        # 每次请求使用独立 output 文件，避免并发 Full Semantic 互相覆盖。
        request_id = request_id or f"{time.time_ns()}_{os.getpid()}"
        request_dir = PROJECT_ROOT / "runtime/go2w/vlm_requests"
        request_dir.mkdir(parents=True, exist_ok=True)
        output_path = request_dir / f"semantic_{request_id}.json"
        # 优先走长驻 VLM daemon；不可用时再回退到 subprocess（legacy fallback）。
        try:
            from app.detectors.siliconflow_vision_protocol import (
                SiliconFlowDaemonClient,
                VLMRequest,
            )

            daemon_client = SiliconFlowDaemonClient(
                str(PROJECT_ROOT / "runtime/go2w/siliconflow_vlm.sock"),
                timeout=max(45.0, float(settings.siliconflow_timeout_seconds) + 15.0),
            )
            if daemon_client.available():
                daemon_request = VLMRequest(
                    request_id=f"semantic_{request_id}",
                    mode="semantic",
                    image_path=image_path,
                    frame_id=str(frame_id or state.get("frame_id", "semantic_live")),
                    target=args.target,
                    priority="background",
                    extra_instructions=(
                        "完整列出当前画面的可见物体与关系，供下一视角选择；不要确认目标。"
                    ),
                    model=vision_model,
                    robot_pose=robot_pose,
                )
                daemon_response = daemon_client.request(daemon_request)
                if daemon_response.ok and isinstance(daemon_response.payload, dict):
                    daemon_payload = dict(daemon_response.payload)
                    daemon_payload.update({
                        "image_path": image_path,
                        "frame_id": str(frame_id or state.get("frame_id", "semantic_live")),
                        "robot_pose": robot_pose,
                        "source": "siliconflow_full_scene_vlm_daemon",
                        "semantic_status": SEMANTIC_STATUS_FRESH_FULL,
                        "semantic_quality": "full",
                        "semantic_source_frame_id": str(
                            frame_id or state.get("frame_id", "semantic_live")
                        ),
                        "semantic_capture_timestamp": time.time(),
                        "semantic_completed_timestamp": time.time(),
                        "semantic_age_ms": 0.0,
                        "semantic_source_pose": robot_pose,
                    })
                    return daemon_payload
        except Exception:
            # 任何 daemon IPC 问题都走 subprocess fallback，不中断搜索。
            pass
        # 始终走全场景分析（objects + relations 一起拿）。快速“无目标”捷径只返回
        # objects、不返回 relations，会导致拓扑只有孤立节点；全场景更长但能
        # 既建节点又连边（用 max_tokens=2048，不截断）。
        python = env.get(
            "SILICONFLOW_PYTHON",
            env.get(
                "GROUNDED_SAM_PYTHON",
                sys.executable,
            ),
        )
        worker = PROJECT_ROOT / "app/detectors/siliconflow_vision_worker.py"
        command = [
            python, str(worker), "--image", image_path,
            "--output", str(output_path), "--target", args.target,
            "--extra-instructions",
            "完整列出当前画面的可见物体与关系，供下一视角选择；不要确认目标。",
            "--model", vision_model,
        ]
        last_err = ""
        completed = None
        semantic_timeout = max(
            35.0,
            min(90.0, float(settings.siliconflow_timeout_seconds) + 15.0),
        )
        for attempt in range(2):
            try:
                completed = subprocess.run(
                    command, cwd=str(PROJECT_ROOT), env=env, text=True,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    timeout=semantic_timeout, check=False,
                )
            except subprocess.TimeoutExpired as exc:
                # 计划书 §3.2 不变量 1：timeout ≠ empty scene。必须抛结构化
                # 异常让后台管理器标记语义不可用，绝不返回“空场景成功”。
                raise PerceptionFailure(
                    "FULL_SEMANTIC_TIMEOUT: full-scene analysis exceeded "
                    f"{semantic_timeout:.0f}s",
                    code="FULL_SEMANTIC_TIMEOUT",
                    recoverable=True,
                    detail=str(exc),
                ) from exc
            if completed.returncode == 0:
                break
            last_err = completed.stderr[-600:]
        if completed is None or completed.returncode != 0:
            # 单帧视觉解析失败绝不中断整场搜索：后台管理器保持最近一次成功
            # 的 latest_success，当前帧用 Quick 快路径结果继续（计划书 §8.4）。
            node.get_logger().warn(
                f"semantic observer failed (2 tries): {last_err}"
            )
            raise PerceptionFailure(
                f"FULL_SEMANTIC_ERROR: {last_err}",
                code="FULL_SEMANTIC_ERROR",
                recoverable=True,
                detail=last_err,
            )
        if not output_path.is_file():
            raise PerceptionFailure(
                f"semantic output missing: {output_path}",
                code="VLM_PARSE_ERROR",
                recoverable=True,
            )
        try:
            payload = json.loads(output_path.read_text(encoding="utf-8"))
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            raise PerceptionFailure(
                f"semantic output unreadable: {exc}",
                code="VLM_PARSE_ERROR",
                recoverable=True,
            ) from exc
        payload.update({
            "image_path": image_path,
            "frame_id": str(frame_id or state.get("frame_id", "semantic_live")),
            "robot_pose": robot_pose,
            "source": "siliconflow_full_scene_existing_pipeline",
            "semantic_status": SEMANTIC_STATUS_FRESH_FULL,
            "semantic_quality": "full",
            "semantic_source_frame_id": str(frame_id or state.get("frame_id", "semantic_live")),
            "semantic_capture_timestamp": time.time(),
            "semantic_completed_timestamp": time.time(),
            "semantic_age_ms": 0.0,
            "semantic_source_pose": robot_pose,
        })
        return payload

    semantic_observer = LiveSemanticObserver(
        analyze_semantic,
        ttl_seconds=settings.live_search_reasoner_scene_ttl_seconds,
    )
    # 后台 Full Semantic 管理器：普通帧不再同步等待全场景分析。
    semantic_manager = AsyncSemanticObservationManager(
        analyze_semantic,
        enabled=settings.vlm_runtime_semantic_background_enabled,
        max_inflight=settings.vlm_runtime_semantic_max_inflight,
        ttl_seconds=max(5.0, settings.vlm_runtime_semantic_ttl_seconds),
        translation_refresh_m=settings.vlm_runtime_semantic_translation_refresh_m,
        heading_sector_deg=settings.vlm_runtime_semantic_heading_sector_deg,
        initial_warmup_blocking=settings.vlm_runtime_semantic_initial_warmup_blocking,
        visual_change_enabled=settings.vlm_runtime_semantic_visual_change_enabled,
        visual_change_threshold=None,
        event_sink=node._write,
    )
    # Keep the manager reachable from the outer lifecycle cleanup.  Its
    # request threads are daemonized, but they must be disabled before the
    # ROS node/output stream is torn down.
    node._semantic_manager = semantic_manager
    observed_sectors: set[int] = set()

    # Optional D435 atomic RGB-D source (plan §17/§22).
    rgbd_source = None
    depth_localizer = None
    if args.rgbd_source:
        from app.perception.depth_object_localizer import DepthObjectLocalizer
        from app.perception.realsense_http_rgbd_source import RealSenseHTTPRGBDSource

        rgbd_source = RealSenseHTTPRGBDSource(
            args.rgbd_base_url,
            cache_dir=str(PROJECT_ROOT / "runtime/go2w/rgbd_cache"),
        )
        depth_localizer = DepthObjectLocalizer()

    # Optional SemanticNavigation V2 spatial exploration state (plan §62-§90).
    place_graph = None
    semantic_map = None
    entity_graph = None
    route_planner = None
    spatial_memory = None
    camera_provider = None
    bev_mapper = None
    pose_validator = None
    frontier_extractor = None
    psg_provider = None
    spatial_reasoner = None
    local_executor = None
    if args.spatial_v2:
        from app.navigation.decision_record import build_decision_record
        from app.navigation.local_goal_executor import LocalGoalExecutor
        from app.navigation.long_term_goal_selector import LongTermGoalSelector
        from app.navigation.semantic_route_planner import SemanticRoutePlanner
        from app.reasoning.semantic_navigation.semantic_prior_provider import RuleSemanticPriorProvider
        from app.reasoning.semantic_navigation.spatial_reasoner import SpatialSearchReasoner
        from app.spatial.camera_local_spatial_provider import CameraLocalSpatialProvider
        from app.spatial.frontier_extractor import FrontierExtractor
        from app.spatial.lightweight_depth_bev import LightweightDepthBEVMapper
        from app.spatial.place_graph import PlaceGraph
        from app.spatial.semantic_entity_graph import SemanticEntityGraph
        from app.spatial.semantic_object_map import SemanticObjectMap
        from app.spatial.spatial_memory import SpatialMemory

        place_graph = PlaceGraph(
            merge_distance_m=0.35,
            relocation_min_displacement_m=0.50,
        )
        pose_validator = SpatialPoseValidator()
        semantic_map = SemanticObjectMap(
            merge_distance_m=0.4,
            confirm_min_observations=2,
        )
        entity_graph = SemanticEntityGraph(
            place_graph=place_graph, object_map=semantic_map, frame_id="map"
        )
        route_planner = SemanticRoutePlanner(
            inflation_radius_m=0.25,
            allow_unknown=False,
            max_waypoints=32,
        )
        spatial_memory = SpatialMemory()
        camera_provider = None
        if args.spatial_provider == "plain_slam":
            from app.spatial.plain_slam_spatial_provider import PlainSlamSpatialProvider

            camera_provider = PlainSlamSpatialProvider(
                enable_ros=True,
                map_topic="/go2w/slam/map_2d",
                odom_topic="/go2w/slam/odom_base",
                fallback=CameraLocalSpatialProvider(
                    relocate_distance_m=args.forward_step_m or 0.25,
                ),
            )
        elif args.spatial_provider == "rtabmap":
            from app.spatial.rtabmap_spatial_provider import RtabmapSpatialProvider

            camera_provider = RtabmapSpatialProvider(
                enable_ros=True,
                map_topic="/rtabmap/map",
                odom_topic="/rtabmap/odom",
            )
        else:
            camera_provider = CameraLocalSpatialProvider(
                relocate_distance_m=args.forward_step_m or 0.25,
            )
        node.get_logger().info(f"spatial provider = {args.spatial_provider}")
        bev_mapper = LightweightDepthBEVMapper()
        frontier_extractor = FrontierExtractor(min_component_size=1)
        psg_provider = RuleSemanticPriorProvider()
        spatial_reasoner = SpatialSearchReasoner(
            LongTermGoalSelector(
                psg_zero_weight=1.0,
                psg_partial_weight=0.7,
                psg_strong_weight=0.2,
                psg_verify_weight=0.0,
            )
        )
        local_executor = LocalGoalExecutor(
            forward_step_m=getattr(node, "_experiment_forward_step_m", None)
            or args.forward_step_m or 0.25,
            max_turn_deg=getattr(node, "_experiment_max_turn_deg", None)
            or args.max_turn_deg,
            turn_only=bool(args.turn_only),
        )
        state["spatial_v2"] = {
            "place_graph": place_graph,
            "semantic_map": semantic_map,
            "entity_graph": entity_graph,
            "route_planner": route_planner,
            "spatial_memory": spatial_memory,
            "local_executor": local_executor,
        }

    # ---- observer -----------------------------------------------------------
    observe_cache_dir = PROJECT_ROOT / "runtime/go2w/semantic_observe_cache"
    observe_cache_dir.mkdir(parents=True, exist_ok=True)

    def _cached_image(image_path: str, frame_id: Any) -> str:
        """Copy the bundle image to a stable path so later LLM steps (which can
        take minutes) never lose it to spool rotation."""
        import shutil

        target = observe_cache_dir / f"bundle_{frame_id}.jpg"
        try:
            shutil.copy2(image_path, target)
        except OSError:
            return image_path
        # keep only the newest 60 cached frames
        cached = sorted(observe_cache_dir.glob("bundle_*.jpg"),
                        key=lambda p: p.stat().st_mtime, reverse=True)
        for old in cached[60:]:
            try:
                old.unlink()
            except OSError:
                pass
        return str(target)

    def observe() -> LiveObservation:
        state["planning_cycle"] = int(state.get("planning_cycle", 0)) + 1
        profiler = LatencyProfiler(
            planning_cycle=int(state.get("planning_cycle", 0)),
            frame_id=str(state.get("frame_id", "")),
        )
        profiler.mark("capture_start")
        for _ in range(4):
            rclpy.spin_once(node, timeout_sec=0.05)
        rgbd_frame = None
        try:
            if rgbd_source is not None:
                rgbd_frame = rgbd_source.get_latest(timeout_seconds=5.0)
                frame_id = rgbd_frame.frame_id
                stable_image = _cached_image(rgbd_frame.color_ref, frame_id)
                state["image_path"] = stable_image
                state["frame_id"] = frame_id
                profiler.profile.frame_id = str(frame_id)
                # 计划书 §5.1：RGB-D 帧缓存（有界 60 帧），供“语义来自旧帧但
                # 仍要按 source frame 做 depth localization”的场景使用。
                rgbd_frame_cache[str(frame_id)] = rgbd_frame
                if len(rgbd_frame_cache) > 60:
                    for stale_id in list(rgbd_frame_cache)[: len(rgbd_frame_cache) - 60]:
                        rgbd_frame_cache.pop(stale_id, None)
                node._write({"event": "camera_frame_selected", "frame_id": frame_id,
                             "source": "d435", "capture_timestamp": rgbd_frame.timestamp,
                             "host_s": node._host_s()})
            else:
                image_path, frame_id = node._latest_bundle_image(spool_root)
                stable_image = _cached_image(image_path, frame_id)
                state["image_path"] = stable_image
                state["frame_id"] = frame_id
                profiler.profile.frame_id = str(frame_id)
                node._write({"event": "camera_frame_selected", "frame_id": frame_id,
                             "source": "spool", "host_s": node._host_s()})
        except Exception as exc:  # noqa: BLE001 - 感知失败可恢复，交给 explorer retry
            raise PerceptionFailure(
                f"RGB-D capture failed: {type(exc).__name__}: {exc}",
                code=_classify_perception_error(exc),
                recoverable=True,
                detail=str(exc),
            ) from exc
        # 计划书 §14：运动后必须拿到新帧才允许规划下一步。运动刚结束时旧帧
        # 短暂可接受（相机流有延迟），超过 3 秒仍无新帧 -> FRAME_STALE retry。
        motion_end_ts = float(motion_state.get("motion_end_timestamp") or 0.0)
        last_motion_frame = motion_state.get("last_frame_id_before_motion")
        if motion_end_ts > 0.0 and last_motion_frame is not None:
            if str(frame_id) == str(last_motion_frame):
                waited = time.time() - motion_end_ts
                if waited > 3.0:
                    raise PerceptionFailure(
                        "FRAME_STALE: no new camera frame after motion ended "
                        f"(frame={frame_id}, waited={waited:.1f}s)",
                        code="FRAME_STALE",
                        recoverable=True,
                        detail="motion ended but camera frame did not advance",
                    )
                # 短时间内再取一次，期望拿到运动后的新帧。
                for _ in range(2):
                    time.sleep(0.5)
                    rclpy.spin_once(node, timeout_sec=0.1)
                    try:
                        if rgbd_source is not None:
                            rgbd_frame = rgbd_source.get_latest(timeout_seconds=2.0)
                            frame_id = rgbd_frame.frame_id
                            stable_image = _cached_image(rgbd_frame.color_ref, frame_id)
                            state["image_path"] = stable_image
                            state["frame_id"] = frame_id
                            rgbd_frame_cache[str(frame_id)] = rgbd_frame
                        else:
                            image_path, frame_id = node._latest_bundle_image(
                                spool_root, retries=0
                            )
                            stable_image = _cached_image(image_path, frame_id)
                            state["image_path"] = stable_image
                            state["frame_id"] = frame_id
                    except Exception:  # noqa: BLE001 - 保留旧帧信息，由上层判断
                        pass
                    if str(frame_id) != str(last_motion_frame):
                        break
        profiler.record("capture_ms", "capture_start")
        x, y, yaw = node._odom_snapshot()
        pose = {"x": x, "y": y, "yaw_rad": yaw,
                "yaw_deg": math.degrees(yaw)}
        capture_ts = time.time()
        # 计划书 §6.2：navigation heading sector 是运动状态，永远由当前
        # capture pose 计算，与语义是否 stale 无关。
        navigation_sector = _navigation_sector(
            yaw_rad=yaw,
            sectors=max(1, int(policy.candidates.heading_sectors)),
        )
        state["navigation_heading_sector"] = navigation_sector
        node._write({
            "event": "navigation_sector_observed",
            "frame_id": str(frame_id),
            "navigation_heading_sector": navigation_sector,
            "yaw_deg": round(math.degrees(yaw), 3),
            "host_s": node._host_s(),
        })
        # 后台 Full Semantic single-flight（计划书 §3.5）：首帧也提交后台，
        # 当前轮不再同步等待 75 秒；失败/超时绝不进入 latest_success。
        semantic_manager.submit_if_needed(
            image_path=stable_image,
            frame_id=str(frame_id),
            capture_timestamp=capture_ts,
            robot_pose=pose,
            target_profile=controller.target_profile,
            scene_signature=compute_scene_signature(stable_image),
        )
        profiler.mark("quick_start")
        node._write({"event": "quick_vlm_started", "frame_id": str(frame_id),
                     "host_s": node._host_s()})
        quick_start_perf = time.perf_counter()
        try:
            objects = node._detect(stable_image, prompt, env)
        except PerceptionFailure:
            raise
        except Exception as exc:  # noqa: BLE001 - 结构化错误交给 explorer retry
            raise PerceptionFailure(
                f"Quick VLM failed: {type(exc).__name__}: {exc}",
                code=_classify_perception_error(exc),
                recoverable=True,
                detail=str(exc),
                last_success_age_s=_last_success_age_s(),
            ) from exc
        quick_latency_ms = (time.perf_counter() - quick_start_perf) * 1000.0
        profiler.record("quick_vlm_ms", "quick_start")
        profiler.incr("quick_api_calls")
        node._write({"event": "quick_vlm_completed", "frame_id": str(frame_id),
                     "latency_ms": round(float(quick_latency_ms), 3),
                     "objects": len(objects),
                     "host_s": node._host_s()})
        # Quick VLM 新契约：objects 只含真正目标候选；target_decision 是目标 gate。
        quick_payload = None
        if getattr(node, "_detector", "llm") == "llm":
            quick_payload = getattr(node, "_last_llm_detection_payload", None)
        if quick_payload is not None:
            decision = quick_payload.get("target_decision") or {}
            target_objects = list(
                quick_payload.get("target_objects")
                or quick_payload.get("objects")
                or []
            )
            detections = []
            for item in target_objects:
                raw_bbox = item.get("bbox_2d") or item.get("bbox")
                if not isinstance(raw_bbox, (list, tuple)) or len(raw_bbox) != 4:
                    # POSSIBLE candidates are still carried by target_state,
                    # but an invalid box must never be used as a full-frame
                    # verification crop.
                    continue
                bbox = [float(v) for v in raw_bbox]
                detections.append({
                    "label": str(item.get("label", "object")),
                    "score": float(item.get("score", 0.0)),
                    "bbox_2d": bbox,
                })
            target_present = quick_target_present(quick_payload, args.target_score_min)
            target_state = quick_target_state(quick_payload, args.target_score_min)
        else:
            detections = []
            for item in objects:
                bbox = [float(v) for v in item.get("bbox_2d", [0.0, 0.0, 1.0, 1.0])]
                detections.append({
                    "label": str(item.get("label", "object")),
                    "score": float(item.get("score", 0.0)),
                    "bbox_2d": bbox,
                })
            target_present = bool(detections)
            target_state = TARGET_PRESENT if target_present else "ABSENT"
        if detections:
            best = max(detections, key=lambda item: item["score"])
            node._feed_detection(best["label"], best["score"], best["bbox_2d"])
        # ---- 计划书 §4.1：Quick 快路径场景物体（轻量 fallback）-------------
        # Quick 一次调用既给目标判断，又给少量显著普通物体；Full Semantic
        # 成功后再用完整 object list/relations 增强。绝不把普通物体混进
        # target_objects。
        quick_scene_objects: list[dict[str, Any]] = []
        if quick_payload is not None:
            quick_scene_objects = list(
                quick_payload.get("scene_objects_light")
                or quick_payload.get("scene_objects")
                or []
            )
        quick_semantic_payload = {
            "scene_objects": quick_scene_objects,
            "scene_relations": [],
            "scene_summary_zh": str(
                (quick_payload or {}).get("scene_summary_zh") or ""
            ),
            "image_path": stable_image,
            "frame_id": str(frame_id),
            "source": "siliconflow_quick_scene_light",
            "semantic_status": SEMANTIC_STATUS_FRESH_QUICK,
            "semantic_quality": "quick_scene_light",
            "semantic_source_frame_id": str(frame_id),
            "semantic_capture_timestamp": capture_ts,
            "semantic_completed_timestamp": capture_ts,
            "semantic_age_ms": 0.0,
            "semantic_source_pose": pose,
            "target_state": target_state,
            "target_decision": (quick_payload or {}).get("target_decision", {}),
        }
        profiler.mark("semantic_start")
        # 先收割后台已完成结果；未完成时继续用 latest completed。
        for completed_semantic in semantic_manager.poll_completed():
            node._write({
                "event": "semantic_background_applied",
                "frame_id": str(getattr(completed_semantic, "frame_id", "")),
                "result_age_ms": round(
                    max(0.0, (time.time() - float(
                        getattr(completed_semantic, "semantic_completed_timestamp", 0.0)
                        or completed_semantic.timestamp_sec)) * 1000.0),
                    3,
                ),
                "objects": len(completed_semantic.objects or []),
                "relations": len(completed_semantic.relations or []),
                "status": getattr(completed_semantic, "semantic_status", "fresh_full"),
                "host_s": node._host_s(),
            })
        latest_semantic = semantic_manager.get_latest_completed()
        # 计划书 §4.2 语义优先级：
        #   当前帧 Full Semantic 成功 > 当前帧 scene_objects_light > 当前帧无语义
        # 禁止：111 秒前的 Full Semantic 冒充“当前场景确认无物体”。
        semantic: SemanticObservation | None = None
        if latest_semantic is not None and (
            latest_semantic.semantic_source_frame_id == str(frame_id)
            or latest_semantic.frame_id == str(frame_id)
        ):
            semantic = latest_semantic
            profiler.set_counter("semantic_cache_hit", True)
        elif latest_semantic is not None and not quick_scene_objects:
            # 旧帧 Full Semantic 仅在很短的新鲜度窗口内作为“历史记忆”兜底，
            # 明确标记 age/status，绝不假装是当前帧事实。
            age_s = max(
                0.0,
                time.time() - float(
                    latest_semantic.semantic_completed_timestamp
                    or latest_semantic.timestamp_sec
                ),
            )
            soft_stale_s = max(
                0.0, float(settings.vlm_runtime_planner_semantic_soft_stale_seconds)
            )
            if age_s <= soft_stale_s:
                semantic = latest_semantic
                semantic.semantic_age_ms = age_s * 1000.0
                if semantic.semantic_status in {"fresh_full", "fresh_quick_scene"}:
                    semantic.semantic_status = "stale"
                    semantic.semantic_quality = "full_older_frame"
                profiler.set_counter("semantic_cache_hit", True)
            else:
                node._write({"event": "semantic_stale", "frame_id": str(frame_id),
                             "source_frame_id": str(latest_semantic.frame_id),
                             "age_ms": round(age_s * 1000.0, 3),
                             "host_s": node._host_s()})
        if semantic is None:
            # 当前帧 Quick 快路径（scene_objects_light）；没有普通物体时也
            # 构造“当前帧无语义”状态，而不是拿旧语义顶替。
            quick_semantic = _from_payload(
                quick_semantic_payload,
                robot_pose=pose,
                sector=navigation_sector,
                now=capture_ts,
            )
            if not quick_scene_objects:
                quick_semantic.semantic_status = SEMANTIC_STATUS_PENDING
                quick_semantic.semantic_quality = "unavailable"
                node._write({
                    "event": "semantic_unavailable",
                    "frame_id": str(frame_id),
                    "detail": "Full Semantic pending and quick scene empty",
                    "host_s": node._host_s(),
                })
            semantic = quick_semantic
            profiler.incr("semantic_api_calls")
            profiler.set_counter("semantic_cache_hit", False)
        if semantic.semantic_age_ms is None:
            semantic.semantic_age_ms = max(
                0.0,
                (time.time() - float(semantic.semantic_capture_timestamp
                                     or semantic.timestamp_sec)) * 1000.0,
            )
        state["semantic"] = semantic
        profiler.set_counter("semantic_source_frame_id", str(getattr(semantic, "frame_id", "")))
        profiler.set_counter("semantic_result_age_ms", max(0.0, float(semantic.semantic_age_ms or 0.0)))
        profiler.set_counter("semantic_status", str(semantic.semantic_status))
        # 计划书 §6.4：所有“机器人看过哪些方向”的记录都使用 navigation sector。
        observed_sectors.add(navigation_sector)
        node._write({
            "event": "semantic_status",
            "frame_id": str(frame_id),
            "semantic_status": semantic.semantic_status,
            "semantic_quality": semantic.semantic_quality,
            "semantic_source_frame_id": semantic.semantic_source_frame_id,
            "semantic_age_ms": round(float(semantic.semantic_age_ms or 0.0), 3),
            "semantic_object_count": len(semantic.objects or []),
            "semantic_error_code": semantic.semantic_error_code,
            "host_s": node._host_s(),
        })

        spatial_quality = "RGB_ONLY"
        camera_xyz = None
        depth_ref = None
        intrinsics = None
        depth_scale = None
        localized_frame_id: str | None = None
        localized: list[Any] = []
        if depth_localizer is not None:
            # 计划书 §5.2 / 不变量：old semantic bbox 严禁配 current depth。
            # 只能使用语义 source frame 对应的深度帧（当前帧或 RGB-D 缓存）。
            from app.perception.depth_object_localizer import resolve_depth_frame

            semantic_frame_id = str(
                getattr(semantic, "semantic_source_frame_id", None)
                or semantic.frame_id
            )
            depth_frame = resolve_depth_frame(
                semantic_frame_id, rgbd_frame, rgbd_frame_cache
            )
            if depth_frame is None and rgbd_frame is not None:
                node._write({
                    "event": "semantic_depth_frame_mismatch",
                    "semantic_frame_id": semantic_frame_id,
                    "current_frame_id": str(rgbd_frame.frame_id),
                    "action": "SEMANTIC_2D_ONLY",
                    "host_s": node._host_s(),
                })
                spatial_quality = "SEMANTIC_2D_ONLY"
            if depth_frame is not None:
                localized_frame_id = str(depth_frame.frame_id)
                localized = depth_localizer.localize(semantic.objects, depth_frame)
                # Enrich object dicts with spatial fields and remember the best
                # camera-local 3D position for the LiveObservation payload.
                enriched_objects: list[dict[str, Any]] = []
                for index, obj in enumerate(semantic.objects):
                    item = dict(obj)
                    if index < len(localized):
                        spatial = localized[index]
                        item["depth_m"] = spatial.depth_m
                        item["bearing_deg"] = spatial.bearing_deg
                        item["camera_xyz"] = list(spatial.camera_xyz) if spatial.camera_xyz else None
                        item["spatial_quality"] = spatial.spatial_quality
                    enriched_objects.append(item)
                semantic.objects = enriched_objects
                # Also publish 3D fields into the observed SceneGraph node
                # attributes so PSG can bind hypotheses to real anchors.
                if semantic.scene_graph is not None:
                    for sg_node in semantic.scene_graph.nodes:
                        label = str(getattr(sg_node, "label_zh", None) or getattr(sg_node, "label", None) or "")
                        for obj in enriched_objects:
                            if str(obj.get("label_zh") or obj.get("label") or "") == label:
                                attrs = dict(getattr(sg_node, "attributes", {}) or {})
                                for key in ("depth_m", "bearing_deg", "camera_xyz", "spatial_quality"):
                                    if obj.get(key) is not None:
                                        attrs[key] = obj[key]
                                sg_node.attributes = attrs
                                break
                localized_with_xyz = [item for item in localized if item.camera_xyz is not None]
                if localized_with_xyz:
                    best_spatial = max(localized_with_xyz, key=lambda item: item.confidence)
                    camera_xyz = list(best_spatial.camera_xyz)
                    spatial_quality = best_spatial.spatial_quality
                depth_ref = depth_frame.depth_ref
                intrinsics = {
                    "fx": depth_frame.fx, "fy": depth_frame.fy,
                    "cx": depth_frame.cx, "cy": depth_frame.cy,
                }
                depth_scale = depth_frame.depth_unit_m

        # Quick and Full are both allowed to omit basic geometric relations.
        # Derive conservative same-frame edges after RGB-D enrichment and
        # before persistent association; endpoint ids remain frame-local here.
        geometric_relations = extract_geometric_relations(semantic.objects)
        if geometric_relations:
            existing_relation_keys = {
                (
                    str(item.get("subject_id") or item.get("source_id") or ""),
                    str(item.get("object_id") or item.get("target_id") or ""),
                    str(item.get("relation") or item.get("relation_type") or "").lower(),
                )
                for item in semantic.relations
                if isinstance(item, dict)
            }
            semantic.relations = list(semantic.relations) + [
                relation for relation in geometric_relations
                if (
                    relation["subject_id"], relation["object_id"], relation["relation"]
                ) not in existing_relation_keys
            ]
            node._write({
                "event": "geometric_relations_extracted",
                "frame_id": str(frame_id),
                "relation_count": len(geometric_relations),
                "host_s": node._host_s(),
            })

        observation = semantic_observation_to_live(
            semantic,
            bundle_id=f"bundle_{frame_id}",
            detections=detections,
            target_present=target_present,
            target_state=target_state,
            pose=pose,
            image_ref=stable_image,
            depth_ref=depth_ref,
            rgbd_frame_id=localized_frame_id,
            intrinsics=intrinsics,
            depth_scale=depth_scale,
            spatial_quality=spatial_quality,
            camera_xyz=camera_xyz,
            navigation_heading_sector=navigation_sector,
            sensor_health={
                "camera": True,
                "lidar": node._lidar_fresh is True,
                "sport_mode_ok": (
                    node._sport is not None
                    and int(node._sport.mode) == 1
                    and int(node._sport.error_code) == 0
                ),
            },
        )

        # ---- SemanticNavigation V2 spatial state update -------------------------------
        if place_graph is not None and pose is not None:
            from app.spatial.camera_local_spatial_provider import CameraLocalSpatialProvider
            from app.spatial.models import SpatialPose

            # 计划书 §9.2 / 不变量 3：空间地图位姿必须来自 SpatialProvider 自己的
            # 世界坐标系（plain_slam -> pslam_odom）。wheel odom 只用于相对运动/
            # 当前 yaw/运动验证，绝不冒充 pslam pose。
            spatial_pose = None
            raw_spatial_pose = None
            provider_quality = "CAMERA_LOCAL"
            if camera_provider is not None:
                spin = getattr(camera_provider, "spin_once", None)
                if spin is not None:
                    spin()
                try:
                    spatial_pose = camera_provider.get_pose()
                except Exception:  # noqa: BLE001 - degraded pose never crashes
                    spatial_pose = None
                provider_quality = getattr(
                    camera_provider, "quality", lambda: "CAMERA_LOCAL"
                )()
                if spatial_pose is None and isinstance(camera_provider, CameraLocalSpatialProvider):
                    # 仅 camera-local 提供者接受 wheel odom 相对位姿（无全局地图）。
                    spatial_pose = SpatialPose(
                        x=float(pose["x"]),
                        y=float(pose["y"]),
                        yaw=float(pose["yaw_rad"]),
                        frame_id="odom",
                        quality="relative",
                        source="go2w_wheel_odom",
                    )
                    camera_provider.set_pose(spatial_pose)
                elif spatial_pose is None:
                    node._write({
                        "event": "spatial_frame_mismatch",
                        "detail": "NO_GLOBAL_SPATIAL_POSE: wheel odom not injected "
                                  "into metric provider",
                        "provider": type(camera_provider).__name__,
                        "host_s": node._host_s(),
                    })
            raw_spatial_pose = spatial_pose
            motion_evidence = motion_state.get("pending_motion_evidence")
            validation = (
                pose_validator.validate(
                    spatial_pose,
                    motion_evidence,
                    timestamp=observation.timestamp,
                )
                if pose_validator is not None else None
            )
            if validation is not None:
                if validation.accepted:
                    spatial_pose = validation.accepted_pose
                else:
                    # Preserve only the last good pose for diagnostics; do not
                    # write the rejected raw pose into metric map/place data.
                    spatial_pose = None
                    provider_quality = "DEGRADED_LIO"
                node._write({
                    "event": "spatial_pose_validation",
                    "frame_id": str(frame_id),
                    "accepted": validation.accepted,
                    "health": validation.health,
                    "reason_code": validation.reason_code,
                    "pslam_delta_xy_m": validation.pslam_delta_xy_m,
                    "pslam_delta_yaw_deg": validation.pslam_delta_yaw_deg,
                    "wheel_delta_xy_m": validation.wheel_delta_xy_m,
                    "raw_pose": raw_spatial_pose.to_dict() if raw_spatial_pose else None,
                    "accepted_pose": validation.accepted_pose.to_dict() if validation.accepted_pose else None,
                    "host_s": node._host_s(),
                })
                observation.sensor_health.update({
                    "spatial_pose_health": validation.health,
                    "spatial_pose_reason": validation.reason_code,
                    "spatial_pose_accepted": validation.accepted,
                })
            spatial_provenance = {}
            if camera_provider is not None and hasattr(camera_provider, "transform_provenance"):
                try:
                    spatial_provenance = camera_provider.transform_provenance()
                except Exception:  # noqa: BLE001
                    spatial_provenance = {}
            # Phase 2: write map_xyz into each localized observation BEFORE
            # entity association so cross-view fusion can use world coords.
            if camera_provider is not None and spatial_pose is not None:
                for obs_item in localized:
                    if getattr(obs_item, "camera_xyz", None) is None:
                        continue
                    try:
                        mapped = camera_provider.camera_point_to_spatial(
                            obs_item.camera_xyz, pose=spatial_pose
                        )
                    except SpatialFrameMismatch as exc:
                        node._write({
                            "event": "spatial_frame_mismatch",
                            "error": str(exc),
                            "host_s": node._host_s(),
                        })
                        mapped = None
                    except Exception:  # noqa: BLE001 - degraded mapping must not crash
                        mapped = None
                    if mapped is not None:
                        obs_item.map_xyz = mapped
                        obs_item.spatial_quality = provider_quality
                        obs_item.provenance = {
                            **dict(obs_item.provenance or {}),
                            **spatial_provenance,
                            "map_frame": spatial_provenance.get("target_frame", "map"),
                        }
            place_id, created = place_graph.register_observation(
                observation_id=observation.bundle_id,
                heading_sector=(
                    observation.navigation_heading_sector
                    or observation.heading_sector
                ),
                objects=observation.object_labels,
                rgbd_frame_id=observation.rgbd_frame_id,
                pose=spatial_pose,
                observed_displacement_m=(
                    motion_evidence.wheel_delta_xy_m
                    if motion_evidence is not None else None
                ),
                timestamp=observation.timestamp,
                target_candidate=observation.target_present,
            )
            state["place_id"] = place_id
            state["created_place"] = created
            update_result = None
            if semantic_map is not None:
                update_result = semantic_map.update_with_associations(
                    localized,
                    place_id=place_id,
                    now=observation.timestamp,
                    frame_id=observation.rgbd_frame_id,
                )
            if bev_mapper is not None and rgbd_frame is not None and spatial_pose is not None:
                bev_mapper.update(rgbd_frame, spatial_pose)
            state["localized"] = localized

            # Identity bridge (plan §5): frame_object_id -> entity association
            # -> persistent_object_id.  The WebUI object events and the object
            # relation graph must NEVER use label-based identity, so that two
            # chairs / bins keep distinct persistent ids.
            association_by_index: dict[int, Any] = {}
            persistent_by_source_id: dict[str, str] = {}
            if update_result is not None:
                association_by_index = {
                    assoc.observation_index: assoc
                    for assoc in update_result.associations
                }
                persistent_by_source_id = {
                    str(assoc.source_object_id): assoc
                    for assoc in update_result.associations
                    if assoc.source_object_id
                }

            # Phase 3: publish spatial events for the WebUI, including the real
            # map_xyz on every localized object.
            if camera_provider is not None and observation.rgbd_frame_id:
                node._write({
                    "event": "rgbd_frame_updated",
                    "frame_id": observation.rgbd_frame_id,
                    "depth_ref": observation.depth_ref,
                    "intrinsics": observation.intrinsics,
                    "depth_scale": observation.depth_scale,
                    "spatial_quality": getattr(camera_provider, "quality", lambda: "CAMERA_LOCAL")(),
                    "host_s": node._host_s(),
                })
            node._write({
                "event": "spatial_pose_updated",
                "pose": spatial_pose.to_dict() if spatial_pose is not None else None,
                "quality": (
                    getattr(camera_provider, "quality", lambda: "CAMERA_LOCAL")()
                    if camera_provider is not None else "CAMERA_LOCAL"
                ),
                "host_s": node._host_s(),
            })
            for index, obs_item in enumerate(localized):
                if getattr(obs_item, "map_xyz", None) is None:
                    continue
                assoc = association_by_index.get(index)
                if assoc is None:
                    # No reliable persistent identity this frame; do not guess
                    # from the label (two same-label objects must stay distinct).
                    continue
                entity = None
                if semantic_map is not None:
                    entity = semantic_map.objects.get(assoc.persistent_object_id)
                event_object = {}
                if entity is not None:
                    event_object = entity.to_dict()
                event_object.update({
                    "object_id": assoc.persistent_object_id,
                    "label": obs_item.label,
                    "camera_xyz": list(obs_item.camera_xyz) if obs_item.camera_xyz else None,
                    "map_xyz": list(obs_item.map_xyz),
                    "spatial_quality": obs_item.spatial_quality,
                    "source_object_id": assoc.source_object_id,
                    "association_score": assoc.association_score,
                    "association_action": assoc.action,
                })
                node._write({
                    "event": "semantic_object_localized",
                    "object": event_object,
                    "host_s": node._host_s(),
                })
            node._write({
                "event": "place_updated",
                "place": place_graph.places[place_id].to_dict(),
                "host_s": node._host_s(),
            })
            if update_result is not None and entity_graph is not None:
                entity_graph.sync_from_observation(
                    observation_id=observation.bundle_id,
                    heading_sector=(
                        observation.navigation_heading_sector
                        or observation.heading_sector
                    ),
                    labels=observation.object_labels,
                    spatial_objects=localized,
                    pose=spatial_pose,
                    timestamp=observation.timestamp,
                    place_id=place_id,
                    update_result=update_result,
                    relations=list(getattr(semantic, "relations", None) or []),
                )
            # One motion action is consumed by exactly one post-motion
            # observation.  Leaving it pending would double-count a later
            # stationary frame as a relocation.
            motion_state["pending_motion_evidence"] = None

        profiler.record("blocking_decision_ms", "quick_start")
        profiler.record("cycle_total_ms", "capture_start")
        state["last_success_ts"] = time.time()
        node._write({
            "event": "latency_profile",
            "planning_cycle": profiler.profile.planning_cycle,
            "frame_id": profiler.profile.frame_id,
            "timings_ms": profiler.profile.timings_ms,
            "api_calls": profiler.profile.api_calls,
            "counters": profiler.profile.counters,
            "host_s": node._host_s(),
        })
        return observation

    # ---- matcher -------------------------------------------------------------
    def matcher(observation: LiveObservation) -> SemanticMatch:
        semantic = state.get("semantic")
        scene_graph = getattr(semantic, "scene_graph", None)
        context = SearchReasoningContext(
            target_profile=controller.target_profile,
            scene_graph=scene_graph,
            negative_memory=semantic_memory,
            robot_pose=observation.pose,
            robot_yaw_deg=float((observation.pose or {}).get("yaw_deg", 0.0)),
            observed_heading_sectors=sorted(observed_sectors),
        )
        directive = controller.propose(context)
        graph_match = context.graph_match
        anchor_labels: list[str] = []
        if graph_match is not None:
            anchor_ids = set(graph_match.supporting_anchor_scene_node_ids)
            nodes = (
                scene_graph.nodes if scene_graph is not None
                and hasattr(scene_graph, "nodes") else []
            )
            for node in nodes:
                node_id = getattr(node, "node_id", None)
                label = getattr(node, "label_zh", None) or getattr(node, "label", None)
                if node_id in anchor_ids and label:
                    anchor_labels.append(str(label))
        match = SemanticMatch(
            has_candidate=bool(observation.target_present),
            graph_match=graph_match,
            directive=directive,
            target_profile=controller.target_profile,
            anchor_labels=anchor_labels,
            target_score=graph_match.score if graph_match is not None else 0.0,
            target_match_level=(
                graph_match.state.value if graph_match is not None else "none"
            ),
            target_state=observation.target_state,
            provenance={"source": "semantic_navigation_matcher"},
        )
        state["match"] = match
        return match

    # ---- verifier -------------------------------------------------------------
    # 同一 frame_id + bbox + target 最多调一次 Verify API；跨 fresh frame 可重试。
    verify_cache = VerificationCache()

    def verifier(observation: LiveObservation,
                 match: SemanticMatch) -> VerificationOutcome:
        image_path = observation.image_ref
        if not isinstance(image_path, str):
            return VerificationOutcome(confirmed=False, attempts=1,
                                       reason_zh="no image for verification")
        best = max(observation.detections, key=lambda item: item["score"],
                   default=None)
        if best is None:
            return VerificationOutcome(confirmed=False, attempts=1,
                                       reason_zh="no detection to verify")
        key = verify_cache.make_key(
            observation.bundle_id, args.target, list(best["bbox_2d"])
        )
        cached = verify_cache.get(key)
        if cached is not None:
            return VerificationOutcome(
                confirmed=cached.confirmed,
                attempts=1,
                reason_zh=(cached.reason or "cached same-frame verification"),
                details=dict(cached.details or {}),
            )
        result = node._verify_target(image_path, list(best["bbox_2d"]), env)
        confirmed = bool(result.get("is_target", False)) and float(
            result.get("confidence", 0.0)
        ) >= args.verify_min_confidence
        outcome = VerificationOutcome(
            confirmed=confirmed,
            attempts=1,
            reason_zh=str(result.get("reason_zh", "")),
            details=result,
        )
        verify_cache.put(
            key,
            VerificationCacheEntry(
                confirmed=confirmed,
                confidence=float(result.get("confidence", 0.0)),
                reason=str(result.get("reason_zh", "")),
                details=result,
            ),
        )
        return outcome

    # ---- SemanticNavigation V2 spatial candidate generator / planner ------------------
    def _scan_info_marker(place) -> tuple:
        """计划书 §7.4“新信息”定义：新 navigation sector / 新 object / 新 label。"""
        return (
            sorted(observed_sectors),
            len(semantic_map.objects) if semantic_map is not None else 0,
            sorted(place.observed_object_labels) if place is not None else [],
        )

    def _place_scan_has_new_info(place_id: str, place) -> bool:
        marker_key = f"scan_info_marker_{place_id}"
        previous = state.get(marker_key)
        current = _scan_info_marker(place)
        state[marker_key] = current
        return previous is not None and current != previous

    def spatial_candidate_generator(**kwargs: Any) -> list[Any]:
        """V2 candidate generator: selects a long-term spatial intent and
        returns the next local primitive for the current intent."""
        observation = kwargs.get("observation")
        capabilities = kwargs.get("capabilities")
        current_yaw_deg = float(kwargs.get("current_yaw_deg") or 0.0)

        # Continue an active local intent if it still has primitives.
        if local_executor is not None and local_executor.active:
            goal = local_executor.next_goal(
                current_yaw_deg=current_yaw_deg, capabilities=capabilities
            )
            if goal is not None:
                return [goal]
            local_executor.finish()

        if place_graph is None or spatial_reasoner is None:
            return []

        match = state.get("match")
        graph_match = getattr(match, "graph_match", None) if match is not None else None
        match_state = (
            graph_match.state.value if graph_match is not None else "zero_match"
        )

        # ---- bounded LOCAL_SCAN (计划书 §7 / §57-§58) ----------------------
        # 配额语义：同一个 Place 最多执行 max_local_rotations 次 LOCAL_SCAN
        # 旋转动作（local_scan_steps），与 heading coverage 覆盖数完全解耦。
        if match_state in {"zero_match", "partial_match"} and place_graph is not None:
            current_place = place_graph.current_place()
            if current_place is not None:
                max_local_rotations = max(0, int(args.max_local_rotations))
                scan_states = state.setdefault("local_scan_states", {})
                scan_state = LocalScanState.from_dict(
                    scan_states.get(current_place.place_id)
                )
                has_new_info = _place_scan_has_new_info(
                    current_place.place_id, current_place
                )
                goal_fields, next_state = select_local_scan_goal(
                    current_yaw_deg=current_yaw_deg,
                    heading_coverage=current_place.heading_coverage,
                    max_local_rotations=max_local_rotations,
                    state=scan_state,
                    sector_deg=30.0,
                    sectors=12,
                    new_information=has_new_info,
                )
                scan_states[current_place.place_id] = next_state.to_dict()
                state["local_scan_states"] = scan_states
                if goal_fields is None:
                    if next_state.steps >= max_local_rotations > 0:
                        node._write({
                            "event": "local_scan_quota_exhausted",
                            "place_id": current_place.place_id,
                            "steps": next_state.steps,
                            "max_local_rotations": max_local_rotations,
                            "host_s": node._host_s(),
                        })
                    # 无合法候选/配额耗尽：落到下方长期目标规划分支。
                else:
                    node._write({
                        "event": "local_scan_candidate",
                        "place_id": current_place.place_id,
                        "candidates": [
                            {"delta_sector": delta, "sector": (int(round(current_yaw_deg / 30.0)) + delta) % 12}
                            for delta in (1, -1, 2, -2)
                        ],
                        "host_s": node._host_s(),
                    })
                    goal = ExplorationGoal(
                        **{**goal_fields,
                           "goal_id": f"local_scan_{next_state.steps:03d}",
                           "provenance": {
                               **goal_fields["provenance"],
                               "place_id": current_place.place_id,
                           }},
                    )
                    node._write({
                        "event": "local_scan_selected",
                        "goal_id": goal.goal_id,
                        "relative_dyaw": goal.relative_dyaw,
                        "heading_sector": goal.heading_sector,
                        "steps": next_state.steps,
                        "max_local_rotations": max_local_rotations,
                        "same_direction_count": next_state.same_direction_count,
                        "host_s": node._host_s(),
                    })
                    return [goal]

        # Metric map first (plain_slam / RTAB-Map), then BEV fallback,
        # otherwise relative frontier.
        frontiers: list[Any] = []
        if camera_provider is not None and hasattr(camera_provider, "get_map"):
            spin = getattr(camera_provider, "spin_once", None)
            if spin is not None:
                spin()
            map_snap = camera_provider.get_map()
            if map_snap is not None and map_snap.revision > 0:
                pose = camera_provider.get_pose()
                frontiers = frontier_extractor.extract(map_snap, pose) if frontier_extractor else []
        if not frontiers and bev_mapper is not None:
            map_snap = bev_mapper.get_map()
            if map_snap is not None and map_snap.revision > 0:
                pose = camera_provider.get_pose() if camera_provider is not None else None
                frontiers = frontier_extractor.extract(map_snap, pose) if frontier_extractor else []
        if not frontiers and camera_provider is not None:
            frontiers = camera_provider.get_frontiers()

        # Acceptance diagnostics (plan §28): log which metric map fed the
        # frontier extraction so the data chain is auditable.
        metric_map = None
        if camera_provider is not None and hasattr(camera_provider, "get_map"):
            metric_map = camera_provider.get_map()
        if metric_map is not None and metric_map.revision > 0:
            node.get_logger().info(
                "map source = %s, map revision = %d, "
                "pose source = %s, frontier count = %d",
                metric_map.source,
                metric_map.revision,
                getattr(camera_provider.get_pose(), "source", "none"),
                len(frontiers),
            )
            node.get_logger().info(
                "route planner consumes %s (revision %d)",
                metric_map.source, metric_map.revision,
            )
        elif camera_provider is not None:
            node.get_logger().warn(
                "metric map not fresh; falling back (provider=%s, frontier count=%d)",
                args.spatial_provider,
                len(frontiers),
            )

        semantic = state.get("semantic")
        scene_graph = getattr(semantic, "scene_graph", None) if semantic is not None else None
        psg_prior = (
            psg_provider.predict(
                controller.goal_graph,
                scene_graph,
                camera_provider,
                semantic_map,
            )
            if psg_provider is not None else None
        )
        frontier_memory = (
            {key: value.to_dict() for key, value in spatial_memory.frontiers.items()}
            if spatial_memory is not None else {}
        )
        # Build route costs for every frontier through the shared route
        # planner so the LongTermGoalSelector sees real path cost / reachability.
        route_costs: dict[str, dict[str, Any]] = {}
        if route_planner is not None and camera_provider is not None:
            rt_pose = camera_provider.get_pose() or state.get("spatial_pose")
            for f in frontiers:
                rp = route_planner.plan(
                    start_pose=rt_pose if rt_pose is not None else None,
                    target_type="FRONTIER_CANDIDATE",
                    target_id=f.frontier_id,
                    target_position=f.position,
                    map_snapshot=(
                        camera_provider.get_map()
                        if hasattr(camera_provider, "get_map") else None
                    ),
                    place_graph=place_graph,
                    object_map=semantic_map,
                    frame_id="map",
                )
                if rp is not None:
                    route_plan_dict = rp.to_dict()
                    route_costs[f.frontier_id] = route_plan_dict
                    if entity_graph is not None:
                        entity_graph.set_route_plan(route_plan_dict)

        # Memory context: observation memory + in-session frontier memory.
        memory_context: dict[str, Any] = {"frontier_priors": {}, "observation_priors": {}}
        for fid, fm in frontier_memory.items():
            memory_context["frontier_priors"][fid] = float(
                fm.get("semantic_prior", fm.get("semantic_score", 0.0)) or 0.0
            )
        if semantic_memory is not None:
            long_term = semantic_memory.retrieve_long_term(args.target, top_k=5)
            for item in long_term:
                # Observation memory does not know frontier ids; expose a
                # conservative place-level prior for the current Place.
                memory_context["observation_priors"] = {
                    fid: min(1.0, float(item.get("target_related", False) or 0.0) * 0.5)
                    for fid in (f.frontier_id for f in frontiers)
                }

        # Structured common-sense prior; never allowed to confirm targets or
        # command motion.  Refresh in background (non-blocking) with a TTL so
        # it does not sit on the motion critical path.
        common_sense: dict[str, Any] = state.get("common_sense") or {}
        prior_ttl = 30.0
        now_ts = time.time()
        if (
            settings.llm_commonsense_prior_enabled
            and (
                not common_sense
                or (now_ts - float(state.get("common_sense_updated_ts", 0.0))) >= prior_ttl
            )
            and not bool(state.get("common_sense_pending"))
        ):
            state["common_sense_pending"] = True
            _semantic_summary = (
                semantic.get("scene_summary", "")
                if isinstance(semantic, dict)
                else str(getattr(semantic, "scene_summary", "") or "")
            )
            prior_input = LLMPriorInput(
                target=args.target,
                scene_summary=str(_semantic_summary),
                observed_objects=(
                    observation.scene_objects if observation is not None else []
                ),
                observed_relations=(
                    observation.scene_relations if observation is not None else []
                ),
                robot_capabilities={
                    "motion_primitives": list(getattr(capabilities, "allowed_motion_primitives", []))
                },
            )

            def _refresh_prior() -> None:
                try:
                    new_prior = llm_prior_generator.generate(prior_input)
                    state["common_sense"] = new_prior
                    state["common_sense_updated_ts"] = time.time()
                except Exception as exc:  # noqa: BLE001 - prior failure is non-fatal
                    state["common_sense_error"] = f"{type(exc).__name__}: {exc}"
                finally:
                    state["common_sense_pending"] = False

            threading.Thread(target=_refresh_prior, daemon=True).start()

        scored = spatial_reasoner.propose(
            graph_match=graph_match,
            frontiers=frontiers,
            place_graph=place_graph,
            semantic_map=semantic_map,
            psg_prior=psg_prior,
            frontier_memory=frontier_memory,
            route_costs=route_costs,
            semantic_relevance={f.frontier_id: 0.0 for f in frontiers},
            current_yaw_deg=current_yaw_deg,
            memory_context=memory_context,
            common_sense=common_sense,
        )
        if scored is None:
            return []
        intent = scored.intent
        if intent.target_frontier_id and spatial_memory is not None:
            spatial_memory.mark_frontier_selected(intent.target_frontier_id)
        goal = None
        route = None
        current_place = place_graph.current_place()
        if intent.target_frontier_id and current_place is not None:
            from app.spatial.topological_frontier import TopologicalFrontier

            current_place_id = current_place.place_id
            tf = next(
                (
                    TopologicalFrontier(
                        frontier_id=f.frontier_id,
                        parent_place_id=current_place_id,
                        bearing_deg=float(f.bearing_deg or 0.0),
                        local_distance_hint_m=f.distance_m,
                        information_gain=float(f.spatial_information_gain or 0.0),
                    )
                    for f in frontiers
                    if f.frontier_id == intent.target_frontier_id
                ),
                None,
            )
            if tf is not None:
                route = TopologyRoutePlanner().plan(
                    place_graph=place_graph,
                    current_place_id=current_place_id,
                    frontier=tf,
                )
            if route is not None:
                goal = TopologyRouteExecutor(
                    forward_step_m=float(
                        getattr(node, "_experiment_forward_segment_m", 0.30)
                    )
                ).next_goal(
                    route=route,
                    current_place_id=current_place_id,
                    place_graph=place_graph,
                    current_yaw_deg=current_yaw_deg,
                    capabilities=capabilities,
                    frontier=tf,
                )
        if goal is None:
            local_executor.begin(intent)
            goal = local_executor.next_goal(
                current_yaw_deg=current_yaw_deg, capabilities=capabilities
            )
            if goal is None:
                local_executor.finish()
                return []
        goal.semantic_relevance = max(0.0, min(1.0, scored.score))
        goal.expected_information_gain = intent.spatial_gain
        goal.provenance = {
            **goal.provenance,
            "intent": intent.to_dict(),
            "scored_intent": scored.to_dict(),
            "route": route.to_dict() if route is not None else None,
            "route_plan": route_costs.get(intent.target_frontier_id)
            if intent.target_frontier_id else None,
        }
        # Publish real long-term goal selection + decision record.
        if entity_graph is not None:
            selected_route = route_costs.get(intent.target_frontier_id)
            if selected_route:
                entity_graph.set_route_plan(selected_route)
        _publish_decision_record(
            cycle=state.get("cycle", 0),
            match_state=match_state,
            scored=scored,
            goal=goal,
            intent=intent,
            observation=observation,
            route_plan=route_costs.get(intent.target_frontier_id),
        )
        return [goal]

    def _publish_decision_record(
        *,
        cycle: int,
        match_state: str,
        scored: Any,
        goal: Any,
        intent: Any,
        observation: Any,
        route_plan: dict[str, Any] | None,
    ) -> None:
        """Build and emit a real structured DecisionRecord (plan §10)."""
        from app.navigation.decision_record import (
            alternative_from_candidate,
            build_decision_record,
        )

        breakdown = dict(scored.components or {})
        matched_normalized = {
            "ZERO": "ZERO", "PARTIAL": "PARTIAL", "STRONG": "STRONG",
            "VERIFY": "VERIFY",
        }.get(str(match_state).upper(), str(match_state).upper())
        record = build_decision_record(
            cycle=int(cycle),
            match_state=matched_normalized,
            selected_intent=intent.to_dict(),
            selected_goal=goal.to_dict(),
            next_motion_command={"instruction_zh": goal.semantic_reason},
            score=float(scored.score),
            score_breakdown=breakdown,
            evidence={
                "anchor_object_ids": [
                    entry.object_id for entry in (semantic_map.objects.values() if semantic_map else [])
                    if getattr(entry, "status", "") == "CONFIRMED"
                ][:3],
                "anchor_labels": getattr(observation, "scene_objects", None)
                and [
                    str(item.get("label_zh") or item.get("label") or "")
                    for item in observation.scene_objects[:3]
                ]
                or [],
                "current_place_id": place_graph.current_place().place_id
                if place_graph and place_graph.current_place() else None,
                "spatial_quality": getattr(camera_provider, "quality", lambda: "CAMERA_LOCAL")(),
                "route_id": (route_plan or {}).get("route_id"),
            },
            alternatives=[],
            map_revision=entity_graph.revision if entity_graph is not None else 0,
            canonical_target=args.target,
            task_text=args.target,
            current_place_id=place_graph.current_place().place_id
            if place_graph and place_graph.current_place() else None,
            session_id=args.session_id or "",
        )
        state["last_decision"] = record.to_dict()
        node._write({
            "event": "long_term_goal_selected",
            "intent": intent.to_dict(),
            "route_plan": route_plan,
            "scored": scored.to_dict(),
            "host_s": node._host_s(),
        })
        node._write({
            "event": "decision_recorded",
            "decision": record.to_dict(),
            "host_s": node._host_s(),
        })
        node._write({
            "event": "local_goal_progress",
            "progress": {
                "goal_id": goal.goal_id,
                "goal_type": goal.goal_type,
                "decision_id": record.decision_id,
            },
            "host_s": node._host_s(),
        })

    def spatial_planner(candidates: list[Any], **kwargs: Any) -> Any:
        """Real explainable planner: score the produced goal from the intent.

        The score and components come from the LongTermGoalSelector's real
        ScoredIntent rather than a hard-coded ``spatial_v2=1.0``.
        """
        from app.navigation.exploration_planner import ScoredGoal

        if not candidates:
            return None
        goal = candidates[0]
        prov = dict(goal.provenance or {})
        scored_intent = prov.get("scored_intent") or {}
        score = float(scored_intent.get("score", 0.0) or 0.0)
        components = dict(scored_intent.get("components", {}) or {})
        reasons = list(scored_intent.get("reasons", []) or [])
        if not reasons:
            reasons = [goal.semantic_reason or "semantic navigation intent"]
        return ScoredGoal(
            goal=goal,
            score=score if score > 0 else 0.5,
            components=components if components else {"semantic_relevance": score or 0.5},
            reasons=reasons,
        )

    # ---- backend ---------------------------------------------------------------
    from app.navigation.go2w_experimental_backend import (
        Go2WBackendConfig,
        Go2WExperimentalBackend,
    )
    step_index = [0]

    def execute_step(step: str) -> tuple[bool, str, dict[str, Any]]:
        if args.dry_run_motion:
            # WebUI dry-run: run the full real perception / reasoning pipeline
            # but never send a motion command (plan book §60).
            return True, "dry_run_motion", {"step": step, "dry_run": True}
        index = step_index[0]
        step_index[0] += 1
        before = node._odom_snapshot()
        ok, reason = node._execute_step(index, step)
        detail = {"step": step, "index": index}
        reason_code = str(reason or "").split(":", 1)[0].strip().upper()
        if reason_code in {"MOTION_ACCEPT_TIMEOUT", "MOTION_RESULT_TIMEOUT"}:
            detail["error_type"] = reason_code
            detail["non_retryable"] = True
        if ok:
            after = node._odom_snapshot()
            wheel_delta_xy = math.hypot(after[0] - before[0], after[1] - before[1])
            requested_dyaw = _requested_dyaw_deg(step)
            motion_state["pending_motion_evidence"] = MotionEvidence(
                command_type=("ROTATE" if abs(requested_dyaw) > 0.0 else "TRANSLATE"),
                requested_turn_deg=requested_dyaw,
                requested_forward_m=_requested_forward_m(step),
                wheel_delta_xy_m=wheel_delta_xy,
                wheel_delta_yaw_deg=math.degrees(_wrap_pi(after[2] - before[2])),
                motion_completed_at=time.time(),
            )
            motion_state["motion_end_timestamp"] = time.time()
            motion_state["last_frame_id_before_motion"] = str(
                state.get("frame_id") or ""
            )
            motion_state["last_yaw_before"] = float(before[2])
            # 计划书 §7.5：用运动后的真实 odom 计算 reached sector，绝不拿
            # “计划想去的 sector”冒充“实际到达 sector”。
            sectors = max(1, int(policy.candidates.heading_sectors))
            observed_dyaw = math.degrees(_wrap_pi(after[2] - before[2]))
            sector_before = _navigation_sector(yaw_rad=before[2], sectors=sectors)
            sector_reached = _navigation_sector(yaw_rad=after[2], sectors=sectors)
            current_place = (
                place_graph.current_place() if place_graph is not None else None
            )
            node._write({
                "event": "motion_sector_reached",
                "step": step,
                "requested_dyaw_deg": round(requested_dyaw, 3),
                "observed_dyaw_deg": round(observed_dyaw, 3),
                "wheel_delta_xy_m": round(wheel_delta_xy, 4),
                "sector_before": sector_before,
                "sector_requested": _navigation_sector(
                    yaw_rad=before[2] + math.radians(requested_dyaw),
                    sectors=sectors,
                ),
                "sector_reached": sector_reached,
                "coverage_before": (
                    dict(current_place.heading_coverage or {})
                    if current_place is not None else {}
                ),
                "coverage_after": "next_observation",
                "host_s": node._host_s(),
            })
        return ok, reason, detail

    def stop() -> None:
        node._emergency_stop()

    def cancel() -> None:
        node._emergency_stop()

    def health_probe() -> dict[str, Any]:
        try:
            motion_ready = bool(node._client.server_is_ready())
        except Exception:
            motion_ready = False
        try:
            arm_ready = bool(node._arm_client.service_is_ready())
            stop_ready = bool(node._stop_srv.service_is_ready())
        except Exception:
            arm_ready = stop_ready = False
        mode_ok = bool(
            node._sport is not None
            and int(node._sport.mode) == 1
            and int(node._sport.error_code) == 0
        )
        action_server_count, action_graph_detail = _motion_action_server_count()
        action_process_count, action_process_pids = (
            _motion_action_server_process_count()
        )
        odom_publisher_count, odom_graph_detail = _topic_publisher_count(
            args.odom_topic
        )
        odom_process_count, odom_process_pids = _wheel_odom_process_count()
        return {
            "motion_action_available": motion_ready,
            "motion_action_server_count": action_server_count,
            "motion_action_graph_detail": action_graph_detail,
            "motion_action_server_process_count": action_process_count,
            "motion_action_server_pids": action_process_pids,
            "odom_topic": args.odom_topic,
            "odom_publisher_count": odom_publisher_count,
            "odom_graph_detail": odom_graph_detail,
            "wheel_odom_process_count": odom_process_count,
            "wheel_odom_process_pids": odom_process_pids,
            "arm_service_available": arm_ready,
            "emergency_stop_service_available": stop_ready,
            "robot_mode_error": not mode_ok,
            "lidar_fresh": node._lidar_fresh is True,
            "operator_authorized_rotation": operator_authorized,
            "experiment_profile": getattr(
                node, "_experiment_profile_name", "operator_supervised_experiment"
            ),
            "allowed_motion_primitives": [
                name for name, enabled in getattr(
                    node, "_experiment_motion_primitives", {}
                ).items() if enabled
            ],
        }

    backend = Go2WExperimentalBackend(
        execute_step=execute_step,
        odometry=node._odom_snapshot,
        stop=stop,
        cancel=cancel,
        health_probe=health_probe,
        config=Go2WBackendConfig(
            dry_run=bool(args.dry_run_motion),
            max_turn_deg_per_action=getattr(
                node, "_experiment_max_turn_deg", args.max_turn_deg
            ),
            forward_step_m=getattr(
                node, "_experiment_forward_segment_m", 0.30
            ),
            max_forward_step_m=max(
                0.01,
                float(getattr(
                    node, "_experiment_forward_step_m", args.forward_step_m
                )),
            ),
        ),
    )

    if args.record_video:
        try:
            from run_autonomous_loop import BundleVideoRecorder
            node._video = BundleVideoRecorder(args.record_video, 15.0, 0.4)
            node._video.start()
            node.get_logger().info(f"recording camera to {args.record_video}")
        except RuntimeError as exc:
            node.get_logger().warn(f"video recording disabled: {exc}")

    # ---- explorer ---------------------------------------------------------------
    from app.task_understanding.search_task_context import SearchTaskContext
    task_context = _task_context_from_args(args)
    graph = ExplorationGraph(session_id=args.session_id or time.strftime("explore_go2w_%Y%m%d_%H%M%S"))
    events: list[dict[str, Any]] = []
    holder: dict[str, Any] = {"explorer": None}

    def on_event(event: dict[str, Any]) -> None:
        if event_hook is not None:
            explorer = holder.get("explorer")
            if explorer is not None and event.get("event") in _MAP_RELEVANT_EVENTS:
                event = {**event, "graph": explorer.graph.to_dict()}
            event = event_hook(event, holder)
        if event is None:
            return
        events.append(event)
        node._write(event)

    explorer_kwargs: dict[str, Any] = {}
    if args.spatial_v2:
        explorer_kwargs["candidate_generator"] = spatial_candidate_generator
        explorer_kwargs["planner"] = spatial_planner
    explorer = AutonomousExplorer(
        target=task_context.canonical_target,
        task_context=task_context,
        observer=observe,
        matcher=matcher,
        verifier=verifier,
        backend=backend,
        policy=policy,
        graph=graph,
        negative_memory=semantic_memory,
        negative_target_key=profile.canonical_name_zh,
        on_event=on_event,
        finish_on_visual_confirmation=args.finish_on_visual_confirmation,
        turn_only=bool(args.turn_only),
        session_id=graph.session_id,
        executor_id=args.executor_id or None,
        worker_generation=args.worker_generation,
        **explorer_kwargs,
    )
    holder["explorer"] = explorer
    node.get_logger().info(
        f"starting semantic exploration target={args.target} "
        f"session={graph.session_id}"
    )
    result = explorer.run()
    if node._armed_by_runner:
        node._emergency_stop()
        try:
            node._arm(False)
        except RuntimeError as exc:
            node.get_logger().error(str(exc))
    if camera_provider is not None:
        close = getattr(camera_provider, "close", None)
        if close is not None:
            close()
    _write_session_artifacts(args, explorer, result, events)
    if place_graph is not None:
        run_dir = Path(args.session_dir) / explorer.session_id
        run_dir.mkdir(parents=True, exist_ok=True)
        (run_dir / "place_graph.json").write_text(
            json.dumps(place_graph.to_dict(), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        if semantic_map is not None:
            (run_dir / "semantic_map.json").write_text(
                json.dumps(semantic_map.to_dict(), ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
        if spatial_memory is not None:
            (run_dir / "spatial_memory.json").write_text(
                json.dumps(spatial_memory.to_dict(), ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
    return 0 if result.result == "TARGET_FOUND" else 3


def _task_context_from_args(args: argparse.Namespace):
    from app.task_understanding.search_task_context import SearchTaskContext

    if getattr(args, "task_context_json", ""):
        try:
            value = json.loads(args.task_context_json)
            if isinstance(value, dict):
                return SearchTaskContext.from_dict(value)
        except (TypeError, ValueError, json.JSONDecodeError):
            pass
    return SearchTaskContext.mock_fallback(args.target)


def _probe_readiness(args, node):
    """Automatic health probe for the experiment profile (plan section 17)."""
    from app.live_robot.frame_bundle_reader import FrameBundleReader
    bundle_ok = False
    camera_ok = False
    try:
        reader = FrameBundleReader(args.spool_root)
        bundle = reader.read_latest(timeout_seconds=2.0)
        bundle_ok = bundle is not None
        camera_ok = bool((bundle.payload.get("sensor_health") or {}).get("camera"))
    except Exception:
        pass
    try:
        motion_ready = bool(node._client.server_is_ready())
    except Exception:
        motion_ready = False
    try:
        arm_ready = bool(node._arm_client.service_is_ready())
        stop_ready = bool(node._stop_srv.service_is_ready())
    except Exception:
        arm_ready = stop_ready = False
    mode_ok = bool(
        node._sport is not None
        and int(node._sport.mode) == 1
        and int(node._sport.error_code) == 0
    )
    pose_fresh = bool(node._odom is not None)
    from app.navigation.robot_backend import RobotCapabilities

    capabilities = RobotCapabilities(
        supports_relative_translation=True,
        supports_relative_rotation=True,
        supports_heading_control=True,
        supports_navigation_cancel=True,
        supports_navigation_feedback=True,
        allowed_motion_primitives=("FORWARD", "ROTATE_LEFT", "ROTATE_RIGHT"),
    )
    # The production path uses the long-running VLM daemon.  In that mode the
    # ROS worker intentionally does not need to inherit the provider key: the
    # daemon owns the API client and may have loaded the key from .env before
    # it was started.  Treat a live daemon socket as an available LLM runtime;
    # otherwise retain the legacy direct-key check.
    llm_daemon_available = False
    try:
        from app.detectors.siliconflow_vision_protocol import SiliconFlowDaemonClient

        llm_daemon_available = SiliconFlowDaemonClient(
            str(PROJECT_ROOT / "runtime/go2w/siliconflow_vlm.sock")
        ).available()
    except Exception:
        llm_daemon_available = False
    llm_key_configured = bool(os.getenv("SILICONFLOW_API_KEY"))
    llm_runtime_available = llm_key_configured or llm_daemon_available
    return compute_experiment_readiness(
        camera_fresh=camera_ok,
        bundle_fresh=bundle_ok,
        llm_available=llm_runtime_available,
        motion_action_available=motion_ready,
        robot_mode_ok=mode_ok,
        emergency_stop_available=stop_ready,
        backend_healthy=True,
        pose_freshness_if_available=pose_fresh,
        capabilities=capabilities,
        check_llm_key=not llm_daemon_available,
    )


if __name__ == "__main__":
    raise SystemExit(main())
