#!/usr/bin/env python3
"""Offline, no-API A/B replay for legacy, SemanticNavigation and hybrid policies.

The evaluator separates three things that must not be conflated:

* recorded-session evidence and costs;
* each backend's proposed directive and adapted StepPlan;
* the step shadow mode would actually execute (always the legacy step).
"""

from __future__ import annotations

import argparse
from dataclasses import fields
import json
import math
from pathlib import Path
import sys
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from app.live_robot.search_directive_adapter import directive_to_step_plan
from app.live_robot.step_planner import plan_scan_step
from app.live_robot.step_search_runner import StepSearchConfig
from app.reasoning.semantic_navigation.models import SearchReasoningContext
from app.reasoning.semantic_navigation.router import SemanticSearchController
from app.reasoning.semantic_navigation.semantic_memory import SemanticSearchMemory
from app.video.schemas import SceneGraph, SceneGraphEdge, SceneGraphNode
from app.reasoning.target_profile import TargetProfile, TargetProfileResolver


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True, help="session dir or scene_graph.json")
    parser.add_argument("--target", required=True)
    parser.add_argument("--output-dir", default="outputs/reasoner_evaluation")
    parser.add_argument("--scan-index", type=int, default=0)
    parser.add_argument(
        "--event-log",
        default="",
        help="optional runner JSONL used for executed-step metrics",
    )
    return parser.parse_args(argv)


def _session_root(path: Path) -> Path:
    return path if path.is_dir() else path.parent


def _load_graph(path: Path) -> SceneGraph:
    graph_path = path / "scene_graph.json" if path.is_dir() else path
    payload = json.loads(graph_path.read_text(encoding="utf-8"))
    return SceneGraph(
        nodes=[SceneGraphNode(**item) for item in payload.get("nodes", [])],
        edges=[SceneGraphEdge(**item) for item in payload.get("edges", [])],
    )


def _load_profile(root: Path, target: str) -> TargetProfile:
    """Reuse the session's exact task profile; never make a replay API call."""

    payload = _read_json(root / "target_profile.json", {})
    required = {"raw_query", "canonical_name_zh"}
    if not required.issubset(payload):
        return TargetProfileResolver().resolve(target, use_llm=False)
    kwargs = {
        item.name: payload[item.name]
        for item in fields(TargetProfile)
        if item.name in payload and item.name != "grounding_prompt_plan"
    }
    # The persisted grounding plan is auxiliary and may use a separate
    # dataclass schema. GoalGraph uses the explicit profile fields above.
    kwargs["grounding_prompt_plan"] = None
    return TargetProfile(**kwargs)


def _read_json(path: Path, default: Any) -> Any:
    if not path.is_file():
        return default
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return default


def _read_jsonl(path: Path | None) -> list[dict[str, Any]]:
    if path is None or not path.is_file():
        return []
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            payload = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(payload, dict):
            rows.append(payload)
    return rows


def _turn_degrees(step: str) -> float:
    if not isinstance(step, str) or not step.startswith(("l", "r")):
        return 0.0
    try:
        return abs(float(step[1:]))
    except ValueError:
        return 0.0


def _recorded_metrics(root: Path, event_log: Path | None) -> dict[str, Any]:
    observations = _read_json(root / "frame_observations.json", [])
    target_search = _read_json(root / "target_search.json", {})
    gate = _read_json(root / "evidence_gating_report.json", {})
    crop_report = _read_json(root / "crop_verify_results.json", {})
    profile = _read_json(root / "target_profile.json", {})
    directive = _read_json(root / "search_directive.json", {})
    runner_rows = _read_jsonl(event_log)

    timeline = list(target_search.get("timeline") or [])
    candidate_times = [
        float(item["timestamp_sec"])
        for item in timeline
        if item.get("type") == "direct_detection"
        and isinstance(item.get("timestamp_sec"), (int, float))
    ]
    gate_candidates = list(gate.get("candidates") or [])
    verify_reject_count = sum(
        bool(item.get("blocking_rules")) for item in gate_candidates
    )

    verified_steps = [
        row for row in runner_rows if row.get("event") == "step_verified"
    ]
    started_steps = [
        row for row in runner_rows
        if row.get("event") in {"step_start", "motion_start"}
    ]
    # Prefer verified steps so retries and rejected attempts are not counted as
    # physical progress. Observe-only sessions correctly stay at zero.
    physical_steps = verified_steps or started_steps
    total_turn_deg = sum(_turn_degrees(str(row.get("step", ""))) for row in physical_steps)
    distance_values = [
        float(row["distance_m"])
        for row in verified_steps
        if isinstance(row.get("distance_m"), (int, float))
        and math.isfinite(float(row["distance_m"]))
    ]

    detector_calls = len(observations) if isinstance(observations, list) else 0
    crop_calls = int(crop_report.get("attempted") or 0)
    profile_calls = 1 if profile.get("resolver_source") == "llm" else 0
    detector_name = str((target_search.get("task") or {}).get("detector") or "")
    inferred_recorded_llm_calls = (
        (detector_calls if detector_name == "llm" else 0)
        + crop_calls
        + profile_calls
    )
    semantic_observer_calls = 1 if directive else 0
    target_found = bool(gate.get("target_found"))
    best_evidence = target_search.get("best_evidence") or {}
    target_time = (
        float(best_evidence["timestamp_sec"])
        if target_found
        and isinstance(best_evidence.get("timestamp_sec"), (int, float))
        else None
    )

    return {
        "search_steps": len(physical_steps),
        "total_turn_deg": round(total_turn_deg, 6),
        "estimated_distance_m": round(sum(distance_values), 6),
        "repeat_observation_count": 0 if semantic_observer_calls <= 1 else None,
        "negative_revisit_count": 0 if not physical_steps else None,
        "detector_calls": detector_calls,
        "semantic_observer_calls": semantic_observer_calls,
        "recorded_llm_calls_inferred": inferred_recorded_llm_calls,
        "recorded_llm_call_provenance": {
            "full_scene_detection": detector_calls if detector_name == "llm" else 0,
            "target_profile_resolution": profile_calls,
            "crop_verification": crop_calls,
            "note": "derived from persisted artifacts; no API was called by replay",
        },
        "time_to_candidate_s": min(candidate_times) if candidate_times else None,
        "time_to_target_s": target_time,
        "verify_reject_count": verify_reject_count,
        "success_proxy": bool(best_evidence),
        "success_proxy_definition": "persisted visual best_evidence exists; not target confirmation",
        "target_confirmed": target_found,
        "event_log_rows": len(runner_rows),
    }


def evaluate(
    *,
    session: str,
    target: str,
    output_dir: str,
    scan_index: int = 0,
    event_log: str = "",
) -> dict:
    session_path = Path(session)
    root = _session_root(session_path)
    graph = _load_graph(session_path)
    profile = _load_profile(root, target)
    config = StepSearchConfig(target=target)
    legacy = plan_scan_step(
        scan_index,
        scan_turn_deg=config.scan_turn_deg,
        scan_span=config.scan_span,
        distance_m=0.0,
        max_radius_m=config.max_radius_m,
        forward_estimate_m=config.forward_estimate_m,
    )
    rows = []
    for backend in ("legacy", "semantic_navigation", "hybrid"):
        controller = SemanticSearchController(profile, backend=backend)
        context = SearchReasoningContext(
            scene_graph=graph,
            negative_memory=SemanticSearchMemory(),
            scan_index=scan_index,
            legacy_scan_candidate=legacy,
            safety_context={"allow_forward": False, "offline_replay": True},
        )
        directive = controller.propose(context)
        proposed_plan = directive_to_step_plan(
            directive,
            scan_index=scan_index,
            distance_m=0.0,
            step_config=config,
            min_confidence=config.reasoner_min_confidence,
            allow_forward=False,
            max_turn_deg=config.reasoner_max_turn_deg,
        )
        dangerous_forward_request = bool(
            directive.allow_forward or directive.preferred_distance_m is not None
        )
        rows.append({
            "backend": backend,
            "directive": directive.to_dict(),
            "graph_match": context.graph_match.to_dict() if context.graph_match else None,
            "legacy_step": legacy.step,
            "proposed_step": proposed_plan.step,
            "shadow_executed_step": legacy.step,
            "shadow_matches_legacy": True,
            "disagrees_with_legacy": proposed_plan.step != legacy.step,
            "dangerous_forward_request": dangerous_forward_request,
        })

    event_path = Path(event_log) if event_log else None
    recorded = _recorded_metrics(root, event_path)
    fallback_count = sum(bool(row["directive"]["fallback_to_legacy"]) for row in rows)
    disagreement_count = sum(bool(row["disagrees_with_legacy"]) for row in rows)
    dangerous_forward_count = sum(bool(row["dangerous_forward_request"]) for row in rows)
    report = {
        "schema_version": "2.0",
        "evaluation_mode": "offline_shadow_no_api_no_motion",
        "target": target,
        "source_session": str(session_path),
        "source_event_log": str(event_path) if event_path else None,
        "scene_nodes": len(graph.nodes),
        "scene_edges": len(graph.edges),
        "detector_calls": recorded["detector_calls"],
        "semantic_observer_calls": recorded["semantic_observer_calls"],
        "reasoner_calls": len(rows),
        "llm_calls": 0,
        "recorded_llm_calls_inferred": recorded["recorded_llm_calls_inferred"],
        "search_steps": recorded["search_steps"],
        "total_turn_deg": recorded["total_turn_deg"],
        "estimated_distance": recorded["estimated_distance_m"],
        "repeat_observation_count": recorded["repeat_observation_count"],
        "negative_revisit_count": recorded["negative_revisit_count"],
        "fallback_count": fallback_count,
        "legacy_vs_semantic_disagreement_count": disagreement_count,
        "dangerous_forward_request_count": dangerous_forward_count,
        "time_to_candidate": recorded["time_to_candidate_s"],
        "time_to_target": recorded["time_to_target_s"],
        "verify_reject_count": recorded["verify_reject_count"],
        "success_proxy": recorded["success_proxy"],
        "success_proxy_definition": recorded["success_proxy_definition"],
        "target_confirmed": recorded["target_confirmed"],
        "actual_shadow_behavior_matches_legacy": all(
            row["shadow_matches_legacy"] for row in rows
        ),
        "recorded_metrics": recorded,
        "results": rows,
        "limitations": [
            "offline graph-policy replay only; no claim of real-robot success",
            "success_proxy is candidate evidence, never target confirmation",
            "repeat/revisit metrics require event traces when physical steps exist",
            "recorded LLM calls are inferred from persisted frame/profile/crop artifacts",
        ],
    }
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    (output / "reasoner_comparison.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    (output / "reasoner_metrics.json").write_text(
        json.dumps(recorded, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    shadow_comparison = {
        "schema_version": report["schema_version"],
        "source_session": report["source_session"],
        "legacy_step": legacy.step,
        "actual_shadow_behavior_matches_legacy": report[
            "actual_shadow_behavior_matches_legacy"
        ],
        "dangerous_forward_request_count": dangerous_forward_count,
        "results": rows,
    }
    (output / "shadow_comparison.json").write_text(
        json.dumps(
            shadow_comparison, ensure_ascii=False, indent=2, allow_nan=False
        ) + "\n",
        encoding="utf-8",
    )
    lines = [
        "# Live Search Reasoner Comparison", "",
        f"- target: {target}",
        f"- scene nodes/edges: {len(graph.nodes)}/{len(graph.edges)}",
        f"- recorded detector / inferred LLM calls: "
        f"{recorded['detector_calls']} / {recorded['recorded_llm_calls_inferred']}",
        f"- time to candidate / target: "
        f"{recorded['time_to_candidate_s']} / {recorded['time_to_target_s']}",
        f"- verify rejects: {recorded['verify_reject_count']}",
        f"- semantic disagreements: {disagreement_count}",
        f"- dangerous forward requests: {dangerous_forward_count}",
        "- shadow executed behavior equals legacy: yes", "",
    ]
    for row in rows:
        directive = row["directive"]
        lines.append(
            f"- {row['backend']}: {directive['kind']} -> proposed "
            f"{row['proposed_step']}, shadow executes {row['shadow_executed_step']} / "
            f"confidence={directive['confidence']:.2f} / {directive['reason_zh']}"
        )
    (output / "reasoner_comparison.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )
    return report


def main(argv=None) -> int:
    args = _parse_args(argv)
    report = evaluate(
        session=args.session,
        target=args.target,
        output_dir=args.output_dir,
        scan_index=args.scan_index,
        event_log=args.event_log,
    )
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
