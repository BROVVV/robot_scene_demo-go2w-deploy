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
from app.reasoning.semantic_navigation.task_agent import (
    attach_trace,
    enrich_context,
    retrieve_knowledge,
    summarize_topology,
)
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
    parser.add_argument(
        "--node-index",
        type=int,
        default=-1,
        help="当会话没有 scene_graph.json 时，从 exploration_graph.json 的"
             "第几个可回放观测节点取图（默认 -1 = 最后一个，即搜索结束时"
             "看到的最全的一帧；按时间顺序，0 是最早那次观测）",
    )
    return parser.parse_args(argv)


def _session_root(path: Path) -> Path:
    return path if path.is_dir() else path.parent


def _exploration_view(root: Path, node_index: int) -> tuple[dict[str, Any], float, list[int]]:
    """从真机会话的 exploration_graph.json 取一次观测 —— 连当时的机器人朝向和
    已观测扇区一起取。

    `run_semantic_exploration.py`（§22 验收走的就是它）**不写**独立的
    scene_graph.json —— 每次观测的 SceneGraph 嵌在 exploration_graph.json 的
    `nodes[i].scene_graph` 里，schema 与 SceneGraphNode/SceneGraphEdge 逐字段一致。
    2026-09-16 实测：outputs/live_runs 下 128 个真机会话里有 102 个可这样回放，
    而带独立 scene_graph.json 的是 **0 个**。没有这条回退，本评估器就只能吃
    手工构造的 fixture，吃不到任何一次真机跑批。

    后两个返回值不是装饰：真机调用点 `run_semantic_exploration.py:2318-2322`
    会传 `robot_yaw_deg` 与 `observed_heading_sectors`，而
    `search_reasoner.py:_rank_candidate_headings` 的候选打分里 `visited_cost` 依赖
    后者。回放时若不传（空集），六个候选方向全部平局、退化成恒选 −30 ——
    那会造出一个**看起来像发现**的假结论。node 的 `heading` 就是当时的机器人
    yaw：用 §22 那次核对过，三个观测的 heading −3.48/28.93/−31.83 与
    `heading_sector` 0/1/11 自洽，且正好对上 +30/−60 两次转向。
    """
    path = root / "exploration_graph.json"
    payload = json.loads(path.read_text(encoding="utf-8"))
    observations = [
        node for node in (payload.get("nodes") or [])
        if (node.get("scene_graph") or {}).get("nodes")
    ]
    if not observations:
        raise ValueError(f"{path} 里没有带 scene_graph 的观测节点，无法离线回放")
    try:
        chosen = observations[node_index]
    except IndexError:
        raise ValueError(
            f"--node-index {node_index} 越界：{path} 只有 "
            f"{len(observations)} 个可回放观测节点"
        ) from None
    # 扇区取"到这次观测为止"，不取整场结束后的终值 —— 否则早期观测会被
    # 喂进它当时还看不到的扇区。
    upto = observations[: node_index + 1] if node_index >= 0 else observations
    sectors = sorted({
        int(node["heading_sector"]) for node in upto
        if isinstance(node.get("heading_sector"), int)
    })
    heading = chosen.get("heading")
    yaw = float(heading) if isinstance(heading, (int, float)) else 0.0
    return chosen["scene_graph"], yaw, sectors


def _load_session(
    path: Path, node_index: int = -1
) -> tuple[SceneGraph, float, list[int], str]:
    """返回 (图, robot_yaw_deg, observed_heading_sectors, 图是从哪读的)。"""
    graph_path = path / "scene_graph.json" if path.is_dir() else path
    if graph_path.is_file():
        source = str(graph_path)
        payload = json.loads(graph_path.read_text(encoding="utf-8"))
        # 视频管线只落图，不落当时的朝向与扇区 —— 报 0.0/[] 并在报告里
        # 用 replay_context 挑明，不假装知道。
        yaw, sectors = 0.0, []
    else:
        source = str(_session_root(path) / "exploration_graph.json")
        payload, yaw, sectors = _exploration_view(_session_root(path), node_index)
    try:
        graph = SceneGraph(
            nodes=[SceneGraphNode(**item) for item in payload.get("nodes", [])],
            edges=[SceneGraphEdge(**item) for item in payload.get("edges", [])],
        )
    except TypeError as exc:
        # 实测 outputs/live_runs 的 102 个可回放会话里有 45 个是 mock/早期会话，
        # 内嵌 scene_graph 只写了 node_id/label/label_zh/attributes 四个键，装不下
        # SceneGraphNode。直接抛 TypeError 会看起来像评估器自己的 bug，所以说清
        # 是**这份录制本身**不是真机感知产物。
        raise ValueError(
            f"{source} 的 SceneGraph 字段不全（{exc}）—— 多半是 mock 或早期会话，"
            f"不是真机感知产物，无法回放"
        ) from None
    return graph, yaw, sectors, source


def _scene_graph_source(source: str) -> str:
    """报告里标明这张图是从哪读来的 —— 免得把回退读到的图当成独立工件。"""
    if source.endswith("/scene_graph.json"):
        return "scene_graph.json"
    if source.endswith("/exploration_graph.json"):
        return "exploration_graph.json:nodes[].scene_graph"
    return "unknown"


def _load_task_context(root: Path) -> dict[str, Any] | None:
    """会话里落盘的 `SearchTaskContext`。

    这是**任务输入**，在搜索开始之前就存在，不是录制结束后的产物 ——
    回放把它喂给推理器不构成未来信息泄漏（计划书 §4.2）。没有就返回 None，
    不补一个空壳冒充"任务已解析"。
    """
    payload = _read_json(root / "task.json", None)
    return payload if isinstance(payload, dict) else None


def _load_topology_summary(root: Path) -> dict[str, Any] | None:
    payload = _read_json(root / "place_graph.json", None)
    return summarize_topology(payload)


def _frozen_consumer_accepts(directive, config, graph) -> bool:
    """冻结的 `generate_live_exploration_candidates` 是否真的会采纳这条 directive。

    §4.3 要求同时验证两件事：reasoner 输出改变且可解释，**以及**新 directive
    能通过冻结消费者契约。只做前者不算交付。这里直接调冻结消费者，不复制它的
    判定逻辑（复制出来的那份迟早和它不一致）。
    """
    from app.navigation.candidate_goal_generator import generate_live_exploration_candidates
    from app.navigation.exploration_graph import ExplorationGraph
    from app.navigation.models import LiveObservation

    observation = LiveObservation(
        bundle_id="offline_replay", timestamp=0.0, scene_objects=[],
        scene_graph={"nodes": [], "edges": []},
        target_match={"target_present": False},
        pose={"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
        heading_sector=0, provenance={"source": "offline_replay"},
    )
    # 传空图：只考验 directive 这一条入口，不让未访问扇区/图节点候选把
    # "消费者接受了 directive"这个判断混掉。
    goals = generate_live_exploration_candidates(
        observation=observation,
        graph=graph if isinstance(graph, ExplorationGraph) else ExplorationGraph(session_id="replay"),
        directive=directive,
        current_yaw_deg=0.0,
    )
    return any(
        goal.provenance.get("directive_id") == directive.directive_id for goal in goals
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
    node_index: int = -1,
) -> dict:
    session_path = Path(session)
    root = _session_root(session_path)
    graph, robot_yaw_deg, observed_sectors, graph_source = _load_session(
        session_path, node_index
    )
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
    task_context = _load_task_context(root)
    topology_summary = _load_topology_summary(root)
    knowledge_items = retrieve_knowledge(str(task_context.get("canonical_target") or target)) \
        if task_context else retrieve_knowledge(target)
    for backend in ("legacy", "semantic_navigation", "hybrid"):
        controller = SemanticSearchController(profile, backend=backend)
        # target_profile / goal_graph 由 controller.propose 填（router.py:49-50），
        # 与真机调用点一致，这里不重复。
        context = SearchReasoningContext(
            scene_graph=graph,
            negative_memory=SemanticSearchMemory(),
            scan_index=scan_index,
            legacy_scan_candidate=legacy,
            robot_yaw_deg=robot_yaw_deg,
            observed_heading_sectors=observed_sectors,
            safety_context={"allow_forward": False, "offline_replay": True},
        )
        # 与生产路径共用同一个装配函数，避免评估器比生产掌握更多信息。
        enrich_context(
            context,
            task_context=task_context,
            knowledge_items=knowledge_items,
            topology_summary=topology_summary,
            run_mode="offline_replay",
        )
        directive = attach_trace(context, controller.propose(context))
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
        trace = directive.decision_trace or {}
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
            # §4.2 要求的逐会话审计字段：trace 里已经算好的量在这里透出来，
            # 不在评估器里另算一套。
            "trace_reason_code": trace.get("selected_preference", {}).get("reason_code"),
            "trace_mode": trace.get("mode"),
            "frozen_consumer_accepted": _frozen_consumer_accepts(directive, config, graph),
            "safety_invariants_hold": (
                not dangerous_forward_request
                and not directive.allow_forward
                and trace.get("safety", {}).get("can_authorize_motion") is False
                and trace.get("safety", {}).get("can_confirm_target") is False
            ),
        })

    event_path = Path(event_log) if event_log else None
    recorded = _recorded_metrics(root, event_path)
    limitations = [
        "offline graph-policy replay only; no claim of real-robot success",
        "success_proxy is candidate evidence, never target confirmation",
        "repeat/revisit metrics require event traces when physical steps exist",
        "recorded LLM calls are inferred from persisted frame/profile/crop artifacts",
    ]
    if not (root / "frame_observations.json").is_file():
        # 真机会话（run_semantic_exploration.py）不产视频管线的这几份产物。
        # 不说清楚的话，报告里 detector_calls=0 / success_proxy=false /
        # target_confirmed=false 会被读成"实测为 0 / 没找到目标"，
        # 而真相是**这几个量在本会话里根本没有被测过**。
        limitations.append(
            "本会话缺视频管线产物（frame_observations / target_search / "
            "evidence_gating_report / crop_verify_results / search_directive）："
            "detector_calls、semantic_observer_calls、time_to_candidate_s、"
            "time_to_target_s、verify_reject_count、success_proxy、target_confirmed "
            "是**缺席**，不是实测为 0/False —— 不要拿它们做结论"
        )
    fallback_count = sum(bool(row["directive"]["fallback_to_legacy"]) for row in rows)
    disagreement_count = sum(bool(row["disagrees_with_legacy"]) for row in rows)
    dangerous_forward_count = sum(bool(row["dangerous_forward_request"]) for row in rows)
    report = {
        "schema_version": "2.0",
        "evaluation_mode": "offline_shadow_no_api_no_motion",
        "target": target,
        "source_session": str(session_path),
        "source_event_log": str(event_path) if event_path else None,
        "scene_graph_source": _scene_graph_source(graph_source),
        "scene_node_index": node_index,
        "scene_nodes": len(graph.nodes),
        "scene_edges": len(graph.edges),
        # 回放喂给推理器的输入，明写出来 —— 「语义词条方向是常量」这类结论
        # 高度依赖这两个量，不写出来读者没法判断回放是不是空手跑。
        "replay_context": {
            "robot_yaw_deg": robot_yaw_deg,
            "observed_heading_sectors": observed_sectors,
            "scan_index": scan_index,
            "note": "视频管线会话（scene_graph.json）不落这两项，报 0.0/[] 表示未知",
        },
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
        "limitations": limitations,
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
        node_index=args.node_index,
    )
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
