"""确定性任务 Agent：把已有上下文汇总成可审计的决策依据。

职责边界（计划书 §3.1）：根据 `SearchTaskContext` 生成总体计划、汇总已传入的
上下文、把 reasoner 的真实候选分量整理成 `decision_trace`、生成简短中文说明
和不确定性列表。

它**不**决定底盘速度、不调用网络或模型 API、不改地图、不确认目标、不创建导航
候选，也**不重算**一套与 reasoner 不同的排名 —— 候选分量一律照抄
`directive.decision_trace` 里 reasoner 自己写下的原始值。这里没有全局状态可读，
所有输入都从参数进来，因此同样的输入必然得到同样的输出。
"""

from __future__ import annotations

import dataclasses
from typing import Any

from app.reasoning.semantic_navigation.models import (
    SearchDirective,
    SearchReasoningContext,
)

SCHEMA_VERSION = "task_agent_trace_v1"

# 当前执行链真正能做的事：搜索/探索/观察。其余子任务必须如实标注，
# 不能假装已经调度（计划书 §4.6）。判定只用子任务自己声明的能力字段，
# 不另造一套动作词表。
_PLAN_STAGES = (
    ("understand_task", "理解任务", "解析目标、属性、关系与限制。"),
    ("retrieve_context", "汇总上下文", "加载本地知识、目标图、场景图、记忆与拓扑摘要。"),
    ("search_or_explore", "选择搜索方向", "按证据感知的候选排序给出本轮搜索偏好。"),
    ("verify_target", "等待视觉复核", "目标是否成立只能由视觉判据决定，知识不能代替检测。"),
    ("report_result", "汇报结果", "仅在收到任务结束事件后才算完成。"),
)


def build_plan(task_context: dict[str, Any] | None, *, active_id: str) -> list[dict[str, Any]]:
    """总体计划快照。

    任务自带 `subtasks` 时保留原顺序与原始文本，只根据子任务**自己声明的**
    能力字段标注状态；没有 `subtasks` 就用 §4.6 的五阶段默认计划。
    绝不根据最终结果倒填一份"看起来更聪明"的计划。
    """
    task_context = task_context or {}
    subtasks = list(task_context.get("subtasks") or [])
    if not subtasks:
        stages = []
        for stage_id, title, detail in _PLAN_STAGES:
            if stage_id in {"understand_task", "retrieve_context"}:
                status = "completed"
            elif stage_id == active_id:
                status = "active"
            else:
                status = "pending"
            stages.append({
                "id": stage_id, "title": title, "status": status,
                "detail": detail, "capability": "supported",
            })
        return stages

    # §3.2：「**当前搜索目标**对应的子任务标为 active」——是单数。认不出当前目标
    # 是哪一个（没有 canonical_target，或没有一个子任务对得上）时，一个都不标：
    # 把"所有导航子任务"一起说成正在进行中，比不标更糟。
    current_target = str(task_context.get("canonical_target") or "").strip()
    active_taken = False
    plan = []
    for index, item in enumerate(subtasks):
        if not isinstance(item, dict):
            item = {"id": f"subtask_{index + 1}", "raw_text": str(item)}
        blocked_reason = item.get("blocked_reason")
        if (blocked_reason or item.get("allowed_by_capability") is False
                or item.get("allowed_by_safety") is False):
            status, capability = "blocked", "unsupported"
        elif item.get("requires_manipulation") or item.get("requires_physical_contact"):
            status, capability = "unsupported_by_current_executor", "unsupported"
        else:
            status, capability = "pending", "supported"
        if (status == "pending" and item.get("is_navigation_relevant")
                and not active_taken and current_target
                and str(item.get("object") or "").strip() == current_target):
            status, active_taken = "active", True
        plan.append({
            "id": str(item.get("id") or f"subtask_{index + 1}"),
            "title": str(item.get("object") or item.get("raw_text") or item.get("id") or ""),
            "status": status,
            "detail": str(blocked_reason or item.get("llm_reason") or ""),
            "capability": capability,
            "depends_on": [str(prior) for prior in (item.get("depends_on") or [])],
        })
    return plan


def summarize_context(context: SearchReasoningContext) -> dict[str, Any]:
    """上下文摘要。只汇总传进来的东西，不自己读取任何全局状态。"""
    match = context.graph_match
    scene_nodes = _scene_nodes(context)
    return {
        "graph_match": match.to_dict() if match is not None else None,
        "goal_graph": context.goal_graph.to_dict() if context.goal_graph else None,
        "scene_graph": {
            "node_count": len(scene_nodes),
            "labels": [
                label for label in (_node_label(node) for node in scene_nodes) if label
            ],
        },
        "observation": {
            "scan_index": context.scan_index,
            "robot_yaw_deg": context.robot_yaw_deg,
            "distance_from_origin_m": context.distance_from_origin_m,
            "observed_heading_sectors": list(context.observed_heading_sectors),
            "has_semantic_observation": context.semantic_observation is not None,
        },
        "memory": {
            "observation_memory_count": len(context.observation_memory),
            "negative_memory_present": context.negative_memory is not None,
            "negative_target_key": getattr(context.target_profile, "canonical_name_zh", None),
        },
        "topology": context.topology_summary,
        "world_state": context.world_state_snapshot,
        # 拿不到就是拿不到，列进 missing_sources，不折叠成 0 / {} 冒充测量值。
        "missing_sources": _missing_sources(context),
    }


def build_decision_trace(
    context: SearchReasoningContext,
    directive: SearchDirective,
) -> dict[str, Any]:
    """§5.3 的 `task_agent_trace_v1`。候选分量原样取自 reasoner。"""
    raw = directive.decision_trace or {}
    candidates = list(raw.get("candidate_ranking") or [])
    evidence = list(raw.get("evidence") or [])
    task_context = context.task_context or {}
    # legacy 后端只重放既有扫描序列，本来就没有候选排名。给它一个明确的原因码，
    # 而不是留一个含义不明的 null —— UI 才能显示"这个后端没有候选审计"的空态。
    reason_code = raw.get("reason_code") or (None if raw else "legacy_scan_sequence")

    return {
        "schema_version": SCHEMA_VERSION,
        "trace_id": directive.directive_id,
        "mode": context.run_mode,
        "task": {
            "raw_text": task_context.get("raw_text"),
            "canonical_target": task_context.get("canonical_target"),
            "attributes": _as_list(task_context.get("target_attributes")),
            "relations": _as_list(task_context.get("target_relations")),
            "constraints": _as_list(task_context.get("constraints")),
        },
        "plan": build_plan(task_context, active_id="search_or_explore"),
        "active_subtask": {
            "id": "search_or_explore",
            "title": "搜索目标",
            "reason": "目标尚未通过视觉判据确认，本轮只产生搜索偏好。",
        },
        "inputs": summarize_context(context),
        "knowledge": _knowledge_trace(context, candidates),
        "candidates": candidates,
        "selected_preference": {
            "directive_kind": directive.kind.value,
            "relative_heading_deg": directive.preferred_heading_delta_deg,
            "reason_code": reason_code,
            "reason_zh": _reason_zh(directive, reason_code, task_context),
            "evidence_refs": list(directive.evidence_refs),
            # evidence_refs 只有 ID，说不出这些证据各自把方向指到哪、那个方向
            # 又是怎么来的。`evidence` 从上面读出来一直没被用过（这就是它此前
            # 不在 v1 信封里的原因），现在原样透出：每条带 ref / label /
            # heading_delta_deg / heading_source。
            "evidence": evidence,
        },
        "uncertainties": _uncertainties(context, raw, candidates),
        "safety": {
            # 这三项都不是"策略上先填 false"：推理层结构上就没有确认目标或
            # 授权运动的能力。allow_forward 照抄 directive 的真实值。
            "can_confirm_target": False,
            "can_authorize_motion": False,
            "allow_forward": bool(directive.allow_forward),
        },
        "downstream": {
            "status": "pending",
            "decision_id": None,
            "reason": None,
        },
    }


def attach_trace(
    context: SearchReasoningContext,
    directive: SearchDirective,
) -> SearchDirective:
    """返回挂了完整 v1 trace 的 directive 副本。

    `SearchDirective` 是 frozen dataclass，这里用 `dataclasses.replace` 生成新对象，
    不改原对象、也不动任何冻结字段的语义。
    """
    return dataclasses.replace(
        directive, decision_trace=build_decision_trace(context, directive)
    )


def retrieve_knowledge(
    target_text: str,
    *,
    room_type: str | None = None,
    location_hint: str | None = None,
) -> list[dict[str, Any]]:
    """复用既有本地检索，不新增检索逻辑、不创造知识、不改置信度。

    生产路径与离线评估器都调这一个函数，避免评估器掌握生产没有的信息
    （计划书 §4.2）。检索不到就是空列表 —— 那是"本轮没有"，不是负证据。
    """
    from app.knowledge.retrieval import retrieve_relevant_knowledge

    items = retrieve_relevant_knowledge(
        target_text=target_text, room_type=room_type, location_hint=location_hint
    )
    return [item.model_dump() for item in items]


def summarize_topology(place_graph: Any) -> dict[str, Any] | None:
    """只读拓扑摘要。拿不到就返回 `None`，绝不用 0 冒充"测量值为零"。

    §4.5：第一阶段拓扑只做上下文和下游关联，**不参与方向排序**。

    生产路径传的是活的 `PlaceGraph`，而且是**每帧**读一次，所以这里只读它的
    `places` / `edges`（都是 O(1)）—— 不走 `to_dict()`，那会把整个会话的观测
    序列化一遍，逐帧调用就是 O(n²)。离线评估器传的是 `place_graph.json` 读出来
    的 dict。两种输入都要认，否则两条路喂给推理器的口径就不一致（§4.1 / §4.2）。
    """
    if isinstance(place_graph, dict):
        places = place_graph.get("places")
        edges = place_graph.get("edges")
        current_place_id = place_graph.get("current_place_id")
    else:
        places = getattr(place_graph, "places", None)
        edges = getattr(place_graph, "edges", None)
        current = (
            place_graph.current_place()
            if hasattr(place_graph, "current_place") else None
        )
        current_place_id = getattr(current, "place_id", None)
    if places is None and edges is None:
        return None
    return {
        "current_place_id": current_place_id,
        "place_count": len(places or []),
        "edge_count": len(edges or []),
    }


def enrich_context(
    context: SearchReasoningContext,
    *,
    task_context: dict[str, Any] | None = None,
    knowledge_items: list[dict[str, Any]] | None = None,
    topology_summary: dict[str, Any] | None = None,
    run_mode: str = "live",
) -> SearchReasoningContext:
    """把任务上下文、知识与拓扑摘要补进推理上下文。

    生产路径（`scripts/go2w/run_semantic_exploration.py`）和离线评估器
    （`scripts/evaluate_live_search_reasoners.py`）都走这一个函数，这样
    两边喂给推理器的输入口径一致（计划书 §4.1 / §4.2）。
    """
    context.task_context = task_context
    context.knowledge_items = list(knowledge_items or [])
    context.topology_summary = topology_summary
    context.run_mode = run_mode
    return context


def _knowledge_trace(
    context: SearchReasoningContext, candidates: list[dict[str, Any]]
) -> list[dict[str, Any]]:
    """每条知识是"被用于定向"还是"仅作背景"，由候选的 support_refs 反查。

    没有落到本轮可见实体上的知识只能是 `context_only` —— 知识库里写着
    "垃圾桶常在桌边"，而本轮没看见桌子，那它给不出任何方向（计划书 §4.4）。
    """
    rows = []
    for item in context.knowledge_items:
        if not isinstance(item, dict):
            continue
        item_id = str(item.get("id") or item.get("source") or "")
        supports = [
            candidate["id"] for candidate in candidates
            if item_id in (candidate.get("knowledge_support_refs") or [])
        ]
        rows.append({
            "id": item_id,
            "content": str(item.get("content_zh") or ""),
            "source": str(item.get("source") or ""),
            "confidence": item.get("confidence"),
            "status": "used" if supports else "context_only",
            "supports": supports,
            "reason": (
                "该条知识的相关对象出现在本轮场景图中，参与了方向排序。"
                if supports else
                "该条知识的相关对象未出现在本轮场景图中，仅作背景，未参与方向排序。"
            ),
        })
    return rows


def _uncertainties(
    context: SearchReasoningContext, raw: dict[str, Any], candidates: list[dict[str, Any]]
) -> list[dict[str, Any]]:
    """不确定性只记录"确实拿不到/定不出"的东西，不编造风险清单。"""
    items = [
        {"code": code, "detail": _UNCERTAINTY_ZH.get(code, code)}
        for code in (raw.get("uncertainties") or [])
    ]
    for source in _missing_sources(context):
        items.append({
            "code": f"missing_{source}",
            "detail": f"未拿到 {source}，相关判断只能建立在其余来源上。",
        })
    if not context.knowledge_items:
        items.append({
            "code": "no_knowledge_retrieved",
            "detail": "本轮未检索到本地知识；这不构成任何方向的负证据。",
        })
    if not candidates:
        items.append({
            "code": "no_candidate_ranking",
            "detail": "该后端不产生候选方向排名，本轮没有可审计的候选分量。",
        })
    if candidates and all(not row.get("auxiliary_support_refs") for row in candidates):
        items.append({
            "code": "no_auxiliary_hint",
            "detail": "本轮没有可用的辅助情境提示。",
        })
    return items


_UNCERTAINTY_ZH = {
    "no_anchor_candidate": "目标图里没有可检查的锚点候选，方向只能由覆盖度与可见证据决定。",
    "anchor_direction_unknown": "锚点候选存在，但没有任何可转成相对方向的空间信息。",
}


def _missing_sources(context: SearchReasoningContext) -> list[str]:
    missing = []
    if context.graph_match is None:
        missing.append("graph_match")
    if context.goal_graph is None:
        missing.append("goal_graph")
    if context.scene_graph is None:
        missing.append("scene_graph")
    if context.topology_summary is None:
        missing.append("topology_summary")
    if context.task_context is None:
        missing.append("task_context")
    return missing


def _reason_zh(
    directive: SearchDirective, reason_code: str | None, task_context: dict[str, Any]
) -> str:
    target = task_context.get("canonical_target") or "目标"
    prefix = f"任务目标「{target}」："
    heading = directive.preferred_heading_delta_deg
    if reason_code == "partial_without_anchor":
        # 计划书 §1.2 锁定的原文，一个字都不许改。
        return (
            prefix
            + "仅目标节点形成 PARTIAL，缺少可检查锚点，因此转入未观测方向探索"
            + (f"，本轮建议相对转向 {heading:+g}°。" if isinstance(heading, (int, float)) else "。")
        )
    return prefix + directive.reason_zh


def _scene_nodes(context: SearchReasoningContext) -> list:
    scene_graph = context.scene_graph
    if scene_graph is None:
        return []
    nodes = getattr(scene_graph, "nodes", None)
    if nodes is None and isinstance(scene_graph, dict):
        nodes = scene_graph.get("nodes")
    return list(nodes or [])


def _node_label(node) -> str | None:
    get = node.get if isinstance(node, dict) else lambda name, d=None: getattr(node, name, d)
    return get("label_zh") or get("label")


def _as_list(value: Any) -> list:
    if value is None:
        return []
    if isinstance(value, dict):
        return [f"{key}={item}" for key, item in value.items()]
    if isinstance(value, str):
        return [value]
    return list(value)
