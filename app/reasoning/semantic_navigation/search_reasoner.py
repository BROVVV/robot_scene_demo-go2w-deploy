"""Explainable next-view policies; none of them execute motion or confirm targets."""

from __future__ import annotations

import math
from typing import Protocol
from uuid import uuid4

from app.reasoning.semantic_navigation.models import (
    GraphMatchState,
    SearchDirective,
    SearchDirectiveKind,
    SearchReasoningContext,
)

# 无证据时的方向距离。用一个哨兵而不是 0：0 会被排序当成"正对着证据"，
# 于是没有证据的候选反而赢了有证据的候选。哨兵只活在排序键里，
# 进 decision_trace 之前一律换成 None（计划书 §5.3：未知不能填 0 冒充测量结果）。
_NO_EVIDENCE = float("inf")

# 方位是怎么来的。两个值都是"相对方向"，但含义差着量级：`measured` 是节点自己
# 带的方位角，`coarse_lr` 只是按 `stable_position_2d` 的左右推出来的一个写死常数
# （左右各一次，见 COARSE_LR_DEG），仓库里没有任何测量支持 25 这个数。
#
# 真机上 `measured` 一次都没出现过：1036 个场景节点里 `heading_delta_deg` 0 个
# （`observed_heading_deg` 倒有 883 个，但那是机器人自己的偏航角，见下），所以
# **真实数据上能命中的只有 `coarse_lr` 一条路**。既然只有一个值会实际出现，
# 现在标不标都读不出差别；标出来是为了它一旦命中时不至于被当成测量值 —— 两种
# 来源产出的 evidence 逐字节相同，光看角度分不出来。
HEADING_SOURCE_MEASURED = "measured"
HEADING_SOURCE_COARSE_LR = "coarse_lr"

# `stable_position_2d` 的左右两档到角度。它是个约定，不是标定值：仓库里没有
# 任何一处测量支持 25 这个数，`candidate_goal_generator.py` 里另有一份同样的
# 常数（它给下游消费者用），两处都只是同一个约定。
COARSE_LR_DEG = 25.0


class SearchReasoner(Protocol):
    def propose(self, context: SearchReasoningContext) -> SearchDirective: ...


class LegacyReasoner:
    def propose(self, context: SearchReasoningContext) -> SearchDirective:
        step = getattr(context.legacy_scan_candidate, "step", "")
        heading = _heading_from_step(step)
        return SearchDirective(
            directive_id=_id(), kind=SearchDirectiveKind.LEGACY_SCAN,
            source_backend="legacy", match_state="legacy",
            confidence=1.0, preferred_heading_delta_deg=heading,
            allow_forward=step == "f",
            reason_zh="保持既有短步扫描序列。",
            fallback_to_legacy=False,
        )


class SemanticNavigationReasoner:
    def propose(
        self,
        context: SearchReasoningContext,
        *,
        use_auxiliary_hints: bool = False,
    ) -> SearchDirective:
        match = context.graph_match
        if match is None:
            return _fallback("缺少可解释的图匹配结果，回退 legacy。", "semantic_navigation")
        target_key = getattr(context.target_profile, "canonical_name_zh", "target")
        anchor, anchor_heading, anchor_heading_source = _valid_anchor(context)

        # 有可检查的锚点时才走锚点分支；锚点缺失或定不出方向一律回落到
        # 同一个候选方向排序器（计划书 §1.2 / §2.2）。原来 PARTIAL 无条件发
        # inspect_anchor(anchor=None) 并硬编 +30°，对着一个不存在的锚点宣称
        # "已匹配到任务锚点" —— 这正是 §5.3 的裸目标缺陷。
        if match.state == GraphMatchState.PARTIAL and anchor is not None:
            return SearchDirective(
                directive_id=_id(), kind=SearchDirectiveKind.INSPECT_ANCHOR,
                source_backend="semantic_navigation", match_state=match.state.value,
                confidence=max(0.45, min(0.88, match.score + 0.18)),
                preferred_heading_delta_deg=anchor_heading,
                anchor_scene_node_id=_node_field(anchor, "node_id"),
                anchor_label=_node_field(anchor, "label_zh") or _node_field(anchor, "label"),
                allow_forward=False,
                reason_zh="已匹配到任务锚点，优先朝该锚点的观测方向重观测目标。",
                evidence_refs=match.evidence_refs,
                # 这两个锚点分支此前不带 decision_trace，于是信封里
                # `raw.get("reason_code")` 取空、回落到 "legacy_scan_sequence" ——
                # 一个语义锚点决策被面板说成"旧扫描后端产生的"，同时那句方位
                # 也丢了来历。补上最小的一份 trace 就同时解决这两件事。
                decision_trace={
                    "reason_code": "anchor_inspect",
                    "selected_heading_delta_deg": anchor_heading,
                    "candidate_ranking": [],
                    "evidence": _anchor_evidence(
                        anchor, anchor_heading, anchor_heading_source
                    ),
                    "uncertainties": [],
                },
            )
        if match.state == GraphMatchState.STRONG and anchor is not None:
            # Strong is still only a search-priority signal, never confirmation.
            return SearchDirective(
                directive_id=_id(), kind=SearchDirectiveKind.REOBSERVE_SECTOR,
                source_backend="semantic_navigation", match_state=match.state.value,
                confidence=max(0.55, min(0.95, match.score)),
                preferred_heading_delta_deg=anchor_heading,
                anchor_scene_node_id=_node_field(anchor, "node_id"),
                anchor_label=_node_field(anchor, "label_zh") or _node_field(anchor, "label"),
                allow_forward=False,
                reason_zh="目标图得到强支持，但不能据此确认目标；换角度重观测并继续视觉复核。",
                evidence_refs=match.evidence_refs,
                decision_trace={
                    "reason_code": "anchor_reobserve",
                    "selected_heading_delta_deg": anchor_heading,
                    "candidate_ranking": [],
                    "evidence": _anchor_evidence(
                        anchor, anchor_heading, anchor_heading_source
                    ),
                    "uncertainties": [],
                },
            )

        # 需要探索的三个分支共用同一个候选方向评估器。
        rows, evidence = _rank_candidate_headings(
            context, target_key, use_auxiliary_hints=use_auxiliary_hints
        )
        best = rows[0]
        heading = best["heading_delta_deg"]
        refs = best["memory_refs"]
        auxiliary_refs = best["auxiliary_refs"]
        # 去重但保序：图匹配自带的 ref 与按方向收上来的 goal_graph 证据常常是
        # 同一个节点，原样拼接会让面板把一处证据显示成两处。
        evidence_refs = list(dict.fromkeys(
            [*match.evidence_refs, *[item["ref"] for item in evidence], *auxiliary_refs]
        ))

        if match.state == GraphMatchState.ZERO:
            kind = SearchDirectiveKind.EXPLORE_UNSEEN
            confidence = max(0.35, 0.62 - 0.2 * len(refs))
            reason_code = "zero_match"
            reason_zh = "未匹配到目标或锚点，优先观察尚未覆盖且负证据较少的方向。"
            if auxiliary_refs:
                reason_zh += " 同分方向使用了低优先级 PSG/情境先验。"
        elif match.state == GraphMatchState.PARTIAL:
            kind = SearchDirectiveKind.EXPLORE_UNSEEN
            confidence = max(0.35, 0.62 - 0.2 * len(refs))
            reason_code = "partial_without_anchor"
            reason_zh = "仅目标节点形成局部匹配，缺少可检查锚点，转入未观测方向探索。"
        else:
            kind = SearchDirectiveKind.REOBSERVE_SECTOR
            confidence = max(0.55, min(0.95, match.score))
            reason_code = "strong_without_anchor"
            reason_zh = "目标图得到强支持但缺少可检查锚点，按覆盖与证据重新选择观测方向。"

        return SearchDirective(
            directive_id=_id(), kind=kind,
            source_backend="semantic_navigation", match_state=match.state.value,
            confidence=confidence,
            preferred_heading_delta_deg=heading,
            allow_forward=False,
            reason_zh=reason_zh,
            evidence_refs=evidence_refs,
            memory_penalties=refs,
            decision_trace={
                "reason_code": reason_code,
                "selected_heading_delta_deg": heading,
                "candidate_ranking": [
                    _candidate_trace(row, rank, best, evidence)
                    for rank, row in enumerate(rows)
                ],
                "evidence": [_evidence_trace(item) for item in evidence],
                "uncertainties": _uncertainties(match, anchor),
            },
        )


class HybridReasoner:
    """Prefer observed graph facts, then retain legacy as deterministic fallback."""

    def __init__(self) -> None:
        self.semantic = SemanticNavigationReasoner()
        self.legacy = LegacyReasoner()

    def propose(self, context: SearchReasoningContext) -> SearchDirective:
        directive = self.semantic.propose(
            context, use_auxiliary_hints=True
        )
        if directive.fallback_to_legacy:
            legacy = self.legacy.propose(context)
            return SearchDirective(
                **{**legacy.__dict__, "source_backend": "hybrid",
                   "reason_zh": f"{directive.reason_zh} {legacy.reason_zh}"}
            )
        return SearchDirective(
            **{**directive.__dict__, "source_backend": "hybrid"}
        )


def _rank_candidate_headings(context: SearchReasoningContext,
                             target_key: str, *,
                             use_auxiliary_hints: bool = False,
                             ) -> tuple[list[dict], list[dict]]:
    """给所有离散候选方向打分排序，并返回参与排序的方向证据。

    排序键是**字典序**，不是加权和：加权和需要给"覆盖度"和"语义支持"定权重，
    而权重在这个项目里没有标定依据，凭空定一个就是发明参数。

    键里新增的证据项位于覆盖度/负记忆之后，无证据时全部候选都取同一个哨兵值
    （`_NO_EVIDENCE`），于是它们整体退化成恒等项，排序完全回落到原有的
    `visited_cost + penalty` / 辅助提示 / `abs(heading)` 三级 —— 这就是
    "在现有键上增加字段，而不是重写全部逻辑"（计划书 §4.3）。
    """
    candidates = [-30, 30, -60, 60, -90, 90]
    observed = set(context.observed_heading_sectors)
    memory = context.negative_memory
    evidence = _directional_evidence(context)
    rows: list[dict] = []
    for index, heading in enumerate(candidates):
        sector = int(round((context.robot_yaw_deg + heading) / 30.0))
        visited_cost = 0.45 if sector in observed else 0.0
        penalty, refs = memory.sector_penalty(target_key, sector) if memory else (0.0, [])
        auxiliary_score, auxiliary_refs = (
            _auxiliary_heading_score(context, heading)
            if use_auxiliary_hints else (0.0, [])
        )
        rows.append({
            "heading_delta_deg": heading,
            "sector": sector,
            "visited_cost": visited_cost,
            "memory_penalty": penalty,
            "memory_refs": refs,
            "auxiliary_score": auxiliary_score,
            "auxiliary_refs": auxiliary_refs,
            "semantic_distance_deg": _nearest_evidence_distance(evidence, heading, "goal_graph"),
            "knowledge_distance_deg": _nearest_evidence_distance(evidence, heading, "knowledge"),
            "index": index,
        })
    rows.sort(key=_rank_key)
    return rows, evidence


def _rank_key(row: dict) -> tuple:
    # 覆盖度/负记忆排在最前：它们是**本轮已经确认过的事实**（这个扇区看过没有、
    # 上次朝这个方向吃过亏），而方向证据只是**一个实体的左右**，比"看过没有"弱。
    # 证据一旦排在覆盖度前面就会把候选推回已经看过的扇区 —— 真机实测恰好 2 例
    # （explore_go2w_20260914_125819 会选 −30、explore_go2w_20260916_200905 会选 +30，
    # 两条都是 obs=True 的已观测扇区，唯一理由是它们的语义距离最小，都是 5.0）。
    # `index` 兜底保证同分时按原候选顺序取（与改动前 `min()` 取首个最小项一致）。
    return (
        row["visited_cost"] + row["memory_penalty"],
        row["semantic_distance_deg"],
        row["knowledge_distance_deg"],
        -row["auxiliary_score"],
        abs(row["heading_delta_deg"]),
        row["index"],
    )


def _nearest_evidence_distance(evidence: list[dict], heading: float, source: str) -> float:
    """候选方向与该类证据的最近夹角。没有该类证据就返回哨兵。"""
    distances = [
        abs((item["heading_delta_deg"] - heading + 180.0) % 360.0 - 180.0)
        for item in evidence if item["source"] == source
    ]
    return min(distances) if distances else _NO_EVIDENCE


def _directional_evidence(context: SearchReasoningContext) -> list[dict]:
    """收集"能定出方向、且能追溯到出处"的实体，作为排序证据。

    定不出方向的实体**不进来**：拿一个猜出来的角度当证据，比没有证据更坏。
    它们只出现在 `uncertainties` 里。

    `goal_graph` 一类的来源是本轮已匹配上的场景实体与锚点候选；`knowledge`
    一类只认"本地知识条目指出、且该对象确实出现在本轮场景图里"的实体
    （计划书 §2.3）。知识库说垃圾桶常在桌边，而本轮没看见桌子 —— 那不是证据。
    """
    evidence: list[dict] = []
    match = context.graph_match
    if match is not None:
        ids = list(dict.fromkeys(
            [*match.matched_scene_node_ids, *match.supporting_anchor_scene_node_ids]
        ))
        for node in _scene_nodes(context.scene_graph):
            if _node_field(node, "node_id") not in ids:
                continue
            heading, heading_source = _entity_heading_delta(node)
            if heading is None:
                continue
            evidence.append({
                "source": "goal_graph",
                "ref": str(_node_field(node, "node_id")),
                "label": _node_field(node, "label_zh") or _node_field(node, "label"),
                "heading_delta_deg": heading,
                "heading_source": heading_source,
            })
    for item in context.knowledge_items:
        if not isinstance(item, dict):
            continue
        metadata = item.get("metadata") or {}
        for name in metadata.get("related_objects") or []:
            node = _scene_node_by_name(context.scene_graph, name)
            if node is None:
                continue
            heading, heading_source = _entity_heading_delta(node)
            if heading is None:
                continue
            evidence.append({
                "source": "knowledge",
                # KnowledgeItem 的主键叫 `id`；`item_id` 只是历史猜测。取不到
                # 主键就退回来源，绝不凭空编一个 ID —— evidence_refs 要能指回去。
                "ref": str(item.get("id") or item.get("item_id")
                           or item.get("source") or name),
                "label": _node_field(node, "label_zh") or _node_field(node, "label"),
                "heading_delta_deg": heading,
                "heading_source": heading_source,
            })
    return evidence


def _uncertainties(match, anchor) -> list[str]:
    codes: list[str] = []
    if anchor is None:
        cands = match.supporting_anchor_scene_node_ids if match else []
        if not cands:
            codes.append("no_anchor_candidate")
        else:
            codes.append("anchor_direction_unknown")
    return codes


def _candidate_trace(row: dict, rank: int, best: dict, evidence: list[dict]) -> dict:
    """§5.3 的 candidates 条目。

    `rank_key` 直接取排序键本身（inf 换成 null），不是另算一套只供 UI 好看的
    解释 —— 展示的分量、顺序必须就是代码真正用的那一套。
    """
    return {
        "id": f"heading:{row['heading_delta_deg']:+d}",
        "relative_heading_deg": row["heading_delta_deg"],
        "absolute_sector": row["sector"],
        "observed": row["visited_cost"] > 0.0,
        "negative_memory_penalty": row["memory_penalty"],
        "semantic_support_refs": _nearest_evidence_refs(evidence, row["heading_delta_deg"], "goal_graph"),
        "knowledge_support_refs": _nearest_evidence_refs(evidence, row["heading_delta_deg"], "knowledge"),
        "auxiliary_support_refs": list(row["auxiliary_refs"]),
        "rank_key": [_finite_or_none(part) if isinstance(part, float) else part
                     for part in _rank_key(row)],
        "selected": rank == 0,
        "rejection_reasons": [] if rank == 0 else _rejection_reasons(row, best),
    }


def _rejection_reasons(row: dict, best: dict) -> list[str]:
    """没被选中的原因，逐条对应排序键里真正输掉的那一项。

    罗列顺序**必须与 `_rank_key` 的元组顺序一致**：面板按序念出来，第一条读起来
    就是"它主要输在哪"。两处顺序一旦错开，就会出现"输在覆盖度、却说成离语义证据
    更远"这种假解释 —— 和 `_candidate_trace` 那句"顺序必须就是代码真正用的那一套"
    是同一个要求。
    """
    reasons = []
    if row["visited_cost"] + row["memory_penalty"] > best["visited_cost"] + best["memory_penalty"]:
        reasons.append("sector_already_observed" if row["visited_cost"] else "negative_memory")
    if row["semantic_distance_deg"] > best["semantic_distance_deg"]:
        reasons.append("farther_from_semantic_evidence")
    if row["knowledge_distance_deg"] > best["knowledge_distance_deg"]:
        reasons.append("farther_from_knowledge_evidence")
    if row["auxiliary_score"] < best["auxiliary_score"]:
        reasons.append("weaker_auxiliary_hint")
    if abs(row["heading_delta_deg"]) > abs(best["heading_delta_deg"]):
        reasons.append("larger_turn")
    reasons.append("lower_rank")
    return reasons


def _nearest_evidence_refs(evidence: list[dict], heading: float, source: str) -> list[str]:
    matching = [item for item in evidence if item["source"] == source]
    if not matching:
        return []
    nearest = min(
        abs((item["heading_delta_deg"] - heading + 180.0) % 360.0 - 180.0)
        for item in matching
    )
    return [
        item["ref"] for item in matching
        if abs((item["heading_delta_deg"] - heading + 180.0) % 360.0 - 180.0) == nearest
    ]


def _evidence_trace(item: dict) -> dict:
    return {
        "source": item["source"],
        "ref": item["ref"],
        "label": item["label"],
        "heading_delta_deg": item["heading_delta_deg"],
        # 方位是量出来的（measured）还是按左右推的（coarse_lr）。`source` 说的是
        # 证据从哪来（目标图 / 知识库），这一项说的是那个角度怎么来的 —— 两个
        # 正交的轴，面板上要分别显示。
        "heading_source": item["heading_source"],
    }


def _finite_or_none(value: float) -> float | None:
    return None if value == _NO_EVIDENCE else value


def _auxiliary_heading_score(
    context: SearchReasoningContext, heading: float
) -> tuple[float, list[str]]:
    best_score = 0.0
    best_refs: list[str] = []
    for hint in context.auxiliary_hints:
        if not isinstance(hint, dict) or hint.get("can_confirm_target") is not False:
            continue
        try:
            hint_heading = float(hint["heading_delta_deg"])
            confidence = max(0.0, min(1.0, float(hint.get("confidence", 0.0))))
        except (KeyError, TypeError, ValueError):
            continue
        delta = (hint_heading - heading + 180.0) % 360.0 - 180.0
        if abs(delta) > 15.0 or confidence <= best_score:
            continue
        best_score = confidence
        reference = str(
            hint.get("hint_id") or hint.get("source") or "auxiliary_hint"
        )
        best_refs = [reference]
    return best_score, best_refs


def _valid_anchor(context: SearchReasoningContext):
    """返回 `(锚点节点, 相对方向, 方向的来历)`；任何一项拿不到就返回三个 `None`。

    有效锚点必须同时满足（计划书 §2.2）：非空 `node_id`、能追溯到本轮目标图匹配、
    且存在可转成相对方向的空间信息。定不出方向就不算有效锚点 —— 原来
    `_anchor_heading` 对未知位置直接返回 15.0，等于替一个不存在的方向作证。

    **来历要和角度一起出去。** 这个分支是 ±25 那个写死常数**唯一真正走到面板上**的
    路：锚点的方位由 `_entity_heading_delta` 给出，而真机上 `heading_delta_deg`
    一个都没有，所以这里算出来的 +25 恒为 `coarse_lr`。此前调用方把第二个返回值
    丢掉（`heading, _ = ...`），面板上就只剩一句"左前方（+25.0°）"，与量出来的方位
    长得一模一样 —— 那正是 §4 要消掉的读法。
    """
    if context.graph_match is None:
        return None, None, None
    node = _anchor_node(context)
    if node is None or not _node_field(node, "node_id"):
        return None, None, None
    heading, heading_source = _entity_heading_delta(node)
    if heading is None:
        return None, None, None
    return node, heading, heading_source


def _anchor_evidence(node, heading: float, heading_source: str | None) -> list[dict]:
    """把锚点方向写成一条证据，形状与 `_directional_evidence` 一致。"""
    return [{
        "source": "goal_graph",
        "ref": str(_node_field(node, "node_id")),
        "label": _node_field(node, "label_zh") or _node_field(node, "label"),
        "heading_delta_deg": heading,
        "heading_source": heading_source,
    }]


def _anchor_node(context: SearchReasoningContext):
    ids = set(context.graph_match.supporting_anchor_scene_node_ids) if context.graph_match else set()
    for node in _scene_nodes(context.scene_graph):
        if _node_field(node, "node_id") in ids:
            return node
    return None


def _scene_nodes(scene_graph) -> list:
    if scene_graph is None:
        return []
    nodes = getattr(scene_graph, "nodes", None)
    if nodes is None and isinstance(scene_graph, dict):
        nodes = scene_graph.get("nodes")
    return list(nodes or [])


def _scene_node_by_name(scene_graph, name: str):
    name = str(name)
    for node in _scene_nodes(scene_graph):
        labels = [
            _node_field(node, "label_zh"),
            _node_field(node, "label"),
            _node_field(node, "name"),
            _node_field(node, "category"),
        ]
        if name in [str(label) for label in labels if label]:
            return node
    return None


def _entity_heading_delta(node) -> tuple[float | None, str | None]:
    """实体的相对方向**以及这个方向是怎么来的**；定不出来返回 `(None, None)`。

    只有两条路能定方向：节点自己带显式 `heading_delta_deg`（`measured`，量出来的），
    或者带 `stable_position_2d` / `position_2d` 的左右（`coarse_lr`，按左右粗推的
    ±25°）。两者都没有就是 `(None, None)` —— 左右之外的未知位置不再兜底成 15.0，
    `None` 也不许在调用方被猜成某个角。

    **来历必须跟着角度一起返回**，不能只给一个 float：±25 是写死的常数，而
    `heading_delta_deg` 一旦出现就是量出来的，两者在 trace 里长得一模一样（同一
    上下文里 `{'heading_delta_deg': 25.0}` 与 `{'stable_position_2d': 'left'}`
    产出的 evidence 逐字节相同）。实测 1036 个场景节点里 `heading_delta_deg`
    0 个，所以 **`measured` 分支在真机上从未命中过**，能命中的只有 `coarse_lr`。

    ±25 唯一真正走到面板上的路是**锚点分支**：候选表里的方向是 ±30/±60/±90
    这几个固定候选（1026 份已交付的 v1 信封里，面板"本轮选择"那一行只出现过
    −30.0 ×870 / +30 ×120 / −60 ×36，±25 一次都没有）。所以这里标出来历，防的是
    锚点路径那种情况：它此前不带 decision_trace，于是面板一边写着 `inspect_anchor`
    一边把方位说成旧扫描后端给的，而那个角度没有任何来历。

    仓库里**确实存在**一个量出来的方位：`bearing_deg`（附带 `depth_m`、
    `camera_xyz`），由 `run_semantic_exploration.py` 写在 409/1036 个节点上，
    下游由 PSG 先验消费。这里不读它，因为读了会改变排序结果 —— 那是另一件事，
    不在"给证据标来源"这一条里。

    **这里曾经还会读 `observed_heading_deg` 减当前 yaw，2026-09-17 删掉了。**
    那个字段是 `semantic_observer.py:252` 给**每一帧的每一个节点**写下的
    `robot_pose.yaw_deg`（同处还标了 `position_status = "observation_pose_only"`，
    即仓库自己声明"这个节点没有方位"）。拿它减当前 yaw，算出来的是
    "机器人自己转了多少度"，与实体在哪无关；实测 116 个录制（其中 45 个是 mock，
    去掉这 45 个数字不变）、1036 个场景节点里 `heading_delta_deg` 一个都没有、
    `observed_heading_deg` 有 883 个，
    于是这个分支**恒命中**，把下面那条真正有信息的左右分支挡成了死代码，
    还让排序键首项退化成对称的 `|heading|`（看着像测量值，其实没有方向信息）。
    """
    if node is None:
        return None, None
    attrs = _node_attrs(node)
    if isinstance(attrs.get("heading_delta_deg"), (int, float)):
        heading = float(attrs["heading_delta_deg"])
        heading_source = HEADING_SOURCE_MEASURED
    else:
        position = str(attrs.get("stable_position_2d") or attrs.get("position_2d") or "")
        if "left" in position or "左" in position:
            return COARSE_LR_DEG, HEADING_SOURCE_COARSE_LR
        if "right" in position or "右" in position:
            return -COARSE_LR_DEG, HEADING_SOURCE_COARSE_LR
        return None, None
    # 非有限值（inf/nan）等同于"定不出方向"。这里必须挡住，而不是在调用方挡：
    # nan 进排序键首位会吃掉后面所有分量（元组比较先找"第一个不相等的位置"，
    # 而 nan != nan 恒成立，于是第一位就判"不同"，覆盖度和负记忆再也不被读），
    # 结果是"没有方向的候选"赢过"有方向的候选"；它同时也会炸掉评估器
    # `json.dumps(..., allow_nan=False)`。锚点、goal_graph 证据、knowledge
    # 证据三个消费者都走这一个函数，所以守卫落在这里一处就够。
    if not math.isfinite(heading):
        return None, None
    return heading, heading_source


def _node_attrs(node) -> dict:
    attrs = node.get("attributes", {}) if isinstance(node, dict) else getattr(node, "attributes", {})
    return attrs if isinstance(attrs, dict) else {}


def _node_field(node, name: str):
    if node is None:
        return None
    return node.get(name) if isinstance(node, dict) else getattr(node, name, None)


def _heading_from_step(step: str) -> float | None:
    try:
        if step.startswith("l"):
            return float(step[1:])
        if step.startswith("r"):
            return -float(step[1:])
    except ValueError:
        return None
    return None


def _fallback(reason: str, backend: str) -> SearchDirective:
    return SearchDirective(
        directive_id=_id(), kind=SearchDirectiveKind.LEGACY_SCAN,
        source_backend=backend, match_state="unavailable", confidence=0.0,
        reason_zh=reason, fallback_to_legacy=True,
    )


def _id() -> str:
    return f"directive_{uuid4().hex[:12]}"
