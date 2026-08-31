"""Explainable next-view policies; none of them execute motion or confirm targets."""

from __future__ import annotations

from typing import Protocol
from uuid import uuid4

from app.reasoning.semantic_navigation.models import (
    GraphMatchState,
    SearchDirective,
    SearchDirectiveKind,
    SearchReasoningContext,
)


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
        if match.state == GraphMatchState.ZERO:
            heading, penalties, auxiliary_refs = _best_unseen_heading(
                context,
                target_key,
                use_auxiliary_hints=use_auxiliary_hints,
            )
            auxiliary_reason = (
                " 同分方向使用了低优先级 PSG/情境先验。"
                if auxiliary_refs else ""
            )
            return SearchDirective(
                directive_id=_id(), kind=SearchDirectiveKind.EXPLORE_UNSEEN,
                source_backend="semantic_navigation", match_state=match.state.value,
                confidence=max(0.35, 0.62 - 0.2 * len(penalties)),
                preferred_heading_delta_deg=heading, allow_forward=False,
                reason_zh=(
                    "未匹配到目标或锚点，优先观察尚未覆盖且负证据较少的方向。"
                    + auxiliary_reason
                ),
                evidence_refs=[*match.evidence_refs, *auxiliary_refs],
                memory_penalties=penalties,
            )
        anchor = _anchor_node(context)
        heading = _anchor_heading(anchor, context.robot_yaw_deg)
        if match.state == GraphMatchState.PARTIAL:
            return SearchDirective(
                directive_id=_id(), kind=SearchDirectiveKind.INSPECT_ANCHOR,
                source_backend="semantic_navigation", match_state=match.state.value,
                confidence=max(0.45, min(0.88, match.score + 0.18)),
                preferred_heading_delta_deg=heading,
                anchor_scene_node_id=_node_field(anchor, "node_id"),
                anchor_label=_node_field(anchor, "label_zh") or _node_field(anchor, "label"),
                allow_forward=False,
                reason_zh="已匹配到任务锚点，优先朝该锚点的观测方向重观测目标。",
                evidence_refs=match.evidence_refs,
            )
        # Strong is still only a search-priority signal, never confirmation.
        return SearchDirective(
            directive_id=_id(), kind=SearchDirectiveKind.REOBSERVE_SECTOR,
            source_backend="semantic_navigation", match_state=match.state.value,
            confidence=max(0.55, min(0.95, match.score)),
            preferred_heading_delta_deg=heading,
            anchor_scene_node_id=_node_field(anchor, "node_id"),
            anchor_label=_node_field(anchor, "label_zh") or _node_field(anchor, "label"),
            allow_forward=False,
            reason_zh="目标图得到强支持，但不能据此确认目标；换角度重观测并继续视觉复核。",
            evidence_refs=match.evidence_refs,
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


def _best_unseen_heading(context: SearchReasoningContext,
                         target_key: str, *,
                         use_auxiliary_hints: bool = False,
                         ) -> tuple[float, list[str], list[str]]:
    candidates = [-30, 30, -60, 60, -90, 90]
    observed = set(context.observed_heading_sectors)
    memory = context.negative_memory
    ranked: list[tuple[float, float, float, list[str], list[str]]] = []
    for heading in candidates:
        sector = int(round((context.robot_yaw_deg + heading) / 30.0))
        visited_cost = 0.45 if sector in observed else 0.0
        penalty, refs = memory.sector_penalty(target_key, sector) if memory else (0.0, [])
        auxiliary_score, auxiliary_refs = (
            _auxiliary_heading_score(context, heading)
            if use_auxiliary_hints else (0.0, [])
        )
        ranked.append((
            visited_cost + penalty,
            -auxiliary_score,
            heading,
            refs,
            auxiliary_refs,
        ))
    _, _, heading, refs, auxiliary_refs = min(
        ranked, key=lambda item: (item[0], item[1], abs(item[2]))
    )
    return heading, refs, auxiliary_refs


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


def _anchor_node(context: SearchReasoningContext):
    ids = set(context.graph_match.supporting_anchor_scene_node_ids) if context.graph_match else set()
    nodes = getattr(context.scene_graph, "nodes", []) if context.scene_graph is not None else []
    for node in nodes:
        if _node_field(node, "node_id") in ids:
            return node
    return None


def _anchor_heading(node, robot_yaw_deg: float) -> float:
    if node is None:
        return 30.0
    attrs = node.get("attributes", {}) if isinstance(node, dict) else getattr(node, "attributes", {})
    if isinstance(attrs.get("heading_delta_deg"), (int, float)):
        return float(attrs["heading_delta_deg"])
    if isinstance(attrs.get("observed_heading_deg"), (int, float)):
        delta = float(attrs["observed_heading_deg"]) - robot_yaw_deg
        return (delta + 180.0) % 360.0 - 180.0
    position = str(attrs.get("stable_position_2d") or attrs.get("position_2d") or "")
    if "left" in position or "左" in position:
        return 25.0
    if "right" in position or "右" in position:
        return -25.0
    return 15.0


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
