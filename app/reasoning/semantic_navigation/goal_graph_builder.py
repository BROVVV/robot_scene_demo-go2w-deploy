"""Build a compact Goal Graph from the project's existing TargetProfile."""

from __future__ import annotations

import hashlib
import re
from typing import TYPE_CHECKING, Any

from app.reasoning.semantic_navigation.models import GoalGraph, GoalGraphEdge, GoalGraphNode

if TYPE_CHECKING:
    from app.reasoning.target_profile import TargetProfile


_RELATION_ALIASES = {
    "next to": "near", "near": "near", "beside": "near", "旁边": "near",
    "附近": "near", "靠近": "near", "left of": "left_of", "左边": "left_of",
    "right of": "right_of", "右边": "right_of", "in front of": "in_front_of",
    "前面": "in_front_of", "behind": "behind", "后面": "behind", "on": "on",
    "上面": "on", "under": "under", "下面": "under", "inside": "inside",
    "里面": "inside", "contains": "contains", "attached to": "attached_to",
    "挂在": "attached_to", "贴在": "attached_to",
}


class GoalGraphBuilder:
    def build(self, profile: Any) -> GoalGraph:
        task_id = "goal_" + hashlib.sha1(
            profile.raw_query.encode("utf-8")
        ).hexdigest()[:12]
        target_id = "goal_target"
        nodes = [
            GoalGraphNode(
                node_id=target_id,
                label=profile.canonical_name_zh,
                role="target",
                aliases=_dedupe([
                    *profile.primary_labels_en,
                    *profile.aliases_zh,
                    *profile.aliases_en,
                ]),
                attributes=_dedupe([
                    *profile.attributes,
                    *profile.colors,
                    *profile.materials,
                    *profile.affordances,
                ]),
                source="target_profile",
                evidence_level="explicit",
            )
        ]
        edges: list[GoalGraphEdge] = []
        warnings: list[str] = []
        explicit_anchors = self._explicit_anchors(profile)
        used: set[str] = set()
        for label, relation, source_text in explicit_anchors:
            key = _normalize(label)
            if not key or key in used or _is_target_label(label, profile):
                continue
            used.add(key)
            node_id = f"goal_anchor_{len(used):03d}"
            anchor_aliases = _context_aliases(label, profile)
            nodes.append(
                GoalGraphNode(
                    node_id=node_id,
                    label=label,
                    role="anchor",
                    aliases=anchor_aliases,
                    source="user" if source_text == profile.raw_query else "target_profile",
                    evidence_level="explicit",
                )
            )
            edges.append(
                GoalGraphEdge(
                    edge_id=f"goal_edge_{len(edges) + 1:03d}",
                    source_node_id=target_id,
                    target_node_id=node_id,
                    relation=relation,
                    source="user" if source_text == profile.raw_query else "target_profile",
                    evidence_level="explicit",
                )
            )

        contexts = _paired_contexts(profile)
        for label, aliases in contexts:
            key = _normalize(label)
            if not key or key in used or _is_target_label(label, profile):
                continue
            used.add(key)
            node_id = f"goal_context_{len(used):03d}"
            nodes.append(
                GoalGraphNode(
                    node_id=node_id,
                    label=label,
                    role="context",
                    aliases=aliases,
                    source="target_profile",
                    evidence_level="inferred",
                )
            )
            edges.append(
                GoalGraphEdge(
                    edge_id=f"goal_edge_{len(edges) + 1:03d}",
                    source_node_id=target_id,
                    target_node_id=node_id,
                    relation="context_hint",
                    source="target_profile",
                    evidence_level="inferred",
                )
            )
        if not profile.primary_labels_en:
            warnings.append("target_profile_has_no_english_primary_label")
        return GoalGraph(
            task_id=task_id,
            raw_query=profile.raw_query,
            target_node_id=target_id,
            nodes=nodes,
            edges=edges,
            build_source=f"target_profile:{profile.resolver_source}",
            warnings=warnings,
        )

    def _explicit_anchors(self, profile: Any) -> list[tuple[str, str, str]]:
        result: list[tuple[str, str, str]] = []
        # The raw user relation is the strongest source and avoids producing
        # duplicate Chinese/English anchors from an LLM-expanded profile.
        texts = [(profile.raw_query, profile.raw_query)]
        if _relation_in(profile.raw_query) is None:
            texts.extend((item, item) for item in profile.relation_constraints)
        for text, source in texts:
            relation = _relation_in(text)
            if not relation:
                continue
            anchor = _anchor_from_text(text, relation, profile)
            if anchor:
                result.append((anchor, relation, source))
        return result


def _relation_in(text: str) -> str | None:
    lowered = text.lower()
    for alias in sorted(_RELATION_ALIASES, key=len, reverse=True):
        if alias in lowered:
            return _RELATION_ALIASES[alias]
    return None


def _anchor_from_text(text: str, relation: str, profile: Any) -> str:
    lowered = text.lower()
    aliases = [key for key, value in _RELATION_ALIASES.items() if value == relation]
    for alias in sorted(aliases, key=len, reverse=True):
        position = lowered.find(alias)
        if position < 0:
            continue
        before = text[:position].strip(" ，,。的")
        after = text[position + len(alias):].strip(" ，,。的")
        contains_chinese = any("\u4e00" <= ch <= "\u9fff" for ch in text)
        if contains_chinese and relation in {"attached_to", "on", "under", "inside"}:
            candidate = re.split(r"[上下里中的]", after, maxsplit=1)[0]
        else:
            # Chinese "anchor + 旁边 + target" and English
            # "target + next to + anchor" use opposite sides.
            candidate = before if contains_chinese else after
        candidate = _strip_task_words(candidate)
        candidate = _strip_target_words(candidate, profile)
        if candidate:
            return candidate
    return ""


def _strip_task_words(value: str) -> str:
    return re.sub(r"^(请|帮我|请帮我)?(寻找|找到|找|看一下)", "", value).strip()


def _strip_target_words(value: str, profile: Any) -> str:
    result = value
    for term in [profile.canonical_name_zh, *profile.aliases_zh]:
        if term:
            result = result.replace(term, "")
    # Attributes belong to the target, not to the anchor.
    for term in [*profile.colors, *profile.attributes]:
        if term:
            result = result.replace(term, "")
    result = re.sub(r"^(蓝色|红色|灰色|黑色|白色|黄色|绿色)", "", result)
    return result.strip(" 的，,。")


def _paired_contexts(profile: Any) -> list[tuple[str, list[str]]]:
    result: list[tuple[str, list[str]]] = []
    width = max(len(profile.context_labels_zh), len(profile.context_labels_en))
    for index in range(width):
        zh = profile.context_labels_zh[index] if index < len(profile.context_labels_zh) else ""
        en = profile.context_labels_en[index] if index < len(profile.context_labels_en) else ""
        label = zh or en
        aliases = [item for item in [zh, en] if item and item != label]
        result.append((label, aliases))
    return result


def _context_aliases(label: str, profile: Any) -> list[str]:
    key = _normalize(label)
    for context_label, aliases in _paired_contexts(profile):
        terms = [context_label, *aliases]
        if key in {_normalize(item) for item in terms}:
            return [item for item in terms if _normalize(item) != key]
    return []


def _is_target_label(label: str, profile: Any) -> bool:
    normalized = _normalize(label)
    return normalized in {_normalize(item) for item in profile.direct_terms()}


def _normalize(value: str) -> str:
    return "".join(re.findall(r"[a-z0-9\u4e00-\u9fff]+", value.lower()))


def _dedupe(values: list[str]) -> list[str]:
    result: list[str] = []
    seen: set[str] = set()
    for value in values:
        key = _normalize(value)
        if key and key not in seen:
            seen.add(key)
            result.append(value)
    return result
