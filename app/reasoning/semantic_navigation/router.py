"""Backend router and reusable controller for live semantic reasoning."""

from __future__ import annotations

from typing import Any

from app.reasoning.semantic_navigation.goal_graph_builder import GoalGraphBuilder
from app.reasoning.semantic_navigation.graph_matcher import SemanticNavigationGraphMatcher
from app.reasoning.semantic_navigation.models import SearchDirective, SearchReasoningContext
from app.reasoning.semantic_navigation.search_reasoner import HybridReasoner, LegacyReasoner, SemanticNavigationReasoner


class SearchReasonerRouter:
    def __init__(self, backend: str = "legacy", *, partial_threshold: float = 0.30,
                 strong_threshold: float = 0.72) -> None:
        if backend not in {"legacy", "semantic_navigation", "hybrid"}:
            raise ValueError(f"unsupported search reasoner backend: {backend}")
        self.backend = backend
        self.matcher = SemanticNavigationGraphMatcher(partial_threshold, strong_threshold)
        self.reasoner = {
            "legacy": LegacyReasoner(),
            "semantic_navigation": SemanticNavigationReasoner(),
            "hybrid": HybridReasoner(),
        }[backend]

    def propose(self, context: SearchReasoningContext) -> SearchDirective:
        if self.backend != "legacy" and context.goal_graph is not None:
            context.graph_match = self.matcher.match(
                context.goal_graph, context.scene_graph or {"nodes": [], "edges": []},
                target_profile=context.target_profile,
            )
        return self.reasoner.propose(context)


class SemanticSearchController:
    """Own the task graph and route each event-driven semantic snapshot."""

    def __init__(self, target_profile: Any, *, backend: str = "hybrid",
                 partial_threshold: float = 0.30,
                 strong_threshold: float = 0.72) -> None:
        self.target_profile = target_profile
        self.goal_graph = GoalGraphBuilder().build(target_profile)
        self.router = SearchReasonerRouter(
            backend, partial_threshold=partial_threshold,
            strong_threshold=strong_threshold,
        )

    def propose(self, context: SearchReasoningContext) -> SearchDirective:
        context.target_profile = self.target_profile
        context.goal_graph = self.goal_graph
        return self.router.propose(context)
