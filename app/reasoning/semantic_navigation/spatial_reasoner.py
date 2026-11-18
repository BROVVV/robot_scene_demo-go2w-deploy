"""SemanticNavigation V2 spatial reasoner: choose a long-term spatial goal from semantic
match state + frontiers + PSG + PlaceGraph.

This is the V2 replacement for the V1 next-view policy.  It still returns an
``ExplorationIntent`` (or ``None``), never a raw motion primitive.
"""

from __future__ import annotations

from typing import Any

from app.perception.entity_roles import classify_roles

from app.navigation.long_term_goal_selector import (
    MATCH_PARTIAL,
    MATCH_STRONG,
    MATCH_VERIFY,
    MATCH_ZERO,
    LongTermGoalSelector,
    ScoredIntent,
)
from app.reasoning.semantic_navigation.models import GraphMatchResult, GraphMatchState
from app.spatial.models import ExplorationIntent, FrontierCandidate, SemanticPrior
from app.spatial.place_graph import PlaceGraph
from app.spatial.semantic_object_map import SemanticObjectMap


class SpatialSearchReasoner:
    def __init__(self, selector: LongTermGoalSelector | None = None) -> None:
        self.selector = selector or LongTermGoalSelector()

    def propose(
        self,
        *,
        graph_match: GraphMatchResult | None,
        frontiers: list[FrontierCandidate],
        place_graph: PlaceGraph | None = None,
        semantic_map: SemanticObjectMap | None = None,
        target_profile: Any = None,
        observation_id: str | None = None,
        psg_prior: SemanticPrior | None = None,
        frontier_memory: dict[str, dict[str, Any]] | None = None,
        route_costs: dict[str, dict[str, Any]] | None = None,
        semantic_relevance: dict[str, float] | None = None,
        current_yaw_deg: float = 0.0,
        current_position: tuple[float, float] | None = None,
        memory_context: dict[str, Any] | None = None,
        common_sense: dict[str, Any] | None = None,
        strategic_candidate_id: str | None = None,
    ) -> ScoredIntent | None:
        match_state = self._match_state(graph_match)
        # Persistent geometry is eligible only when the current observation
        # supports the target class. Historical high-confidence objects and
        # scene/anchor detections must not supply a target direction.
        current_objects = [
            entry.to_dict() for entry in semantic_map.objects.values()
            if observation_id and observation_id in entry.source_observation_ids
        ] if semantic_map is not None else []
        target_ids = [
            item["object_id"] for item in classify_roles(
                current_objects, target_profile,
            ).target_candidates
        ] if target_profile is not None else []
        return self.selector.select(
            match_state=match_state,
            frontiers=frontiers,
            place_graph=place_graph,
            semantic_map=semantic_map,
            target_object_ids=target_ids,
            psg_prior=psg_prior,
            frontier_memory=frontier_memory or {},
            route_costs=route_costs or {},
            semantic_relevance=semantic_relevance or {},
            current_yaw_deg=float(current_yaw_deg),
            current_position=current_position,
            memory_context=memory_context or {},
            common_sense=common_sense or {},
            preferred_candidate_id=strategic_candidate_id,
        )

    @staticmethod
    def _match_state(graph_match: GraphMatchResult | None) -> str:
        if graph_match is None:
            return MATCH_ZERO
        if graph_match.state == GraphMatchState.STRONG:
            return MATCH_STRONG
        if graph_match.state == GraphMatchState.PARTIAL:
            return MATCH_PARTIAL
        return MATCH_ZERO
