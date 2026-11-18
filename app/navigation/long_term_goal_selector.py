"""LongTermGoalSelector: choose the next spatial exploration goal.

This is the module that separates *where to explore* (Frontier / Anchor Region /
Target Viewpoint) from *how to execute it* (LocalGoalExecutor).

It produces a :class:`ScoredIntent` whose ``components`` and ``reasons`` are
real explainable numbers, not placeholder ``spatial_v2=1.0``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from app.spatial.models import (
    INTENT_APPROACH_TARGET,
    INTENT_EXPLORE_FRONTIER,
    INTENT_INSPECT_ANCHOR_REGION,
    INTENT_VERIFY_TARGET,
    ExplorationIntent,
    FrontierCandidate,
    SemanticPrior,
)
from app.spatial.place_graph import PlaceGraph
from app.spatial.semantic_object_map import SemanticObjectMap

MATCH_ZERO = "ZERO"
MATCH_PARTIAL = "PARTIAL"
MATCH_STRONG = "STRONG"
MATCH_VERIFY = "VERIFY"


@dataclass
class ScoredIntent:
    intent: ExplorationIntent
    score: float = 0.0
    components: dict[str, float] = field(default_factory=dict)
    reasons: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "intent": self.intent.to_dict(),
            "score": round(self.score, 4),
            "components": {k: round(v, 4) for k, v in self.components.items()},
            "reasons": self.reasons,
        }


class LongTermGoalSelector:
    def __init__(
        self,
        *,
        psg_zero_weight: float = 1.0,
        psg_partial_weight: float = 0.7,
        psg_strong_weight: float = 0.2,
        psg_verify_weight: float = 0.0,
        travel_cost_weight: float = 0.25,
        visited_penalty: float = 0.2,
        semantic_relevance_weight: float = 0.30,
        spatial_gain_weight: float = 0.25,
        psg_prior_weight: float = 0.20,
        novelty_weight: float = 0.10,
        continuity_weight: float = 0.05,
        route_cost_weight: float = 0.20,
        negative_evidence_weight: float = 0.20,
        experience_memory_weight: float = 0.10,
        observation_memory_weight: float = 0.10,
        commonsense_weight: float = 0.10,
        traversal_failure_penalty: float = 0.15,
        blacklist_penalty_weight: float = 0.6,
    ) -> None:
        self.psg_weights = {
            MATCH_ZERO: psg_zero_weight,
            MATCH_PARTIAL: psg_partial_weight,
            MATCH_STRONG: psg_strong_weight,
            MATCH_VERIFY: psg_verify_weight,
        }
        self.travel_cost_weight = float(travel_cost_weight)
        self.visited_penalty = float(visited_penalty)
        self.weights = {
            "semantic_relevance": float(semantic_relevance_weight),
            "spatial_gain": float(spatial_gain_weight),
            "psg_prior": float(psg_prior_weight),
            "novelty": float(novelty_weight),
            "continuity": float(continuity_weight),
            "route_cost": float(route_cost_weight),
            "visited_penalty": float(visited_penalty),
            "negative_evidence": float(negative_evidence_weight),
            "experience_memory": float(experience_memory_weight),
            "observation_memory": float(observation_memory_weight),
            "commonsense": float(commonsense_weight),
            "traversal_failure": float(traversal_failure_penalty),
            # 2026-10-14：``SpatialMemory.blacklist`` 在整份代码里原本**零消费者**
            # —— ``mark_frontier_visited`` 也是零调用点，所以"试过但到不了的前沿
            # 会被拉黑"这条链两端都断着。这里补上唯一的读取点。
            # 注意是**扣分**不是**剔除**：候选集里少了前沿会让
            # ``run_semantic_exploration`` 的 ``reachable_frontier_count`` 变小，
            # 而那个数喂给 ``_classify_empty_candidates`` 决定 PLANNING_ERROR 还是
            # SEARCH_EXHAUSTED —— 硬过滤会把"只剩一个前沿的房间"判成搜索穷尽。
            "blacklist": float(blacklist_penalty_weight),
        }

    def select(
        self,
        *,
        match_state: str,
        frontiers: list[FrontierCandidate],
        place_graph: PlaceGraph | None = None,
        semantic_map: SemanticObjectMap | None = None,
        target_object_ids: list[str] | None = None,
        psg_prior: SemanticPrior | None = None,
        frontier_memory: dict[str, dict[str, Any]] | None = None,
        route_costs: dict[str, dict[str, Any]] | None = None,
        semantic_relevance: dict[str, float] | None = None,
        current_yaw_deg: float = 0.0,
        current_position: tuple[float, float] | None = None,
        memory_context: dict[str, Any] | None = None,
        common_sense: dict[str, Any] | None = None,
        preferred_candidate_id: str | None = None,
    ) -> ScoredIntent | None:
        match_state = match_state.upper()
        unbound_match_state = None
        if match_state in {MATCH_STRONG, MATCH_VERIFY}:
            verified_direction = self._select_verify(
                semantic_map=semantic_map, match_state=match_state,
                target_object_ids=target_object_ids or [],
                current_yaw_deg=current_yaw_deg, current_position=current_position,
            )
            if verified_direction is not None:
                return verified_direction
            # A graph label alone is not an identified current target. Keep
            # exploring observed frontiers rather than manufacturing a target
            # bearing or failing despite reachable space remaining.
            unbound_match_state, match_state = match_state, MATCH_ZERO
        if (
            preferred_candidate_id is None
            and match_state == MATCH_PARTIAL
            and psg_prior is not None
            and psg_prior.region_hypotheses
        ):
            return self._select_anchor_region(psg_prior)
        eligible_frontiers = frontiers
        if preferred_candidate_id is not None:
            preferred = [
                item for item in frontiers
                if item.frontier_id == str(preferred_candidate_id)
            ]
            if preferred:
                eligible_frontiers = preferred
        selected = self._select_frontier(
            frontiers=eligible_frontiers,
            match_state=match_state,
            place_graph=place_graph,
            psg_prior=psg_prior,
            frontier_memory=frontier_memory or {},
            route_costs=route_costs or {},
            semantic_relevance=semantic_relevance or {},
            current_yaw_deg=current_yaw_deg,
            memory_context=memory_context or {},
            common_sense=common_sense or {},
        )
        if selected is not None and unbound_match_state is not None:
            selected.intent.provenance['unbound_target_match_state'] = unbound_match_state
            selected.reasons.append('no current identified target; explore observed frontier')
        return selected

    def rank_frontiers(self, *, frontiers: list[FrontierCandidate], **kwargs: Any) -> list[ScoredIntent]:
        """Rank program-generated candidates without changing their geometry."""
        ranked = [
            self._select_frontier(frontiers=[frontier], **kwargs)
            for frontier in frontiers
        ]
        return sorted(
            (item for item in ranked if item is not None),
            key=lambda item: item.score,
            reverse=True,
        )

    def _select_frontier(
        self,
        *,
        frontiers: list[FrontierCandidate],
        match_state: str,
        place_graph: PlaceGraph | None,
        psg_prior: SemanticPrior | None,
        frontier_memory: dict[str, dict[str, Any]],
        route_costs: dict[str, dict[str, Any]],
        semantic_relevance: dict[str, float],
        current_yaw_deg: float,
        memory_context: dict[str, Any],
        common_sense: dict[str, Any],
    ) -> ScoredIntent | None:
        if not frontiers:
            return None
        psg_weight = self.psg_weights.get(match_state, 0.5)
        psg_scores = (psg_prior.frontier_scores if psg_prior else {}) or {}
        scored: list[ScoredIntent] = []
        current_place = place_graph.current_place() if place_graph is not None else None
        current_place_id = current_place.place_id if current_place is not None else None
        for frontier in frontiers:
            components: dict[str, float] = {}
            spatial = max(0.0, min(1.0, frontier.spatial_information_gain))
            components["spatial_gain"] = spatial

            semantic = max(
                0.0,
                min(
                    1.0,
                    float(semantic_relevance.get(frontier.frontier_id, 0.0)),
                ),
            )
            components["semantic_relevance"] = semantic

            psg = max(0.0, min(1.0, float(psg_scores.get(frontier.frontier_id, 0.0)))) * psg_weight
            components["psg_prior"] = psg

            memory = frontier_memory.get(frontier.frontier_id, {})
            visit_count = int(memory.get("visit_count", 0) or 0)
            novelty = max(0.0, min(1.0, 1.0 - visit_count / max(1, 3)))
            components["novelty"] = novelty

            bearing = frontier.bearing_deg
            continuity = 1.0
            if bearing is not None:
                dyaw = abs(((float(bearing) - float(current_yaw_deg) + 180.0) % 360.0) - 180.0)
                continuity = max(0.0, min(1.0, 1.0 - dyaw / 180.0))
            components["continuity"] = continuity

            route_info = route_costs.get(frontier.frontier_id, {})
            route_cost = 0.0
            route_reachable = bool(route_info.get("reachable", True))
            if route_info.get("path_length_m") is not None:
                route_cost = min(1.0, float(route_info["path_length_m"]) / 5.0)
            elif frontier.distance_m is not None:
                route_cost = min(1.0, float(frontier.distance_m) / 5.0)
            route_penalty = route_cost if route_reachable else 1.0
            components["route_cost_penalty"] = route_penalty

            visited = min(1.0, visit_count / 3.0)
            components["visited_penalty"] = visited

            negative = 0.0
            if current_place is not None and current_place.place_id in (place_graph.places if place_graph else {}):
                negative = min(1.0, current_place.negative_evidence * 0.35)
            if negative == 0.0:
                negative = min(1.0, float(memory.get("negative_evidence", 0) or 0) * 0.35)
            components["negative_evidence_penalty"] = negative

            # Long-term memory / experience priors (frontier-level).
            frontier_priors = memory_context.get("frontier_priors", {}) or {}
            observation_priors = memory_context.get("observation_priors", {}) or {}
            exp_memory = max(
                0.0, min(1.0, float(frontier_priors.get(frontier.frontier_id, 0.0)))
            )
            obs_memory = max(
                0.0, min(1.0, float(observation_priors.get(frontier.frontier_id, 0.0)))
            )
            components["experience_memory_prior"] = exp_memory
            components["observation_memory_prior"] = obs_memory

            # Structured common-sense prior from LLM/PSG (never safety).
            cs_prior = 0.0
            for hint in (common_sense.get("frontier_hints") or []):
                if not isinstance(hint, dict):
                    continue
                hint_bearing = hint.get("bearing_deg")
                if hint_bearing is None or frontier.bearing_deg is None:
                    continue
                delta = abs(
                    (float(hint_bearing) - float(frontier.bearing_deg) + 180.0)
                    % 360.0 - 180.0
                )
                if delta <= 30.0:
                    cs_prior = max(
                        cs_prior,
                        float(hint.get("score", 0.0) or 0.0) * (1.0 - delta / 30.0),
                    )
            if cs_prior == 0.0:
                cs_prior = max(
                    0.0,
                    min(1.0, float(common_sense.get("place_prior", 0.0) or 0.0)),
                )
            components["commonsense_prior"] = cs_prior

            # Traversal failures along the route should lower the score.
            #
            # 键名修正在 2026-10-14：``frontier_memory`` 来自
            # ``SpatialMemory.frontiers[*].to_dict()``，它写的是 ``fail_count``；
            # 这里读 ``failure_count`` 于是**恒为 0**（那个键属于拓扑前沿
            # ``frontier:{place}:{sector}`` 的另一套命名空间，不是这一套）。
            failure_penalty = min(
                1.0,
                float(route_info.get("failure_count", 0) or 0) * 0.3
                + float(memory.get("fail_count", 0) or 0) * 0.3,
            )
            components["traversal_failure_penalty"] = failure_penalty

            # 被拉黑的前沿：扣一大笔分，但**留在候选集里**。留下是刻意的——
            # 候选集少一个前沿会让 reachable_frontier_count 变小，那正是
            # "卡住"被误判成"穷尽"的入口。
            blacklisted = 1.0 if memory.get("blacklisted") else 0.0
            components["blacklist_penalty"] = blacklisted

            score = (
                self.weights["semantic_relevance"] * components["semantic_relevance"]
                + self.weights["spatial_gain"] * components["spatial_gain"]
                + self.weights["psg_prior"] * components["psg_prior"]
                + self.weights["novelty"] * components["novelty"]
                + self.weights["continuity"] * components["continuity"]
                + self.weights["experience_memory"] * components["experience_memory_prior"]
                + self.weights["observation_memory"] * components["observation_memory_prior"]
                + self.weights["commonsense"] * components["commonsense_prior"]
                - self.weights["route_cost"] * components["route_cost_penalty"]
                - self.weights["visited_penalty"] * components["visited_penalty"]
                - self.weights["negative_evidence"] * components["negative_evidence_penalty"]
                - self.weights["traversal_failure"] * components["traversal_failure_penalty"]
                - self.weights["blacklist"] * components["blacklist_penalty"]
            )
            reasons = []
            if semantic > 0:
                reasons.append(f"semantic relevance {semantic:.2f}")
            if spatial > 0:
                reasons.append(f"spatial gain {spatial:.2f}")
            if psg > 0:
                reasons.append(f"PSG prior {psg:.2f}")
            if route_penalty > 0:
                reasons.append(f"route cost {route_penalty:.2f}")
            if visited > 0:
                reasons.append(f"visited penalty {visited:.2f}")
            if negative > 0:
                reasons.append(f"negative evidence {negative:.2f}")
            if exp_memory > 0:
                reasons.append(f"experience memory {exp_memory:.2f}")
            if obs_memory > 0:
                reasons.append(f"observation memory {obs_memory:.2f}")
            if cs_prior > 0:
                reasons.append(f"common sense {cs_prior:.2f}")
            if failure_penalty > 0:
                reasons.append(f"traversal failure {failure_penalty:.2f}")
            if blacklisted > 0:
                reasons.append(f"blacklisted frontier {-self.weights['blacklist']:.2f}")
            intent = ExplorationIntent(
                intent_id=f"intent_{len(scored) + 1:03d}",
                intent_type=INTENT_EXPLORE_FRONTIER,
                target_frontier_id=frontier.frontier_id,
                preferred_position=frontier.position,
                preferred_bearing_deg=frontier.bearing_deg,
                semantic_reason="; ".join(reasons) or "frontier exploration",
                semantic_score=semantic,
                psg_score=psg,
                spatial_gain=spatial,
                travel_cost=route_cost,
                provenance={
                    "frontier": frontier.to_dict(),
                    "source": str(
                        (frontier.provenance or {}).get("source") or "frontier"
                    ),
                    "match_state": match_state,
                    "route_plan": route_info,
                    "current_place_id": current_place_id,
                },
            )
            scored.append(ScoredIntent(intent, score, components, reasons))
        scored.sort(key=lambda item: item.score, reverse=True)
        return scored[0] if scored else None

    def _select_anchor_region(self, psg_prior: SemanticPrior) -> ScoredIntent:
        # Pick the highest-confidence PSG region that is still predicted.
        regions = [r for r in psg_prior.region_hypotheses if r.state != "REJECTED"]
        if not regions:
            return None
        region = max(regions, key=lambda r: r.confidence)
        intent = ExplorationIntent(
            intent_id="intent_anchor_region",
            intent_type=INTENT_INSPECT_ANCHOR_REGION,
            target_region=region.to_dict(),
            target_object_id=region.anchor_object_id,
            preferred_position=region.center,
            preferred_bearing_deg=(
                (region.bearing_range_deg[0] + region.bearing_range_deg[1]) / 2.0
                if region.bearing_range_deg else None
            ),
            semantic_reason=f"anchor region {region.region_id} confidence {region.confidence:.2f}",
            semantic_score=region.confidence,
            psg_score=region.confidence,
            spatial_gain=0.4,
            travel_cost=0.0,
            provenance={"source": "psg_semantic_region", "region_id": region.region_id},
        )
        return ScoredIntent(
            intent,
            score=0.8,
            components={"semantic": region.confidence, "psg": region.confidence, "spatial_gain": 0.4},
            reasons=["PARTIAL match", "anchor spatially located", "PSG region"],
        )

    def _select_verify(
        self, *, semantic_map: SemanticObjectMap | None, match_state: str,
        target_object_ids: list[str],
        current_yaw_deg: float, current_position: tuple[float, float] | None,
    ) -> ScoredIntent | None:
        intent_type = INTENT_VERIFY_TARGET if match_state == MATCH_VERIFY else INTENT_APPROACH_TARGET
        # Graph strength does not identify a persistent object. In particular,
        # the highest-scoring scene object can be an unrelated chair or anchor.
        # Missing target identity must not manufacture a navigation bearing.
        allowed = set(target_object_ids)
        best = max(
            (item for item in semantic_map.objects.values()
             if item.object_id in allowed and item.status not in {"STALE", "REJECTED"}
             and math.isfinite(item.confidence)
             and item.bearing_deg is not None and math.isfinite(item.bearing_deg)),
            key=lambda item: item.confidence, default=None,
        ) if semantic_map is not None else None
        if best is None:
            return None
        object_id, confidence = best.object_id, best.confidence
        if (current_position is not None and best.map_xyz is not None
                and best.provenance.get('map_frame') == 'map'
                and all(math.isfinite(v) for v in (*current_position, *best.map_xyz))):
            bearing = math.degrees(math.atan2(best.map_xyz[1]-current_position[1],
                                             best.map_xyz[0]-current_position[0]))
            bearing_source = 'current_map_position_to_target'
        else:
            # Depth-localizer bearing is camera optical +x (RIGHT), whereas
            # navigation yaw is +z (LEFT). A forward camera gives yaw-bearing.
            bearing = (current_yaw_deg-best.bearing_deg+180.) % 360.-180.
            bearing_source = 'optical_bearing_forward_camera_assumption'
        intent = ExplorationIntent(
            intent_id="intent_verify_target",
            intent_type=intent_type,
            target_object_id=object_id,
            preferred_bearing_deg=bearing,
            semantic_reason="STRONG/VERIFY target candidate requires real visual verification",
            semantic_score=confidence,
            psg_score=0.0,
            spatial_gain=0.0,
            travel_cost=0.0,
            provenance={"source": "semantic_navigation_v2", "match_state": match_state,
                        "bearing_source": bearing_source},
        )
        return ScoredIntent(
            intent,
            score=confidence,
            components={"semantic": confidence, "spatial_gain": 0.0, "route_cost_penalty": 0.0},
            reasons=[f"{match_state} match -> {intent_type}, detected candidate confidence {confidence:.2f}"],
        )
