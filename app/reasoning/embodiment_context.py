"""计划书 §15：把几何/感知/拓扑/脱困状态压成 LLM 能读的结构化上下文。

只传扇区级结论与 Top-K frontier，绝不传原始点云（§15.2）。这里同时是一道
约束闸门：clearance 未知的扇区永远不会以 ``traversable=True`` 出现在提示里。
"""

from __future__ import annotations

from typing import Any, Iterable, Mapping, Sequence

from app.navigation.safety_models import LocalSafetySnapshot, PerceptionQuality
from app.navigation.stuck_detector import StuckState, requires_recovery
from app.schemas import (
    LLMFrontierSummary,
    LLMLocalSafetyContext,
    LLMRecoveryContext,
    LLMSafetySector,
    LLMTopologyContext,
)

# §15.2：Top-K 建议 5-8 个。
DEFAULT_FRONTIER_TOP_K = 6


def build_local_safety_context(
    snapshot: LocalSafetySnapshot | None,
) -> LLMLocalSafetyContext | None:
    if snapshot is None:
        return None
    sectors = [
        LLMSafetySector(
            bearing_deg=round(sector.bearing_deg, 2),
            clearance_m=(
                None if sector.clearance_m is None else round(sector.clearance_m, 3)
            ),
            state=sector.state,
            # 未知距离不许被描述成可通行，否则 LLM 会把盲区当成开阔方向。
            traversable=bool(sector.traversable and sector.clearance_m is not None),
            confidence=max(0.0, min(1.0, float(sector.confidence))),
        )
        for sector in snapshot.sectors
    ]
    return LLMLocalSafetyContext(
        front_clearance_m=snapshot.front_clearance_m,
        rear_clearance_m=snapshot.rear_clearance_m,
        left_clearance_m=snapshot.left_clearance_m,
        right_clearance_m=snapshot.right_clearance_m,
        forward_safe=snapshot.forward_safe,
        backward_safe=snapshot.backward_safe,
        nearest_free_sector_deg=snapshot.nearest_free_sector_deg,
        lidar_fresh=snapshot.lidar_fresh,
        geometry_validated=snapshot.geometry_validated,
        sectors=sectors,
    )


def build_perception_quality_context(
    perception: PerceptionQuality | None,
) -> dict[str, Any] | None:
    return None if perception is None else perception.to_dict()


def _clamp01(value: Any) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    return max(0.0, min(1.0, number))


def _frontier_summary(
    frontier: Mapping[str, Any],
    provenance: Mapping[str, Any] | None,
) -> LLMFrontierSummary:
    """frontier 自身给语义，视点 provenance 给几何可达性。"""
    provenance = provenance or {}
    return LLMFrontierSummary(
        frontier_id=str(frontier.get("frontier_id") or ""),
        bearing_deg=float(
            provenance.get("bearing_deg", frontier.get("bearing_deg") or 0.0)
        ),
        distance_m=provenance.get("distance_m"),
        semantic_relevance=_clamp01(frontier.get("semantic_relevance")),
        spatial_information_gain=_clamp01(frontier.get("information_gain")),
        # 没有视点证据时按不可达上报：可达性必须由几何证明，不能默认成立。
        route_reachable=bool(provenance.get("route_reachable", False)),
        route_cost_m=provenance.get("route_cost_m"),
        path_min_clearance_m=provenance.get("path_min_clearance_m"),
        view_quality=_clamp01(provenance.get("view_quality")),
        traversal_failure_count=int(frontier.get("failure_count") or 0),
    )


def build_topology_context(
    *,
    current_place_id: str | None = None,
    recent_place_ids: Sequence[str] = (),
    frontiers: Iterable[Mapping[str, Any]] = (),
    viewpoint_provenance: Iterable[Mapping[str, Any]] = (),
    top_k: int = DEFAULT_FRONTIER_TOP_K,
) -> LLMTopologyContext:
    by_frontier = {
        str(item.get("frontier_id")): item
        for item in viewpoint_provenance
        if item.get("frontier_id")
    }
    summaries = [
        _frontier_summary(entry, by_frontier.get(str(entry.get("frontier_id"))))
        for entry in frontiers
        if entry.get("frontier_id")
    ]
    summaries.sort(
        key=lambda item: (
            item.route_reachable,
            item.spatial_information_gain + item.semantic_relevance,
            item.view_quality,
        ),
        reverse=True,
    )
    return LLMTopologyContext(
        current_place_id=current_place_id,
        recent_place_ids=[str(item) for item in recent_place_ids],
        frontiers=summaries[: max(1, int(top_k))],
    )


def build_recovery_context(
    *,
    stuck_state: StuckState | str | None = None,
    navigation_failures: int = 0,
    recoveries: int = 0,
    breadcrumb_backward_available: bool = False,
) -> LLMRecoveryContext:
    if isinstance(stuck_state, StuckState):
        state = stuck_state.value
    else:
        state = str(stuck_state or StuckState.FREE.value)
    return LLMRecoveryContext(
        stuck_state=state,
        recent_navigation_failures=int(navigation_failures),
        recent_recoveries=int(recoveries),
        breadcrumb_backward_available=bool(breadcrumb_backward_available),
    )


def recovery_required(context: LLMRecoveryContext | None) -> bool:
    if context is None:
        return False
    try:
        return requires_recovery(StuckState(context.stuck_state))
    except ValueError:
        return False
