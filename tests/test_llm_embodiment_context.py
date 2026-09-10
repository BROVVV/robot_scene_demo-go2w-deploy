"""计划书 §15/§17：LLM 只看结构化几何摘要，而且它的偏好会被安全层否决。

三条不变量：未知方向不得以可通行出现在提示里；不可达 frontier 不得被选为
preferred_frontier_id；脱困状态下必须先脱困再谈探索。
"""

from __future__ import annotations

import json
from types import SimpleNamespace

from app.navigation.safety_models import (
    SECTOR_COUNT,
    SECTOR_WIDTH_DEG,
    STATE_CLEAR,
    STATE_OCCUPIED,
    SafetySector,
    build_local_safety_snapshot,
    evaluate_perception_quality,
    wrap_deg,
)
from app.navigation.stuck_detector import StuckState
from app.reasoning.embodiment_context import (
    build_local_safety_context,
    build_recovery_context,
    build_topology_context,
)
from app.reasoning.llm_situated_search_reasoner import LLMSituatedSearchReasoner
from app.reasoning.observed_facts_builder import build_observed_scene_facts
from app.schemas import (
    LLMReasoningRequest,
    SceneAnalysisResult,
    default_quadruped_capability,
)


def _snapshot(*, unknown_from_deg: float | None = None):
    evidence = []
    for index in range(SECTOR_COUNT):
        bearing = wrap_deg(index * SECTOR_WIDTH_DEG)
        if unknown_from_deg is not None and abs(bearing) <= unknown_from_deg:
            # 完全不上报证据 -> 该扇区留在 UNKNOWN。
            continue
        evidence.append({"bearing_deg": bearing, "clearance_m": 2.4,
                         "state": STATE_CLEAR, "confidence": 0.9})
    return build_local_safety_snapshot(sector_evidence=evidence, lidar_fresh=True,
                                      geometry_validated=True)


def _frontier(frontier_id: str, *, gain: float = 0.5) -> dict:
    return {"frontier_id": frontier_id, "position": (1.0, 0.0), "status": "OPEN",
            "bearing_deg": 0.0, "information_gain": gain,
            "semantic_relevance": 0.3, "failure_count": 0}


def _provenance(frontier_id: str) -> dict:
    return {"frontier_id": frontier_id, "viewpoint_id": f"vp_{frontier_id}",
            "bearing_deg": 12.0, "distance_m": 0.9, "route_reachable": True,
            "route_cost_m": 1.2, "path_min_clearance_m": 0.62,
            "path_mean_clearance_m": 0.9, "view_quality": 0.71,
            "camera_occlusion_risk": 0.1, "spatial_information_gain": 0.5}


# 未知扇区不许被描述成可通行 —— 哪怕上游把标记写错了。
def test_unknown_sector_is_never_traversable() -> None:
    context = build_local_safety_context(_snapshot(unknown_from_deg=20.0))
    unknown = [item for item in context.sectors if item.clearance_m is None]
    assert unknown, "±20° 内应留出未观测扇区"
    assert all(item.traversable is False for item in unknown)
    assert context.lidar_fresh is True and context.geometry_validated is True


def test_malformed_sector_claim_is_overridden() -> None:
    snapshot = _snapshot()
    broken = SafetySector(bearing_deg=0.0, clearance_m=None, state=STATE_OCCUPIED,
                          observable=True, traversable=True, confidence=1.0)
    snapshot = type(snapshot)(**{**snapshot.__dict__, "sectors": (broken,)})
    context = build_local_safety_context(snapshot)
    assert context.sectors[0].traversable is False


# 没有几何证据的 frontier 只能是 route_reachable=False，且 Top-K 有上限。
def test_topology_context_needs_geometry_and_is_top_k() -> None:
    frontiers = [_frontier(f"f{index}", gain=0.1 * index) for index in range(10)]
    context = build_topology_context(
        current_place_id="place_1", recent_place_ids=["place_0"],
        frontiers=frontiers, viewpoint_provenance=[_provenance("f9")])
    assert len(context.frontiers) == 6
    assert context.frontiers[0].frontier_id == "f9"
    assert context.frontiers[0].route_reachable is True
    assert all(item.route_reachable is False for item in context.frontiers[1:])
    assert context.current_place_id == "place_1"


def _request(*, topology=None, safety=None, recovery=None) -> LLMReasoningRequest:
    scene = SceneAnalysisResult.model_validate(json.loads(
        (__import__("pathlib").Path(__file__).resolve().parents[1]
         / "examples/mock_knowledge_aware_result.json").read_text(encoding="utf-8")
    )["base_scene"])
    facts = build_observed_scene_facts(scene, "找到蓝色垃圾桶")
    facts = facts.model_copy(update={"target_observed": False})
    return LLMReasoningRequest(
        target_text="找到蓝色垃圾桶",
        observed_facts=facts,
        capability_contract=default_quadruped_capability(),
        local_safety=build_local_safety_context(safety),
        perception_quality=evaluate_perception_quality(
            depth_valid_ratio=0.9, near_field_ratio=0.1, visual_feature_count=200,
            stable_object_count=4, estimated_information_gain=0.4,
            front_clearance_m=2.4).to_dict(),
        topology_context=topology,
        recovery_context=recovery,
    )


def _client(payload: dict):
    response = SimpleNamespace(choices=[SimpleNamespace(
        message=SimpleNamespace(content=json.dumps(payload, ensure_ascii=False)))])
    return SimpleNamespace(chat=SimpleNamespace(completions=SimpleNamespace(
        create=lambda **kwargs: response)))


def _llm_payload(**overrides) -> dict:
    payload = {
        "scene_interpretation_zh": "走廊",
        "target_search_logic_zh": "沿走廊探索",
        "hypotheses": [],
        "global_uncertainty_zh": "未知",
        "recommended_next_observation_zh": "转向后重观测",
        "no_target_found_policy_zh": "保持 inferred",
        "preferred_intent": "EXPLORE_FRONTIER",
    }
    payload.update(overrides)
    return payload


def _reason(payload: dict, request: LLMReasoningRequest):
    return LLMSituatedSearchReasoner(client=_client(payload)).reason(request)


# 不可达 frontier 只能进 avoid 列表。
def test_unreachable_preferred_frontier_is_rejected() -> None:
    topology = build_topology_context(frontiers=[_frontier("f_far")])
    result = _reason(_llm_payload(preferred_frontier_id="f_far"),
                     _request(topology=topology, safety=_snapshot()))
    assert result.preferred_frontier_id is None
    assert result.avoid_frontier_ids == ["f_far"]


def test_reachable_preferred_frontier_survives() -> None:
    topology = build_topology_context(frontiers=[_frontier("f_ok")],
                                      viewpoint_provenance=[_provenance("f_ok")])
    result = _reason(_llm_payload(preferred_frontier_id="f_ok"),
                     _request(topology=topology, safety=_snapshot()))
    assert result.preferred_frontier_id == "f_ok"
    assert result.avoid_frontier_ids == []
    assert result.preferred_intent == "EXPLORE_FRONTIER"


# 指向未观测方向的 "开阔方向" 必须被丢掉。
def test_blocked_open_direction_is_dropped() -> None:
    request = _request(safety=_snapshot(unknown_from_deg=20.0))
    result = _reason(_llm_payload(preferred_open_direction_deg=0.0), request)
    assert result.preferred_open_direction_deg is None
    result = _reason(_llm_payload(preferred_open_direction_deg=90.0), request)
    assert result.preferred_open_direction_deg == 90.0


# 卡住时不许先探索，必须先脱困。
def test_stuck_state_forces_recovery_first() -> None:
    topology = build_topology_context(frontiers=[_frontier("f_ok")],
                                      viewpoint_provenance=[_provenance("f_ok")])
    recovery = build_recovery_context(stuck_state=StuckState.CAMERA_OCCLUDED,
                                     navigation_failures=1, recoveries=0,
                                     breadcrumb_backward_available=True)
    result = _reason(_llm_payload(preferred_frontier_id="f_ok"),
                     _request(topology=topology, safety=_snapshot(),
                              recovery=recovery))
    assert result.requires_recovery_first is True
    assert result.preferred_intent == "RECOVER_THEN_EXPLORE"
    assert result.preferred_frontier_id is None


# 传感器退化不是脱困状态：禁止盲退，但也不该强制 recovery。
def test_sensor_degraded_does_not_force_recovery() -> None:
    recovery = build_recovery_context(stuck_state=StuckState.SENSOR_DEGRADED)
    result = _reason(_llm_payload(), _request(safety=_snapshot(), recovery=recovery))
    assert result.requires_recovery_first is False


def test_unknown_intent_is_dropped() -> None:
    result = _reason(_llm_payload(preferred_intent="JUST_GO"),
                     _request(safety=_snapshot()))
    assert result.preferred_intent is None
