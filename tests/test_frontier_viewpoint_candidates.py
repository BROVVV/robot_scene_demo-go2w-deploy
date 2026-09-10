"""计划书 §7.2/§8/§10/§29：候选目标必须是站得住的观察位姿，不是 frontier 质心。

三条硬线在这里被钉住：
1. frontier 贴墙时，候选目标是退开后的 standoff 位姿；
2. 找不到硬安全位姿时，frontier 变成拒绝证据，绝不静默消失；
3. 没有净空地图（旧会话）时，候选生成行为与改造前完全一致。
"""

from __future__ import annotations

import math

from app.live_robot.autonomous_explorer import AutonomousExplorer
from app.live_robot.mock_observation_scene import (
    MockObservationScene,
    MockSceneStep,
    scenario_no_target,
)
from app.navigation.backend_factory import MockBackend
from app.navigation.candidate_goal_generator import (
    generate_live_exploration_candidates,
)
from app.navigation.clearance_map import clearance_map_from_points
from app.navigation.exploration_config import (
    ExplorationBudget,
    load_exploration_policy,
)
from app.navigation.exploration_graph import ExplorationGraph
from app.navigation.models import (
    GOAL_NAVIGATE_POSE,
    LiveObservation,
)
from app.navigation.robot_backend import RobotCapabilities
from app.navigation.safety_models import (
    SECTOR_COUNT,
    SECTOR_WIDTH_DEG,
    STATE_CLEAR,
    build_local_safety_snapshot,
    evaluate_perception_quality,
    wrap_deg,
)
from app.navigation.viewpoint_planner import ViewpointPlanner, ViewpointPlannerConfig

WALL_X = 2.0


def _vertical_wall(x: float, *, span: float = 6.0, step: float = 0.02):
    count = int(span / step) + 1
    return [(x, -span / 2.0 + index * step) for index in range(count)]


def _planner() -> ViewpointPlanner:
    # 2m 走廊 + footprint 0.25 的有效净空只有 0.75m，低于偏好净空 0.8m；
    # 单元测试统一用 0.20 让 "偏好净空达标" 这件事可被断言。
    return ViewpointPlanner(ViewpointPlannerConfig(footprint_radius_m=0.20))


def _observation() -> LiveObservation:
    return LiveObservation(bundle_id="b1", timestamp=1.0, heading_sector=0,
                           pose={"x": 0.0, "y": 0.0, "yaw_rad": 0.0})


def _frontier(frontier_id: str, position) -> dict:
    return {"frontier_id": frontier_id, "label": "F01", "position": position,
            "status": "OPEN", "information_gain": 0.6,
            "semantic_relevance": 0.2, "failure_count": 0}


def _metric_caps() -> RobotCapabilities:
    return RobotCapabilities(supports_global_pose=True,
                             supports_metric_navigation=True)


def _generate(*, clearance_map=None, frontiers=(), safety_rejections=None,
              capabilities=None):
    return generate_live_exploration_candidates(
        observation=_observation(),
        graph=ExplorationGraph(session_id="test"),
        capabilities=capabilities or _metric_caps(),
        clearance_map=clearance_map,
        viewpoint_planner=_planner() if clearance_map is not None else None,
        frontiers=frontiers,
        robot_pose=(0.0, 0.0, 0.0),
        safety_rejections=safety_rejections,
    )


# 场景 1：frontier 质心贴在墙上 -> 目标是退开的观察位姿，不是质心。
def test_wall_frontier_yields_standoff_goal() -> None:
    clearance_map = clearance_map_from_points(
        _vertical_wall(WALL_X), known_bounds=(-3.0, -3.0, 3.0, 3.0))
    centroid = (1.9, 0.0)
    # 前提：质心本身站不住，这正是 "直接去质心" 会撞墙的原因。
    assert (clearance_map.pose_clearance(centroid, 0.20) or 0.0) < 0.40
    goals = _generate(clearance_map=clearance_map,
                      frontiers=[_frontier("frontier:p1:00", centroid)])
    viewpoints = [goal for goal in goals
                  if goal.provenance.get("source") == "frontier_viewpoint"]
    assert len(viewpoints) == 1
    goal = viewpoints[0]
    assert goal.goal_type == GOAL_NAVIGATE_POSE
    assert goal.target_node_id == "frontier:p1:00"
    # 目标点被推离墙面，且自身净空过硬边界。
    assert math.hypot(goal.position[0] - centroid[0],
                      goal.position[1] - centroid[1]) >= 0.60
    assert (clearance_map.pose_clearance(goal.position, 0.20) or 0.0) >= 0.40
    provenance = goal.provenance
    assert provenance["route_reachable"] is True
    assert provenance["path_min_clearance_m"] >= 0.40
    assert provenance["local_clearance_m"] >= 0.40
    for key in ("viewpoint_id", "view_quality", "camera_occlusion_risk",
                "route_cost_m", "spatial_information_gain", "viewpoint_score",
                "traversal_failure_count", "viewpoint_components"):
        assert key in provenance


# 场景 2：几何未知的 frontier -> 只能进拒绝证据，不许变成目标。
def test_unobserved_frontier_becomes_safety_rejection() -> None:
    # 已知区域只有身边 0.5m，frontier 在 3m 外的未知空间里。
    clearance_map = clearance_map_from_points(
        [], known_bounds=(-0.5, -0.5, 0.5, 0.5))
    rejections: list[dict] = []
    goals = _generate(clearance_map=clearance_map,
                      frontiers=[_frontier("frontier:p1:06", (3.0, 0.0))],
                      safety_rejections=rejections)
    assert not [goal for goal in goals
                if goal.provenance.get("source") == "frontier_viewpoint"]
    assert len(rejections) == 1
    entry = rejections[0]
    assert entry["frontier_id"] == "frontier:p1:06"
    assert entry["reason"].startswith("UNKNOWN_")
    assert entry["rejected"], "拒绝证据必须带上被拒候选，便于复盘"


# 场景 3：没有净空地图（旧会话/自定义生成器）-> 候选与改造前一致。
def test_without_clearance_map_no_viewpoint_candidates() -> None:
    goals = _generate(frontiers=[_frontier("frontier:p1:00", (1.9, 0.0))])
    assert not [goal for goal in goals
                if goal.provenance.get("source") == "frontier_viewpoint"]
    assert goals, "legacy 扇区候选仍应照常产生"


def _snapshot():
    evidence = [{"bearing_deg": wrap_deg(index * SECTOR_WIDTH_DEG),
                 "clearance_m": 2.5, "state": STATE_CLEAR}
                for index in range(SECTOR_COUNT)]
    return build_local_safety_snapshot(sector_evidence=evidence,
                                      lidar_fresh=True,
                                      geometry_validated=True)


def _perception(observation, gain):
    return evaluate_perception_quality(
        depth_valid_ratio=0.9, near_field_ratio=0.1, visual_feature_count=250,
        stable_object_count=5, estimated_information_gain=float(gain),
        front_clearance_m=2.5)


# 场景 4：整链路 E2E -> 安全/感知/视点事件与 §29 决策字段必须都在。
def test_explorer_emits_safety_and_viewpoint_events() -> None:
    scene = scenario_no_target(empty_scenes=3)
    policy = load_exploration_policy()
    policy.budget = ExplorationBudget(max_motion_steps=6,
                                      max_planning_cycles=6)
    snapshot = _snapshot()
    explorer = AutonomousExplorer(
        target="蓝色垃圾桶",
        observer=scene.observer(),
        matcher=scene.matcher(),
        verifier=scene.verifier(),
        backend=MockBackend(),
        policy=policy,
        graph=ExplorationGraph(session_id="test"),
        negative_target_key="蓝色垃圾桶",
        local_safety_provider=lambda obs: snapshot,
        perception_quality_provider=_perception,
    )
    explorer.run()
    events = {event["event"] for event in explorer.events}
    assert "local_safety_snapshot" in events
    assert "perception_quality" in events
    assert "viewpoint_candidates" in events
    assert "viewpoint_selected" in events
    assert "llm_embodiment_context" in events
    context = next(event for event in explorer.events
                   if event["event"] == "llm_embodiment_context")
    assert context["local_safety"]["lidar_fresh"] is True
    assert context["recovery_context"]["stuck_state"]
    assert explorer.decision_records
    record = explorer.decision_records[0].to_dict()
    assert record["local_safety_summary"] is not None
    assert record["perception_quality"] is not None
    assert "hard_safety_rejections" in record
    chosen = [item for item in explorer.decision_records
              if item.selected_viewpoint_id]
    assert chosen, "至少一个决策应选中 standoff 视点"
    assert chosen[0].path_min_clearance_m is not None
    assert chosen[0].selected_frontier_id
