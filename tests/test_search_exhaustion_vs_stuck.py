"""计划书 §12/§32：低信息不等于搜索穷尽。

"什么都没看到" 必须先归因到物理原因（相机贴墙 / 前方受阻 / 传感器退化 /
还有 frontier 没走），只有真正没有可达 frontier 且空间证据稳定时才允许
SEARCH_EXHAUSTED。
"""

from __future__ import annotations

from types import SimpleNamespace

from app.live_robot.autonomous_explorer import AutonomousExplorer
from app.live_robot.mock_observation_scene import (
    MockObservationScene,
    MockSceneStep,
    scenario_target_appears_after,
)
from app.navigation.backend_factory import MockBackend
from app.navigation.exploration_config import load_exploration_policy
from app.navigation.exploration_graph import ExplorationGraph
from app.navigation.models import LiveObservation
from app.navigation.robot_backend import NavigationStatus
from app.navigation.safety_models import (
    SECTOR_COUNT,
    SECTOR_WIDTH_DEG,
    STATE_CLEAR,
    STATE_OCCUPIED,
    build_local_safety_snapshot,
    evaluate_perception_quality,
    wrap_deg,
)
from app.navigation.stuck_detector import StuckState


def _snapshot(*, blocked_half_angle_deg: float = 0.0, lidar_fresh: bool = True,
              geometry_validated: bool = True):
    evidence = []
    for index in range(SECTOR_COUNT):
        bearing = wrap_deg(index * SECTOR_WIDTH_DEG)
        if blocked_half_angle_deg > 0.0 and abs(bearing) <= blocked_half_angle_deg:
            state, clearance = STATE_OCCUPIED, 0.30
        else:
            state, clearance = STATE_CLEAR, 2.5
        evidence.append({"bearing_deg": bearing, "clearance_m": clearance,
                         "state": state})
    return build_local_safety_snapshot(
        sector_evidence=evidence, lidar_fresh=lidar_fresh,
        geometry_validated=geometry_validated)


def _perception(*, camera_blocked: bool):
    if camera_blocked:
        return evaluate_perception_quality(
            depth_valid_ratio=0.4, near_field_ratio=0.9,
            visual_feature_count=40, stable_object_count=0,
            estimated_information_gain=0.02, front_clearance_m=0.28)
    return evaluate_perception_quality(
        depth_valid_ratio=0.9, near_field_ratio=0.1,
        visual_feature_count=250, stable_object_count=5,
        estimated_information_gain=0.6, front_clearance_m=2.5)


def _explorer(*, safety=None, camera_blocked: bool = False, backend=None,
              open_frontiers: int = 0, **kwargs) -> AutonomousExplorer:
    scene = MockObservationScene(scenes=[MockSceneStep(objects=["wall"])])
    explorer = AutonomousExplorer(
        target="蓝色垃圾桶",
        observer=scene.observer(),
        matcher=scene.matcher(),
        verifier=scene.verifier(),
        backend=backend or MockBackend(),
        policy=load_exploration_policy(),
        graph=ExplorationGraph(session_id="test"),
        negative_target_key="蓝色垃圾桶",
        local_safety_provider=(None if safety is None else (lambda obs: safety)),
        perception_quality_provider=(
            lambda obs, gain: _perception(camera_blocked=camera_blocked)),
        **kwargs,
    )
    for index in range(open_frontiers):
        explorer.semantic_graph.frontiers[f"f{index}"] = SimpleNamespace(status="OPEN")
    return explorer


def _observation() -> LiveObservation:
    return LiveObservation(bundle_id="b1", timestamp=1.0, heading_sector=0,
                           pose={"x": 0.0, "y": 0.0, "yaw_deg": 0.0})


# 场景 1：相机贴墙 -> 必须要求脱困，绝不判搜索穷尽。
def test_camera_blocked_is_not_exhaustion() -> None:
    explorer = _explorer(safety=_snapshot(), camera_blocked=True)
    decision = explorer._classify_no_information(_observation(), None, 0)
    assert decision["decision"] == "RECOVERY_REQUIRED"
    assert decision["stuck_state"] is StuckState.CAMERA_OCCLUDED


# 场景 2：前方受阻 -> 走 recovery，而不是穷尽。
def test_front_blocked_requires_recovery() -> None:
    explorer = _explorer(safety=_snapshot(blocked_half_angle_deg=10.0),
                         camera_blocked=False)
    decision = explorer._classify_no_information(_observation(), None, 0)
    assert decision["decision"] == "RECOVERY_REQUIRED"
    assert decision["stuck_state"] is StuckState.FRONT_BLOCKED


# 传感器退化 -> 等地图，不许盲退也不许判穷尽。
def test_sensor_degraded_waits_for_map() -> None:
    explorer = _explorer(safety=_snapshot(lidar_fresh=False))
    decision = explorer._classify_no_information(_observation(), None, 0)
    assert decision["decision"] == "WAITING_FOR_MAP"
    assert decision["stuck_state"] is StuckState.SENSOR_DEGRADED


# 场景 3：仍有 OPEN frontier -> 重新规划。
def test_open_frontier_replans() -> None:
    explorer = _explorer(safety=_snapshot(), open_frontiers=2)
    decision = explorer._classify_no_information(_observation(), None, 0)
    assert decision["decision"] == "REPLAN"
    assert decision["open_frontiers"] == 2


# 场景 4：无 frontier、证据稳定 -> 才是真正的穷尽。
def test_no_frontier_is_exhaustion() -> None:
    explorer = _explorer(safety=_snapshot())
    decision = explorer._classify_no_information(_observation(), None, 0)
    assert decision["decision"] == "SEARCH_EXHAUSTED"
    assert decision["stuck_state"] is StuckState.FREE


# REPLAN 有上限，不允许无限空转。
def test_replan_budget_falls_back_to_exhaustion() -> None:
    explorer = _explorer(safety=_snapshot(), open_frontiers=1)
    explorer._no_information_replans = 2
    decision = explorer._classify_no_information(_observation(), None, 0)
    assert decision["decision"] == "SEARCH_EXHAUSTED"


# 没有 LiDAR 证据注入时保持旧行为（不许凭空自主脱困）。
def test_without_safety_provider_keeps_legacy_behaviour() -> None:
    explorer = _explorer(safety=None)
    decision = explorer._classify_no_information(_observation(), None, 0)
    assert decision == {"decision": "SEARCH_EXHAUSTED", "stuck_state": None}


# 导航失败必须先跑 §13.2 的脱困流程再重规划。
def test_navigation_failure_runs_recovery_before_replan() -> None:
    scene = scenario_target_appears_after(3)
    backend = MockBackend(outcome_sequence=[NavigationStatus.FAILED])
    safety = _snapshot()
    explorer = AutonomousExplorer(
        target="蓝色垃圾桶",
        observer=scene.observer(),
        matcher=scene.matcher(),
        verifier=scene.verifier(),
        backend=backend,
        policy=load_exploration_policy(),
        graph=ExplorationGraph(session_id="test"),
        negative_target_key="蓝色垃圾桶",
        local_safety_provider=lambda obs: safety,
        perception_quality_provider=lambda obs, gain: _perception(camera_blocked=False),
    )
    explorer.run()
    events = [event["event"] for event in explorer.events]
    assert "recovery_triggered" in events
    assert "recovery_result" in events
    # 没有 breadcrumb -> 禁止盲退，只能原地安全转向。
    triggered = next(event for event in explorer.events
                     if event["event"] == "recovery_triggered")
    assert triggered["decision"]["backward_allowed"] is False
    assert events.index("recovery_triggered") < events.index("replan")
