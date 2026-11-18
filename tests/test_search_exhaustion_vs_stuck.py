"""计划书 §12/§32：低信息不等于搜索穷尽。

"什么都没看到" 必须先归因到物理原因（相机贴墙 / 前方受阻 / 传感器退化 /
还有 frontier 没走），只有真正没有可达 frontier 且空间证据稳定时才允许
SEARCH_EXHAUSTED。
"""

from __future__ import annotations

from dataclasses import replace

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
    # **形状必须和生产一致**：``SemanticNavigationGraph.frontiers`` 声明的是
    # ``dict[str, dict[str, Any]]``，唯一的写入点
    # （``app/spatial/semantic_navigation_graph.py:387``）也写普通 dict。
    #
    # 这里以前写的是 ``SimpleNamespace(status="OPEN")`` —— 于是一个只在**对象**上
    # 成立的 ``getattr(frontier, "status", "")`` 在这个夹具上照样绿，而它在生产
    # 的 dict 上恒得 "" ⇒ ``open_frontiers`` 永远为空 ⇒ REPLAN 那一支不可达、
    # 直接判 SEARCH_EXHAUSTED。**夹具编码了 bug 的形状**，和
    # ``MockBackend``（相对后端）测不出 nav2（地图系后端）是同一类错误：
    # 判据没错，校准在了错的那一臂。
    for index in range(open_frontiers):
        explorer.semantic_graph.frontiers[f"f{index}"] = {
            "frontier_id": f"f{index}", "state": "OPEN", "status": "OPEN",
        }
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


# 配对负控：前沿的“OPEN”判定必须真的有分辨力 —— 走完的前沿不算还能去。
# 没有这一条，把 ``open_frontiers`` 改成 `len(frontiers)` 也能让上面那条变绿。
#
# 注：这是对**判据**的分辨力测试，不是在描述生产行为。拓扑前沿的 ``state`` /
# ``status`` 全仓库只有一个写入点（``semantic_navigation_graph.py:387``），写死
# "OPEN"、之后**没有任何代码把它翻成 VISITED**；被 ``mark_frontier_visited``
# 标成 VISITED 的是 ``SpatialMemory.frontiers``（另一套账）。
# 后果：``_classify_no_information`` 里 `open_frontiers` 在生产上恒 ≥1
# ⇒ 穷尽这一支实际由 ``_no_information_replans < 2`` 那个上限兜住
# （先重规划 2 轮、再穷尽）。方向是 fail-safe（推后收工，不会提前收工）。
def test_visited_frontier_is_not_an_open_frontier() -> None:
    explorer = _explorer(safety=_snapshot(), open_frontiers=2)
    explorer.semantic_graph.frontiers["f0"]["state"] = "VISITED"
    explorer.semantic_graph.frontiers["f0"]["status"] = "VISITED"
    decision = explorer._classify_no_information(_observation(), None, 0)
    assert decision["decision"] == "REPLAN"
    assert decision["open_frontiers"] == 1, "只该剩下没走的那一个"

    explorer.semantic_graph.frontiers["f1"]["status"] = "VISITED"
    decision = explorer._classify_no_information(_observation(), None, 0)
    assert decision["decision"] == "SEARCH_EXHAUSTED"
    assert decision["open_frontiers"] == 0


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


def test_preflight_rejection_replans_without_physical_recovery() -> None:
    """A rejected candidate that never sent an Action must not move the dog."""

    class PreflightRejectBackend(MockBackend):
        def execute_goal(self, goal):
            before = list(self._pose)
            handle = super().execute_goal(goal)
            self._pose[:] = before
            handle.result = replace(
                handle.result,
                observed_motion={},
                provenance={
                    **handle.result.provenance,
                    "motion_action_sent": False,
                    "reject_reason": "TOO_NARROW",
                },
            )
            self._executed[-1] = handle.result
            return handle

    scene = scenario_target_appears_after(3)
    backend = PreflightRejectBackend(
        outcome_sequence=[NavigationStatus.REJECTED]
    )
    rejected = []
    explorer = AutonomousExplorer(
        target="蓝色垃圾桶",
        observer=scene.observer(),
        matcher=scene.matcher(),
        verifier=scene.verifier(),
        backend=backend,
        policy=load_exploration_policy(),
        graph=ExplorationGraph(session_id="preflight-reject"),
        negative_target_key="蓝色垃圾桶",
        local_safety_provider=lambda obs: _snapshot(),
        perception_quality_provider=lambda obs, gain: _perception(
            camera_blocked=False
        ),
        preflight_rejection_handler=lambda goal, result: rejected.append(
            (goal.goal_id, result.provenance["reject_reason"])
        ),
    )

    result = explorer.run()
    events = [event["event"] for event in explorer.events]
    assert result.result == "TARGET_FOUND"
    assert result.navigation_failures == 0
    assert rejected
    assert "candidate_preflight_rejected" in events
    assert "recovery_triggered" not in events
    assert all(item.provenance["motion_action_sent"] is False
               for item in backend._executed)


# 普通导航失败不自动等于“需要物理脱困”。RecoveryManager 已明确返回
# trigger="" / no recovery trigger 时，探索器只能停稳并重规划，不能自行补一个
# escape turn。现场反例会白白消耗一次不可退款的运动账本。
def test_navigation_failure_without_recovery_trigger_does_not_move() -> None:
    from app.navigation.backend_factory import MockMetricBackend

    scene = scenario_target_appears_after(3)
    backend = MockMetricBackend(
        outcome_sequence=[NavigationStatus.FAILED, NavigationStatus.SUCCEEDED])
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
    recorded: list[tuple[str, dict]] = []
    original = explorer._record_execution_history

    def spy(goal, pose, result):
        original(goal, pose, result)
        recorded.append((goal.goal_id, dict(explorer._command_history[-1])))

    explorer._record_execution_history = spy  # type: ignore[method-assign]
    explorer.run()

    assert recorded, "整场一条运动都没执行"
    escape_ids = [gid for gid, _ in recorded if gid.endswith("_escape_turn")]
    assert escape_ids == [], (
        "RecoveryManager returned trigger='' / no recovery trigger, but an "
        f"escape turn was still executed: {escape_ids}"
    )
    recovery = next(
        event for event in explorer.events if event["event"] == "recovery_result"
    )
    assert recovery["executed"] is None
    assert recovery["reason"] == "no recovery trigger"


# 倒车脱困是同一条纪律的另一半：位姿也必须取在下发之前。
#
# 地图系后端下倒车目标就是面包屑的 ``start_pose``（``recovery_manager`` 的
# ``_backward_goal``），``MockMetricBackend`` 成功时把位姿**精确搬到**目标点 ——
# 取在执行之后，记下来的就是"最后停在离目标多远"＝0，脱困倒车在
# ``detect_stuck`` 的命令/位姿窗口里彻底消失（和原地打转漏数是同一个病）。
def test_metric_backward_recovery_records_the_commanded_backoff() -> None:
    from app.live_robot.autonomous_explorer import _MIN_COMMANDED_TRANSLATION_M
    from app.live_robot.recovery_motion import SafeMotionSegment
    from app.navigation.backend_factory import MockMetricBackend

    scene = scenario_target_appears_after(3)
    backend = MockMetricBackend(
        outcome_sequence=[NavigationStatus.FAILED, NavigationStatus.SUCCEEDED])
    # 正前方受阻才会真正触发脱困（``safety`` 全清时 ``_trigger`` 返回空串，
    # ``decide`` 直接给 "no recovery trigger"，倒车支路根本走不到）。
    safety = _snapshot(blocked_half_angle_deg=20.0)
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
        # 面包屑是倒车唯一的合法走廊：没有它 ``backward_allowed`` 恒为 False。
        breadcrumb_provider=lambda: SafeMotionSegment(
            start_pose=(-0.5, 0.0, 0.0),
            end_pose=(0.0, 0.0, 0.0),
            signed_distance_m=0.5,
            heading_rad=0.0,
            source_step="FORWARD",
        ),
    )
    recorded: list[tuple[str, dict]] = []
    original = explorer._record_execution_history

    def spy(goal, pose, result):
        original(goal, pose, result)
        recorded.append((goal.goal_id, dict(explorer._command_history[-1])))

    explorer._record_execution_history = spy  # type: ignore[method-assign]
    explorer.run()

    backward = [entry for gid, entry in recorded
                if gid.startswith("recovery_") and not gid.endswith("_escape_turn")]
    assert backward, f"倒车脱困没有进命令窗口：{[gid for gid, _ in recorded]}"
    # 命令量＝"从当前位姿挪到面包屑起点"，0.5 m ≠ 0。
    assert backward[-1]["distance_m"] > _MIN_COMMANDED_TRANSLATION_M
