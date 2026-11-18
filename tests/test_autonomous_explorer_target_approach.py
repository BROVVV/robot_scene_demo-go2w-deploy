"""计划书 §18/§21/§23 + §32.11：视觉确认只是接近的开始，不是终点。

四个 case 钉住整条状态机：
1. `finish_on_visual_confirmation=False` -> 确认后必须走完接近 + 最终确认才 TARGET_FOUND；
2. legacy flag=True -> 老行为一字不动（远处看到就成功）；
3. 确认了但深度不新鲜 -> 只许停车重观察，绝不许成功；
4. 接近途中目标丢失 -> 进 lost 分支，也绝不许成功。
"""

from __future__ import annotations

import math
from typing import Any

import pytest

from app.live_robot.autonomous_explorer import (
    AutonomousExplorer,
    SemanticMatch,
    VerificationOutcome,
)
from app.navigation.backend_factory import MockBackend, MockBackendConfig
from app.navigation.exploration_config import (
    ExplorationBudget,
    load_exploration_policy,
)
from app.navigation.exploration_graph import ExplorationGraph
from app.navigation.models import LiveObservation
from app.navigation.safety_models import (
    SECTOR_COUNT,
    SECTOR_WIDTH_DEG,
    STATE_CLEAR,
    build_local_safety_snapshot,
    evaluate_perception_quality,
    wrap_deg,
)

TARGET = "蓝色垃圾桶"
START_DEPTH_M = 3.0


class ApproachScene:
    """目标固定在正前方 3m；depth 由 mock backend 的里程直接推出来。

    ``visible_frames`` 之后目标不再可见（case 4），``depth_valid`` 关掉表示
    深度不新鲜（case 3）。
    """

    def __init__(self, *, visible_frames: int = 999, depth_valid: bool = True,
                 bearing_deg: float = 0.0) -> None:
        self.backend = MockBackend(config=MockBackendConfig(forward_step_m=0.30))
        self.visible_frames = int(visible_frames)
        self.depth_valid = bool(depth_valid)
        self.bearing_deg = float(bearing_deg)
        self.frames = 0

    # ---- 真值 -----------------------------------------------------------

    def depth_m(self) -> float:
        pose = self.backend.get_pose()
        return round(START_DEPTH_M - float(pose.x), 3)

    def target_visible(self) -> bool:
        return self.frames <= self.visible_frames

    # ---- 注入给 explorer 的四个 callable --------------------------------

    def observer(self):
        def observe() -> LiveObservation:
            self.frames += 1
            depth = self.depth_m()
            target_object: dict[str, Any] = {
                "label": TARGET, "label_zh": TARGET, "name": TARGET,
                "category": "target", "confidence": 0.95,
                "id": f"frame_obj_{self.frames:03d}",
                "frame_object_id": f"frame_obj_{self.frames:03d}",
                "bbox_2d": [0.45, 0.35, 0.55, 0.65],
                "depth_m": depth, "bearing_deg": -self.bearing_deg,
                "camera_xyz": [0.0, 0.0, depth],
                "spatial_quality": "CAMERA_LOCAL",
            }
            return LiveObservation(
                bundle_id=f"approach_{self.frames:03d}",
                timestamp=float(self.frames),
                scene_objects=[target_object],
                scene_relations=[],
                target_match={"target_present": True, "score": 0.95},
                heading_sector=0,
                spatial_quality="CAMERA_LOCAL",
                sensor_health={"camera": True, "lidar": True},
            )
        return observe

    def matcher(self):
        def match(observation: LiveObservation) -> SemanticMatch:
            return SemanticMatch(has_candidate=True, target_score=0.95,
                                 target_match_level="candidate",
                                 target_match=observation.target_match)
        return match

    def verifier(self):
        def verify(observation: LiveObservation,
                   match: SemanticMatch) -> VerificationOutcome:
            return VerificationOutcome(confirmed=True, attempts=1,
                                       reason_zh="mock 确认目标")
        return verify

    def safety_provider(self):
        def provide(observation: LiveObservation):
            # 前向净空就是目标距离：LiDAR 看到的就是目标那个面。
            front = max(0.05, self.depth_m())
            evidence = []
            for index in range(SECTOR_COUNT):
                bearing = wrap_deg(index * SECTOR_WIDTH_DEG)
                evidence.append({
                    "bearing_deg": bearing,
                    "clearance_m": front if abs(bearing) <= 45.0 else 2.5,
                    "state": STATE_CLEAR,
                })
            return build_local_safety_snapshot(
                sector_evidence=evidence, timestamp=observation.timestamp,
                lidar_fresh=True, geometry_validated=True,
                pose_quality="relative")
        return provide

    def perception_provider(self):
        def provide(observation: LiveObservation, gain: float):
            visible = self.target_visible()
            return evaluate_perception_quality(
                depth_valid_ratio=0.9, near_field_ratio=0.1,
                visual_feature_count=260, stable_object_count=4,
                estimated_information_gain=float(gain),
                front_clearance_m=max(0.05, self.depth_m()),
                target_visible=visible,
                target_depth_valid=visible and self.depth_valid)
        return provide


def _explorer(scene: ApproachScene, *, finish_on_visual_confirmation: bool,
              max_planning_cycles: int = 3) -> AutonomousExplorer:
    policy = load_exploration_policy()
    policy.budget = ExplorationBudget(max_motion_steps=6,
                                      max_planning_cycles=max_planning_cycles,
                                      max_search_seconds=120.0)
    return AutonomousExplorer(
        target=TARGET,
        observer=scene.observer(),
        matcher=scene.matcher(),
        verifier=scene.verifier(),
        backend=scene.backend,
        policy=policy,
        graph=ExplorationGraph(session_id="test_approach"),
        session_id="test_approach",
        negative_target_key=TARGET,
        finish_on_visual_confirmation=finish_on_visual_confirmation,
        local_safety_provider=scene.safety_provider(),
        perception_quality_provider=scene.perception_provider(),
    )


def _events(explorer: AutonomousExplorer, name: str) -> list[dict[str, Any]]:
    return [event for event in explorer.events if event["event"] == name]


# Case 1：确认后不得立刻成功，必须一步步走到 standoff 再最终确认。
def test_confirmation_triggers_approach_before_target_found() -> None:
    scene = ApproachScene()
    explorer = _explorer(scene, finish_on_visual_confirmation=False)
    result = explorer.run()

    names = [event["event"] for event in explorer.events]
    assert "target_confirmed" in names
    # target_confirmed 必须排在 target_found 之前，中间夹着接近过程。
    assert names.index("target_confirmed") < names.index("target_found")
    assert names.index("final_verification") < names.index("target_found")
    assert _events(explorer, "target_localized")
    assert _events(explorer, "target_reached")

    steps = _events(explorer, "approach_step")
    forward = [item for item in steps if item["action"] == "APPROACH_FORWARD"]
    assert len(forward) >= 5, f"3m 目标至少要走好几步，实际 {len(forward)}"
    assert forward[0]["relative_dx"] == 0.25, "远距离必须用 coarse 步长"
    assert any(item["relative_dx"] == 0.10 for item in forward), "近距离要切 fine 步长"
    assert steps[-1]["action"] == "APPROACH_COMPLETE"

    # 最终停在 standoff 区间内，且路上任何一步都没有越过硬净空。
    final = _events(explorer, "final_verification")[-1]
    assert final["reached"] is True
    assert final["reason"] == "TARGET_REACHED"
    assert 0.70 <= final["target"]["depth_m"] <= 1.10
    for item in forward:
        assert item["details"]["forward_swept_clearance_m"] - item["relative_dx"] >= 0.40

    assert result.result == "TARGET_FOUND"
    assert scene.backend._stop_called is True


def test_session_start_records_nonlegacy_approach_contract() -> None:
    explorer = _explorer(ApproachScene(), finish_on_visual_confirmation=False)
    explorer.run()

    started = _events(explorer, "session_start")[0]
    assert started["finish_on_visual_confirmation"] is False
    assert started["target_approach_enabled"] is True


# Case 2：legacy flag 打开 -> 老行为完全不变，一步都不许走。
def test_legacy_flag_keeps_finish_on_visual_confirmation() -> None:
    scene = ApproachScene()
    explorer = _explorer(scene, finish_on_visual_confirmation=True)
    result = explorer.run()

    names = [event["event"] for event in explorer.events]
    assert "target_found" in names
    assert "target_confirmed" not in names
    assert not _events(explorer, "approach_step")
    assert not _events(explorer, "final_verification")
    assert result.result == "TARGET_FOUND"
    assert scene.depth_m() == START_DEPTH_M, "legacy 模式不许产生任何位移"


# Case 3：确认了但深度不新鲜 -> 只能停车重观察，绝不许判成功。
def test_stale_depth_after_confirmation_never_succeeds() -> None:
    scene = ApproachScene(depth_valid=False)
    explorer = _explorer(scene, finish_on_visual_confirmation=False,
                         max_planning_cycles=1)
    result = explorer.run()

    assert "target_confirmed" in [event["event"] for event in explorer.events]
    steps = _events(explorer, "approach_step")
    assert steps, "至少要走一次接近决策"
    assert {item["action"] for item in steps} == {"STOP_REOBSERVE"}
    assert all(item["reason"].startswith("STALE_TARGET_DEPTH") for item in steps)
    aborted = _events(explorer, "approach_aborted")
    assert aborted, "接近失败必须显式发 approach_aborted"
    assert aborted[-1]["decision_reason"].startswith("STALE_TARGET_DEPTH")
    assert not _events(explorer, "target_found")
    assert not _events(explorer, "target_reached")
    assert result.result != "TARGET_FOUND"
    assert scene.depth_m() == START_DEPTH_M, "没有有效深度就不许有任何位移"


# Case 4：接近途中目标丢失 -> 进 lost 分支并放弃这次接近。
def test_target_lost_during_approach_enters_lost_strategy() -> None:
    scene = ApproachScene(visible_frames=2)
    explorer = _explorer(scene, finish_on_visual_confirmation=False,
                         max_planning_cycles=1)
    result = explorer.run()

    steps = _events(explorer, "approach_step")
    lost = [item for item in steps if item["lost_target"]]
    assert lost, "目标丢失必须走 STOP_REOBSERVE 分支"
    assert lost[0]["action"] == "STOP_REOBSERVE"
    assert lost[0]["relative_dx"] == 0.0
    aborted = _events(explorer, "approach_aborted")
    assert aborted[-1]["reason"] == "TARGET_LOST_DURING_APPROACH"
    assert not _events(explorer, "target_found")
    assert result.result != "TARGET_FOUND"


# ---------------------------------------------------------------------------
# 计划书 §3.2：位姿这一帧丢了，不能当成「机器人在原点」
#
# `_pose_xy_yaw` 对缺失字段一律回落到 0，于是「位姿真的是原点」和「这帧没有
# 位姿」在它的返回值上长得一模一样。凡是拿它去**发地图系目标**的地方，都必须
# 先问 `_has_xy`——否则位姿一丢，§6.7 的原地观察转向就变成「开回地图原点」。
# ---------------------------------------------------------------------------


def test_pose_reader_distinguishes_the_origin_from_a_missing_pose() -> None:
    """判据本身：原点返回 True，缺位姿返回 False —— 这正是 _pose_xy_yaw 做不到的。"""
    from app.live_robot.autonomous_explorer import _has_xy, _pose_xy_yaw

    # 两件事的**读取结果**完全相同，所以只看 _pose_xy_yaw 分不出来。
    assert _pose_xy_yaw(None) == _pose_xy_yaw({"x": 0.0, "y": 0.0, "yaw_rad": 0.0}) == (
        0.0, 0.0, 0.0)
    assert _has_xy({"x": 0.0, "y": 0.0}) is True, "真的站在原点是个合法位姿"
    assert _has_xy(None) is False
    assert _has_xy({}) is False
    assert _has_xy({"yaw_rad": 1.0}) is False, "只有朝向没有坐标，等于没有位姿"
    assert _has_xy({"x": None, "y": 2.0}) is False


def test_plan_context_never_hands_a_synthesized_origin_to_the_generator() -> None:
    """没有位姿时交给候选生成器的必须是 None，不是一个编出来的原点。

    分辨力在于两遍跑的是**同一个** explorer、同一份 frontier/净空地图，只换
    观测里有没有坐标：有坐标那遍必须真的交出 (x, y)，没有那遍必须交 None。
    少了前半句，一个恒交 None 的实现也能让这条测试变绿。
    """
    from app.navigation.clearance_map import clearance_map_from_local_safety
    from app.navigation.safety_models import (
        SECTOR_COUNT,
        SECTOR_WIDTH_DEG,
        STATE_CLEAR,
        build_local_safety_snapshot,
    )

    scene = ApproachScene()
    explorer = _explorer(scene, finish_on_visual_confirmation=False)

    evidence = [
        {"sector": index, "state": STATE_CLEAR, "distance_m": 3.0,
         "bearing_deg": wrap_deg(index * SECTOR_WIDTH_DEG)}
        for index in range(SECTOR_COUNT)
    ]
    snapshot = build_local_safety_snapshot(
        sector_evidence=evidence, timestamp=1_700_000_000.0,
        lidar_fresh=True, geometry_validated=True, pose_quality="metric")
    clearance_map = clearance_map_from_local_safety(snapshot)
    # 前端点非空 —— `_candidate_plan_context` 里那道闸就是这次回归的现场。
    explorer.semantic_graph.frontiers["frontier:p0:00"] = {
        "frontier_id": "frontier:p0:00", "position": [2.0, 0.0],
        "status": "OPEN", "state": "OPEN", "failure_count": 0,
    }
    perception = scene.perception_provider()(
        LiveObservation(bundle_id="b", timestamp=1_700_000_000.0, pose=None), 0.0)

    def _context(pose):
        observation = LiveObservation(
            bundle_id="b", timestamp=1_700_000_000.0, pose=pose)
        return explorer._candidate_plan_context(
            observation=observation, safety=snapshot, perception=perception,
            clearance_map=clearance_map, safety_rejections=[])

    located = _context({"x": 1.5, "y": -2.0, "yaw_rad": 0.3})
    assert located["robot_pose"] == (1.5, -2.0, 0.3), located["robot_pose"]

    missing = _context(None)
    assert missing.get("frontiers"), "这遍必须真的走进 frontier 分支，否则什么也没测"
    assert "robot_pose" in missing
    assert missing["robot_pose"] is None, (
        "位姿丢了就不能交出一个原点冒充机器人位置："
        f"{missing['robot_pose']!r}"
    )


def test_metric_approach_never_rotates_toward_the_map_origin() -> None:
    """§6.7 的「原地转向」在拿不到位姿时必须不出目标，而不是转到 (0, 0)。

    对照那遍用真实位姿：同一个 decision 必须真的产出以机器人为原地的
    NAVIGATE_POSE —— 少了它，一个恒返回 None 的实现也能让这条测试变绿。
    """
    from app.navigation.models import LiveObservation as _Obs

    class _Decision:
        action = "ROTATE_SMALL"
        relative_dyaw_deg = 30.0
        reason = "turn toward target"
        def to_dict(self):
            return {"action": self.action}

    def _goal(pose):
        return AutonomousExplorer._metric_approach_goal(
            _Decision(), _Obs(bundle_id="b", timestamp=1.0, pose=pose),
            None, object_id="o1", steps=1)

    located = _goal({"x": 1.5, "y": -2.0, "yaw_rad": math.radians(17.0)})
    assert located is not None, "有位姿时这条转向必须表达得出来"
    assert located.goal_type == "NAVIGATE_POSE"
    assert located.position == (1.5, -2.0)
    assert located.yaw == pytest.approx(17.0 + 30.0)
    assert located.frame == "map"

    assert _goal(None) is None, "没有位姿就不许编一个原点当转向目标"
    assert _goal({"yaw_rad": math.radians(17.0)}) is None
