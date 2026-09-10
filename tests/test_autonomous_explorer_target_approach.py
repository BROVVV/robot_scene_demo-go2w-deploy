"""计划书 §18/§21/§23 + §32.11：视觉确认只是接近的开始，不是终点。

四个 case 钉住整条状态机：
1. `finish_on_visual_confirmation=False` -> 确认后必须走完接近 + 最终确认才 TARGET_FOUND；
2. legacy flag=True -> 老行为一字不动（远处看到就成功）；
3. 确认了但深度不新鲜 -> 只许停车重观察，绝不许成功；
4. 接近途中目标丢失 -> 进 lost 分支，也绝不许成功。
"""

from __future__ import annotations

from typing import Any

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
