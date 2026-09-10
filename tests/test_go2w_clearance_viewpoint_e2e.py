"""计划书 §33：GO2W「高净空视觉友好导航 + 目标接近」端到端验收。

六条链路各钉一件事，全程 mock 传感/底盘，不碰真机：

1. 走廊里目标点必须是走中线的 standoff 视点，而不是 frontier 质心；
2. 墙角 + 相机贴墙 -> 必须脱困，绝不判搜索穷尽，也绝不盲目前进；
3. frontier 质心站不住 -> 退开的观察位姿，站不住的 frontier 变成拒绝证据；
4. 3m 外的目标 -> 多步接近到 standoff，停稳后最终确认才算找到；
5. 目标贴墙 -> 从开放侧接近，贴墙侧候选被硬安全拒掉；
6. 上面这些本体证据必须原样出现在 WebUI 快照里。
"""

from __future__ import annotations

import json
import math
import time
from typing import Any

from app.live_robot.autonomous_explorer import (
    AutonomousExplorer,
    SemanticMatch,
    VerificationOutcome,
)
from app.live_robot.explorer_search_adapter import ExplorerSearchAdapter
from app.live_robot.recovery_motion import SafeMotionSegment
from app.live_robot.search_event_bus import SearchEventBus
from app.live_robot.search_state_store import SearchStateStore
from app.navigation.backend_factory import (
    MockBackend,
    MockBackendConfig,
    MockMetricBackend,
)
from app.navigation.clearance_map import clearance_map_from_points
from app.navigation.exploration_config import (
    ExplorationBudget,
    load_exploration_policy,
)
from app.navigation.exploration_graph import ExplorationGraph
from app.navigation.models import GOAL_NAVIGATE_POSE, LiveObservation
from app.navigation.recovery_manager import RecoveryManager
from app.navigation.robot_backend import NavigationStatus
from app.navigation.safety_models import (
    FRONT_HALF_ANGLE_DEG,
    SECTOR_COUNT,
    SECTOR_WIDTH_DEG,
    STATE_CLEAR,
    STATE_OCCUPIED,
    build_local_safety_snapshot,
    evaluate_perception_quality,
    wrap_deg,
)
from app.navigation.stuck_detector import StuckState

TARGET = "蓝色垃圾桶"


# ---------------------------------------------------------------------------
# 场景积木
# ---------------------------------------------------------------------------


def _vertical_wall(x: float, *, y_min: float, y_max: float, step: float = 0.05):
    count = int(round((y_max - y_min) / step)) + 1
    return [(x, y_min + index * step) for index in range(count)]


def _horizontal_wall(y: float, *, x_min: float, x_max: float, step: float = 0.05):
    count = int(round((x_max - x_min) / step)) + 1
    return [(x_min + index * step, y) for index in range(count)]


def _corridor_points(*, half_width: float = 1.2, x_min: float = -1.0,
                     x_max: float = 5.0):
    return (_horizontal_wall(half_width, x_min=x_min, x_max=x_max)
            + _horizontal_wall(-half_width, x_min=x_min, x_max=x_max))


def _snapshot(*, front_clearance_m: float = 2.5,
              blocked_half_angle_deg: float = 0.0,
              lidar_fresh: bool = True, geometry_validated: bool = True,
              pose_quality: str = "metric"):
    """24 扇区证据：正前方一段可以按需压低，其余保持开阔。"""
    evidence = []
    for index in range(SECTOR_COUNT):
        bearing = wrap_deg(index * SECTOR_WIDTH_DEG)
        if blocked_half_angle_deg > 0.0 and abs(bearing) <= blocked_half_angle_deg:
            state, clearance = STATE_OCCUPIED, front_clearance_m
        elif abs(bearing) <= FRONT_HALF_ANGLE_DEG:
            state, clearance = STATE_CLEAR, front_clearance_m
        else:
            state, clearance = STATE_CLEAR, 2.5
        evidence.append({"bearing_deg": bearing, "clearance_m": clearance,
                         "state": state})
    return build_local_safety_snapshot(
        sector_evidence=evidence, lidar_fresh=lidar_fresh,
        geometry_validated=geometry_validated, pose_quality=pose_quality)


def _perception(*, camera_blocked: bool, gain: float = 0.6,
                target_visible: bool = False, target_depth_valid: bool = False,
                front_clearance_m: float = 2.5):
    if camera_blocked:
        return evaluate_perception_quality(
            depth_valid_ratio=0.4, near_field_ratio=0.9,
            visual_feature_count=40, stable_object_count=0,
            estimated_information_gain=0.02, front_clearance_m=0.28,
            target_visible=target_visible,
            target_depth_valid=target_depth_valid)
    return evaluate_perception_quality(
        depth_valid_ratio=0.9, near_field_ratio=0.1,
        visual_feature_count=250, stable_object_count=5,
        estimated_information_gain=float(gain),
        front_clearance_m=front_clearance_m,
        target_visible=target_visible, target_depth_valid=target_depth_valid)


def _frontier(frontier_id: str, position, *, gain: float = 0.6) -> dict:
    return {"frontier_id": frontier_id, "label": frontier_id[-3:],
            "position": list(position), "status": "OPEN",
            "information_gain": gain, "semantic_relevance": 0.2,
            "failure_count": 0}


class FrontierPin:
    """把拓扑自动补出的 0.8m 近距 frontier 换成本场景要考察的开口。

    等价于「身边这些方向都已经看过了，只剩下面这些没探」，让断言能落在
    「净空怎么选点」上，而不是被自动 frontier 的几何巧合牵着走。
    每个规划周期都要重钉：``update_observation`` 会围着机器人重新补一圈近距
    frontier，只在 run() 之前钉一次会被冲掉。净空 provider 正好在候选生成之前
    调用，所以钉在这里就够了。
    """

    def __init__(self, entries, inner) -> None:
        self.entries = list(entries)
        self.inner = inner
        self.explorer: AutonomousExplorer | None = None

    def __call__(self, observation, safety):
        frontiers = self.explorer.semantic_graph.frontiers
        frontiers.clear()
        for entry in self.entries:
            frontiers[entry["frontier_id"]] = dict(entry)
        return self.inner(observation, safety)


def _events(explorer: AutonomousExplorer, name: str) -> list:
    return [event for event in explorer.events if event["event"] == name]


def _names(explorer: AutonomousExplorer) -> list:
    return [event["event"] for event in explorer.events]


def _budget(**kwargs) -> ExplorationBudget:
    defaults = dict(max_motion_steps=8, max_planning_cycles=3,
                    max_search_seconds=120.0)
    defaults.update(kwargs)
    return ExplorationBudget(**defaults)


class SearchScene:
    """一个可编排的 mock 感知源：走廊里探索，若干帧之后目标出现。

    ``target_x`` 是目标在 map 系的 x（y=0）；depth 由 backend 里程直接推出，
    所以「走近了 depth 就变小」这件事不需要额外脚本。
    """

    def __init__(self, backend, *, target_x: float = 3.0,
                 spatial_quality: str = "CAMERA_LOCAL",
                 visible_after: int = 0, obstacles=(),
                 known_bounds=None) -> None:
        self.backend = backend
        self.target_x = float(target_x)
        self.spatial_quality = spatial_quality
        self.visible_after = int(visible_after)
        self.obstacles = list(obstacles)
        self.known_bounds = known_bounds
        self.frames = 0

    # ---- 真值 ---------------------------------------------------------
    def pose_xy_yaw(self):
        pose = self.backend.get_pose()
        return float(pose.x), float(pose.y), float(pose.yaw)

    def depth_m(self) -> float:
        x, y, _ = self.pose_xy_yaw()
        return round(math.hypot(self.target_x - x, 0.0 - y), 3)

    def target_visible(self) -> bool:
        return self.frames > self.visible_after

    # ---- 注入给 explorer 的 callable ----------------------------------
    def observer(self):
        def observe() -> LiveObservation:
            self.frames += 1
            x, y, yaw = self.pose_xy_yaw()
            pose = {"x": x, "y": y, "yaw_rad": yaw}
            objects: list = [{"label": "wall", "label_zh": "墙面",
                              "name": "wall", "confidence": 0.8,
                              "id": f"wall_{self.frames:03d}"}]
            match: dict = {"target_present": False, "score": 0.0}
            if self.target_visible():
                depth = self.depth_m()
                objects.append({
                    "label": TARGET, "label_zh": TARGET, "name": TARGET,
                    "category": "target", "confidence": 0.95,
                    "id": f"frame_obj_{self.frames:03d}",
                    "frame_object_id": f"frame_obj_{self.frames:03d}",
                    "bbox_2d": [0.45, 0.35, 0.55, 0.65],
                    "depth_m": depth, "bearing_deg": 0.0,
                    # camera_xyz = (lateral, vertical, forward)
                    "camera_xyz": [0.0, 0.0, depth],
                    "spatial_quality": self.spatial_quality,
                })
                match = {"target_present": True, "score": 0.95}
            return LiveObservation(
                bundle_id=f"scene_{self.frames:03d}",
                timestamp=float(self.frames),
                scene_objects=objects, scene_relations=[],
                target_match=match, heading_sector=0,
                spatial_quality=self.spatial_quality,
                pose=pose, sensor_health={"camera": True, "lidar": True})
        return observe

    def matcher(self):
        def match(observation: LiveObservation) -> SemanticMatch:
            present = bool((observation.target_match or {}).get("target_present"))
            return SemanticMatch(
                has_candidate=present,
                target_score=0.95 if present else 0.0,
                target_match_level="candidate" if present else "none",
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
            # 走廊里前向净空就取「到目标/走廊尽头的距离」，够开阔。
            front = 2.5 if not self.target_visible() else max(0.05, self.depth_m())
            return _snapshot(front_clearance_m=front)
        return provide

    def perception_provider(self):
        def provide(observation: LiveObservation, gain: float):
            visible = self.target_visible()
            front = 2.5 if not visible else max(0.05, self.depth_m())
            return _perception(camera_blocked=False, gain=float(gain),
                               target_visible=visible,
                               target_depth_valid=visible,
                               front_clearance_m=front)
        return provide

    def clearance_map_provider(self):
        def provide(observation, safety):
            x, y, _ = self.pose_xy_yaw()
            return clearance_map_from_points(
                self.obstacles, known_bounds=self.known_bounds, origin=(x, y))
        return provide


# ---------------------------------------------------------------------------
# E2E-1：走廊 -> 中线 standoff 视点，全程不脱困
# ---------------------------------------------------------------------------
def test_corridor_center_viewpoint_without_recovery() -> None:
    backend = MockMetricBackend()
    obstacles = _corridor_points()
    snapshot = _snapshot()
    policy = load_exploration_policy()
    policy.budget = _budget(max_planning_cycles=1)
    # 走廊尽头是唯一开口：frontier 质心在 (3, 0)，两侧墙在 y=±1.2。
    pin = FrontierPin(
        [_frontier("frontier:p1:00", (3.0, 0.0))],
        lambda obs, safety: clearance_map_from_points(
            obstacles, known_bounds=(-2.0, -2.0, 6.0, 2.0)))
    explorer = AutonomousExplorer(
        target=TARGET,
        observer=SearchScene(backend, visible_after=99).observer(),
        matcher=lambda obs: SemanticMatch(has_candidate=False, target_score=0.0,
                                          target_match_level="none",
                                          target_match={}),
        verifier=lambda obs, match: VerificationOutcome(
            confirmed=False, attempts=1, reason_zh="没有目标"),
        backend=backend,
        policy=policy,
        graph=ExplorationGraph(session_id="e2e_corridor"),
        negative_target_key=TARGET,
        local_safety_provider=lambda obs: snapshot,
        perception_quality_provider=lambda obs, gain: _perception(
            camera_blocked=False, gain=float(gain)),
        clearance_map_provider=pin,
    )
    pin.explorer = explorer
    explorer.run()

    selected = _events(explorer, "selected_goal")
    assert selected, "走廊里必须能选出目标"
    goal = selected[0]["goal"]
    assert goal["goal_type"] == GOAL_NAVIGATE_POSE
    provenance = goal["provenance"]
    assert provenance["source"] == "frontier_viewpoint"
    # 走中线：偏移 ±0.3 的候选路径净空只有 0.65m，达不到 0.8m 偏好净空。
    assert abs(goal["position"][1]) <= 0.05, goal["position"]
    assert provenance["path_min_clearance_m"] >= 0.80
    assert provenance["local_clearance_m"] >= 0.80
    # 站位是退开的观察点，不是 frontier 质心本身。
    assert goal["position"][0] < 2.95

    names = _names(explorer)
    assert "viewpoint_selected" in names
    assert "recovery_triggered" not in names, "开阔走廊里不该脱困"
    assert "search_exhausted" not in names


# ---------------------------------------------------------------------------
# E2E-2：墙角 + 相机贴墙 -> 脱困，不判穷尽，也不盲目前进
# ---------------------------------------------------------------------------
class CornerScene:
    """机器人怼进墙角：前方 0.30m、相机被近景糊住。

    ``opened`` 一旦置位就代表「已经退开了」，几何重新打开。
    """

    def __init__(self, backend) -> None:
        self.backend = backend
        self.opened = False

    def safety_provider(self):
        def provide(observation):
            if self.opened:
                return _snapshot()
            # 只压住正前方那一个扇区：前向不安全，但左右仍有转身余量，
            # 这样 stuck 归因才会落到「相机被挡」而不是「彻底困住」。
            return _snapshot(front_clearance_m=0.30, blocked_half_angle_deg=5.0)
        return provide

    def perception_provider(self):
        def provide(observation, gain):
            return _perception(camera_blocked=not self.opened, gain=float(gain))
        return provide

    def breadcrumb_provider(self):
        def provide():
            # 面包屑记的是「刚刚在 yaw=0 时向前走了 0.30m 怼进墙角」这一段既有
            # 事实，是静态的：真机上它由上一次前进确认后写下，不会随之后的转向
            # 改写。朝向差因此由脱困当时的实际位姿去比对，正是安全门要判的东西。
            return SafeMotionSegment(
                start_pose=(-0.30, 0.0, 0.0),
                end_pose=(0.0, 0.0, 0.0),
                signed_distance_m=0.30, heading_rad=0.0, confirmed=True,
                created_monotonic_s=time.monotonic(),
                source_step="mock_forward")
        return provide


def test_corner_camera_blocked_recovers_instead_of_exhausted() -> None:
    # 第一个目标失败 -> 触发 §13.2 的导航失败脱困；之后一律成功。
    backend = MockBackend(outcome_sequence=[NavigationStatus.FAILED,
                                           NavigationStatus.SUCCEEDED])
    scene = CornerScene(backend)
    search = SearchScene(backend, visible_after=99)
    policy = load_exploration_policy()
    policy.budget = _budget(max_planning_cycles=3)
    explorer = AutonomousExplorer(
        target=TARGET,
        observer=search.observer(),
        matcher=lambda obs: SemanticMatch(has_candidate=False, target_score=0.0,
                                          target_match_level="none",
                                          target_match={}),
        verifier=lambda obs, match: VerificationOutcome(
            confirmed=False, attempts=1, reason_zh="没有目标"),
        backend=backend,
        policy=policy,
        graph=ExplorationGraph(session_id="e2e_corner"),
        negative_target_key=TARGET,
        local_safety_provider=scene.safety_provider(),
        perception_quality_provider=scene.perception_provider(),
        breadcrumb_provider=scene.breadcrumb_provider(),
        recovery_manager=RecoveryManager(),
    )
    # 退开动作一落地就把场景切成「已经退开」，模拟退后之后几何重新打开。
    original_hook = explorer._on_event

    def hook(event):
        if event["event"] == "recovery_backward_result":
            scene.opened = True
        original_hook(event)

    explorer._on_event = hook
    explorer.run()

    names = _names(explorer)
    assert "search_exhausted" not in names, "相机被挡绝不能当搜索穷尽"

    stuck = _events(explorer, "stuck_detected")
    assert stuck, "导航失败 + 前方受阻必须先归因"
    assert stuck[0]["stuck_state"] in {
        StuckState.CAMERA_OCCLUDED.value, StuckState.FRONT_BLOCKED.value,
        StuckState.CORNERED.value}

    triggered = _events(explorer, "recovery_triggered")
    assert triggered, "必须显式发脱困决策"
    decision = triggered[0]["decision"]
    assert decision["backward_allowed"] is True, decision["reason"]
    backward = decision["backward_goal"]
    assert -0.12 <= backward["relative_dx"] <= -0.05, backward
    assert _events(explorer, "recovery_reobserve"), "退开后必须强制重观察"

    # 前方受阻时绝不允许出现盲目前进的兜底候选。
    first_cycle = _events(explorer, "candidates")[0]
    dumped = json.dumps(first_cycle["candidates"], ensure_ascii=False)
    assert "fallback_forward" not in dumped

    # 退开之后几何打开，搜索继续往下走而不是原地卡死。
    assert _events(explorer, "selected_goal")


# ---------------------------------------------------------------------------
# E2E-3：frontier 质心站不住 -> standoff 视点 + 拒绝证据
# ---------------------------------------------------------------------------
def test_unsafe_frontier_centroid_becomes_standoff_viewpoint() -> None:
    backend = MockMetricBackend()
    wall = _vertical_wall(2.0, y_min=-3.0, y_max=3.0)
    clearance_map = clearance_map_from_points(
        wall, known_bounds=(-3.0, -3.0, 3.0, 3.0))
    centroid = (1.9, 0.0)
    # 前提：质心自己就站不住，这正是「直接去质心」会撞墙的原因。
    assert (clearance_map.pose_clearance(centroid, 0.25) or 0.0) < 0.40

    snapshot = _snapshot()
    policy = load_exploration_policy()
    policy.budget = _budget(max_planning_cycles=1)
    pin = FrontierPin([
        _frontier("frontier:p1:00", centroid),
        # 3m 外的未知空间：几何都没观测到，只能进拒绝证据。
        _frontier("frontier:p1:06", (5.0, 0.0)),
    ], lambda obs, safety: clearance_map)
    explorer = AutonomousExplorer(
        target=TARGET,
        observer=SearchScene(backend, visible_after=99).observer(),
        matcher=lambda obs: SemanticMatch(has_candidate=False, target_score=0.0,
                                          target_match_level="none",
                                          target_match={}),
        verifier=lambda obs, match: VerificationOutcome(
            confirmed=False, attempts=1, reason_zh="没有目标"),
        backend=backend,
        policy=policy,
        graph=ExplorationGraph(session_id="e2e_standoff"),
        negative_target_key=TARGET,
        local_safety_provider=lambda obs: snapshot,
        perception_quality_provider=lambda obs, gain: _perception(
            camera_blocked=False, gain=float(gain)),
        clearance_map_provider=pin,
    )
    pin.explorer = explorer
    explorer.run()

    selected = _events(explorer, "selected_goal")
    assert selected
    goal = selected[0]["goal"]
    assert goal["provenance"]["source"] == "frontier_viewpoint"
    position = goal["position"]
    assert math.hypot(position[0] - centroid[0],
                      position[1] - centroid[1]) >= 0.60, position
    assert (clearance_map.pose_clearance(position, 0.25) or 0.0) >= 0.40

    rejected = _events(explorer, "candidate_rejected_safety")
    assert rejected, "站不住的 frontier 必须留下拒绝证据，不许静默消失"
    ids = {item["frontier_id"] for item in rejected[0]["rejections"]}
    assert "frontier:p1:06" in ids
    entry = next(item for item in rejected[0]["rejections"]
                 if item["frontier_id"] == "frontier:p1:06")
    assert entry["reason"].startswith("UNKNOWN_")
    assert entry["rejected"], "拒绝证据要带上被拒候选，便于复盘"


# ---------------------------------------------------------------------------
# E2E-4：3m 外的目标 -> 多步接近 + 最终确认
# ---------------------------------------------------------------------------
def _approach_explorer(scene: SearchScene, **kwargs) -> AutonomousExplorer:
    policy = load_exploration_policy()
    policy.budget = _budget(max_planning_cycles=kwargs.pop("max_planning_cycles", 3))
    return AutonomousExplorer(
        target=TARGET,
        observer=scene.observer(),
        matcher=scene.matcher(),
        verifier=scene.verifier(),
        backend=scene.backend,
        policy=policy,
        graph=ExplorationGraph(session_id="e2e_approach"),
        session_id="e2e_approach",
        negative_target_key=TARGET,
        finish_on_visual_confirmation=False,
        local_safety_provider=scene.safety_provider(),
        perception_quality_provider=scene.perception_provider(),
        **kwargs,
    )


def test_distant_target_multi_step_approach() -> None:
    backend = MockBackend(config=MockBackendConfig(forward_step_m=0.30))
    scene = SearchScene(backend, target_x=3.0, spatial_quality="CAMERA_LOCAL")
    explorer = _approach_explorer(scene)
    result = explorer.run()

    names = _names(explorer)
    assert "target_confirmed" in names
    assert names.index("target_confirmed") < names.index("target_found"), \
        "视觉确认只是接近的开始，不是终点"
    assert names.index("final_verification") < names.index("target_found")

    steps = _events(explorer, "approach_step")
    forward = [item for item in steps if item["action"] == "APPROACH_FORWARD"]
    assert len(forward) >= 5, f"3m 目标至少要走好几步，实际 {len(forward)}"
    assert forward[0]["relative_dx"] == 0.25, "远距离用 coarse 步长"
    assert any(item["relative_dx"] == 0.10 for item in forward), "近距离切 fine 步长"
    assert steps[-1]["action"] == "APPROACH_COMPLETE"

    final = _events(explorer, "final_verification")[-1]
    assert final["reached"] is True
    assert 0.70 <= final["target"]["depth_m"] <= 1.10, final["target"]
    assert result.result == "TARGET_FOUND"


# ---------------------------------------------------------------------------
# E2E-5：目标贴墙 -> 从开放侧接近
# ---------------------------------------------------------------------------
def test_target_against_wall_approaches_from_open_side() -> None:
    backend = MockBackend(config=MockBackendConfig(forward_step_m=0.30))
    # 目标在 (3.0, 0)，墙在 x=3.30：贴墙只剩 0.30m，机器人只能从来的那侧看。
    # 目标本体故意不进障碍点云 —— standoff 环本身保证不会撞上目标，把目标
    # 塞进点云反而会让所有候选失去视线（§44：不许为接近目标删 LiDAR 点，
    # 也不许把目标当墙）。
    wall = _vertical_wall(3.30, y_min=-2.0, y_max=2.0)
    scene = SearchScene(backend, target_x=3.0, spatial_quality="METRIC_RGBD",
                        obstacles=wall, known_bounds=(-1.0, -2.0, 4.0, 2.0))
    explorer = _approach_explorer(
        scene, clearance_map_provider=scene.clearance_map_provider())
    explorer.run()

    picks = _events(explorer, "approach_candidate_selected")
    assert picks, "metric 目标必须排 standoff 环视点"
    first = picks[0]
    selected = first["selected"]
    assert selected is not None, "开放侧应当有硬安全的接近位姿"
    assert selected["hard_safe"] is True
    assert selected["line_of_sight"] is True
    assert 0.70 <= selected["subject_distance_m"] <= 1.10, selected
    assert selected["local_clearance_m"] >= 0.40
    # 从机器人来的那一侧接近，不许绕到目标和墙之间的 0.30m 夹缝里。
    assert selected["pose"][0] < 3.0, selected["pose"]

    rejected = [item for item in first["candidates"] if not item["hard_safe"]]
    assert any(item["rejection_reason"].startswith("LOCAL_CLEARANCE_TOO_LOW")
               for item in rejected), "贴墙侧候选必须被硬净空拒掉"


# ---------------------------------------------------------------------------
# E2E-6：本体证据必须原样到 WebUI 快照
# ---------------------------------------------------------------------------
def test_embodiment_snapshot_reaches_webui() -> None:
    backend = MockMetricBackend()
    obstacles = _corridor_points()
    # 第 1 帧只有走廊（走 PLAN 选视点），之后目标出现走接近闭环。
    scene = SearchScene(backend, target_x=3.0, spatial_quality="METRIC_RGBD",
                        visible_after=1, obstacles=obstacles,
                        known_bounds=(-2.0, -2.0, 6.0, 2.0))
    store = SearchStateStore()
    bus = SearchEventBus()
    adapter = ExplorerSearchAdapter(bus, store, session_id="e2e_webui")
    pin = FrontierPin([_frontier("frontier:p1:00", (3.0, 0.0))],
                      scene.clearance_map_provider())
    explorer = _approach_explorer(
        scene, clearance_map_provider=pin,
        on_event=adapter.on_explorer_event)
    pin.explorer = explorer
    explorer.run()

    embodiment = store.snapshot()["embodiment"]
    for key in ("local_safety", "stuck_state", "selected_viewpoint",
                "recovery", "target_approach"):
        assert key in embodiment, key
    assert embodiment["local_safety"]["lidar_fresh"] is True
    assert embodiment["local_safety"]["forward_safe"] is True
    assert embodiment["selected_viewpoint"]["viewpoint_id"], embodiment
    assert embodiment["selected_viewpoint"]["path_min_clearance_m"] >= 0.40
    assert embodiment["target_approach"]["status"] == "REACHED", embodiment
    assert store.snapshot()["status"] == "TARGET_FOUND"
