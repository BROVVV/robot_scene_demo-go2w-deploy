"""计划书 §21/§22/§23 + §32.9：目标接近控制器的七条硬线。

这些用例钉住的是「怎么都不许发生」的事：
1. 没有 RGBD 真值就不许前进（禁止固定 1m 启发式）；
2. 偏航大先转，转向余量为 0 就判 BLOCKED；
3. 前向 swept 未知/不安全/余量不足时一律不前进；
4. 到达判定必须九项全过，缺一条就不算到达。
"""

from __future__ import annotations

from app.navigation.approach_controller import (
    ACTION_APPROACH_BLOCKED,
    ACTION_APPROACH_COMPLETE,
    ACTION_APPROACH_FORWARD,
    ACTION_ROTATE_SMALL,
    ACTION_STOP_REOBSERVE,
    ACTION_TARGET_REACHED_OR_TOO_CLOSE,
    decide_approach,
    evaluate_target_reached,
    robot_bearing_from_camera,
)
from app.navigation.safety_models import (
    SECTOR_COUNT,
    SECTOR_WIDTH_DEG,
    STATE_CLEAR,
    STATE_OCCUPIED,
    STATE_UNKNOWN,
    build_local_safety_snapshot,
    evaluate_perception_quality,
    wrap_deg,
)
from app.navigation.search_policy_config import TargetApproachConfig
from app.navigation.viewpoint_models import TargetApproachContext

POLICY = TargetApproachConfig()


def _snapshot(*, front_clearance_m: float = 3.0, front_state: str = STATE_CLEAR,
              side_clearance_m: float = 2.5,
              lidar_fresh: bool = True,
              geometry_validated: bool = True):
    """24 扇区全部有观测；前向 ±45° 可单独设成 OCCUPIED/UNKNOWN。"""
    evidence = []
    for index in range(SECTOR_COUNT):
        bearing = wrap_deg(index * SECTOR_WIDTH_DEG)
        is_front = abs(bearing) <= 45.0
        evidence.append({
            "bearing_deg": bearing,
            "clearance_m": front_clearance_m if is_front else side_clearance_m,
            "state": front_state if is_front else STATE_CLEAR,
        })
    return build_local_safety_snapshot(
        sector_evidence=evidence, lidar_fresh=lidar_fresh,
        geometry_validated=geometry_validated, pose_quality="relative")


def _blocked_turn_snapshot():
    """只有正前方那一格空旷，相邻扇区全被占据 -> 左右转向余量都是 0。"""
    evidence = []
    for index in range(SECTOR_COUNT):
        bearing = wrap_deg(index * SECTOR_WIDTH_DEG)
        if bearing == 0.0:
            evidence.append({"bearing_deg": bearing, "clearance_m": 3.0,
                             "state": STATE_CLEAR})
        else:
            evidence.append({"bearing_deg": bearing, "clearance_m": 0.15,
                             "state": STATE_OCCUPIED})
    return build_local_safety_snapshot(
        sector_evidence=evidence, lidar_fresh=True, geometry_validated=True)


def _perception(*, target_visible: bool = True, target_depth_valid: bool = True,
                front_clearance_m: float = 3.0):
    return evaluate_perception_quality(
        depth_valid_ratio=0.9, near_field_ratio=0.1, visual_feature_count=250,
        stable_object_count=5, estimated_information_gain=0.6,
        front_clearance_m=front_clearance_m,
        target_visible=target_visible, target_depth_valid=target_depth_valid)


def _context(*, depth_m: float | None = 3.0, bearing_deg: float = 0.0,
             spatial_quality: str = "CAMERA_LOCAL"):
    return TargetApproachContext(
        object_id="obj_1", label="蓝色垃圾桶", depth_m=depth_m,
        bearing_deg=bearing_deg, spatial_quality=spatial_quality,
        confidence=0.9, last_seen_timestamp=1.0)


# 相机右为正 -> 机器人左为正，符号必须翻转，否则会朝反方向转。
def test_camera_bearing_is_mirrored_into_robot_frame() -> None:
    assert robot_bearing_from_camera(15.0) == -15.0
    assert robot_bearing_from_camera(-15.0) == 15.0
    assert robot_bearing_from_camera(None) is None


# 3m 正前方 -> 取 coarse 步长 0.25m。
def test_far_target_takes_coarse_step() -> None:
    decision = decide_approach(_context(depth_m=3.0), safety=_snapshot(),
                               perception=_perception(), policy=POLICY)
    assert decision.action == ACTION_APPROACH_FORWARD
    assert decision.relative_dx == POLICY.max_approach_step_m == 0.25
    assert decision.relative_dyaw_deg == 0.0
    # 终点必须仍在最小 standoff 之外，目标表面永远不在这一步的路径上。
    assert 3.0 - decision.relative_dx > POLICY.minimum_standoff_m


# bearing 15° > 8° 阈值 -> 先转，绝不带着偏航前进。
def test_off_axis_target_turns_before_moving() -> None:
    decision = decide_approach(_context(depth_m=3.0, bearing_deg=15.0),
                               safety=_snapshot(), perception=_perception(),
                               policy=POLICY)
    assert decision.action == ACTION_ROTATE_SMALL
    assert decision.relative_dx == 0.0
    assert decision.relative_dyaw_deg == 15.0
    right = decide_approach(_context(depth_m=3.0, bearing_deg=-15.0),
                            safety=_snapshot(), perception=_perception(),
                            policy=POLICY)
    assert right.relative_dyaw_deg == -15.0


# 该转的方向没有余量 -> BLOCKED，不许硬转也不许直接前进。
def test_turn_without_clearance_is_blocked() -> None:
    decision = decide_approach(_context(depth_m=3.0, bearing_deg=20.0),
                               safety=_blocked_turn_snapshot(),
                               perception=_perception(), policy=POLICY)
    assert decision.action == ACTION_APPROACH_BLOCKED
    assert decision.reason.startswith("ROTATION_NOT_CLEAR")


# 1.0m -> fine 步长；0.95m 已落在 preferred 附近 -> 直接算走到位。
def test_near_target_uses_fine_step_then_completes() -> None:
    fine = decide_approach(_context(depth_m=1.0), safety=_snapshot(),
                           perception=_perception(), policy=POLICY)
    assert fine.action == ACTION_APPROACH_FORWARD
    assert fine.relative_dx == POLICY.fine_approach_step_m == 0.10

    done = decide_approach(_context(depth_m=0.95), safety=_snapshot(),
                           perception=_perception(), policy=POLICY)
    assert done.action == ACTION_APPROACH_COMPLETE
    assert done.reached is True
    assert done.relative_dx == 0.0


# 0.65m 已经进到最小 standoff 以内 -> 不许再前进。
def test_too_close_target_never_moves_forward() -> None:
    decision = decide_approach(_context(depth_m=0.65), safety=_snapshot(),
                               perception=_perception(), policy=POLICY)
    assert decision.action == ACTION_TARGET_REACHED_OR_TOO_CLOSE
    assert decision.relative_dx == 0.0
    assert decision.reached is False


# 目标丢失 -> 停车重新观察，且标记 lost_target。
def test_lost_target_stops_and_reobserves() -> None:
    decision = decide_approach(_context(depth_m=2.0), safety=_snapshot(),
                               perception=_perception(target_visible=False),
                               policy=POLICY)
    assert decision.action == ACTION_STOP_REOBSERVE
    assert decision.lost_target is True
    assert decision.relative_dx == 0.0

    explicit = decide_approach(_context(depth_m=2.0), safety=_snapshot(),
                               perception=_perception(), policy=POLICY,
                               target_lost=True)
    assert explicit.action == ACTION_STOP_REOBSERVE


# 深度不新鲜 / 没有深度 / RGB_ONLY -> 一律不前进（§19 禁止固定 1m）。
def test_missing_or_stale_depth_never_moves_forward() -> None:
    stale = decide_approach(_context(depth_m=2.0), safety=_snapshot(),
                            perception=_perception(target_depth_valid=False),
                            policy=POLICY)
    assert stale.action == ACTION_STOP_REOBSERVE
    assert stale.reason.startswith("STALE_TARGET_DEPTH")

    no_depth = decide_approach(_context(depth_m=None), safety=_snapshot(),
                               perception=_perception(), policy=POLICY)
    assert no_depth.action == ACTION_STOP_REOBSERVE

    rgb_only = decide_approach(
        _context(depth_m=2.0, spatial_quality="RGB_ONLY"), safety=_snapshot(),
        perception=_perception(), policy=POLICY)
    assert rgb_only.action == ACTION_STOP_REOBSERVE
    assert "spatial_quality=RGB_ONLY" in rgb_only.reason


# 前向不安全 / 未知 / 未验证几何 -> 不前进，UNKNOWN 绝不当空旷。
def test_unsafe_or_unknown_front_blocks_approach() -> None:
    occupied = decide_approach(
        _context(depth_m=2.0),
        safety=_snapshot(front_clearance_m=0.30, front_state=STATE_OCCUPIED),
        perception=_perception(front_clearance_m=0.30), policy=POLICY)
    assert occupied.action == ACTION_APPROACH_BLOCKED
    assert occupied.reason.startswith("FORWARD_NOT_SAFE")

    unknown = decide_approach(
        _context(depth_m=2.0),
        safety=_snapshot(front_clearance_m=None, front_state=STATE_UNKNOWN),
        perception=_perception(), policy=POLICY)
    assert unknown.action == ACTION_APPROACH_BLOCKED
    assert unknown.reason.startswith("UNKNOWN_FORWARD_GEOMETRY")

    unvalidated = decide_approach(
        _context(depth_m=2.0), safety=_snapshot(geometry_validated=False),
        perception=_perception(), policy=POLICY)
    assert unvalidated.action == ACTION_APPROACH_BLOCKED
    assert unvalidated.reason.startswith("UNVALIDATED_LOCAL_GEOMETRY")


# swept 净空扣掉步长后低于硬边界 -> 不许走这一步。
def test_step_larger_than_hard_clearance_margin_is_blocked() -> None:
    decision = decide_approach(
        _context(depth_m=2.0), safety=_snapshot(front_clearance_m=0.55),
        perception=_perception(front_clearance_m=0.55), policy=POLICY)
    assert decision.action == ACTION_APPROACH_BLOCKED
    assert decision.reason.startswith("STEP_EXCEEDS_HARD_CLEARANCE")


# §23：九项全过才算到达。
def test_target_reached_requires_every_condition() -> None:
    reached, reason = evaluate_target_reached(
        _context(depth_m=0.85, bearing_deg=3.0), safety=_snapshot(),
        perception=_perception(), verified_again=True, robot_stationary=True,
        policy=POLICY)
    assert reached is True
    assert reason == "TARGET_REACHED"

    for kwargs, expected in (
        ({"verified_again": False}, "FINAL_VERIFICATION_MISSING"),
        ({"robot_stationary": False}, "ROBOT_NOT_STATIONARY"),
    ):
        ok, why = evaluate_target_reached(
            _context(depth_m=0.85, bearing_deg=3.0), safety=_snapshot(),
            perception=_perception(), policy=POLICY,
            **{"verified_again": True, "robot_stationary": True, **kwargs})
        assert ok is False and why == expected

    far, why = evaluate_target_reached(
        _context(depth_m=1.60, bearing_deg=3.0), safety=_snapshot(),
        perception=_perception(), verified_again=True, robot_stationary=True,
        policy=POLICY)
    assert far is False and why == "DEPTH_OUT_OF_STANDOFF_RANGE"

    skew, why = evaluate_target_reached(
        _context(depth_m=0.85, bearing_deg=30.0), safety=_snapshot(),
        perception=_perception(), verified_again=True, robot_stationary=True,
        policy=POLICY)
    assert skew is False and why == "BEARING_OUT_OF_RANGE"

    weak, why = evaluate_target_reached(
        _context(depth_m=0.85, bearing_deg=3.0, spatial_quality="RGB_ONLY"),
        safety=_snapshot(), perception=_perception(), verified_again=True,
        robot_stationary=True, policy=POLICY)
    assert weak is False and why.startswith("SPATIAL_QUALITY_TOO_WEAK")
