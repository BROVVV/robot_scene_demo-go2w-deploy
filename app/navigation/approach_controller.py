"""计划书 §21/§22/§23：目标接近控制器。

只做一件事：把「目标此刻的 depth/bearing + 本地安全快照」变成「下一个极小的动作」。

三条不可动摇的前提：

1. 距离永远来自 RGB-D 真值。没有有效 depth 就只能 stop/reobserve，
   绝不允许用固定 1m 之类的启发式距离驱动前进（§19）。
2. 计划步长永远小于 ``depth - minimum_standoff``，所以目标自身表面
   永远落在这一步的终点之外——这就是第一版不做点云 cluster 关联、
   也不会「因为看见目标而永远不敢接近」的原因（§22.2）。
3. 前进前必须过 forward 硬门（几何已验证 + swept clearance 够），
   UNKNOWN 一律当作不可通行。

角度约定：本模块内部所有 bearing / dyaw 都是**机器人系**，左正右负，
与 ``safety_models`` 的扇区和 ``ExplorationGoal.relative_dyaw`` 一致。
相机给出的 bearing 是「图像右为正」，进来之前必须过
``robot_bearing_from_camera()`` 取反。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .robot_backend import RobotCapabilities
from .safety_models import LocalSafetySnapshot, PerceptionQuality
from .search_policy_config import ClearanceNavigationConfig, TargetApproachConfig
from .viewpoint_models import TargetApproachContext

ACTION_STOP_REOBSERVE = "STOP_REOBSERVE"
ACTION_ROTATE_SMALL = "ROTATE_SMALL"
ACTION_APPROACH_FORWARD = "APPROACH_FORWARD"
ACTION_TARGET_REACHED_OR_TOO_CLOSE = "TARGET_REACHED_OR_TOO_CLOSE"
ACTION_APPROACH_COMPLETE = "APPROACH_COMPLETE"
ACTION_APPROACH_BLOCKED = "APPROACH_BLOCKED"

# §19.1：只有这三档 spatial quality 才算「知道目标真实距离」。
# RGB_ONLY / SEMANTIC_2D_ONLY 一律不能驱动前进，也不能判 TARGET_REACHED。
APPROACH_CAPABLE_SPATIAL_QUALITY = ("METRIC_RGBD", "RELATIVE_RGBD", "CAMERA_LOCAL")


def robot_bearing_from_camera(camera_bearing_deg: float | None) -> float | None:
    """相机 bearing（图像右为正）→ 机器人 bearing（左为正）。"""
    if camera_bearing_deg is None:
        return None
    return -float(camera_bearing_deg)


@dataclass
class ApproachDecision:
    action: str
    relative_dx: float = 0.0
    relative_dyaw_deg: float = 0.0
    reached: bool = False
    lost_target: bool = False
    reason: str = ""
    details: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "action": self.action,
            "relative_dx": round(self.relative_dx, 3),
            "relative_dyaw_deg": round(self.relative_dyaw_deg, 1),
            "reached": self.reached,
            "lost_target": self.lost_target,
            "reason": self.reason,
            "details": dict(self.details),
        }


def decide_approach(
    context: TargetApproachContext,
    *,
    safety: LocalSafetySnapshot,
    perception: PerceptionQuality,
    policy: TargetApproachConfig | None = None,
    clearance: ClearanceNavigationConfig | None = None,
    capabilities: RobotCapabilities | None = None,
    target_lost: bool = False,
) -> ApproachDecision:
    """§21.3：返回下一步接近动作。standoff/步长只认 policy，不认 context 自带字段。"""
    policy = policy or TargetApproachConfig()
    clearance = clearance or ClearanceNavigationConfig()

    if target_lost or not perception.target_visible:
        return ApproachDecision(
            action=ACTION_STOP_REOBSERVE, lost_target=True,
            reason="TARGET_NOT_VISIBLE: 停车重新观察，禁止盲目前进",
        )
    if context.spatial_quality not in APPROACH_CAPABLE_SPATIAL_QUALITY or context.depth_m is None:
        return ApproachDecision(
            action=ACTION_STOP_REOBSERVE, lost_target=False,
            reason=f"NO_METRIC_TARGET_DEPTH: spatial_quality={context.spatial_quality}",
        )
    if policy.require_fresh_depth and not perception.target_depth_valid:
        return ApproachDecision(
            action=ACTION_STOP_REOBSERVE, lost_target=False,
            reason="STALE_TARGET_DEPTH: 需要一帧新的有效 RGB-D 深度",
        )

    depth = float(context.depth_m)
    bearing = 0.0 if context.bearing_deg is None else float(context.bearing_deg)

    # 先对准再前进：偏航大的时候前进只会让目标滑出视野。
    if abs(bearing) > policy.align_yaw_threshold_deg:
        budget = safety.safe_turn_left_deg if bearing > 0 else safety.safe_turn_right_deg
        if budget <= 0.0:
            return ApproachDecision(
                action=ACTION_APPROACH_BLOCKED,
                reason=f"ROTATION_NOT_CLEAR: bearing={bearing:.1f}° 方向无可用转向余量",
                details={"depth_m": depth, "bearing_deg": bearing},
            )
        dyaw = (1.0 if bearing > 0 else -1.0) * min(abs(bearing), budget)
        return ApproachDecision(
            action=ACTION_ROTATE_SMALL, relative_dyaw_deg=dyaw,
            reason=f"ALIGN_YAW: bearing={bearing:.1f}° 超过 {policy.align_yaw_threshold_deg}°",
            details={"depth_m": depth, "bearing_deg": bearing, "turn_budget_deg": budget},
        )

    if depth <= policy.minimum_standoff_m:
        return ApproachDecision(
            action=ACTION_TARGET_REACHED_OR_TOO_CLOSE,
            reached=depth >= policy.minimum_standoff_m,
            reason=f"AT_OR_INSIDE_MIN_STANDOFF: depth={depth:.2f}m",
            details={"depth_m": depth, "bearing_deg": bearing},
        )

    error = depth - policy.preferred_standoff_m
    if error > policy.max_approach_step_m:
        step = min(policy.max_approach_step_m, error)
    elif error > policy.fine_approach_step_m:
        step = min(policy.fine_approach_step_m, error)
    else:
        return ApproachDecision(
            action=ACTION_APPROACH_COMPLETE, reached=True,
            reason=f"AT_PREFERRED_STANDOFF: depth={depth:.2f}m",
            details={"depth_m": depth, "bearing_deg": bearing},
        )

    # §22.1：终点永远停在目标表面之外，目标自身因此不是 path collision。
    step = min(step, depth - policy.minimum_standoff_m)

    details = {
        "depth_m": depth,
        "bearing_deg": bearing,
        "distance_error_m": round(error, 3),
        "forward_swept_clearance_m": safety.forward_swept_clearance_m,
        "requested_step_m": round(step, 3),
    }
    blocked = _forward_block_reason(safety, step, clearance, capabilities)
    if blocked:
        return ApproachDecision(action=ACTION_APPROACH_BLOCKED, reason=blocked, details=details)
    return ApproachDecision(
        action=ACTION_APPROACH_FORWARD, relative_dx=round(step, 3),
        reason=f"APPROACH_STEP: depth={depth:.2f}m -> standoff {policy.preferred_standoff_m}m",
        details=details,
    )


def _forward_block_reason(
    safety: LocalSafetySnapshot,
    step_m: float,
    clearance: ClearanceNavigationConfig,
    capabilities: RobotCapabilities | None,
) -> str:
    # 计划书 §6.7：这条门管的是「相对前进原语」。metric 后端（nav2）走的是
    # NAVIGATE_POSE + 路径规划，同一个位移由 Nav2 负责，本来就不会用 FORWARD
    # 原语，所以对它不适用——否则 nav2 模式下接近永远停在 BLOCKED。
    metric = bool(getattr(capabilities, "supports_metric_navigation", False))
    if not metric and capabilities is not None and capabilities.allowed_motion_primitives:
        if "FORWARD" not in capabilities.allowed_motion_primitives:
            return "FORWARD_PRIMITIVE_NOT_ALLOWED: 平台未开放前进原语"
    if not (safety.lidar_fresh and safety.geometry_validated):
        return "UNVALIDATED_LOCAL_GEOMETRY: LiDAR 不新鲜或几何未交叉验证"
    swept = safety.forward_swept_clearance_m
    if swept is None:
        return "UNKNOWN_FORWARD_GEOMETRY: 前向 swept 扇区无观测，禁止把未知当空旷"
    if not safety.forward_safe:
        return f"FORWARD_NOT_SAFE: swept={swept:.2f}m"
    if swept - step_m < clearance.hard_clearance_m:
        return (
            f"STEP_EXCEEDS_HARD_CLEARANCE: swept={swept:.2f}m step={step_m:.2f}m "
            f"hard={clearance.hard_clearance_m}m"
        )
    return ""


def evaluate_target_reached(
    context: TargetApproachContext,
    *,
    safety: LocalSafetySnapshot,
    perception: PerceptionQuality,
    verified_again: bool,
    robot_stationary: bool,
    policy: TargetApproachConfig | None = None,
) -> tuple[bool, str]:
    """§23：只有全部条件同时成立才算真正到达；返回 (reached, 第一条不满足的原因)。"""
    policy = policy or TargetApproachConfig()
    depth = context.depth_m
    bearing = context.bearing_deg
    checks = (
        (not policy.require_final_verification or verified_again, "FINAL_VERIFICATION_MISSING"),
        (robot_stationary, "ROBOT_NOT_STATIONARY"),
        (safety.geometry_validated, "GEOMETRY_NOT_VALIDATED"),
        (perception.target_visible, "TARGET_NOT_VISIBLE"),
        (perception.target_depth_valid, "TARGET_DEPTH_NOT_FRESH"),
        (
            context.spatial_quality in APPROACH_CAPABLE_SPATIAL_QUALITY,
            f"SPATIAL_QUALITY_TOO_WEAK: {context.spatial_quality}",
        ),
        (depth is not None, "NO_TARGET_DEPTH"),
        (
            depth is not None
            and policy.minimum_standoff_m <= depth <= policy.maximum_standoff_m,
            "DEPTH_OUT_OF_STANDOFF_RANGE",
        ),
        (
            bearing is not None and abs(bearing) <= policy.max_finish_bearing_deg,
            "BEARING_OUT_OF_RANGE",
        ),
    )
    for ok, reason in checks:
        if not ok:
            return False, reason
    return True, "TARGET_REACHED"
