"""ExperimentReadiness: automatic-only readiness for the operator-supervised
experiment.

This deliberately does NOT require any manual calibration (chessboard, tape
measure, four-direction obstacle placement, swept-envelope acceptance, LIO
route, Nav2 map).  It only checks what the high-level experiment actually
needs, and every check is automatic.  The formal Stage-2 readiness
(``stage2_readiness.py``) stays untouched.
"""

from __future__ import annotations

import os
from dataclasses import asdict, dataclass, field
from typing import Any

from app.navigation.robot_backend import RobotCapabilities


@dataclass
class ExperimentReadiness:
    ready: bool
    backend: str = "nav2"
    degraded: list[str] = field(default_factory=list)
    capabilities: dict[str, Any] = field(default_factory=dict)
    checks: dict[str, Any] = field(default_factory=dict)
    reason: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "ready": self.ready,
            "backend": self.backend,
            "degraded": self.degraded,
            "capabilities": self.capabilities,
            "checks": self.checks,
            "reason": self.reason,
        }


def compute_experiment_readiness(
    *,
    camera_fresh: bool,
    bundle_fresh: bool,
    motion_action_available: bool,
    robot_mode_ok: bool,
    emergency_stop_available: bool,
    llm_available: bool,
    backend_healthy: bool = True,
    pose_freshness_if_available: bool | None = None,
    capabilities: RobotCapabilities | None = None,
    check_llm_key: bool = True,
    backend: str = "nav2",
) -> ExperimentReadiness:
    """Machine-readable automatic readiness (plan section 17)."""
    checks: dict[str, Any] = {
        "camera_fresh": bool(camera_fresh),
        "bundle_fresh": bool(bundle_fresh),
        "llm_available": bool(llm_available),
        "motion_action_available": bool(motion_action_available),
        "robot_mode_ok": bool(robot_mode_ok),
        "emergency_stop_available": bool(emergency_stop_available),
        "backend_healthy": bool(backend_healthy),
    }
    if pose_freshness_if_available is not None:
        checks["pose_freshness_if_available"] = bool(pose_freshness_if_available)
    if check_llm_key:
        checks["llm_api_key_configured"] = bool(os.getenv("SILICONFLOW_API_KEY"))

    # 调用方实传的能力表就是这次运行的后端的真实能力；不传时退化成全 False 的
    # 空表（``RobotCapabilities`` 的每个字段默认都是 False），于是「没有声明
    # metric 导航」会被如实记成降级 —— 与旧行为一致，不会因为这次改动而变红。
    caps_obj = capabilities or RobotCapabilities()

    degraded: list[str] = []
    if not camera_fresh:
        degraded.append("camera_unavailable")
    if not bundle_fresh:
        degraded.append("bundle_unavailable")
    if not llm_available:
        degraded.append("llm_unavailable")
    if not motion_action_available:
        degraded.append("motion_action_unavailable")
    if not robot_mode_ok:
        degraded.append("robot_mode_error")
    if not emergency_stop_available:
        degraded.append("emergency_stop_unavailable")
    if not backend_healthy:
        degraded.append("backend_unhealthy")
    if pose_freshness_if_available is False:
        degraded.append("pose_stale")
    if checks.get("llm_api_key_configured") is False:
        degraded.append("llm_api_key_missing")
    # 「没有地图位姿」这一条要**按能力实报**，不能无条件追加。
    #
    # 它此前是一句 `degraded.append("metric_pose_unavailable")`，无条件执行，
    # 于是每一份报告都带着它：2026-10-15 11:05 那一场，同一次会话的
    # `backend_health` 写着 `supports_metric_navigation: true` / `pose_quality:
    # metric`，`experiment_readiness` 却写着 `degraded:
    # ["metric_pose_unavailable"]`。无条件的降级项等于没有信息 —— 读的人分不出
    # 「真的没有地图位姿」和「这份报告压根没判断」。现在只有后端确实没声明
    # metric 导航时才算降级。
    #
    # 注意本条**不参与** ready 的判定（历史上也不参与）：这个实验剖面允许在相对
    # 位姿下半自主运行，缺地图位姿是降级、不是阻塞。
    metric_pose_available = bool(caps_obj.supports_metric_navigation)
    checks["metric_pose_available"] = metric_pose_available
    if not metric_pose_available:
        degraded.append("metric_pose_unavailable")

    ready = all(
        checks[key] for key in (
            "camera_fresh", "bundle_fresh", "llm_available",
            "motion_action_available", "robot_mode_ok",
            "emergency_stop_available", "backend_healthy",
        )
    )
    if checks.get("llm_api_key_configured") is False:
        ready = False
    reason = ""
    if not ready:
        reason = "experiment readiness failed: " + "; ".join(degraded)
    caps = caps_obj.to_dict()
    return ExperimentReadiness(
        ready=ready,
        # 计划书 §4.3 / §6.6：go2w_experimental 不再出现在生产入口、默认配置
        # 和可选参数里，这份报告也不该再声称自己跑在那个后端上。默认取生产
        # 默认值 nav2，调用方（run_semantic_exploration）按 --backend 实传。
        backend=backend,
        degraded=sorted(set(degraded)),
        capabilities=caps,
        checks=checks,
        reason=reason,
    )
