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
    backend: str = "go2w_experimental"
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
    # Metric pose is never required for this experiment.
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
    caps = (capabilities or RobotCapabilities()).to_dict()
    return ExperimentReadiness(
        ready=ready,
        backend="go2w_experimental",
        degraded=sorted(set(degraded)),
        capabilities=caps,
        checks=checks,
        reason=reason,
    )
