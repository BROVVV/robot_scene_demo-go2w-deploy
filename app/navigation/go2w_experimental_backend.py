"""Go2-W operator-supervised experimental backend.

Implements :class:`RobotBackend` for the current Go2-W rig through the *high
level* control interface (``/go2w/motion`` action, ``/go2w/arm`` service,
``/go2w/emergency_stop`` service).  LowCmd / joint control / firmware changes
are forbidden by project policy.

Like the existing ``StepSearchRunner``, all hardware access is injected as
callables so this module stays ROS-independent and unit-testable; the CLI wires
the callables to the real ROS node (reusing the audited motion executor from
``scripts/go2w/run_autonomous_loop.py``).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Callable

from .models import ExplorationGoal
from .robot_backend import (
    BackendHealth,
    NavigationHandle,
    NavigationResult,
    NavigationStatus,
    PoseQuality,
    RobotBackend,
    RobotCapabilities,
    RobotPose,
    navigation_result,
)


@dataclass
class Go2WBackendConfig:
    # Hard safety limit for ONE physical turn primitive.  A logical goal larger
    # than this is split into several primitives and closed on measured yaw
    # (计划书 §5.1); the limit itself is never raised to reach the goal.
    max_turn_deg_per_action: float = 30.0
    # Do not issue a primitive for a residual smaller than this - the platform
    # cannot execute it reliably.
    min_turn_segment_deg: float = 5.0
    # A logical rotation goal is confirmed only when the accumulated measured
    # yaw is within this tolerance (计划书 §5.4).
    turn_confirm_tolerance_deg: float = 8.0
    forward_step_m: float = 0.20
    max_forward_step_m: float = 0.30
    allow_lateral: bool = False
    motion_timeout_sec: float = 60.0
    # Opportunistic request-vs-observed correction learning (never a blocker).
    correction_min_samples: int = 8
    correction_min_confidence: float = 0.6
    apply_correction: bool = True
    # Pose freshness window; None disables the check.
    pose_max_age_sec: float | None = 10.0
    # Camera/LLM-only validation mode.  It intentionally never authorizes
    # motion; missing Action/arm services therefore do not make the
    # perception loop look like a backend crash.
    dry_run: bool = False
    # Backward recovery is formal but recovery-only and breadcrumb-safe.
    allow_backward_recovery: bool = True
    backward_step_m: float = 0.10
    max_backward_step_m: float = 0.12
    min_backward_step_m: float = 0.05
    backward_heading_tolerance_deg: float = 8.0
    backward_max_age_sec: float = 8.0


GO2W_OPERATOR_SUPERVISED_PRIMITIVES = (
    "FORWARD",
    "BACKWARD_RECOVERY",
    "ROTATE_LEFT",
    "ROTATE_RIGHT",
)


@dataclass
class MotionCorrection:
    rotation_scale: float = 1.0
    forward_scale: float = 1.0
    samples: int = 0
    confidence: str = "low"

    def to_dict(self) -> dict[str, Any]:
        return {
            "rotation_scale": round(self.rotation_scale, 4),
            "forward_scale": round(self.forward_scale, 4),
            "samples": self.samples,
            "confidence": self.confidence,
        }


class Go2WExperimentalBackend(RobotBackend):
    """Relative/topological backend for the current Go2-W."""

    def __init__(
        self,
        *,
        execute_step: Callable[[str], tuple[bool, str, dict[str, Any]]],
        odometry: Callable[[], tuple[float, float, float]],
        stop: Callable[[], None] | None = None,
        cancel: Callable[[], None] | None = None,
        health_probe: Callable[[], dict[str, Any]] | None = None,
        config: Go2WBackendConfig | None = None,
        now: Callable[[], float] = time.time,
    ) -> None:
        self._execute_step = execute_step
        self._odometry = odometry
        self._stop = stop or (lambda: None)
        self._cancel = cancel or (lambda: None)
        self._health_probe = health_probe
        self.config = config or Go2WBackendConfig()
        self._now = now
        self._correction = MotionCorrection()
        self._pending: dict[str, tuple[float, float, float]] = {}
        self._executed_goals: list[dict[str, Any]] = []
        self._last_health: dict[str, Any] = field(default_factory=dict)

    # ---- RobotBackend -----------------------------------------------------

    def capabilities(self) -> RobotCapabilities:
        return RobotCapabilities(
            supports_global_pose=False,
            supports_metric_navigation=False,
            supports_relative_translation=True,
            supports_relative_rotation=True,
            supports_heading_control=True,
            supports_navigation_cancel=True,
            supports_navigation_feedback=True,
            supports_platform_obstacle_avoidance=False,
            allowed_motion_primitives=GO2W_OPERATOR_SUPERVISED_PRIMITIVES,
        )

    def get_pose(self) -> RobotPose | None:
        try:
            x, y, yaw = self._odometry()
        except Exception:
            return None
        if not all(math.isfinite(float(v)) for v in (x, y, yaw)):
            return None
        return RobotPose(
            x=float(x), y=float(y), yaw=float(yaw),
            frame_id="odom", quality=PoseQuality.RELATIVE,
            timestamp=self._now(), source="wheel_fused_odom",
        )

    def execute_goal(self, goal: ExplorationGoal) -> NavigationHandle:
        handle = NavigationHandle(goal_id=goal.goal_id)
        try:
            if (
                goal.goal_type not in {"STOP", "REOBSERVE"}
                and not self.config.dry_run
            ):
                health = self.health()
                if not health.ready:
                    result = navigation_result(
                        goal.goal_id,
                        NavigationStatus.BACKEND_UNAVAILABLE,
                        message=(
                            "motion preflight failed: "
                            + ", ".join(health.degraded)
                        ),
                        requested_motion=_requested_motion(goal),
                        provenance={"backend_health": health.to_dict()},
                    )
                else:
                    result = self._dispatch(goal)
            else:
                result = self._dispatch(goal)
        except Exception as exc:  # backend failure -> structured result
            result = navigation_result(
                goal.goal_id, NavigationStatus.BACKEND_UNAVAILABLE,
                message=f"go2w backend error: {type(exc).__name__}: {exc}",
                requested_motion=_requested_motion(goal),
                provenance={"error": f"{type(exc).__name__}: {exc}"},
            )
        handle.result = result
        self._executed_goals.append(result.to_dict())
        return handle

    def get_navigation_status(self, handle: NavigationHandle) -> NavigationResult:
        if handle.result is None:
            return navigation_result(
                handle.goal_id, NavigationStatus.RUNNING,
                message="motion still running",
            )
        return handle.result

    def cancel(self, handle: NavigationHandle | None = None) -> bool:
        try:
            self._cancel()
            return True
        except Exception:
            return False

    def stop(self) -> None:
        self._stop()

    def health(self) -> BackendHealth:
        details: dict[str, Any] = {
            "pose_source": "wheel_fused_odom",
            "motion_correction": self._correction.to_dict(),
            "executed_goals": len(self._executed_goals),
        }
        if self._health_probe is not None:
            try:
                probe = self._health_probe()
            except Exception as exc:
                probe = {"error": f"{type(exc).__name__}: {exc}"}
            details.update(probe)
        degraded = ["metric_pose_unavailable"]
        ready = True
        if details.get("motion_action_available") is False and not self.config.dry_run:
            ready = False
            degraded.append("motion_action_unavailable")
        server_count = details.get("motion_action_server_count")
        if (
            "motion_action_server_count" in details
            and server_count != 1
            and not self.config.dry_run
        ):
            ready = False
            degraded.append(
                "motion_action_graph_probe_failed"
                if server_count is None
                else "motion_action_server_missing"
                if int(server_count) < 1
                else "duplicate_motion_action_servers"
            )
        process_count = details.get("motion_action_server_process_count")
        # Same split topology as the wheel odom gate below: the WebUI search
        # worker runs on the robot while the single canonical action server runs
        # on the control host, where a local /proc scan finds nothing.  The ROS
        # graph server-count is the authority there.  An unknown probe or extra
        # local servers still fail closed.
        remote_action_authority = process_count == 0 and server_count == 1
        if (
            "motion_action_server_process_count" in details
            and process_count != 1
            and not remote_action_authority
            and not self.config.dry_run
        ):
            ready = False
            marker = (
                "motion_action_process_probe_failed"
                if process_count is None
                else "motion_action_server_process_missing"
                if int(process_count) < 1
                else "duplicate_motion_action_server_processes"
            )
            if marker not in degraded:
                degraded.append(marker)
        odom_publishers = details.get("odom_publisher_count")
        if (
            "odom_publisher_count" in details
            and odom_publishers != 1
            and not self.config.dry_run
        ):
            ready = False
            degraded.append(
                "odom_graph_probe_failed"
                if odom_publishers is None
                else "odom_publisher_missing"
                if int(odom_publishers) < 1
                else "duplicate_odom_publishers"
            )
        odom_processes = details.get("wheel_odom_process_count")
        # The Go2-W WebUI may run on the robot while the single canonical
        # wheel-odom publisher runs on the control host.  In that topology
        # there is intentionally no local executable to count; the ROS graph
        # publisher-count gate is the authority.  Still fail closed on an
        # unknown process probe or on multiple local odom processes.
        remote_odom_authority = (
            odom_processes == 0 and odom_publishers == 1
        )
        if (
            "wheel_odom_process_count" in details
            and odom_processes not in {1, 0}
            and not self.config.dry_run
        ):
            ready = False
            marker = (
                "wheel_odom_process_probe_failed"
                if odom_processes is None
                else "wheel_odom_process_missing"
                if int(odom_processes) < 1
                else "duplicate_wheel_odom_processes"
            )
            if marker not in degraded:
                degraded.append(marker)
        if (
            "wheel_odom_process_count" in details
            and odom_processes == 0
            and not remote_odom_authority
            and not self.config.dry_run
        ):
            ready = False
            if "wheel_odom_process_missing" not in degraded:
                degraded.append("wheel_odom_process_missing")
        if details.get("robot_mode_error"):
            ready = False
            degraded.append("robot_mode_error")
        self._last_health = details
        return BackendHealth(
            ready=ready,
            backend="go2w_experimental",
            degraded=degraded,
            capabilities=self.capabilities(),
            pose_quality=PoseQuality.RELATIVE,
            details=details,
        )

    # ---- internals --------------------------------------------------------

    def _dispatch(self, goal: ExplorationGoal) -> NavigationResult:
        goal_type = goal.goal_type
        if goal_type == "STOP":
            self.stop()
            return navigation_result(
                goal.goal_id, NavigationStatus.SUCCEEDED,
                message="backend stop requested", requested_motion=_requested_motion(goal),
            )
        if goal_type in {"REOBSERVE"}:
            return navigation_result(
                goal.goal_id, NavigationStatus.SUCCEEDED,
                message="reobserve without motion", requested_motion=_requested_motion(goal),
            )
        if goal_type == "NAVIGATE_POSE":
            return navigation_result(
                goal.goal_id, NavigationStatus.REJECTED,
                message="metric navigation unsupported on go2w_experimental backend",
                requested_motion=_requested_motion(goal),
            )
        # Heading-driven actions (ROTATE_VIEW / INSPECT_ANCHOR / REVISIT_NODE).
        dyaw = goal.relative_dyaw
        if goal_type in {"ROTATE_VIEW", "INSPECT_ANCHOR", "REVISIT_NODE"}:
            if dyaw is None:
                dyaw = 0.0
            return self._execute_turn(goal, float(dyaw))
        if goal_type == "RELATIVE_MOVE":
            dx = float(goal.relative_dx if goal.relative_dx is not None else 0.0)
            dy = float(goal.relative_dy or 0.0)
            if dx < -0.01:
                if not self.config.allow_backward_recovery:
                    return navigation_result(
                        goal.goal_id, NavigationStatus.REJECTED,
                        message=(
                            "backward recovery disabled by backend config; "
                            "replan with rotate+forward"
                        ),
                        requested_motion=_requested_motion(goal),
                        provenance={
                            "backend": "go2w_experimental",
                            "replan_required": True,
                            "unsupported_primitive": "BACKWARD_RECOVERY_DISABLED",
                        },
                    )
                requested = abs(float(dx))
                if requested < self.config.min_backward_step_m - 1e-9:
                    return navigation_result(
                        goal.goal_id, NavigationStatus.REJECTED,
                        message=(
                            f"backward recovery step {requested:.3f}m below "
                            f"minimum {self.config.min_backward_step_m:.2f}m"
                        ),
                        requested_motion=_requested_motion(goal),
                        provenance={
                            "backend": "go2w_experimental",
                            "replan_required": True,
                            "unsupported_primitive": "BACKWARD_TOO_SHORT",
                        },
                    )
                allowed = min(requested, self.config.max_backward_step_m)
                return self._execute_backward(goal, allowed)
            if dx < 0.01 and abs(dy) <= 0.01:
                # A pure re-position with no forward demand.
                return navigation_result(
                    goal.goal_id, NavigationStatus.SUCCEEDED,
                    message="no translation demanded", requested_motion=_requested_motion(goal),
                )
            if not self.config.allow_lateral and abs(dy) > 0.01:
                return navigation_result(
                    goal.goal_id, NavigationStatus.REJECTED,
                    message="lateral motion disabled; replan with rotate+forward",
                    requested_motion=_requested_motion(goal),
                    provenance={
                        "backend": "go2w_experimental",
                        "replan_required": True,
                        "unsupported_primitive": "LATERAL",
                    },
                )
            return self._execute_forward(goal, dx)
        return navigation_result(
            goal.goal_id, NavigationStatus.REJECTED,
            message=f"unsupported goal type {goal_type}",
            requested_motion=_requested_motion(goal),
        )

    def _execute_turn(self, goal: ExplorationGoal, dyaw_deg: float) -> NavigationResult:
        """Execute a *logical* rotation goal as bounded 30° primitives.

        计划书 §5：规划层的 ±60° 目标不再被静默限幅成一个 30° 动作。逻辑目标按
        ``max_turn_deg_per_action`` 拆段，每段之间用 canonical fused odometry 闭环，
        累计实测 yaw 决定逻辑目标是否成功。
        """
        requested_total = _wrap_deg(float(dyaw_deg))
        limit = abs(self.config.max_turn_deg_per_action)
        tolerance = abs(self.config.turn_confirm_tolerance_deg)
        min_segment = abs(self.config.min_turn_segment_deg)
        if abs(requested_total) < min_segment:
            return navigation_result(
                goal.goal_id, NavigationStatus.SUCCEEDED,
                message="no rotation demanded",
                requested_motion={
                    "step": "turn_segmented",
                    "requested_total_deg": round(requested_total, 3),
                    "segment_count": 0,
                },
                observed_motion={"observed_total_deg": 0.0,
                                 "remaining_deg": round(requested_total, 3)},
                provenance={"backend": "go2w_experimental", "segments": []},
            )

        started = self._now()
        observed_total = 0.0
        segments: list[dict[str, Any]] = []
        ok = True
        reason = ""
        detail: dict[str, Any] = {}
        # One corrective segment beyond the nominal split absorbs per-primitive
        # undershoot without ever exceeding the logical goal.
        max_segments = int(math.ceil(abs(requested_total) / max(1.0, limit))) + 1
        for _ in range(max_segments):
            remaining = requested_total - observed_total
            if abs(remaining) < min_segment:
                break
            segment_deg = max(-limit, min(limit, remaining))
            degrees = max(1, int(round(abs(segment_deg))))
            step = f"l{degrees}" if segment_deg >= 0.0 else f"r{degrees}"
            before = self._odometry()
            segment_ok, segment_reason, segment_detail = self._execute_step(step)
            after = self._odometry()
            segment_observed = _wrap_deg(math.degrees(after[2] - before[2]))
            observed_total += segment_observed
            segments.append({
                "segment_index": len(segments) + 1,
                "segment_step": step,
                "requested_deg": round(segment_deg, 3),
                "observed_deg": round(segment_observed, 3),
                "success": bool(segment_ok),
                "message": segment_reason,
                "detail": segment_detail,
            })
            if not segment_ok:
                # 计划书 §5.3.6：第一段失败后禁止继续执行下一段。
                ok = False
                reason = segment_reason or f"turn segment {len(segments)} failed"
                detail = segment_detail
                self.stop()
                break

        elapsed = max(0.0, self._now() - started)
        remaining = requested_total - observed_total
        self._learn_correction(
            requested=requested_total, observed=observed_total, kind="rotation"
        )
        if ok and abs(remaining) > tolerance:
            ok = False
            reason = (
                f"ROTATION_NOT_CONFIRMED: requested {requested_total:.1f}° but "
                f"odometry measured {observed_total:.1f}° "
                f"(remaining {remaining:.1f}° > {tolerance:.1f}°)"
            )
            detail = {**detail, "error_type": "ROTATION_NOT_CONFIRMED"}
            self.stop()
        status = _motion_status(ok, reason, detail)
        last = segments[-1] if segments else {}
        return navigation_result(
            goal.goal_id, status,
            message=reason or (
                f"turn {requested_total:+.0f}° completed in {len(segments)} "
                f"segment(s), measured {observed_total:+.1f}°"
            ),
            requested_motion={
                "step": "turn_segmented",
                "requested_total_deg": round(requested_total, 3),
                "relative_yaw_deg": round(requested_total, 3),
                "segment_count": len(segments),
                "segment_limit_deg": round(limit, 3),
                "segment_index": last.get("segment_index", 0),
                "segment_step": last.get("segment_step", ""),
                "segment_steps": [item["segment_step"] for item in segments],
            },
            observed_motion={
                "observed_total_deg": round(observed_total, 3),
                "remaining_deg": round(remaining, 3),
                # Backwards-compatible alias for existing consumers.
                "yaw_delta_deg": round(observed_total, 3),
            },
            elapsed_sec=round(elapsed, 3),
            provenance={
                "backend": "go2w_experimental",
                "detail": detail,
                "segments": segments,
            },
        )

    def _execute_backward(
        self, goal: ExplorationGoal, distance_m: float
    ) -> NavigationResult:
        """Execute one breadcrumb-safe recovery backward primitive.

        The safety evidence itself (valid breadcrumb / heading / age) is
        enforced by the injected ``execute_step`` gate in the real ROS runner;
        this backend only clamps distance and records structured provenance.
        """
        allowed = max(
            self.config.min_backward_step_m,
            min(distance_m, self.config.max_backward_step_m),
        )
        step = f"b{allowed:.3f}"
        before = self._odometry()
        started = self._now()
        ok, reason, detail = self._execute_step(step)
        elapsed = max(0.0, self._now() - started)
        after = self._odometry()
        dx = after[0] - before[0]
        dy = after[1] - before[1]
        signed_progress = (
            dx * math.cos(before[2]) + dy * math.sin(before[2])
        )
        lateral_progress = (
            -dx * math.sin(before[2]) + dy * math.cos(before[2])
        )
        observed_d = math.hypot(dx, dy)
        status = _motion_status(ok, reason, detail)
        recovery_reason = str(
            (goal.provenance or {}).get("recovery_reason")
            or (goal.provenance or {}).get("reason")
            or "front_blocked_recovery"
        )
        return navigation_result(
            goal.goal_id, status,
            message=reason or f"backward recovery {step}",
            requested_motion={
                "primitive": "BACKWARD_RECOVERY",
                "step": step,
                "distance_m": round(allowed, 3),
                "requested_distance_m": round(distance_m, 3),
                "safety_limited": distance_m > allowed,
            },
            observed_motion={
                "displacement_m": round(observed_d, 3),
                "signed_progress_m": round(signed_progress, 3),
                "lateral_progress_m": round(lateral_progress, 3),
            },
            elapsed_sec=round(elapsed, 3),
            provenance={
                "backend": "go2w_experimental",
                "primitive": "BACKWARD_RECOVERY",
                "safety_source": "breadcrumb",
                "recovery_reason": recovery_reason,
                "detail": detail,
            },
        )

    def _execute_forward(self, goal: ExplorationGoal, dx_m: float) -> NavigationResult:
        requested_dx_m = max(0.0, float(dx_m))
        executable_dx_m = min(requested_dx_m, self.config.max_forward_step_m)
        if executable_dx_m <= 0.0:
            return navigation_result(
                goal.goal_id, NavigationStatus.SUCCEEDED,
                message="no forward demand", requested_motion=_requested_motion(goal),
            )
        before = self._odometry()
        started = self._now()
        remaining = executable_dx_m
        segment_limit = max(
            0.01,
            min(self.config.forward_step_m, self.config.max_forward_step_m),
        )
        segments: list[dict[str, Any]] = []
        ok = True
        reason = ""
        detail: dict[str, Any] = {}
        while remaining > 0.005:
            segment_m = min(segment_limit, remaining)
            step = f"f{segment_m:.3f}"
            segment_ok, segment_reason, segment_detail = self._execute_step(step)
            segments.append({
                "index": len(segments),
                "requested_distance_m": round(segment_m, 3),
                "step": step,
                "success": bool(segment_ok),
                "message": segment_reason,
                "detail": segment_detail,
            })
            if not segment_ok:
                ok = False
                reason = segment_reason or (
                    f"forward segment {len(segments)} failed"
                )
                detail = segment_detail
                break
            remaining -= segment_m
        elapsed = max(0.0, self._now() - started)
        after = self._odometry()
        observed_d = math.hypot(after[0] - before[0], after[1] - before[1])
        self._learn_correction(
            requested=executable_dx_m, observed=observed_d, kind="forward"
        )
        status = _motion_status(ok, reason, detail)
        return navigation_result(
            goal.goal_id, status,
            message=reason or f"forward completed in {len(segments)} segment(s)",
            requested_motion={
                "step": "forward_segmented",
                "planner_distance_m": round(requested_dx_m, 3),
                "distance_m": round(executable_dx_m, 3),
                "segment_limit_m": round(segment_limit, 3),
                "segment_count": len(segments),
                "safety_limited": executable_dx_m < requested_dx_m,
            },
            observed_motion={"displacement_m": round(observed_d, 3)},
            elapsed_sec=round(elapsed, 3),
            provenance={
                "backend": "go2w_experimental",
                "detail": detail,
                "segments": segments,
            },
        )

    # ---- opportunistic request-vs-observed learning -----------------------

    def _learn_correction(self, *, requested: float, observed: float,
                          kind: str) -> None:
        if not self.config.apply_correction:
            return
        if requested == 0.0 or not math.isfinite(observed) or observed <= 0.0:
            return
        scale = abs(observed / requested)
        if not 0.2 <= scale <= 5.0:
            return
        if kind == "rotation":
            total = self._correction.rotation_scale * self._correction.samples
            self._correction.samples += 1
            self._correction.rotation_scale = (
                (total + scale) / self._correction.samples
            )
        else:
            total = self._correction.forward_scale * self._correction.samples
            self._correction.samples += 1
            self._correction.forward_scale = (
                (total + scale) / self._correction.samples
            )
        if self._correction.samples >= self.config.correction_min_samples:
            deviation = abs(self._correction.rotation_scale - 1.0) + abs(
                self._correction.forward_scale - 1.0
            )
            self._correction.confidence = (
                "medium" if deviation <= 0.2 else "low"
            )
        else:
            self._correction.confidence = "low"

    def correction(self) -> MotionCorrection:
        return self._correction


def _wrap_deg(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def _motion_status(
    ok: bool, reason: str, detail: dict[str, Any] | None
) -> NavigationStatus:
    if ok:
        return NavigationStatus.SUCCEEDED
    error_type = str((detail or {}).get("error_type") or "").upper()
    reason_upper = str(reason or "").upper()
    if error_type in {"MOTION_ACCEPT_TIMEOUT", "MOTION_RESULT_TIMEOUT"} or (
        reason_upper.startswith("MOTION_ACCEPT_TIMEOUT:")
        or reason_upper.startswith("MOTION_RESULT_TIMEOUT:")
    ):
        return NavigationStatus.TIMEOUT
    return NavigationStatus.FAILED


def _requested_motion(goal: ExplorationGoal) -> dict[str, Any]:
    return {
        "goal_type": goal.goal_type,
        "relative_dx": goal.relative_dx,
        "relative_dy": goal.relative_dy,
        "relative_dyaw": goal.relative_dyaw,
        "position": list(goal.position) if goal.position else None,
        "yaw": goal.yaw,
    }
