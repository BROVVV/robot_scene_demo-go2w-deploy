"""Backend factory plus pure-software mock backends for offline E2E.

The mocks simulate pose, goal success / failure / timeout, operator stop and
metric navigation so the AutonomousExplorer can be tested without the robot
(and proves the high level is not bound to Go2-W).
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


def create_backend(kind: str, **kwargs: Any) -> RobotBackend:
    """Build a backend by name: go2w_experimental / mock / mock_metric."""
    normalized = str(kind or "").strip().lower()
    if normalized in {"go2w", "go2w_experimental", "go2w_experimental_backend"}:
        from .go2w_experimental_backend import Go2WExperimentalBackend
        return Go2WExperimentalBackend(**kwargs)
    if normalized in {"mock", "mock_backend"}:
        return MockBackend(**kwargs)
    if normalized in {"mock_metric", "metric", "mock_metric_backend",
                      "future"}:
        return MockMetricBackend(**kwargs)
    raise ValueError(f"unsupported backend kind: {kind}")


# ---------------------------------------------------------------------------
# MockBackend (relative / topological, mirrors Go2-W capabilities)
# ---------------------------------------------------------------------------


@dataclass
class MockBackendConfig:
    max_turn_deg_per_action: float = 30.0
    forward_step_m: float = 0.20
    latency_sec: float = 0.0


class MockBackend(RobotBackend):
    """Scripted relative backend: outcomes replay a provided status list.

    With no script, every goal succeeds.  ``outcome_sequence`` may contain
    NavigationStatus values or strings; the final entry repeats indefinitely.
    """

    def __init__(
        self,
        *,
        outcome_sequence: list[NavigationStatus | str] | None = None,
        pose_provider: Callable[[], tuple[float, float, float]] | None = None,
        config: MockBackendConfig | None = None,
        now: Callable[[], float] = time.time,
    ) -> None:
        self.config = config or MockBackendConfig()
        self._now = now
        self._sequence = [NavigationStatus(item) for item in (outcome_sequence or [])]
        self._index = 0
        self._pose = [0.0, 0.0, 0.0]
        self._pose_provider = pose_provider
        self._stop_called = False
        self._cancel_called = False
        self._executed: list[NavigationResult] = []

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
            allowed_motion_primitives=("FORWARD", "ROTATE_LEFT", "ROTATE_RIGHT"),
        )

    def get_pose(self) -> RobotPose | None:
        if self._pose_provider is not None:
            x, y, yaw = self._pose_provider()
            self._pose = [float(x), float(y), float(yaw)]
        return RobotPose(
            x=self._pose[0], y=self._pose[1], yaw=self._pose[2],
            frame_id="odom", quality=PoseQuality.RELATIVE,
            timestamp=self._now(), source="mock_relative",
        )

    def execute_goal(self, goal: ExplorationGoal) -> NavigationHandle:
        if self.config.latency_sec > 0.0:
            time.sleep(self.config.latency_sec)
        status = self._next_status()
        requested = _mock_requested(goal)
        observed = _apply_mock_motion(self._pose, goal, self.config)
        result = navigation_result(
            goal.goal_id, status,
            message=f"mock {status.value}",
            requested_motion=requested,
            observed_motion=observed,
            elapsed_sec=0.01,
            provenance={"backend": "mock"},
        )
        self._executed.append(result)
        return NavigationHandle(goal_id=goal.goal_id, result=result)

    def get_navigation_status(self, handle: NavigationHandle) -> NavigationResult:
        if handle.result is None:
            return navigation_result(handle.goal_id, NavigationStatus.RUNNING)
        return handle.result

    def cancel(self, handle: NavigationHandle | None = None) -> bool:
        self._cancel_called = True
        return True

    def stop(self) -> None:
        self._stop_called = True

    def health(self) -> BackendHealth:
        return BackendHealth(
            ready=True,
            backend="mock",
            degraded=["metric_pose_unavailable"],
            capabilities=self.capabilities(),
            pose_quality=PoseQuality.RELATIVE,
            details={"executed_goals": len(self._executed)},
        )

    # ---- helpers ----------------------------------------------------------

    def _next_status(self) -> NavigationStatus:
        if not self._sequence:
            return NavigationStatus.SUCCEEDED
        status = self._sequence[min(self._index, len(self._sequence) - 1)]
        self._index += 1
        return status


# ---------------------------------------------------------------------------
# MockMetricBackend (future mature robot)
# ---------------------------------------------------------------------------


class MockMetricBackend(RobotBackend):
    """Simulates a mature robot with map pose + metric navigation."""

    def __init__(
        self,
        *,
        outcome_sequence: list[NavigationStatus | str] | None = None,
        start_pose: tuple[float, float, float] = (0.0, 0.0, 0.0),
        now: Callable[[], float] = time.time,
    ) -> None:
        self._now = now
        self._sequence = [NavigationStatus(item) for item in (outcome_sequence or [])]
        self._index = 0
        self._pose = [float(v) for v in start_pose]
        self._stop_called = False
        self._executed: list[NavigationResult] = []

    def capabilities(self) -> RobotCapabilities:
        return RobotCapabilities(
            supports_global_pose=True,
            supports_metric_navigation=True,
            supports_relative_translation=True,
            supports_relative_rotation=True,
            supports_heading_control=True,
            supports_navigation_cancel=True,
            supports_navigation_feedback=True,
            supports_platform_obstacle_avoidance=True,
            allowed_motion_primitives=(
                "FORWARD", "ROTATE_LEFT", "ROTATE_RIGHT", "REVERSE", "LATERAL"
            ),
        )

    def get_pose(self) -> RobotPose | None:
        return RobotPose(
            x=self._pose[0], y=self._pose[1], yaw=self._pose[2],
            frame_id="map", quality=PoseQuality.METRIC,
            timestamp=self._now(), source="mock_map_localization",
        )

    def execute_goal(self, goal: ExplorationGoal) -> NavigationHandle:
        status = self._next_status()
        requested = _mock_requested(goal)
        observed: dict[str, Any] = {}
        if goal.goal_type == "NAVIGATE_POSE" and goal.position is not None:
            observed = {
                "x": round(goal.position[0], 3),
                "y": round(goal.position[1], 3),
                "yaw": round(goal.yaw or 0.0, 3),
            }
            if status == NavigationStatus.SUCCEEDED:
                self._pose = [goal.position[0], goal.position[1], goal.yaw or 0.0]
        else:
            observed = _apply_mock_motion(self._pose, goal, MockBackendConfig())
        result = navigation_result(
            goal.goal_id, status,
            message=f"mock_metric {status.value}",
            requested_motion=requested,
            observed_motion=observed,
            elapsed_sec=0.01,
            provenance={"backend": "mock_metric"},
        )
        self._executed.append(result)
        return NavigationHandle(goal_id=goal.goal_id, result=result)

    def get_navigation_status(self, handle: NavigationHandle) -> NavigationResult:
        if handle.result is None:
            return navigation_result(handle.goal_id, NavigationStatus.RUNNING)
        return handle.result

    def cancel(self, handle: NavigationHandle | None = None) -> bool:
        return True

    def stop(self) -> None:
        self._stop_called = True

    def health(self) -> BackendHealth:
        return BackendHealth(
            ready=True,
            backend="mock_metric",
            degraded=[],
            capabilities=self.capabilities(),
            pose_quality=PoseQuality.METRIC,
            details={"executed_goals": len(self._executed)},
        )

    def _next_status(self) -> NavigationStatus:
        if not self._sequence:
            return NavigationStatus.SUCCEEDED
        status = self._sequence[min(self._index, len(self._sequence) - 1)]
        self._index += 1
        return status


# ---------------------------------------------------------------------------
# shared helpers
# ---------------------------------------------------------------------------


def _mock_requested(goal: ExplorationGoal) -> dict[str, Any]:
    return {
        "goal_type": goal.goal_type,
        "relative_dx": goal.relative_dx,
        "relative_dy": goal.relative_dy,
        "relative_dyaw": goal.relative_dyaw,
        "position": list(goal.position) if goal.position else None,
        "yaw": goal.yaw,
    }


def _apply_mock_motion(pose: list[float], goal: ExplorationGoal,
                       config: MockBackendConfig) -> dict[str, Any]:
    before = (pose[0], pose[1], pose[2])
    if goal.goal_type in {"ROTATE_VIEW", "INSPECT_ANCHOR", "REVISIT_NODE"}:
        dyaw = float(goal.relative_dyaw if goal.relative_dyaw is not None else 0.0)
        dyaw = max(
            -abs(config.max_turn_deg_per_action),
            min(abs(config.max_turn_deg_per_action), dyaw),
        )
        pose[2] += math.radians(dyaw)
        return {"yaw_delta_deg": round(dyaw, 3)}
    if goal.goal_type == "RELATIVE_MOVE":
        dx = float(goal.relative_dx if goal.relative_dx is not None else 0.0)
        dx = max(0.0, min(config.forward_step_m, dx))
        pose[0] += dx * math.cos(pose[2])
        pose[1] += dx * math.sin(pose[2])
        return {"displacement_m": round(dx, 3)}
    return {}
