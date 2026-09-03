"""Demo-local configuration for the Go2-W manual WASD+QE web demo.

This module deliberately reuses the project SiliconFlow configuration
(``app.config.get_settings``) for the vision model. It only adds the demo's
own scheduling, safety-gate and motion-pulse parameters under the
``MANUAL_DEMO_*`` prefix, exactly as the plan book requires.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[2]


def _env_value(name: str, default: str | None = None) -> str | None:
    value = os.getenv(name)
    if value is None or value.strip() == "":
        return default
    return value.strip()


def _env_float(name: str, default: float) -> float:
    value = _env_value(name)
    if value is None:
        return default
    try:
        return float(value)
    except ValueError:
        return default


def _env_int(name: str, default: int) -> int:
    value = _env_value(name)
    if value is None:
        return default
    try:
        return int(value)
    except ValueError:
        return default


def _env_bool(name: str, default: bool) -> bool:
    value = _env_value(name)
    if value is None:
        return default
    normalized = value.lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    return default


@dataclass(frozen=True)
class ManualDemoSettings:
    """Runtime settings for the manual web demo.

    Motion pulse semantics come from the real ``/go2w/motion`` Action schema
    (``go2w_motion_interfaces/action/MotionCommand.action``):
    ``MODE_TIMED_VELOCITY`` carries ``vx/vy/yaw_rate/duration_sec`` and
    ``MODE_RELATIVE_YAW`` carries ``relative_yaw_deg/max_yaw_rate``.
    """

    host: str = "127.0.0.1"
    port: int = 8765

    camera_max_fps: float = 10.0
    camera_stale_seconds: float = 1.0

    llm_enabled: bool = False
    llm_interval_seconds: float = 5.0
    llm_hide_low_confidence: bool = True

    control_enabled_default: bool = False
    deadman_ms: int = 300
    ros_worker_deadman_ms: int = 500
    repeat_interval_ms: int = 250

    allow_forward: bool = True
    allow_backward: bool = False
    allow_strafe: bool = False
    allow_turn: bool = True
    allow_turn_override: bool = False
    turn_step_deg: float = 8.0

    # Motion velocities for the continuous hold mode. While a key is held the
    # demo keeps one timed-velocity goal running (renewed on completion) so the
    # robot moves continuously; turns use a continuous ``yaw_rate`` so there is
    # no per-step angle limit.
    pulse_vx: float = 0.12
    pulse_vx_backward: float = 0.10
    pulse_vy: float = 0.06
    turn_yaw_rate: float = 1.0
    hold_duration_sec: float = 30.0

    # Forward safety: the formal LiDAR gate is published on /go2w/safety/*.
    min_front_clearance_m: float = 0.30

    save_analysis_frames: bool = False

    runtime_dir: str = "outputs/manual_web_demo/runtime"
    logs_dir: str = "outputs/manual_web_demo/logs"
    analysis_frames_dir: str = "outputs/manual_web_demo/analysis_frames"
    slam_map_snapshot: str = "outputs/autonomous_search/runtime/slam_map_3d.json"

    # ROS worker subprocess. It runs under the system Python with the ROS
    # environment sourced by the start script; the web process only spawns it.
    ros_worker_cmd: tuple[str, ...] = (
        "/usr/bin/python3",
        "scripts/go2w/manual_web_demo_ros_worker.py",
    )

    # The demo's own SiliconFlow interval guard: whether the project's
    # SiliconFlow configuration is loadable. Values are not duplicated here.
    project_root: str = str(_PROJECT_ROOT)

    @property
    def deadman_sec(self) -> float:
        return max(0.05, self.deadman_ms / 1000.0)

    @property
    def ros_worker_deadman_sec(self) -> float:
        return max(0.05, self.ros_worker_deadman_ms / 1000.0)

    @property
    def repeat_interval_sec(self) -> float:
        return max(0.05, self.repeat_interval_ms / 1000.0)

    @property
    def runtime_dir_path(self) -> Path:
        return _PROJECT_ROOT / self.runtime_dir

    @property
    def logs_dir_path(self) -> Path:
        return _PROJECT_ROOT / self.logs_dir

    @property
    def analysis_frames_dir_path(self) -> Path:
        return _PROJECT_ROOT / self.analysis_frames_dir

    @property
    def latest_frame_path(self) -> Path:
        return self.runtime_dir_path / "latest.jpg"

    @property
    def camera_status_path(self) -> Path:
        return self.runtime_dir_path / "camera_status.json"

    @property
    def worker_pid_path(self) -> Path:
        return self.runtime_dir_path / "worker.pid"

    @property
    def web_pid_path(self) -> Path:
        return self.runtime_dir_path / "web.pid"

    @property
    def slam_map_snapshot_path(self) -> Path:
        path = Path(self.slam_map_snapshot)
        return path if path.is_absolute() else _PROJECT_ROOT / path


def get_manual_demo_settings() -> ManualDemoSettings:
    """Load demo settings from ``MANUAL_DEMO_*`` environment variables.

    ``.env`` is loaded so operator unlocks (ALLOW_BACKWARD / ALLOW_STRAFE /
    ALLOW_TURN_OVERRIDE / LLM off) survive a demo restart.
    """
    try:
        from dotenv import load_dotenv

        load_dotenv(_PROJECT_ROOT / ".env")
    except ModuleNotFoundError:
        pass
    return ManualDemoSettings(
        host=_env_value("MANUAL_DEMO_HOST", "127.0.0.1") or "127.0.0.1",
        port=_env_int("MANUAL_DEMO_PORT", 8765),
        camera_max_fps=_env_float("MANUAL_DEMO_CAMERA_MAX_FPS", 10.0),
        camera_stale_seconds=_env_float(
            "MANUAL_DEMO_CAMERA_STALE_SECONDS", 1.0
        ),
        llm_enabled=_env_bool("MANUAL_DEMO_LLM_ENABLED", False),
        llm_interval_seconds=_env_float(
            "MANUAL_DEMO_LLM_INTERVAL_SECONDS", 5.0
        ),
        llm_hide_low_confidence=_env_bool(
            "MANUAL_DEMO_LLM_HIDE_LOW_CONFIDENCE", True
        ),
        control_enabled_default=_env_bool(
            "MANUAL_DEMO_CONTROL_ENABLED", False
        ),
        deadman_ms=_env_int("MANUAL_DEMO_DEADMAN_MS", 300),
        ros_worker_deadman_ms=_env_int(
            "MANUAL_DEMO_ROS_WORKER_DEADMAN_MS", 500
        ),
        repeat_interval_ms=_env_int("MANUAL_DEMO_REPEAT_INTERVAL_MS", 250),
        allow_forward=_env_bool("MANUAL_DEMO_ALLOW_FORWARD", True),
        allow_backward=_env_bool("MANUAL_DEMO_ALLOW_BACKWARD", False),
        allow_strafe=_env_bool("MANUAL_DEMO_ALLOW_STRAFE", False),
        allow_turn=_env_bool("MANUAL_DEMO_ALLOW_TURN", True),
        allow_turn_override=_env_bool(
            "MANUAL_DEMO_ALLOW_TURN_OVERRIDE", False
        ),
        turn_step_deg=_env_float("MANUAL_DEMO_TURN_STEP_DEG", 8.0),
        pulse_vx=_env_float("MANUAL_DEMO_PULSE_VX", 0.12),
        pulse_vx_backward=_env_float("MANUAL_DEMO_PULSE_VX_BACKWARD", 0.10),
        pulse_vy=_env_float("MANUAL_DEMO_PULSE_VY", 0.06),
        turn_yaw_rate=_env_float("MANUAL_DEMO_TURN_YAW_RATE", 1.0),
        hold_duration_sec=_env_float("MANUAL_DEMO_HOLD_DURATION_SEC", 30.0),
        min_front_clearance_m=_env_float(
            "MANUAL_DEMO_MIN_FRONT_CLEARANCE_M", 0.30
        ),
        save_analysis_frames=_env_bool(
            "MANUAL_DEMO_SAVE_ANALYSIS_FRAMES", False
        ),
        runtime_dir=_env_value(
            "MANUAL_DEMO_RUNTIME_DIR", "outputs/manual_web_demo/runtime"
        ),
        logs_dir=_env_value(
            "MANUAL_DEMO_LOGS_DIR", "outputs/manual_web_demo/logs"
        ),
        analysis_frames_dir=_env_value(
            "MANUAL_DEMO_ANALYSIS_FRAMES_DIR",
            "outputs/manual_web_demo/analysis_frames",
        ),
        slam_map_snapshot=_env_value(
            "GO2W_SLAM_MAP_SNAPSHOT",
            "outputs/autonomous_search/runtime/slam_map_3d.json",
        ),
        ros_worker_cmd=tuple(
            (
                _env_value("MANUAL_DEMO_ROS_WORKER", "")
                or "/usr/bin/python3 scripts/go2w/manual_web_demo_ros_worker.py"
            ).split()
        ),
        project_root=str(_PROJECT_ROOT),
    )
