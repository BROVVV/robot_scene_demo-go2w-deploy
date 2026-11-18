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

# 只读硬门的统一理由串。后端的 403、WS 的拒绝回执与前端横幅共用这一句，
# 免得三处各写一份、日后对不上。
DISPLAY_ONLY_REASON = (
    "display_only: 实体遥控器建图模式，网页只读 —— "
    "不发送运动、急停或自主搜索请求；机器狗请用实体遥控器操作"
)


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
    web_video_max_width: int = 0
    web_video_quality: int = 70
    camera_stale_seconds: float = 1.0

    llm_enabled: bool = False
    llm_interval_seconds: float = 5.0
    llm_hide_low_confidence: bool = True

    control_enabled_default: bool = False
    # 实体遥控器建图模式（计划书 §6.2 F）。为真时这个 Web 进程是**只读**的：
    # 地图、相机、状态照常看，但任何能写运动的入口（启用键盘控制、
    # 方向键、急停/复位、起自主搜索、开 VLM 定时识别）一律在 API/WS 入口被
    # 统一硬门挡住。挡在入口而不是散在各处条件判断里 —— 计划书明确要求。
    display_only: bool = False
    # Runtime profile exposed to the reset supervisor.  These are enums set by
    # the launchers, never arbitrary shell fragments from an HTTP request.
    webui_mode: str = "autonomous_search"
    active_detector: str = "grounded_sam"
    runtime_admin_enabled: bool = False
    web_3d_map_enabled: bool = True
    web_2d_map_enabled: bool = False
    runtime_reset_supervisor: str = "scripts/go2w/reset_webui_runtime.sh"
    mapping_topology_snapshot: str = "outputs/autonomous_search/runtime/semantic_topology.json"
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
    # Desktop keeps the ROS camera. Jetson may prefer the local HTTP writer,
    # with automatic ROS fallback when its independent heartbeat expires.
    camera_owner: str = "ros"
    # Opt-in for measured platforms; the existing executor remains the default.
    ros_executor: str = "multi"

    runtime_dir: str = "outputs/manual_web_demo/runtime"
    logs_dir: str = "outputs/manual_web_demo/logs"
    analysis_frames_dir: str = "outputs/manual_web_demo/analysis_frames"
    slam_map_snapshot: str = "outputs/manual_web_demo/runtime/slam_map_3d.json"
    # §2.3：三维对象与地图靠 map_epoch 配对，三个路径都是显式配置 —— 不按
    # 文件修改时间或目录 glob 去猜"哪份数据属于当前地图"（那条路 2026-09-17
    # 已被证伪：glob 挑中的是 09-10 的归档，当天的物体一个都没上屏）。
    slam_map_meta: str = "outputs/manual_web_demo/runtime/slam_map_meta.json"
    semantic_objects_snapshot: str = (
        "outputs/manual_web_demo/runtime/semantic_objects.json"
    )
    target_watch_request: str = (
        "outputs/remote_control_mapping/runtime/target_watch_request.json"
    )
    target_watch_state: str = (
        "outputs/remote_control_mapping/runtime/target_watch_state.json"
    )
    target_watch_evidence_dir: str = (
        "outputs/remote_control_mapping/runtime/target_watch_evidence"
    )

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

    @property
    def slam_map_meta_path(self) -> Path:
        path = Path(self.slam_map_meta)
        return path if path.is_absolute() else _PROJECT_ROOT / path

    @property
    def semantic_objects_snapshot_path(self) -> Path:
        path = Path(self.semantic_objects_snapshot)
        return path if path.is_absolute() else _PROJECT_ROOT / path

    @property
    def mapping_topology_snapshot_path(self) -> Path:
        path = Path(self.mapping_topology_snapshot)
        return path if path.is_absolute() else _PROJECT_ROOT / path

    @property
    def target_watch_request_path(self) -> Path:
        path = Path(self.target_watch_request)
        return path if path.is_absolute() else _PROJECT_ROOT / path

    @property
    def target_watch_state_path(self) -> Path:
        path = Path(self.target_watch_state)
        return path if path.is_absolute() else _PROJECT_ROOT / path

    @property
    def target_watch_evidence_dir_path(self) -> Path:
        path = Path(self.target_watch_evidence_dir)
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
        web_video_max_width=_env_int("GO2W_WEB_VIDEO_MAX_WIDTH", 0),
        web_video_quality=_env_int("GO2W_WEB_VIDEO_QUALITY", 70),
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
        display_only=_env_bool("MANUAL_DEMO_DISPLAY_ONLY", False),
        webui_mode=_env_value(
            "GO2W_WEBUI_MODE",
            "teleop_mapping" if _env_bool("MANUAL_DEMO_DISPLAY_ONLY", False)
            else "autonomous_search",
        ) or "autonomous_search",
        active_detector=_env_value(
            "GO2W_ACTIVE_DETECTOR",
            _env_value("AUTONOMOUS_SEARCH_DEFAULT_DETECTOR", "grounded_sam"),
        ) or "grounded_sam",
        runtime_admin_enabled=_env_bool("GO2W_RUNTIME_ADMIN_ENABLED", False),
        web_3d_map_enabled=_env_bool("GO2W_WEB_3D_MAP_ENABLED", True),
        web_2d_map_enabled=_env_bool("GO2W_WEB_2D_MAP_ENABLED", False),
        runtime_reset_supervisor=_env_value(
            "GO2W_RUNTIME_RESET_SUPERVISOR",
            "scripts/go2w/reset_webui_runtime.sh",
        ) or "scripts/go2w/reset_webui_runtime.sh",
        mapping_topology_snapshot=_env_value(
            "GO2W_MAPPING_TOPOLOGY_SNAPSHOT",
            "outputs/autonomous_search/runtime/semantic_topology.json") or "",
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
        camera_owner=_env_value("MANUAL_DEMO_CAMERA_OWNER", "ros"),
        ros_executor=_env_value("MANUAL_DEMO_ROS_EXECUTOR", "multi"),
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
            "outputs/manual_web_demo/runtime/slam_map_3d.json",
        ),
        slam_map_meta=_env_value(
            "GO2W_SLAM_MAP_META",
            "outputs/manual_web_demo/runtime/slam_map_meta.json",
        ),
        semantic_objects_snapshot=_env_value(
            "GO2W_SEMANTIC_OBJECTS_SNAPSHOT",
            "outputs/manual_web_demo/runtime/semantic_objects.json",
        ),
        target_watch_request=_env_value(
            "GO2W_TARGET_WATCH_REQUEST",
            "outputs/remote_control_mapping/runtime/target_watch_request.json",
        ),
        target_watch_state=_env_value(
            "GO2W_TARGET_WATCH_STATE",
            "outputs/remote_control_mapping/runtime/target_watch_state.json",
        ),
        target_watch_evidence_dir=_env_value(
            "GO2W_TARGET_WATCH_EVIDENCE_DIR",
            "outputs/remote_control_mapping/runtime/target_watch_evidence",
        ),
        ros_worker_cmd=tuple(
            (
                _env_value("MANUAL_DEMO_ROS_WORKER", "")
                or "/usr/bin/python3 scripts/go2w/manual_web_demo_ros_worker.py"
            ).split()
        ),
        project_root=str(_PROJECT_ROOT),
    )
