"""Environment configuration for the Nav2 bridge."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

from .nav2_models import Nav2Mode, Nav2ValidationError


def env_bool(name: str, default: bool = False) -> bool:
    raw = os.getenv(name)
    if raw is None:
        return default
    value = raw.strip().lower()
    if value in {"true", "1", "yes", "y", "on"}:
        return True
    if value in {"false", "0", "no", "n", "off"}:
        return False
    raise Nav2ValidationError(f"{name} 必须是 true/false/1/0/yes/no")


def resolve_setup_bash(configured: str, ros_distro: str = "humble") -> str:
    """Resolve ROS setup and repair only a recognizable ``setup.bas*`` typo."""
    expanded = str(Path(configured).expanduser())
    if Path(expanded).is_file():
        return expanded
    standard = Path("/opt/ros") / ros_distro / "setup.bash"
    malformed = "setup.bas" in expanded and expanded != str(standard)
    if malformed and standard.is_file():
        return str(standard)
    return expanded


@dataclass(frozen=True)
class Nav2Settings:
    enabled: bool = False
    mode: Nav2Mode = Nav2Mode.DISABLED
    ros_distro: str = "humble"
    system_python: str = "/usr/bin/python3"
    setup_bash: str = "/opt/ros/humble/setup.bash"
    workspace_setup: str = ""
    namespace: str = ""
    map_frame: str = "map"
    odom_frame: str = "odom"
    base_frame: str = "base_link"
    cmd_vel_topic: str = "/go2w/nav2_cmd_vel"
    global_plan_topic: str = "/plan"
    local_plan_topic: str = "/local_plan"
    planner_id: str = "GridBased"
    controller_id: str = "FollowPath"
    goal_checker_id: str = "general_goal_checker"
    behavior_tree: str = ""
    planning_timeout_seconds: float = 30.0
    execution_timeout_seconds: float = 300.0
    feedback_interval_seconds: float = 0.5
    cmd_vel_sample_hz: float = 10.0
    output_dir: str = "outputs/nav2"
    offline_fixture: str = "data/nav2_demo/offline_path_fixture.json"
    allow_execute: bool = False
    footprint_confirmed: bool = False
    emergency_stop_confirmed: bool = False
    no_silent_fallback: bool = True
    legacy_cmd_vel_allowed: bool = False

    @classmethod
    def from_env(cls) -> "Nav2Settings":
        mode = Nav2Mode(os.getenv("NAV2_MODE", "disabled"))
        ros_distro = os.getenv("NAV2_ROS_DISTRO", "humble")
        configured_setup = os.getenv(
            "NAV2_SETUP_BASH", f"/opt/ros/{ros_distro}/setup.bash"
        )
        value = cls(
            enabled=env_bool("NAV2_ENABLED"),
            mode=mode,
            ros_distro=ros_distro,
            system_python=str(Path(os.getenv("NAV2_SYSTEM_PYTHON", "/usr/bin/python3")).expanduser()),
            setup_bash=resolve_setup_bash(configured_setup, ros_distro),
            workspace_setup=os.path.expanduser(os.getenv("NAV2_WORKSPACE_SETUP", "")),
            namespace=os.getenv("NAV2_NAMESPACE", ""),
            map_frame=os.getenv("NAV2_MAP_FRAME", "map"),
            odom_frame=os.getenv("NAV2_ODOM_FRAME", "odom"),
            base_frame=os.getenv("NAV2_BASE_FRAME", "base_link"),
            cmd_vel_topic=os.getenv("NAV2_CMD_VEL_TOPIC", "/go2w/nav2_cmd_vel"),
            global_plan_topic=os.getenv("NAV2_GLOBAL_PLAN_TOPIC", "/plan"),
            local_plan_topic=os.getenv("NAV2_LOCAL_PLAN_TOPIC", "/local_plan"),
            planner_id=os.getenv("NAV2_PLANNER_ID", "GridBased"),
            controller_id=os.getenv("NAV2_CONTROLLER_ID", "FollowPath"),
            goal_checker_id=os.getenv("NAV2_GOAL_CHECKER_ID", "general_goal_checker"),
            behavior_tree=os.getenv("NAV2_BEHAVIOR_TREE", ""),
            planning_timeout_seconds=float(os.getenv("NAV2_PLANNING_TIMEOUT_SECONDS", "30")),
            execution_timeout_seconds=float(os.getenv("NAV2_EXECUTION_TIMEOUT_SECONDS", "300")),
            feedback_interval_seconds=float(os.getenv("NAV2_FEEDBACK_INTERVAL_SECONDS", "0.5")),
            cmd_vel_sample_hz=float(os.getenv("NAV2_CMD_VEL_SAMPLE_HZ", "10")),
            output_dir=os.path.expanduser(os.getenv("NAV2_OUTPUT_DIR", "outputs/nav2")),
            offline_fixture=os.path.expanduser(os.getenv("NAV2_OFFLINE_FIXTURE", "data/nav2_demo/offline_path_fixture.json")),
            allow_execute=env_bool("NAV2_ALLOW_EXECUTE"),
            footprint_confirmed=env_bool("NAV2_FOOTPRINT_CONFIRMED"),
            emergency_stop_confirmed=env_bool("NAV2_EMERGENCY_STOP_CONFIRMED"),
            no_silent_fallback=env_bool("NAV2_NO_SILENT_FALLBACK", True),
            legacy_cmd_vel_allowed=env_bool("NAV2_LEGACY_CMD_VEL_ALLOWED"),
        )
        value.validate()
        return value

    def validate(self) -> None:
        if self.planning_timeout_seconds <= 0 or self.execution_timeout_seconds <= 0:
            raise Nav2ValidationError("Nav2 timeout 必须大于 0")
        if not 1 <= self.cmd_vel_sample_hz <= 30:
            raise Nav2ValidationError("NAV2_CMD_VEL_SAMPLE_HZ 必须在 1～30")
