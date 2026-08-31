"""Navigation2 integration with a ROS-free contract layer."""

# The heavy Nav2 integration modules are imported lazily: they require
# Python >= 3.11 (datetime.UTC) while the Go2-W live runner runs under the
# system Python 3.10 with rclpy.  Submodules remain importable directly on
# both interpreters.
try:  # pragma: no cover - depends on interpreter version
    from .nav2_config import Nav2Settings
    from .nav2_gateway import Nav2Gateway
    from .nav2_models import Nav2JobState, Nav2Mode, Nav2Pose, Nav2Request, Nav2Status
    from .navigation_planning_pipeline import run_video_navigation_planning

    __all__ = [
        "Nav2Gateway",
        "Nav2JobState",
        "Nav2Mode",
        "Nav2Pose",
        "Nav2Request",
        "Nav2Settings",
        "Nav2Status",
        "run_video_navigation_planning",
    ]
except ImportError:  # Python 3.10 without datetime.UTC
    __all__ = []
