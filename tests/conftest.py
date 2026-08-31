"""Shared test fixtures for the robot_scene_demo test suite.

Pytest discovers this file automatically. It exposes the pure Python modules of
the ``go2w_lidar_preprocessor`` ROS package (lidar_evidence, dual_lidar_
observability, hesai_diagnostics, ...) to the conda test environment by adding
the package source directory to ``sys.path``. Those modules are numpy-only and
have no ROS runtime dependency, so the app-level tests can import them without
a built ROS workspace.
"""

from __future__ import annotations

import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
LIDAR_PREPROCESSOR_SRC = (
    PROJECT_ROOT / "ros2_ws" / "src" / "go2w_lidar_preprocessor"
)
if str(LIDAR_PREPROCESSOR_SRC) not in sys.path:
    sys.path.insert(0, str(LIDAR_PREPROCESSOR_SRC))
