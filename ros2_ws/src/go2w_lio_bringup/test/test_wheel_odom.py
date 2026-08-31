import math

import pytest

from go2w_lio_bringup.wheel_odom import (
    fuse_yaw_delta,
    heading_delta_sane,
    step_odometry,
    unwrap_yaw,
)


def test_unwrap_yaw_handles_pi_wraparound():
    assert unwrap_yaw(3.1, None, 0.0) == 0.0
    accumulator = unwrap_yaw(3.1, 3.0, 0.0)
    assert accumulator == pytest.approx(0.1)
    accumulator = unwrap_yaw(-3.1, 3.1, accumulator)
    assert accumulator == pytest.approx(0.1 + (2.0 * math.pi - 6.2))


def test_step_odometry_moves_along_heading():
    x, y = step_odometry(0.0, 0.0, 0.0, [1.0, 1.0, 1.0, 1.0], 0.089)
    assert x == 0.089
    assert y == 0.0
    x, y = step_odometry(0.0, 0.0, math.pi / 2.0, [1.0, 1.0, 1.0, 1.0], 0.089)
    assert x == pytest.approx(0.0)
    assert y == 0.089


def test_step_odometry_pure_turn_does_not_translate():
    x, y = step_odometry(1.0, 2.0, 0.5, [1.0, -1.0, 1.0, -1.0], 0.089)
    assert x == 1.0
    assert y == 2.0


def test_fuse_yaw_delta_blends_with_weight():
    assert fuse_yaw_delta(0.10, 0.09, 0.35) == pytest.approx(
        0.10 + 0.35 * (0.09 - 0.10)
    )
    assert fuse_yaw_delta(0.10, 0.09, 0.0) == pytest.approx(0.10)
    assert fuse_yaw_delta(0.10, 0.09, 1.0) == pytest.approx(0.09)
    assert fuse_yaw_delta(0.10, 0.09, 2.0) == pytest.approx(0.09)


def test_heading_delta_sane_rejects_large_disagreement():
    assert heading_delta_sane(0.01, 0.012, 0.05)
    assert not heading_delta_sane(0.01, 0.2, 0.05)
    # 2*pi wrap should still be considered sane.
    assert heading_delta_sane(0.01, 0.01 + 2.0 * math.pi, 0.05)
