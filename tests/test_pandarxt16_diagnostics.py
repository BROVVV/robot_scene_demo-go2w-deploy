"""Tests for the PandarXT-16 per-frame diagnostics."""

from __future__ import annotations

import math

import pytest

from go2w_lidar_preprocessor.hesai_diagnostics import (
    analyze_pandar_frame,
    azimuth_occupied_bins,
    estimate_self_occlusion_fraction,
)


def test_analyze_pandar_frame_reports_provenance() -> None:
    diag = analyze_pandar_frame(
        xyz=[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [2.0, 0.0, 0.0]],
        ring=[0, 1, 2, 3],
        point_timestamp=[0.0, 0.01, 0.02, 0.03],
        frame_id="pandarxt16_link_unvalidated",
    )
    assert diag["total_points"] == 4
    assert diag["zero_or_near_zero_points"] == 1
    assert diag["zero_return_fraction"] == 0.25
    assert diag["valid_return_fraction"] == 0.75
    assert diag["diagnostic_only"] is True
    assert diag["authorizes_motion"] is False
    assert diag["transform_validated"] is False


def test_analyze_pandar_frame_ring_coverage() -> None:
    diag = analyze_pandar_frame(
        xyz=[[1.0, 0.0, 0.0]] * 16,
        ring=list(range(16)),
        point_timestamp=[0.0] * 16,
    )
    assert diag["valid_rings"] == list(range(16))
    assert diag["all_rings_have_valid_returns"] is True


def test_analyze_pandar_frame_freshness() -> None:
    diag = analyze_pandar_frame(
        xyz=[[1.0, 0.0, 0.0]],
        ring=[0],
        point_timestamp=[0.0],
        freshness_seconds=0.2,
    )
    assert diag["fresh"] is True
    stale = analyze_pandar_frame(
        xyz=[[1.0, 0.0, 0.0]],
        ring=[0],
        point_timestamp=[0.0],
        freshness_seconds=math.nan,
    )
    assert stale["fresh"] is False


def test_estimate_self_occlusion_fraction() -> None:
    body = {
        "x_min": -0.6, "x_max": 0.6,
        "y_min": -0.3, "y_max": 0.3,
        "z_min": -0.6, "z_max": 0.2,
    }
    points = [
        [0.0, 0.0, 0.0],   # zero-range return -> filtered, not counted
        [0.0, 0.0, -0.4],  # inside body region
        [2.0, 0.0, 0.0],   # outside
        [0.0, 2.0, 0.0],   # outside
    ]
    result = estimate_self_occlusion_fraction(xyz=points, body_region=body)
    assert result["valid_points"] == 3
    assert result["inside_body_region"] == 1
    assert result["self_occlusion_fraction"] == pytest.approx(1 / 3, abs=1e-5)


def test_azimuth_occupied_bins() -> None:
    points = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
    result = azimuth_occupied_bins(xyz=points, bearing_bin_count=72)
    assert result["occupied_bins"] == 4
    assert result["occupied_fraction"] == pytest.approx(4 / 72, abs=1e-4)
    assert result["empty_bins"] == 68
