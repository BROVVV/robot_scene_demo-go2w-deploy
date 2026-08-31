"""Tests for the PandarXT-16 clock tiering and statistics."""

from __future__ import annotations

import math

import pytest

from app.live_robot.pandar_clock import (
    DEFAULT_PANDAR_CLOCK_TIER,
    PandarClockStatistics,
    PandarClockTier,
    compute_pandar_clock_statistics,
    require_tier_for_metric,
)


def test_default_tier_is_host_receive_time_only() -> None:
    assert DEFAULT_PANDAR_CLOCK_TIER == PandarClockTier.HOST_RECEIVE_TIME_ONLY


def test_enum_values() -> None:
    assert PandarClockTier.UNVALIDATED.value == "unvalidated"
    assert PandarClockTier.HOST_RECEIVE_TIME_ONLY.value == "host_receive_time_only"
    assert PandarClockTier.HOST_CLOCK_MODEL_VALIDATED.value == "host_clock_model_validated"
    assert PandarClockTier.PTP_VALIDATED.value == "ptp_validated"


def test_nominal_10hz_statistics() -> None:
    stamps = [index * 0.1 for index in range(11)]
    arrivals = [index * 0.1 for index in range(11)]
    stats = compute_pandar_clock_statistics(
        header_stamps=stamps, arrival_monotonic=arrivals
    )
    assert stats.header_rate_hz == pytest.approx(10.0, abs=1e-6)
    assert stats.arrival_rate_hz == pytest.approx(10.0, abs=1e-6)
    assert stats.header_delta_median_s == pytest.approx(0.1, abs=1e-6)
    assert stats.missing_header_delta_count == 0
    assert stats.tier == DEFAULT_PANDAR_CLOCK_TIER


def test_drift_detected() -> None:
    # Header time advances faster than wall/receive time => positive drift.
    headers = [index * 0.11 for index in range(21)]
    arrivals = [index * 0.10 for index in range(21)]
    stats = compute_pandar_clock_statistics(
        header_stamps=headers, arrival_monotonic=arrivals
    )
    assert stats.drift_s_per_s is not None
    assert stats.drift_s_per_s > 0.0


def test_dual_lidar_apparent_offset() -> None:
    # Pandar headers are consistently +0.05 s ahead of the built-in header.
    stats = compute_pandar_clock_statistics(
        header_stamps=[1.05, 1.15, 1.25, 1.35],
        arrival_monotonic=[0.05, 0.15, 0.25, 0.35],
        other_sensor_header_stamps=[1.0, 1.1, 1.2, 1.3],
    )
    assert stats.dual_lidar_apparent_time_offset_s == pytest.approx(0.05, abs=1e-6)


def test_dual_lidar_offset_uses_most_recent_other_header() -> None:
    stats = compute_pandar_clock_statistics(
        header_stamps=[0.95, 1.05, 1.15],
        arrival_monotonic=[0.0, 0.1, 0.2],
        other_sensor_header_stamps=[1.0, 2.0, 3.0],
    )
    # For pandar header 0.95 no prior builtin header exists; for 1.05 the most
    # recent prior builtin header is 1.0 (offset 0.05); for 1.15 the most
    # recent prior is 1.0 (offset 0.15). Median of [0.05, 0.15] = 0.10.
    assert stats.dual_lidar_apparent_time_offset_s == pytest.approx(0.10, abs=1e-6)


def test_non_increasing_deltas_discarded() -> None:
    stats = compute_pandar_clock_statistics(
        header_stamps=[0.0, 0.1, 0.05, 0.2],
        arrival_monotonic=[0.0, 0.1, 0.2, 0.3],
    )
    assert stats.missing_header_delta_count == 1
    assert "non-increasing" in stats.warnings[0]


def test_metric_fusion_tier_gate() -> None:
    assert require_tier_for_metric(PandarClockTier.UNVALIDATED) is False
    assert require_tier_for_metric(PandarClockTier.HOST_RECEIVE_TIME_ONLY) is False
    assert require_tier_for_metric(PandarClockTier.HOST_CLOCK_MODEL_VALIDATED) is True
    assert require_tier_for_metric(PandarClockTier.PTP_VALIDATED) is True
    stats = PandarClockStatistics(tier=PandarClockTier.HOST_RECEIVE_TIME_ONLY, samples=0)
    assert stats.tier_allows_metric_fusion() is False
