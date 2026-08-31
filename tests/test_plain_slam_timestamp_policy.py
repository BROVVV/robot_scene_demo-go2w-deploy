# Copyright 2026 robot_scene_demo maintainers

"""Timestamp auto-policy validation (plan §5.2 / §18.2).

Python reference mirror of the C++ timestamp_policy logic.  It validates the
exact policy the C++ pandar_slam_adapter implements against the seven cases
required by the plan, including the "never guess silently" rule.
"""

from __future__ import annotations

import math

HEADER_S = 1700000000.0
TOLERANCE_S = 5.0
SCAN_PERIOD_S = 0.10


def resolve_timestamps(raw: list[float], header: float = HEADER_S) -> tuple[str, list[float], bool]:
    """Mirror of go2w_plain_slam_bridge/timestamp_policy.hpp.

    Returns (mode, timestamps_s, non_monotonic).
    """
    n = len(raw)
    if n == 0:
        return "TIMESTAMP_SYNTHETIC", [], False
    for value in raw:
        if not math.isfinite(value):
            return "TIMESTAMP_ERROR", [], False
    if all(abs(v) < 1e-12 for v in raw):
        timestamps = [header + SCAN_PERIOD_S * i / (n - 1) for i in range(n)] if n > 1 else [header]
        return "TIMESTAMP_SYNTHETIC", timestamps, False

    sorted_vals = sorted(raw)
    mid = n // 2
    median = sorted_vals[mid] if n % 2 == 1 else 0.5 * (sorted_vals[mid - 1] + sorted_vals[mid])
    span = sorted_vals[-1] - sorted_vals[0]

    if abs(median - header) < TOLERANCE_S:
        mode = "ABSOLUTE_SECONDS"
        timestamps = list(raw)
    else:
        converted = False
        for scale in (1e-9, 1e-6, 1e-3):
            if abs(median * scale - header) < TOLERANCE_S:
                timestamps = [v * scale for v in raw]
                mode = "CONVERTED_UNITS"
                converted = True
                break
        if not converted:
            if abs(median) < 10.0 and span < 0.2:
                timestamps = [header + v for v in raw]
                mode = "RELATIVE_SCAN"
            else:
                return "TIMESTAMP_ERROR", [], False

    non_monotonic = any(
        timestamps[i] + 1e-6 < timestamps[i - 1] for i in range(1, len(timestamps))
    )
    return mode, timestamps, non_monotonic


def test_absolute_seconds() -> None:
    mode, timestamps, non_monotonic = resolve_timestamps(
        [HEADER_S, HEADER_S + 0.02, HEADER_S + 0.04]
    )
    assert mode == "ABSOLUTE_SECONDS"
    assert timestamps[2] == HEADER_S + 0.04
    assert not non_monotonic


def test_relative_scan_time() -> None:
    mode, timestamps, non_monotonic = resolve_timestamps([0.0, 0.033, 0.066, 0.099])
    assert mode == "RELATIVE_SCAN"
    assert timestamps[3] == HEADER_S + 0.099
    assert not non_monotonic


def test_zero_timestamps_synthetic_linear() -> None:
    mode, timestamps, non_monotonic = resolve_timestamps([0.0, 0.0, 0.0, 0.0])
    assert mode == "TIMESTAMP_SYNTHETIC"
    assert abs(timestamps[0] - HEADER_S) < 1e-9
    assert abs(timestamps[3] - (HEADER_S + SCAN_PERIOD_S)) < 1e-9
    assert not non_monotonic


def test_nan_is_error_never_silent() -> None:
    mode, _, _ = resolve_timestamps([HEADER_S, math.nan, HEADER_S])
    assert mode == "TIMESTAMP_ERROR"


def test_microseconds_converted() -> None:
    us_epoch = HEADER_S * 1e6
    mode, timestamps, _ = resolve_timestamps([us_epoch, us_epoch + 20000.0])
    assert mode == "CONVERTED_UNITS"
    assert abs(timestamps[1] - (HEADER_S + 0.02)) < 1e-6


def test_nanoseconds_converted() -> None:
    ns_epoch = HEADER_S * 1e9
    mode, timestamps, _ = resolve_timestamps([ns_epoch, ns_epoch + 50000000.0])
    assert mode == "CONVERTED_UNITS"
    assert abs(timestamps[1] - (HEADER_S + 0.05)) < 1e-6


def test_non_monotonic_outlier_flagged() -> None:
    mode, _, non_monotonic = resolve_timestamps(
        [HEADER_S + 0.0, HEADER_S - 0.1, HEADER_S + 0.03]
    )
    assert mode == "ABSOLUTE_SECONDS"
    assert non_monotonic


def test_unclassifiable_is_error() -> None:
    # Large magnitude, unaligned with header, long span: refuse, don't guess.
    mode, _, _ = resolve_timestamps([123456789.0, 123456790.0])
    assert mode == "TIMESTAMP_ERROR"


def test_empty_input_synthetic() -> None:
    mode, timestamps, _ = resolve_timestamps([])
    assert mode == "TIMESTAMP_SYNTHETIC"
    assert timestamps == []
