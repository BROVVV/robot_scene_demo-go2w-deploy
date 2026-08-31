"""PandarXT-16 timestamp tiering and clock statistics.

The Pandar runs PTP Free Run and currently uses host receive time, so its
point/header timestamps are only weakly ordered. Before any formal metric
fusion uses Pandar time, the tier must be checked explicitly: a higher tier
(validated host clock model or PTP) is required before Pandar geometry may be
treated as a metric co-registered source.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Iterable, Sequence


class PandarClockTier(str, Enum):
    UNVALIDATED = "unvalidated"
    HOST_RECEIVE_TIME_ONLY = "host_receive_time_only"
    HOST_CLOCK_MODEL_VALIDATED = "host_clock_model_validated"
    PTP_VALIDATED = "ptp_validated"


# Default for the current rig: the official driver is configured with
# use_timestamp_type=1 (host receive time), PTP is Free Run.
DEFAULT_PANDAR_CLOCK_TIER = PandarClockTier.HOST_RECEIVE_TIME_ONLY

# Tiers whose timestamps may seed formal metric fusion geometry.
_METRIC_READY_TIERS = {
    PandarClockTier.HOST_CLOCK_MODEL_VALIDATED,
    PandarClockTier.PTP_VALIDATED,
}


@dataclass(frozen=True)
class PandarClockStatistics:
    tier: PandarClockTier
    samples: int
    header_rate_hz: float | None = None
    arrival_rate_hz: float | None = None
    header_delta_median_s: float | None = None
    arrival_delta_median_s: float | None = None
    header_delta_std_s: float | None = None
    arrival_delta_std_s: float | None = None
    jitter_s: float | None = None
    drift_s_per_s: float | None = None
    dual_lidar_apparent_time_offset_s: float | None = None
    long_run_stability_s: float | None = None
    missing_header_delta_count: int = 0
    warnings: list[str] = field(default_factory=list)

    def tier_allows_metric_fusion(self) -> bool:
        return self.tier in _METRIC_READY_TIERS


def _median(values: Sequence[float]) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2 == 1:
        return float(ordered[middle])
    return float((ordered[middle - 1] + ordered[middle]) / 2.0)


def _std(values: Sequence[float]) -> float | None:
    if len(values) < 2:
        return None
    mean = sum(values) / len(values)
    variance = sum((value - mean) ** 2 for value in values) / (len(values) - 1)
    return math.sqrt(variance) if variance >= 0.0 else None


def compute_pandar_clock_statistics(
    *,
    header_stamps: Iterable[float],
    arrival_monotonic: Iterable[float],
    point_times: Iterable[float] | None = None,
    other_sensor_header_stamps: Iterable[float] | None = None,
    tier: PandarClockTier = DEFAULT_PANDAR_CLOCK_TIER,
) -> PandarClockStatistics:
    """Compute Pandar clock statistics from header/arrival timestamp streams.

    ``header_stamps`` and ``arrival_monotonic`` must be aligned element-wise
    (same length). ``other_sensor_header_stamps`` (e.g. the built-in LiDAR
    headers in the same time domain) yields the apparent dual-lidar time
    offset: the median ``pandar_header - most_recent_other_header`` gap.
    """
    headers = [float(value) for value in header_stamps if math.isfinite(float(value))]
    arrivals = [float(value) for value in arrival_monotonic if math.isfinite(float(value))]
    count = min(len(headers), len(arrivals))

    header_deltas: list[float] = []
    arrival_deltas: list[float] = []
    missing = 0
    for index in range(1, count):
        header_delta = headers[index] - headers[index - 1]
        arrival_delta = arrivals[index] - arrivals[index - 1]
        if header_delta <= 0.0:
            missing += 1
            continue
        header_deltas.append(header_delta)
        if arrival_delta > 0.0:
            arrival_deltas.append(arrival_delta)

    header_rate = 1.0 / _median(header_deltas) if header_deltas else None
    arrival_rate = 1.0 / _median(arrival_deltas) if arrival_deltas else None
    header_delta_std = _std(header_deltas)
    arrival_delta_std = _std(arrival_deltas)
    # Jitter: robust spread of the header interval (std, or IQR fallback).
    jitter = header_delta_std if header_delta_std is not None else None

    drift = None
    if header_deltas and arrival_deltas and len(header_deltas) >= 2:
        # Linear drift of header time relative to wall/receive time.
        header_time_span = headers[count - 1] - headers[0]
        wall_span = arrivals[count - 1] - arrivals[0]
        if wall_span > 0.0 and header_time_span > 0.0:
            drift = (header_time_span - wall_span) / wall_span

    offset = None
    if other_sensor_header_stamps is not None:
        others = sorted(
            float(value)
            for value in other_sensor_header_stamps
            if math.isfinite(float(value))
        )
        offsets = []
        for header in headers[:count]:
            prior = [value for value in others if value <= header]
            if prior:
                offsets.append(header - max(prior))
        offset = _median(offsets) if offsets else None

    long_run_stability = None
    if header_deltas and len(header_deltas) >= 16:
        # Sample-to-sample variability of the median interval over the run.
        long_run_stability = header_delta_std if header_delta_std is not None else None

    point_span = None
    if point_times is not None:
        points = [float(value) for value in point_times if math.isfinite(float(value))]
        if points:
            point_span = max(points) - min(points)

    warnings: list[str] = []
    if missing:
        warnings.append(f"{missing} non-increasing header deltas discarded")
    if point_span is not None and point_span <= 0.0:
        warnings.append("point timestamp span is non-positive")

    return PandarClockStatistics(
        tier=tier,
        samples=count,
        header_rate_hz=header_rate,
        arrival_rate_hz=arrival_rate,
        header_delta_median_s=_median(header_deltas),
        arrival_delta_median_s=_median(arrival_deltas),
        header_delta_std_s=header_delta_std,
        arrival_delta_std_s=arrival_delta_std,
        jitter_s=jitter,
        drift_s_per_s=drift,
        dual_lidar_apparent_time_offset_s=offset,
        long_run_stability_s=long_run_stability,
        missing_header_delta_count=missing,
        warnings=warnings,
    )


def require_tier_for_metric(tier: PandarClockTier) -> bool:
    """Explicit gate: is this tier trusted enough for formal metric fusion?"""
    return tier in _METRIC_READY_TIERS
