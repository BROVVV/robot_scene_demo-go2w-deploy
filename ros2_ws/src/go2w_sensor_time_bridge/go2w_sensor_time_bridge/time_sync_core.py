"""Clock fitting and configuration helpers without ROS dependencies."""

from __future__ import annotations

import math
from bisect import bisect_left
from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class ClockFit:
    scale: float
    offset_seconds: float
    drift_ppm: float
    fit_rmse_ms: float
    sample_count: int
    duration_sec: float
    stable: bool


@dataclass(frozen=True)
class RelativeClockFit:
    reference_offset_seconds: float
    relative_drift_ppm: float
    fit_rmse_ms: float
    sample_count: int
    duration_sec: float
    maximum_pair_receive_delta_ms: float
    stable: bool


def fit_clock(
    samples: list[tuple[float, float]],
    *,
    minimum_samples: int = 100,
    minimum_duration_sec: float = 120.0,
    maximum_rmse_ms: float = 10.0,
    maximum_abs_drift_ppm: float = 1000.0,
) -> ClockFit:
    if len(samples) < 2:
        raise ValueError("at least two clock samples are required")
    x0 = samples[0][0]
    y0 = samples[0][1]
    xs = [sensor - x0 for sensor, _ in samples]
    ys = [receive - y0 for _, receive in samples]
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    denominator = sum((value - mean_x) ** 2 for value in xs)
    if denominator <= 0.0:
        raise ValueError("sensor timestamps do not advance")
    scale = sum(
        (x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)
    ) / denominator
    offset = (y0 + mean_y) - scale * (x0 + mean_x)
    residuals = [receive - (scale * sensor + offset) for sensor, receive in samples]
    rmse_ms = math.sqrt(sum(value * value for value in residuals) / len(residuals)) * 1000.0
    duration = max(receive for _, receive in samples) - min(
        receive for _, receive in samples
    )
    drift_ppm = (scale - 1.0) * 1_000_000.0
    stable = (
        len(samples) >= minimum_samples
        and duration >= minimum_duration_sec
        and rmse_ms <= maximum_rmse_ms
        and abs(drift_ppm) <= maximum_abs_drift_ppm
    )
    return ClockFit(
        scale=scale,
        offset_seconds=offset,
        drift_ppm=drift_ppm,
        fit_rmse_ms=rmse_ms,
        sample_count=len(samples),
        duration_sec=duration,
        stable=stable,
    )


def transform_seconds(sensor_seconds: float, scale: float, offset: float) -> float:
    value = scale * sensor_seconds + offset
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("transformed timestamp is invalid")
    return value


def fit_cloud_imu_relative_clock(
    cloud_samples: list[tuple[float, float]],
    imu_samples: list[tuple[float, float]],
    *,
    minimum_samples: int = 100,
    minimum_duration_sec: float = 120.0,
    maximum_rmse_ms: float = 10.0,
    maximum_abs_drift_ppm: float = 1000.0,
    maximum_abs_reference_offset_seconds: float = 1.0,
) -> RelativeClockFit:
    """Fit cloud-minus-IMU header offset for messages received together.

    A constant offset is allowed because a cloud header may describe the start or
    end of a scan while its callback occurs after the scan.  Stability is based on
    offset drift and residual jitter, not regression intercepts extrapolated to
    Unix time zero.
    """
    if len(cloud_samples) < 2 or len(imu_samples) < 2:
        raise ValueError("at least two cloud and IMU clock samples are required")
    sorted_imu = sorted(imu_samples, key=lambda item: item[1])
    imu_receive = [item[1] for item in sorted_imu]
    pairs = []
    receive_deltas = []
    for cloud_stamp, cloud_receive in sorted(cloud_samples, key=lambda item: item[1]):
        index = bisect_left(imu_receive, cloud_receive)
        candidates = []
        if index < len(sorted_imu):
            candidates.append(sorted_imu[index])
        if index:
            candidates.append(sorted_imu[index - 1])
        imu_stamp, imu_receive_time = min(
            candidates, key=lambda item: abs(item[1] - cloud_receive)
        )
        pairs.append((cloud_receive, cloud_stamp - imu_stamp))
        receive_deltas.append(abs(cloud_receive - imu_receive_time))

    reference_receive = sum(item[0] for item in pairs) / len(pairs)
    xs = [item[0] - reference_receive for item in pairs]
    offsets = [item[1] for item in pairs]
    reference_offset = sum(offsets) / len(offsets)
    denominator = sum(value * value for value in xs)
    if denominator <= 0.0:
        raise ValueError("cloud receive timestamps do not advance")
    slope = sum(
        x * (offset - reference_offset) for x, offset in zip(xs, offsets)
    ) / denominator
    residuals = [
        offset - (reference_offset + slope * x)
        for x, offset in zip(xs, offsets)
    ]
    rmse_ms = math.sqrt(
        sum(value * value for value in residuals) / len(residuals)
    ) * 1000.0
    duration = max(item[0] for item in pairs) - min(item[0] for item in pairs)
    drift_ppm = slope * 1_000_000.0
    maximum_pair_delta_ms = max(receive_deltas) * 1000.0
    stable = (
        len(pairs) >= minimum_samples
        and duration >= minimum_duration_sec
        and rmse_ms <= maximum_rmse_ms
        and abs(drift_ppm) <= maximum_abs_drift_ppm
        and abs(reference_offset) <= maximum_abs_reference_offset_seconds
    )
    return RelativeClockFit(
        reference_offset_seconds=reference_offset,
        relative_drift_ppm=drift_ppm,
        fit_rmse_ms=rmse_ms,
        sample_count=len(pairs),
        duration_sec=duration,
        maximum_pair_receive_delta_ms=maximum_pair_delta_ms,
        stable=stable,
    )


def load_time_sync(path: str | Path) -> dict:
    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    required = ("scale", "offset_seconds", "stable", "fit_rmse_ms")
    missing = [key for key in required if key not in payload]
    if missing:
        raise ValueError(f"time sync config missing: {', '.join(missing)}")
    payload["scale"] = float(payload["scale"])
    payload["offset_seconds"] = float(payload["offset_seconds"])
    payload["stable"] = bool(payload["stable"])
    for stream in ("cloud", "imu"):
        if stream in payload:
            stream_config = payload[stream]
            for key in ("scale", "offset_seconds", "stable"):
                if key not in stream_config:
                    raise ValueError(f"time sync {stream} config missing: {key}")
            stream_config["scale"] = float(stream_config["scale"])
            stream_config["offset_seconds"] = float(
                stream_config["offset_seconds"]
            )
            stream_config["stable"] = bool(stream_config["stable"])
    return payload
