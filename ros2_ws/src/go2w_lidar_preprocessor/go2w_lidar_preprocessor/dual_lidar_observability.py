"""Dual-LiDAR rotation swept-band observability.

The built-in L2 has near-field blind zones that overlap the full-body swept
annulus (the existing ``rotation_observability_report`` shows 720/720 bearings
unobservable for in-place rotation).  The externally mounted PandarXT-16 is a
candidate complement: it sits higher, spins 360 degrees and reaches closer to
the body.  This module computes, per bearing, whether *any validated sensor*
fully observes the swept band ``[footprint_radius, envelope_radius]``.

A sensor fully observes a bearing when it has no unobservable interval inside
that band.  The Pandar contributes to formal observability only when its
extrinsic transform is validated; otherwise it is diagnostic-only and the
sector stays unobservable (fail-closed).
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Iterable


@dataclass(frozen=True)
class BearingObservability:
    bearing_deg: float
    band_length_m: float
    builtin_fully_observes: bool
    pandar_fully_observes: bool
    pandar_extrinsics_validated: bool
    observable: bool
    reason: str


@dataclass(frozen=True)
class DualLidarRotationObservability:
    footprint_radius_m: float
    envelope_radius_m: float
    total_bearings: int
    observable_bearings: int
    unobservable_bearings: list[float]
    builtin_observable_bearings: int
    pandar_observable_bearings: int
    pandar_extrinsics_validated: bool
    full_rotation_observability_valid: bool
    requested_turn_observability_valid: bool
    requested_turn_range_deg: float | None
    details: list[BearingObservability] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "footprint_radius_m": self.footprint_radius_m,
            "envelope_radius_m": self.envelope_radius_m,
            "total_bearings": self.total_bearings,
            "observable_bearings": self.observable_bearings,
            "unobservable_bearings": self.unobservable_bearings,
            "builtin_observable_bearings": self.builtin_observable_bearings,
            "pandar_observable_bearings": self.pandar_observable_bearings,
            "pandar_extrinsics_validated": self.pandar_extrinsics_validated,
            "full_rotation_observability_valid": self.full_rotation_observability_valid,
            "requested_turn_observability_valid": self.requested_turn_observability_valid,
            "requested_turn_range_deg": self.requested_turn_range_deg,
            "details": [b.__dict__ for b in self.details],
        }


def _covered_length(
    band_start: float,
    band_end: float,
    unobservable: Iterable[tuple[float, float]],
    *,
    tolerance_m: float = 1e-6,
) -> float:
    """Length of the band NOT covered by any unobservable interval."""
    merged: list[list[float]] = []
    for start, end in sorted(unobservable):
        start = max(band_start, float(start))
        end = min(band_end, float(end))
        if end <= start + tolerance_m:
            continue
        if not merged or start > merged[-1][1] + tolerance_m:
            merged.append([start, end])
        else:
            merged[-1][1] = max(merged[-1][1], end)
    covered = band_end - band_start - sum(end - start for start, end in merged)
    return max(0.0, covered)


def _merge_intervals(
    intervals: Iterable[tuple[float, float]],
) -> list[tuple[float, float]]:
    merged: list[list[float]] = []
    for start, end in sorted(intervals):
        if not merged or float(start) > merged[-1][1] + 1e-9:
            merged.append([float(start), float(end)])
        else:
            merged[-1][1] = max(merged[-1][1], float(end))
    return [(start, end) for start, end in merged]


def generate_pandar_unobservable_profile(
    *,
    bearings_deg: Iterable[float],
    footprint_radius_m: float,
    envelope_radius_m: float,
    pandar_min_range_m: float = 0.30,
    mount_occlusion_azimuths_deg: Iterable[tuple[float, float]] = (),
) -> dict[float, list[tuple[float, float]]]:
    """Default Pandar blind model for the swept band.

    The Pandar is mounted high on the protective frame and spins 360 degrees,
    so inside the swept band its near-field blind zone extends from the band
    inner edge up to ``pandar_min_range_m``.  Optional mount-leg occlusion
    azimuth intervals are additionally treated as blind up to the envelope.
    This is a model, not a measurement: formal self-occlusion validation must
    replace these defaults before the Pandar may authorise rotation.
    """
    profile: dict[float, list[tuple[float, float]]] = {}
    occlusion = _merge_intervals(
        (math.radians(a), math.radians(b)) for a, b in mount_occlusion_azimuths_deg
    )
    for bearing_deg in bearings_deg:
        bearing_rad = math.radians(bearing_deg)
        intervals: list[tuple[float, float]] = []
        if footprint_radius_m + 1e-9 < pandar_min_range_m < envelope_radius_m - 1e-9:
            intervals.append((footprint_radius_m, min(envelope_radius_m, pandar_min_range_m)))
        wrapped = (bearing_rad + math.pi) % (2.0 * math.pi) - math.pi
        for start, end in occlusion:
            if _angle_in_interval(wrapped, start, end):
                intervals.append((footprint_radius_m, envelope_radius_m))
                break
        profile[float(bearing_deg)] = intervals
    return profile


def _angle_in_interval(angle_rad: float, start_rad: float, end_rad: float) -> bool:
    if start_rad <= end_rad:
        return start_rad <= angle_rad <= end_rad
    return angle_rad >= start_rad or angle_rad <= end_rad


def compute_dual_lidar_rotation_observability(
    *,
    footprint_radius_m: float,
    envelope_radius_m: float,
    builtin_unobservable: dict[float, Iterable[tuple[float, float]]],
    pandar_unobservable: dict[float, Iterable[tuple[float, float]]],
    pandar_extrinsics_validated: bool,
    requested_turn_range_deg: float | None = None,
    minimum_coverage_tolerance_m: float = 1e-3,
) -> DualLidarRotationObservability:
    """Compute per-bearing observability of the swept band by both LiDARs.

    A bearing's swept band ``[footprint_radius_m, envelope_radius_m]`` is
    safety-observable when at least one validated sensor fully covers it
    (unobservable coverage within tolerance).  The Pandar counts only when
    ``pandar_extrinsics_validated`` is true.
    """
    if not math.isfinite(footprint_radius_m) or footprint_radius_m < 0.0:
        raise ValueError("footprint_radius_m must be finite and non-negative")
    if not math.isfinite(envelope_radius_m) or envelope_radius_m <= footprint_radius_m:
        raise ValueError("envelope_radius_m must exceed footprint_radius_m")
    band_length = envelope_radius_m - footprint_radius_m
    if band_length <= 0.0:
        raise ValueError("swept band must have positive length")

    bearings = sorted(set(builtin_unobservable) | set(pandar_unobservable))
    if not bearings:
        raise ValueError("at least one bearing must be provided")
    requested_range_deg = float(requested_turn_range_deg) if requested_turn_range_deg is not None else None
    if requested_range_deg is not None and not math.isfinite(requested_range_deg):
        raise ValueError("requested_turn_range_deg must be finite")

    details: list[BearingObservability] = []
    observable_bearings = 0
    builtin_observable_bearings = 0
    pandar_observable_bearings = 0
    unobservable_bearings: list[float] = []

    for bearing in bearings:
        builtin_covered = _covered_length(
            footprint_radius_m,
            envelope_radius_m,
            builtin_unobservable.get(bearing, ()),
            tolerance_m=minimum_coverage_tolerance_m,
        )
        pandar_covered = _covered_length(
            footprint_radius_m,
            envelope_radius_m,
            pandar_unobservable.get(bearing, ()),
            tolerance_m=minimum_coverage_tolerance_m,
        )
        builtin_full = builtin_covered + 1e-9 >= band_length
        pandar_full = pandar_covered + 1e-9 >= band_length
        pandar_validated_full = pandar_extrinsics_validated and pandar_full
        if builtin_full or pandar_validated_full:
            reason = "builtin_l2_full_coverage"
            if not builtin_full:
                reason = "pandarxt16_validated_full_coverage"
        else:
            reason = []
            if not builtin_full:
                reason.append("builtin_l2_partial_coverage")
            if not pandar_full:
                reason.append("pandarxt16_partial_coverage")
            elif not pandar_extrinsics_validated:
                reason.append("pandarxt16_extrinsics_unvalidated")
            reason = "; ".join(reason)
        observable = builtin_full or pandar_validated_full
        details.append(
            BearingObservability(
                bearing_deg=float(bearing),
                band_length_m=band_length,
                builtin_fully_observes=builtin_full,
                pandar_fully_observes=pandar_full,
                pandar_extrinsics_validated=pandar_extrinsics_validated,
                observable=observable,
                reason=reason,
            )
        )
        if observable:
            observable_bearings += 1
        else:
            unobservable_bearings.append(float(bearing))
        builtin_observable_bearings += int(builtin_full)
        pandar_observable_bearings += int(pandar_full)

    requested_valid = True
    if requested_range_deg is not None:
        for detail in details:
            # The requested turn sweeps the robot's envelope over roughly the
            # requested angular span centered on the current heading.  A full
            # 360 observability is required for any requested turn until the
            # current heading is proven inside an observable arc; here we
            # require the whole requested band to be observable.
            if abs(detail.bearing_deg) <= requested_range_deg and not detail.observable:
                requested_valid = False
                break

    return DualLidarRotationObservability(
        footprint_radius_m=footprint_radius_m,
        envelope_radius_m=envelope_radius_m,
        total_bearings=len(details),
        observable_bearings=observable_bearings,
        unobservable_bearings=unobservable_bearings,
        builtin_observable_bearings=builtin_observable_bearings,
        pandar_observable_bearings=pandar_observable_bearings,
        pandar_extrinsics_validated=pandar_extrinsics_validated,
        full_rotation_observability_valid=observable_bearings == len(details),
        requested_turn_observability_valid=requested_valid,
        requested_turn_range_deg=requested_range_deg,
        details=details,
    )
