"""Provenance-preserving dual-LiDAR safety evidence model.

The built-in L2 and the externally mounted PandarXT-16 are never merged into a
single concatenated cloud and then declared clear.  Every per-sensor
observation keeps its own provenance (source, freshness, geometry tier,
bearing, radial interval) and is fused with explicit fail-closed rules:

* any trustworthy OCCUPIED  -> final OCCUPIED
* one full trustworthy CLEAR sweep, no OCCUPIED -> final CLEAR
* everything else           -> final UNKNOWN (UNKNOWN != CLEAR)

A sensor contribution is trustworthy only when it is fresh and its geometry
tier is a validated transform.  A candidate TF or a stale/no-return sensor can
never produce formal CLEAR.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Any, Iterable, Sequence

SensorSource = str
# Canonical sources.
BUILTIN_L2 = "builtin_l2"
PANDARXT16 = "pandarxt16"


class EvidenceState(str, Enum):
    CLEAR = "clear"
    OCCUPIED = "occupied"
    UNKNOWN = "unknown"
    SELF_OCCLUDED = "self_occluded"
    SENSOR_BLIND = "sensor_blind"
    STALE = "stale"
    UNVALIDATED_GEOMETRY = "unvalidated_geometry"


class GeometryTier(str, Enum):
    SENSOR_RAW = "sensor_raw"
    CANDIDATE_TF = "candidate_tf"
    VALIDATED_TF = "validated_tf"


# States that may never be treated as CLEAR.
NON_CLEAR_STATES = {
    EvidenceState.OCCUPIED,
    EvidenceState.UNKNOWN,
    EvidenceState.SELF_OCCLUDED,
    EvidenceState.SENSOR_BLIND,
    EvidenceState.STALE,
    EvidenceState.UNVALIDATED_GEOMETRY,
}

# Default upper bound for a no-return / zero-range filter.
DEFAULT_ZERO_RETURN_MAX_M = 0.05


@dataclass(frozen=True)
class SensorEvidence:
    source: SensorSource
    state: EvidenceState
    freshness_seconds: float | None = None
    geometry_tier: GeometryTier = GeometryTier.SENSOR_RAW
    bearing_rad: float | None = None
    radial_interval_m: tuple[float, float] | None = None
    timestamp_s: float | None = None
    reason: str = ""

    def is_fresh(self, *, max_age_seconds: float) -> bool:
        return (
            self.freshness_seconds is not None
            and math.isfinite(self.freshness_seconds)
            and 0.0 <= self.freshness_seconds <= max_age_seconds
        )

    def has_validated_geometry(self) -> bool:
        return self.geometry_tier == GeometryTier.VALIDATED_TF

    def trustworthy_for_clear(self, *, max_age_seconds: float) -> bool:
        """A sensor may only contribute formal CLEAR when fresh + validated."""
        return self.is_fresh(max_age_seconds=max_age_seconds) and (
            self.has_validated_geometry()
        )

    def trustworthy_for_occupied(self, *, max_age_seconds: float) -> bool:
        """OCCUPIED counts only when fresh + validated geometry.

        An obstacle reported by an unvalidated-TF sensor is real but its
        bearing/range mapping is unreliable, so it must not dominate fusion.
        """
        return self.is_fresh(max_age_seconds=max_age_seconds) and (
            self.has_validated_geometry()
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "source": self.source,
            "state": self.state.value,
            "freshness_seconds": self.freshness_seconds,
            "geometry_tier": self.geometry_tier.value,
            "bearing_rad": self.bearing_rad,
            "radial_interval_m": (
                [self.radial_interval_m[0], self.radial_interval_m[1]]
                if self.radial_interval_m is not None
                else None
            ),
            "timestamp_s": self.timestamp_s,
            "reason": self.reason,
        }


@dataclass(frozen=True)
class FusedDualLidarEvidence:
    state: EvidenceState
    occupied_sources: list[str] = field(default_factory=list)
    full_clear_sweep_sources: list[str] = field(default_factory=list)
    unknown_sources: list[str] = field(default_factory=list)
    stale_sources: list[str] = field(default_factory=list)
    unvalidated_geometry_sources: list[str] = field(default_factory=list)
    reason: str = ""

    def authorizes_clear(self) -> bool:
        return self.state == EvidenceState.CLEAR

    def to_dict(self) -> dict[str, Any]:
        return {
            "state": self.state.value,
            "occupied_sources": self.occupied_sources,
            "full_clear_sweep_sources": self.full_clear_sweep_sources,
            "unknown_sources": self.unknown_sources,
            "stale_sources": self.stale_sources,
            "unvalidated_geometry_sources": self.unvalidated_geometry_sources,
            "reason": self.reason,
        }


def state_for_sensor(
    *,
    source: SensorSource,
    fresh: bool | None,
    finite_returns: bool,
    has_return: bool,
    occupied: bool,
    self_occluded: bool,
    blind: bool,
    geometry_tier: GeometryTier,
    freshness_seconds: float | None,
    bearing_rad: float | None,
    radial_interval_m: tuple[float, float] | None,
    timestamp_s: float | None,
    zero_return_max_m: float = DEFAULT_ZERO_RETURN_MAX_M,
) -> SensorEvidence:
    """Build one sensor's evidence for a swept sector from raw observations.

    ``finite_returns`` indicates the sensor stream is delivering finite points
    (not stalled). ``has_return`` indicates at least one valid (non zero/near-
    zero) return was seen in this sector. A zero/near-zero return is not a
    return and must not become free space.
    """
    if not fresh:
        return SensorEvidence(
            source=source,
            state=EvidenceState.STALE,
            freshness_seconds=freshness_seconds,
            geometry_tier=geometry_tier,
            bearing_rad=bearing_rad,
            radial_interval_m=radial_interval_m,
            timestamp_s=timestamp_s,
            reason="sensor stream is stale",
        )
    if geometry_tier != GeometryTier.VALIDATED_TF:
        return SensorEvidence(
            source=source,
            state=EvidenceState.UNVALIDATED_GEOMETRY,
            freshness_seconds=freshness_seconds,
            geometry_tier=geometry_tier,
            bearing_rad=bearing_rad,
            radial_interval_m=radial_interval_m,
            timestamp_s=timestamp_s,
            reason="transform/geometry is not validated",
        )
    if blind:
        return SensorEvidence(
            source=source,
            state=EvidenceState.SENSOR_BLIND,
            freshness_seconds=freshness_seconds,
            geometry_tier=geometry_tier,
            bearing_rad=bearing_rad,
            radial_interval_m=radial_interval_m,
            timestamp_s=timestamp_s,
            reason="sensor has no coverage of this sector",
        )
    if self_occluded:
        return SensorEvidence(
            source=source,
            state=EvidenceState.SELF_OCCLUDED,
            freshness_seconds=freshness_seconds,
            geometry_tier=geometry_tier,
            bearing_rad=bearing_rad,
            radial_interval_m=radial_interval_m,
            timestamp_s=timestamp_s,
            reason="sector is self-occluded",
        )
    if not finite_returns:
        return SensorEvidence(
            source=source,
            state=EvidenceState.UNKNOWN,
            freshness_seconds=freshness_seconds,
            geometry_tier=geometry_tier,
            bearing_rad=bearing_rad,
            radial_interval_m=radial_interval_m,
            timestamp_s=timestamp_s,
            reason="sensor stream has no finite returns",
        )
    if occupied:
        return SensorEvidence(
            source=source,
            state=EvidenceState.OCCUPIED,
            freshness_seconds=freshness_seconds,
            geometry_tier=geometry_tier,
            bearing_rad=bearing_rad,
            radial_interval_m=radial_interval_m,
            timestamp_s=timestamp_s,
            reason="occupied return in swept sector",
        )
    if not has_return:
        # No return in this sector is NOT clear; it may be blind, absorbed, or
        # a zero-return. It stays UNKNOWN unless a full-coverage CLEAR sweep is
        # proven by a different trustworthy sensor.
        return SensorEvidence(
            source=source,
            state=EvidenceState.UNKNOWN,
            freshness_seconds=freshness_seconds,
            geometry_tier=geometry_tier,
            bearing_rad=bearing_rad,
            radial_interval_m=radial_interval_m,
            timestamp_s=timestamp_s,
            reason="no valid return; no-return is not clear",
        )
    return SensorEvidence(
        source=source,
        state=EvidenceState.CLEAR,
        freshness_seconds=freshness_seconds,
        geometry_tier=geometry_tier,
        bearing_rad=bearing_rad,
        radial_interval_m=radial_interval_m,
        timestamp_s=timestamp_s,
        reason="full swept sector observed clear",
    )


def fuse_dual_lidar_evidence(
    evidence: Iterable[SensorEvidence],
    *,
    max_age_seconds: float,
    unknown_is_clear: bool = False,
) -> FusedDualLidarEvidence:
    """Fuse per-sensor evidence with provenance preserved.

    Hard rules (never violated even when ``unknown_is_clear`` is enabled for
    an explicit operator override):
      UNKNOWN != CLEAR, NO_RETURN != CLEAR, UNVALIDATED_GEOMETRY != CLEAR.
    """
    items = list(evidence)
    occupied_sources: list[str] = []
    full_clear_sources: list[str] = []
    unknown_sources: list[str] = []
    stale_sources: list[str] = []
    unvalidated_sources: list[str] = []

    for item in items:
        if not item.is_fresh(max_age_seconds=max_age_seconds):
            stale_sources.append(item.source)
            continue
        if not item.has_validated_geometry():
            unvalidated_sources.append(item.source)
            continue
        if item.state == EvidenceState.OCCUPIED:
            occupied_sources.append(item.source)
        elif item.state == EvidenceState.CLEAR:
            full_clear_sources.append(item.source)
        else:
            unknown_sources.append(item.source)

    if occupied_sources:
        return FusedDualLidarEvidence(
            state=EvidenceState.OCCUPIED,
            occupied_sources=occupied_sources,
            unknown_sources=unknown_sources,
            stale_sources=stale_sources,
            unvalidated_geometry_sources=unvalidated_sources,
            reason=f"occupied by {', '.join(occupied_sources)}",
        )
    if full_clear_sources and not unknown_sources:
        return FusedDualLidarEvidence(
            state=EvidenceState.CLEAR,
            full_clear_sweep_sources=full_clear_sources,
            occupied_sources=occupied_sources,
            unknown_sources=unknown_sources,
            stale_sources=stale_sources,
            unvalidated_geometry_sources=unvalidated_sources,
            reason=f"full clear sweep from {', '.join(full_clear_sources)}",
        )
    if unknown_is_clear and full_clear_sources:
        # Explicit operator override: an otherwise-unknown sector is treated as
        # clear when at least one trustworthy sensor reports a full clear sweep.
        # This must never be enabled by default.
        return FusedDualLidarEvidence(
            state=EvidenceState.CLEAR,
            full_clear_sweep_sources=full_clear_sources,
            unknown_sources=unknown_sources,
            stale_sources=stale_sources,
            unvalidated_geometry_sources=unvalidated_sources,
            reason="operator override: unknown treated as clear",
        )
    return FusedDualLidarEvidence(
        state=EvidenceState.UNKNOWN,
        occupied_sources=occupied_sources,
        full_clear_sweep_sources=full_clear_sources,
        unknown_sources=unknown_sources,
        stale_sources=stale_sources,
        unvalidated_geometry_sources=unvalidated_sources,
        reason="no trustworthy full clear sweep and no trustworthy occupied return",
    )


def classify_zero_returns(
    ranges: Sequence[float],
    *,
    zero_return_max_m: float = DEFAULT_ZERO_RETURN_MAX_M,
) -> dict[str, Any]:
    """Classify ranges into valid / zero-near-zero / non-finite buckets.

    A return at ``range <= zero_return_max_m`` is an INVALID return: it must
    never enter obstacle, free space, clearance, occupancy, rotation evidence
    or a metric SceneGraph.
    """
    valid = 0
    zero_near_zero = 0
    non_finite = 0
    for value in ranges:
        if value is None or not math.isfinite(float(value)):
            non_finite += 1
        elif float(value) <= zero_return_max_m:
            zero_near_zero += 1
        else:
            valid += 1
    total = max(1, len(ranges))
    return {
        "total": int(len(ranges)),
        "valid": int(valid),
        "zero_or_near_zero": int(zero_near_zero),
        "non_finite": int(non_finite),
        "zero_return_fraction": round(zero_near_zero / total, 6),
        "valid_return_fraction": round(valid / total, 6),
    }
