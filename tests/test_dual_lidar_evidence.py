"""Tests for the provenance-preserving dual-LiDAR evidence fusion rules."""

from __future__ import annotations

from go2w_lidar_preprocessor.lidar_evidence import (
    BUILTIN_L2,
    PANDARXT16,
    EvidenceState,
    FusedDualLidarEvidence,
    GeometryTier,
    SensorEvidence,
    fuse_dual_lidar_evidence,
    state_for_sensor,
)


def _evidence(
    source: str,
    state: EvidenceState,
    *,
    tier: GeometryTier = GeometryTier.VALIDATED_TF,
    freshness: float = 0.1,
) -> SensorEvidence:
    return SensorEvidence(
        source=source,
        state=state,
        freshness_seconds=freshness,
        geometry_tier=tier,
    )


def test_occupied_wins_over_clear() -> None:
    clear = _evidence(BUILTIN_L2, EvidenceState.CLEAR)
    occupied = _evidence(PANDARXT16, EvidenceState.OCCUPIED)
    fused = fuse_dual_lidar_evidence([clear, occupied], max_age_seconds=0.5)
    assert fused.state == EvidenceState.OCCUPIED
    assert fused.occupied_sources == [PANDARXT16]


def test_unknown_plus_unknown_is_unknown() -> None:
    unknown_a = _evidence(BUILTIN_L2, EvidenceState.UNKNOWN)
    unknown_b = _evidence(PANDARXT16, EvidenceState.UNKNOWN)
    fused = fuse_dual_lidar_evidence([unknown_a, unknown_b], max_age_seconds=0.5)
    assert fused.state == EvidenceState.UNKNOWN


def test_clear_plus_unknown_is_unknown() -> None:
    clear = _evidence(BUILTIN_L2, EvidenceState.CLEAR)
    unknown = _evidence(PANDARXT16, EvidenceState.UNKNOWN)
    fused = fuse_dual_lidar_evidence([clear, unknown], max_age_seconds=0.5)
    assert fused.state == EvidenceState.UNKNOWN  # UNKNOWN != CLEAR


def test_full_clear_sweep_is_clear() -> None:
    clear = _evidence(BUILTIN_L2, EvidenceState.CLEAR)
    clear2 = _evidence(PANDARXT16, EvidenceState.CLEAR)
    fused = fuse_dual_lidar_evidence([clear, clear2], max_age_seconds=0.5)
    assert fused.state == EvidenceState.CLEAR
    assert fused.full_clear_sweep_sources == [BUILTIN_L2, PANDARXT16]


def test_unvalidated_geometry_never_clear() -> None:
    candidate = _evidence(
        PANDARXT16,
        EvidenceState.CLEAR,
        tier=GeometryTier.CANDIDATE_TF,
    )
    unknown = _evidence(BUILTIN_L2, EvidenceState.UNKNOWN)
    fused = fuse_dual_lidar_evidence([candidate, unknown], max_age_seconds=0.5)
    assert fused.state == EvidenceState.UNKNOWN
    assert fused.unvalidated_geometry_sources == [PANDARXT16]


def test_stale_sensor_never_contributes_clear() -> None:
    stale_clear = _evidence(
        BUILTIN_L2, EvidenceState.CLEAR, freshness=5.0
    )
    unknown = _evidence(PANDARXT16, EvidenceState.UNKNOWN)
    fused = fuse_dual_lidar_evidence(
        [stale_clear, unknown], max_age_seconds=0.5
    )
    assert fused.state == EvidenceState.UNKNOWN
    assert fused.stale_sources == [BUILTIN_L2]


def test_unknown_is_clear_override_only_when_explicit() -> None:
    clear = _evidence(BUILTIN_L2, EvidenceState.CLEAR)
    unknown = _evidence(PANDARXT16, EvidenceState.UNKNOWN)
    # Default: no override, unknown stays unknown.
    fused = fuse_dual_lidar_evidence([clear, unknown], max_age_seconds=0.5)
    assert fused.state == EvidenceState.UNKNOWN
    # Explicit operator override only.
    overridden = fuse_dual_lidar_evidence(
        [clear, unknown], max_age_seconds=0.5, unknown_is_clear=True
    )
    assert overridden.state == EvidenceState.CLEAR


def test_occupied_from_unvalidated_sensor_is_downgraded() -> None:
    # An occupied return from a sensor whose transform is not validated must
    # not dominate fusion; it is recorded as diagnostic only.
    occupied_candidate = _evidence(
        PANDARXT16, EvidenceState.OCCUPIED, tier=GeometryTier.CANDIDATE_TF
    )
    fused = fuse_dual_lidar_evidence([occupied_candidate], max_age_seconds=0.5)
    assert fused.state == EvidenceState.UNKNOWN
    assert fused.occupied_sources == []
    assert fused.unvalidated_geometry_sources == [PANDARXT16]


def test_sensor_state_builder() -> None:
    evidence = state_for_sensor(
        source=PANDARXT16,
        fresh=True,
        finite_returns=False,
        has_return=False,
        occupied=False,
        self_occluded=True,
        blind=False,
        geometry_tier=GeometryTier.VALIDATED_TF,
        freshness_seconds=0.1,
        bearing_rad=None,
        radial_interval_m=None,
        timestamp_s=None,
    )
    assert evidence.state == EvidenceState.SELF_OCCLUDED
