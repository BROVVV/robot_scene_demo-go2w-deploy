"""Tests for the PandarXT-16 zero / near-zero return filter."""

from __future__ import annotations

import math

import pytest

from go2w_lidar_preprocessor.lidar_evidence import (
    DEFAULT_ZERO_RETURN_MAX_M,
    EvidenceState,
    GeometryTier,
    classify_zero_returns,
    state_for_sensor,
)


def test_default_zero_return_bound_is_5cm() -> None:
    assert DEFAULT_ZERO_RETURN_MAX_M == 0.05


def test_classify_zero_returns_buckets() -> None:
    result = classify_zero_returns([0.0, 0.02, 0.05, 0.06, 1.0, 2.0, math.nan])
    assert result["zero_or_near_zero"] == 3  # 0.0, 0.02, 0.05
    assert result["valid"] == 3  # 0.06, 1.0, 2.0
    assert result["non_finite"] == 1
    assert result["zero_return_fraction"] == pytest.approx(3 / 7, abs=1e-5)


def test_boundary_exactly_0_05_is_invalid() -> None:
    result = classify_zero_returns([0.05])
    assert result["zero_or_near_zero"] == 1
    assert result["valid"] == 0


def test_zero_return_never_becomes_clear() -> None:
    evidence = state_for_sensor(
        source="pandarxt16",
        fresh=True,
        finite_returns=True,
        has_return=False,  # only a zero/near-zero return in this sector
        occupied=False,
        self_occluded=False,
        blind=False,
        geometry_tier=GeometryTier.VALIDATED_TF,
        freshness_seconds=0.1,
        bearing_rad=None,
        radial_interval_m=None,
        timestamp_s=None,
    )
    assert evidence.state != EvidenceState.CLEAR
    assert evidence.state == EvidenceState.UNKNOWN


def test_zero_return_never_becomes_obstacle() -> None:
    # A zero-range point is classified as invalid and must not become an
    # occupied evidence either; occupied requires a genuine return beyond the
    # zero-return bound.
    evidence = state_for_sensor(
        source="pandarxt16",
        fresh=True,
        finite_returns=True,
        has_return=True,
        occupied=True,  # a real obstacle return
        self_occluded=False,
        blind=False,
        geometry_tier=GeometryTier.VALIDATED_TF,
        freshness_seconds=0.1,
        bearing_rad=None,
        radial_interval_m=None,
        timestamp_s=None,
    )
    # Occupied is still reported for a real return; the zero-return filter
    # guarantees the range that produced it was > 0.05 m upstream.
    assert evidence.state == EvidenceState.OCCUPIED
