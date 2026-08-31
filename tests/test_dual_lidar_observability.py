"""Tests for the dual-LiDAR swept-band rotation observability."""

from __future__ import annotations

import pytest

from go2w_lidar_preprocessor.dual_lidar_observability import (
    compute_dual_lidar_rotation_observability,
    generate_pandar_unobservable_profile,
)


BUILTIN_BLIND_ALL = {
    0.0: [(0.35, 0.5)],
    90.0: [(0.35, 0.5)],
    180.0: [(0.35, 0.5)],
    270.0: [(0.35, 0.5)],
}


def test_builtin_alone_cannot_cover_blind_annulus() -> None:
    pandar_clear = {bearing: [] for bearing in BUILTIN_BLIND_ALL}
    obs = compute_dual_lidar_rotation_observability(
        footprint_radius_m=0.35,
        envelope_radius_m=0.511,
        builtin_unobservable=BUILTIN_BLIND_ALL,
        pandar_unobservable=pandar_clear,
        pandar_extrinsics_validated=False,
    )
    assert obs.full_rotation_observability_valid is False
    assert obs.pandar_observable_bearings == 4  # pandar would cover, but...
    assert obs.observable_bearings == 0  # ...it is not validated


def test_validated_pandar_fills_builtin_blind_zone() -> None:
    pandar_clear = {bearing: [] for bearing in BUILTIN_BLIND_ALL}
    obs = compute_dual_lidar_rotation_observability(
        footprint_radius_m=0.35,
        envelope_radius_m=0.511,
        builtin_unobservable=BUILTIN_BLIND_ALL,
        pandar_unobservable=pandar_clear,
        pandar_extrinsics_validated=True,
    )
    assert obs.full_rotation_observability_valid is True
    assert obs.observable_bearings == 4


def test_builtin_full_coverage_is_enough() -> None:
    builtin_clear = {bearing: [] for bearing in BUILTIN_BLIND_ALL}
    pandar_blind = {bearing: [(0.35, 0.5)] for bearing in BUILTIN_BLIND_ALL}
    obs = compute_dual_lidar_rotation_observability(
        footprint_radius_m=0.35,
        envelope_radius_m=0.511,
        builtin_unobservable=builtin_clear,
        pandar_unobservable=pandar_blind,
        pandar_extrinsics_validated=True,
    )
    assert obs.full_rotation_observability_valid is True
    assert obs.builtin_observable_bearings == 4


def test_required_turn_observability() -> None:
    pandar_clear = {bearing: [] for bearing in BUILTIN_BLIND_ALL}
    obs = compute_dual_lidar_rotation_observability(
        footprint_radius_m=0.35,
        envelope_radius_m=0.511,
        builtin_unobservable=BUILTIN_BLIND_ALL,
        pandar_unobservable=pandar_clear,
        pandar_extrinsics_validated=True,
        requested_turn_range_deg=30.0,
    )
    assert obs.requested_turn_observability_valid is True


def test_generate_pandar_profile_near_field_blind() -> None:
    profile = generate_pandar_unobservable_profile(
        bearings_deg=[0.0, 90.0],
        footprint_radius_m=0.35,
        envelope_radius_m=0.511,
        pandar_min_range_m=0.40,
    )
    # Near-field blind extends from footprint to pandar min range (0.40).
    assert profile[0.0] == [(0.35, 0.40)]
    assert profile[90.0] == [(0.35, 0.40)]


def test_rejects_invalid_band() -> None:
    with pytest.raises(ValueError, match="envelope_radius_m"):
        compute_dual_lidar_rotation_observability(
            footprint_radius_m=0.5,
            envelope_radius_m=0.3,
            builtin_unobservable={0.0: []},
            pandar_unobservable={0.0: []},
            pandar_extrinsics_validated=True,
        )
