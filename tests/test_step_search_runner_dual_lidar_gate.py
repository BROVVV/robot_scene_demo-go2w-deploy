"""Tests for the dual-LiDAR rotation gate in the search/motion path.

The gate itself lives in ``app.live_robot.motion_bounds`` and is invoked by
the executor before arming.  This test also verifies the SearchDirective
carries the safety intent the executor consumes.
"""

from __future__ import annotations

import pytest

from app.live_robot.motion_bounds import (
    MotionBoundaryDecision,
    evaluate_dual_lidar_rotation_gate,
)
from app.reasoning.semantic_navigation.models import SearchDirective, SearchDirectiveKind


def _gate(**kwargs) -> MotionBoundaryDecision:
    defaults = {
        "fused_state": "clear",
        "dual_lidar_enabled": True,
        "unknown_is_clear": False,
        "occupied_sources": None,
    }
    defaults.update(kwargs)
    return evaluate_dual_lidar_rotation_gate(**defaults)


def test_gate_passes_when_fusion_disabled() -> None:
    decision = _gate(fused_state="unknown", dual_lidar_enabled=False)
    assert decision.allowed is True


def test_gate_rejects_occupied() -> None:
    decision = _gate(fused_state="occupied", occupied_sources=["builtin_l2"])
    assert decision.allowed is False
    assert "occupied" in decision.reason


def test_gate_rejects_unknown() -> None:
    decision = _gate(fused_state="unknown")
    assert decision.allowed is False
    assert "unknown" in decision.reason


def test_gate_unknown_is_clear_only_when_override() -> None:
    assert _gate(fused_state="unknown").allowed is False
    overridden = _gate(fused_state="unknown", unknown_is_clear=True)
    assert overridden.allowed is True


def test_gate_passes_on_clear() -> None:
    decision = _gate(fused_state="clear")
    assert decision.allowed is True


def test_gate_fails_closed_on_stale_and_unvalidated() -> None:
    for state in ("stale", "sensor_blind", "self_occluded", "unvalidated_geometry", "no_evidence"):
        assert _gate(fused_state=state).allowed is False, state


def test_directive_carries_safety_intent_for_turn() -> None:
    directive = SearchDirective(
        directive_id="d1",
        kind=SearchDirectiveKind.REOBSERVE_SECTOR,
        source_backend="semantic_navigation",
        match_state="strong_match",
        confidence=0.8,
        preferred_heading_delta_deg=20.0,
        safety_intent="turn",
        requires_rotation_clearance=True,
        requires_front_clearance=False,
    )
    payload = directive.to_dict()
    assert payload["safety_intent"] == "turn"
    assert payload["requires_rotation_clearance"] is True
    assert payload["requires_front_clearance"] is False
    # A directive must never carry an authorizes_motion flag.
    assert "authorizes_motion" not in payload


def test_default_directive_is_observe_only() -> None:
    directive = SearchDirective(
        directive_id="d2",
        kind=SearchDirectiveKind.LEGACY_SCAN,
        source_backend="legacy",
        match_state="zero_match",
        confidence=1.0,
    )
    assert directive.safety_intent == "observe"
    assert directive.requires_rotation_clearance is False
