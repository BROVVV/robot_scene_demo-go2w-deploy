"""Pure step-planning helpers for the Go2-W short-step search loop.

These functions mirror the decision logic proven on the real robot in
``scripts/go2w/run_autonomous_loop.py`` but stay independent of ROS and motion
APIs so they can be unit-tested and reused by any executor.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class PlanKind(str, Enum):
    VERIFY = "verify"          # candidate reached; run LLM verification
    MOVE = "move"              # execute a short step
    SCAN = "scan"              # no target; follow the scan sequence
    REACHED = "reached"        # target confirmed; stop
    ABORT_RADIUS = "abort_radius"
    ABORT = "abort"


@dataclass(frozen=True)
class StepPlan:
    kind: PlanKind
    step: str = ""
    description_zh: str = ""
    phase: str = ""


def describe_step(step: str, *, forward_vx: float = 0.12,
                  forward_seconds: float = 2.0) -> str:
    if step == "f":
        return f"前进 {forward_vx:.2f} m/s × {forward_seconds:.0f}s"
    if step.startswith("l"):
        return f"左转 {step[1:]}°"
    if step.startswith("r"):
        return f"右转 {step[1:]}°"
    return step


def scan_sequence(*, scan_turn_deg: float = 30.0,
                  scan_span: int = 3) -> list[str]:
    """Sweep right, return, forward, sweep left, return, forward.

    Net heading returns to zero so the tether is not progressively twisted.
    """
    deg = int(scan_turn_deg)
    right = [f"r{deg}"] * max(1, scan_span)
    left = [f"l{deg}"] * max(1, scan_span)
    return right + left + ["f"] + left + right + ["f"]


def plan_approach_step(
    *,
    center_x: float,
    area_ratio: float,
    distance_m: float = 0.0,
    align_threshold: float = 0.08,
    align_yaw_max_deg: float = 25.0,
    reach_area_ratio: float = 0.15,
    max_radius_m: float = 0.0,
    forward_estimate_m: float = 0.15,
) -> StepPlan:
    """Choose the next approach action for a detected target."""
    if area_ratio >= reach_area_ratio:
        return StepPlan(
            kind=PlanKind.VERIFY,
            description_zh="到达候选，需要 LLM 复核",
            phase="APPROACH",
        )
    offset = center_x - 0.5
    if abs(offset) > align_threshold:
        degrees = max(
            -align_yaw_max_deg,
            min(align_yaw_max_deg, -offset * align_yaw_max_deg * 2.0),
        )
        step = f"l{int(abs(degrees))}" if degrees > 0.0 else f"r{int(abs(degrees))}"
        return StepPlan(
            kind=PlanKind.MOVE,
            step=step,
            description_zh=describe_step(step),
            phase="APPROACH",
        )
    if max_radius_m > 0.0 and distance_m + forward_estimate_m >= max_radius_m:
        return StepPlan(
            kind=PlanKind.ABORT_RADIUS,
            description_zh=f"前进将超过实验半径 {max_radius_m:.1f} m",
            phase="APPROACH",
        )
    return StepPlan(
        kind=PlanKind.MOVE,
        step="f",
        description_zh=describe_step("f"),
        phase="APPROACH",
    )


def plan_scan_step(scan_index: int, *, scan_turn_deg: float = 30.0,
                   scan_span: int = 3, distance_m: float = 0.0,
                   max_radius_m: float = 0.0,
                   forward_estimate_m: float = 0.15) -> StepPlan:
    sequence = scan_sequence(
        scan_turn_deg=scan_turn_deg, scan_span=scan_span
    )
    step = sequence[scan_index % len(sequence)]
    if step == "f" and max_radius_m > 0.0:
        if distance_m + forward_estimate_m >= max_radius_m:
            step = f"r{int(scan_turn_deg)}"
    return StepPlan(
        kind=PlanKind.SCAN,
        step=step,
        description_zh=describe_step(step),
        phase="SEARCH",
    )


def verify_rejection_step(turn_deg: float = 15.0) -> StepPlan:
    return StepPlan(
        kind=PlanKind.SCAN,
        step=f"r{int(turn_deg)}",
        description_zh=f"复核拒绝，右转 {int(turn_deg)}° 重新观察",
        phase="REOBSERVE",
    )
