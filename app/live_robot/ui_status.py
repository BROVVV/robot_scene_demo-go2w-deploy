"""Read-only status aggregation for the Go2-W Streamlit panel."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .frame_bundle_reader import (
    FrameBundleReader,
    FrameBundleUnavailable,
    dual_lidar_status as dual_lidar_status_of,
    pandar_status as pandar_status_of,
)


@dataclass(frozen=True)
class LiveUiStatus:
    sensor_health: dict[str, bool]
    control: dict[str, Any]
    search: dict[str, Any]
    plan_gate: dict[str, Any]
    execute_gate: dict[str, Any]
    latest_frame_id: int | None = None
    latest_session_id: str | None = None
    pandar_status: dict[str, Any] | None = None
    dual_lidar_status: dict[str, Any] | None = None


def load_live_ui_status(
    *,
    spool_root: str | Path | None = None,
    acceptance_root: str | Path = "outputs/go2w_acceptance/navigation_gate",
) -> LiveUiStatus:
    if spool_root is None:
        spool_root = os.environ.get("GO2W_FRAME_SPOOL_DIR", "runtime/go2w/spool")
    sensor = {
        name: False
        for name in (
            "camera",
            "camera_info_calibrated",
            "rgb_lidar_overlay",
            "rgb_lidar_extrinsics",
            "rgb_lidar_fusion",
            "lidar",
            "lio",
            "tf",
            "pandar",
            "dual_lidar",
        )
    }
    frame_id = None
    session_id = None
    pandar_status = None
    dual_lidar_status = None
    try:
        bundle = FrameBundleReader(spool_root).read_latest()
        sensor.update({key: bool(value) for key, value in bundle.payload["sensor_health"].items() if key in sensor})
        frame_id = bundle.frame_id
        session_id = str(bundle.payload["session_id"])
        pandar_status = pandar_status_of(bundle.payload)
        dual_lidar_status = dual_lidar_status_of(bundle.payload)
    except FrameBundleUnavailable:
        pass
    root = Path(acceptance_root)
    return LiveUiStatus(
        sensor_health=sensor,
        control={
            "lease": False,
            "operator_armed": False,
            "remote_override_clear": False,
            "emergency_stop_confirmed": False,
            "control_source": "none",
            "execution_enabled": False,
        },
        search={
            "state": "IDLE",
            "target_evidence": False,
            "safety_events": [],
            "semantic_reasoning_enabled": _env_bool(
                "LIVE_SEARCH_SEMANTIC_REASONING_ENABLED", False
            ),
            "reasoner_backend": os.environ.get(
                "LIVE_SEARCH_REASONER_BACKEND", "legacy"
            ),
            "reasoner_mode": os.environ.get(
                "LIVE_SEARCH_REASONER_MODE", "shadow"
            ),
            "semantic_forward_allowed": _env_bool(
                "LIVE_SEARCH_REASONER_ALLOW_FORWARD", False
            ),
        },
        plan_gate=_read_json(root / "plan_only.json"),
        execute_gate=_read_json(root / "execute.json"),
        latest_frame_id=frame_id,
        latest_session_id=session_id,
        pandar_status=pandar_status,
        dual_lidar_status=dual_lidar_status,
    )


def blockers_for_mode(mode: str, status: LiveUiStatus) -> list[str]:
    if mode == "observe_only":
        return [name for name in ("camera", "lidar") if not status.sensor_health.get(name)]
    if mode == "step_search":
        blockers = blockers_for_mode("observe_only", status)
        if not status.control.get("execution_enabled"):
            blockers.append("motion_execution_disabled")
        if not status.control.get("operator_armed"):
            blockers.append("operator_not_armed")
        return blockers
    if mode == "nav2_plan_only":
        return list(status.plan_gate.get("blocking_conditions") or ["plan_gate_unavailable"])
    if mode == "nav2_execute":
        return list(status.execute_gate.get("blocking_conditions") or ["execute_gate_unavailable"])
    return ["unsupported_mode"]


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError, json.JSONDecodeError):
        return {}


def _env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}
