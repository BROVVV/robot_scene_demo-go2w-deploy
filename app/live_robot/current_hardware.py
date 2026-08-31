"""Load and validate the current Go2-W + PandarXT-16 hardware geometry/state.

Both YAML files are fail-closed and operator-confirmed only. The geometry
config alone never authorizes motion; the state manifest binds which configs
describe the current physical rig so stale sensor evidence cannot be reused
for motion after a mount change.

Everything here is pure Python and ROS-independent so the app and the tests
can import it directly.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import datetime as _datetime
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import yaml


EXPECTED_DIMENSIONS_M = {
    "length": 0.70,
    "width": 0.43,
    "height": 0.70,
}
EXPECTED_HIGHEST_POINT = "pandarxt16_protective_frame"


class HardwareConfigError(RuntimeError):
    """Raised when a hardware geometry/state file violates its contract."""


@dataclass(frozen=True)
class CurrentHardwareGeometry:
    hardware_profile: str
    length_m: float
    width_m: float
    height_m: float
    highest_point: str
    highest_point_fixed_structure: bool
    highest_point_is_loose_cable: bool
    dimension_source_type: str
    dimension_source_date: str
    horizontal_footprint_length_m: float
    horizontal_footprint_width_m: float
    remeasurement_required: dict[str, bool]
    authorizes_motion: bool

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @property
    def horizontal_footprint_half_diagonal_m(self) -> float:
        """Pure half-diagonal of the current horizontal footprint.

        The existing conservative swept radius (0.511 m in lidar_preprocess.yaml)
        remains the safety input; this is informational only.
        """
        return math.hypot(self.length_m / 2.0, self.width_m / 2.0)


@dataclass(frozen=True)
class CurrentHardwareState:
    hardware_profile: str
    geometry_config: str
    pandar_config: str
    pandar_extrinsics: str
    builtin_lidar_present: bool
    pandar_present: bool
    height_m: float
    highest_point: str
    mount_changed_since_calibration: bool
    motion_authorization: dict[str, bool]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _load_yaml(path: str | Path) -> dict[str, Any]:
    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if not isinstance(payload, dict):
        raise HardwareConfigError(f"{path} must be a YAML mapping")
    return payload


def _require_bool(payload: dict[str, Any], key: str, path: str) -> bool:
    value = payload.get(key)
    if not isinstance(value, bool):
        raise HardwareConfigError(f"{path}: {key} must be boolean")
    return value


def _require_str(payload: dict[str, Any], key: str, path: str) -> str:
    value = payload.get(key)
    if isinstance(value, (_datetime.date, _datetime.datetime)):
        value = value.isoformat()
    if not isinstance(value, str) or not value.strip():
        raise HardwareConfigError(f"{path}: {key} must be a non-empty string")
    return value.strip()


def _require_number(payload: dict[str, Any], key: str, path: str) -> float:
    value = payload.get(key)
    if value is None or not math.isfinite(float(value)):
        raise HardwareConfigError(f"{path}: {key} must be a finite number")
    return float(value)


def load_current_hardware_geometry(
    path: str | Path,
) -> CurrentHardwareGeometry:
    """Load and validate ``configs/go2w/current_hardware_geometry.yaml``."""
    config_path = Path(path)
    payload = _load_yaml(config_path)
    label = str(config_path)
    if _require_str(payload, "hardware_profile", label) != (
        "go2w_with_pandarxt16_protective_frame"
    ):
        raise HardwareConfigError(f"{label}: unexpected hardware_profile")
    dimensions = payload.get("dimensions_m")
    if not isinstance(dimensions, dict):
        raise HardwareConfigError(f"{label}: dimensions_m must be a mapping")
    length = _require_number(dimensions, "length", label)
    width = _require_number(dimensions, "width", label)
    height = _require_number(dimensions, "height", label)
    for name, expected in EXPECTED_DIMENSIONS_M.items():
        actual = float(dimensions.get(name, math.nan))
        if not math.isclose(actual, expected, abs_tol=1e-6):
            raise HardwareConfigError(
                f"{label}: dimensions_m.{name}={actual} != expected {expected}"
            )

    source = payload.get("dimension_source") or {}
    source_type = _require_str(source, "type", label)
    source_date = _require_str(source, "date", label)

    highest = payload.get("highest_point") or {}
    highest_point = _require_str(highest, "type", label)
    if highest_point != EXPECTED_HIGHEST_POINT:
        raise HardwareConfigError(
            f"{label}: highest_point.type must be {EXPECTED_HIGHEST_POINT}"
        )
    fixed = _require_bool(highest, "fixed_structure", label)
    loose_cable = _require_bool(highest, "is_loose_cable", label)
    if not fixed or loose_cable:
        raise HardwareConfigError(
            f"{label}: highest point must be the fixed protective frame, "
            "not a loose cable"
        )

    footprint = payload.get("horizontal_footprint") or {}
    fp_length = _require_number(footprint, "length_m", label)
    fp_width = _require_number(footprint, "width_m", label)
    if not math.isclose(fp_length, length, abs_tol=1e-6) or not math.isclose(
        fp_width, width, abs_tol=1e-6
    ):
        raise HardwareConfigError(
            f"{label}: horizontal_footprint must match dimensions_m"
        )

    remeasure = payload.get("remeasurement_required")
    if not isinstance(remeasure, dict) or set(remeasure) != {
        "length",
        "width",
        "height",
    }:
        raise HardwareConfigError(
            f"{label}: remeasurement_required must cover length/width/height"
        )
    if any(_require_bool(remeasure, key, label) for key in ("length", "width", "height")):
        raise HardwareConfigError(
            f"{label}: remeasurement is not required for this rig"
        )

    if _require_bool(payload, "authorizes_motion", label) is not False:
        raise HardwareConfigError(
            f"{label}: the geometry config itself never authorizes motion"
        )

    return CurrentHardwareGeometry(
        hardware_profile="go2w_with_pandarxt16_protective_frame",
        length_m=length,
        width_m=width,
        height_m=height,
        highest_point=highest_point,
        highest_point_fixed_structure=fixed,
        highest_point_is_loose_cable=loose_cable,
        dimension_source_type=source_type,
        dimension_source_date=source_date,
        horizontal_footprint_length_m=fp_length,
        horizontal_footprint_width_m=fp_width,
        remeasurement_required={
            key: bool(remeasure[key]) for key in ("length", "width", "height")
        },
        authorizes_motion=False,
    )


def load_current_hardware_state(
    path: str | Path,
) -> CurrentHardwareState:
    """Load and validate ``configs/go2w/current_hardware_state.yaml``."""
    config_path = Path(path)
    payload = _load_yaml(config_path)
    label = str(config_path)
    if _require_str(payload, "hardware_profile", label) != "go2w_pandarxt16_v1":
        raise HardwareConfigError(f"{label}: unexpected hardware_profile")
    motion_authorization = payload.get("motion_authorization")
    if not isinstance(motion_authorization, dict):
        raise HardwareConfigError(f"{label}: motion_authorization must be a mapping")
    state = CurrentHardwareState(
        hardware_profile="go2w_pandarxt16_v1",
        geometry_config=_require_str(payload, "geometry_config", label),
        pandar_config=_require_str(payload, "pandar_config", label),
        pandar_extrinsics=_require_str(payload, "pandar_extrinsics", label),
        builtin_lidar_present=_require_bool(payload, "builtin_lidar_present", label),
        pandar_present=_require_bool(payload, "pandar_present", label),
        height_m=_require_number(payload, "height_m", label),
        highest_point=_require_str(payload, "highest_point", label),
        mount_changed_since_calibration=_require_bool(
            payload, "mount_changed_since_calibration", label
        ),
        motion_authorization={
            "rotation": _require_bool(motion_authorization, "rotation", label),
            "forward": _require_bool(motion_authorization, "forward", label),
        },
    )
    if not math.isclose(state.height_m, EXPECTED_DIMENSIONS_M["height"], abs_tol=1e-6):
        raise HardwareConfigError(
            f"{label}: height_m must equal the confirmed geometry height"
        )
    if state.highest_point != EXPECTED_HIGHEST_POINT:
        raise HardwareConfigError(
            f"{label}: highest_point must be {EXPECTED_HIGHEST_POINT}"
        )
    if not state.builtin_lidar_present or not state.pandar_present:
        raise HardwareConfigError(f"{label}: both LiDAR sensors must be present")
    if state.motion_authorization["rotation"] or state.motion_authorization["forward"]:
        raise HardwareConfigError(
            f"{label}: the state manifest never authorizes motion"
        )
    return state


def _canonical_json(value: dict[str, Any]) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def geometry_hash(geometry: CurrentHardwareGeometry) -> str:
    """Stable hash binding a lease/evidence to this exact geometry config."""
    return sha256_text(
        _canonical_json(
            {
                "hardware_profile": geometry.hardware_profile,
                "dimensions_m": {
                    "length": geometry.length_m,
                    "width": geometry.width_m,
                    "height": geometry.height_m,
                },
                "highest_point": {
                    "type": geometry.highest_point,
                    "fixed_structure": geometry.highest_point_fixed_structure,
                    "is_loose_cable": geometry.highest_point_is_loose_cable,
                },
                "remeasurement_required": geometry.remeasurement_required,
                "authorizes_motion": geometry.authorizes_motion,
            }
        )
    )


def state_hash(state: CurrentHardwareState) -> str:
    """Stable hash binding a lease/evidence to this exact hardware state."""
    return sha256_text(
        _canonical_json(
            {
                "hardware_profile": state.hardware_profile,
                "geometry_config": state.geometry_config,
                "pandar_config": state.pandar_config,
                "pandar_extrinsics": state.pandar_extrinsics,
                "builtin_lidar_present": state.builtin_lidar_present,
                "pandar_present": state.pandar_present,
                "height_m": state.height_m,
                "highest_point": state.highest_point,
                "mount_changed_since_calibration": state.mount_changed_since_calibration,
                "motion_authorization": state.motion_authorization,
            }
        )
    )


def load_geometry_and_state(
    *,
    geometry_path: str | Path = "configs/go2w/current_hardware_geometry.yaml",
    state_path: str | Path = "configs/go2w/current_hardware_state.yaml",
    project_root: str | Path | None = None,
) -> dict[str, Any]:
    """Load both files relative to the project root and bind them together."""
    root = Path(project_root or _project_root())
    geometry = load_current_hardware_geometry(root / geometry_path)
    state = load_current_hardware_state(root / state_path)
    geometry_relative = str(root / geometry_path)
    if state.geometry_config and str(root / state.geometry_config) != str(
        root / geometry_path
    ):
        raise HardwareConfigError(
            "hardware state binds a different geometry config than the one loaded"
        )
    return {
        "geometry": geometry,
        "state": state,
        "geometry_hash": geometry_hash(geometry),
        "state_hash": state_hash(state),
        "geometry_path": str(root / geometry_path),
        "state_path": str(root / state_path),
    }


def _project_root() -> Path:
    return Path(__file__).resolve().parents[2]
