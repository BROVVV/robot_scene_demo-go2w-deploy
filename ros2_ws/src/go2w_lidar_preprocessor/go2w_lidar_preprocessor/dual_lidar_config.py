"""Config gate for the dual-LiDAR safety policy.

Loads ``configs/go2w/dual_lidar_safety.yaml`` and enforces the fail-closed
contract: dual-lidar safety is disabled by default, the Pandar transform tier
is not validated, and UNKNOWN may never be treated as CLEAR unless the
operator explicitly enables the override.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import yaml


class DualLidarConfigError(RuntimeError):
    pass


def load_dual_lidar_safety_config(path: str) -> dict:
    config = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if config.get("schema_version") != 1:
        raise DualLidarConfigError("dual_lidar_safety schema_version must be 1")
    if config.get("enabled") is True:
        # Enabling dual-lidar safety requires validated extrinsics and a
        # non-default UNKNOWN policy; a bare boolean cannot enable it.
        if config.get("require_validated_extrinsics") is not True:
            raise DualLidarConfigError(
                "dual_lidar_safety requires validated extrinsics"
            )
        if config.get("unknown_is_clear") is not False:
            raise DualLidarConfigError(
                "dual_lidar_safety unknown_is_clear must stay false"
            )
    sources = config.get("sources") or {}
    pandar = sources.get("pandarxt16") or {}
    if pandar.get("can_contribute_clear") is True:
        # A sensor that can contribute formal CLEAR must carry a validated TF.
        if pandar.get("transform_tier") not in {"validated_tf", "validated_geometry"}:
            raise DualLidarConfigError(
                "Pandar cannot contribute CLEAR without a validated transform tier"
            )
    max_age = float(config.get("max_evidence_age_seconds", math.nan))
    if not math.isfinite(max_age) or max_age <= 0.0:
        raise DualLidarConfigError("max_evidence_age_seconds must be positive")
    return config


def dual_lidar_safety_enabled(path: str) -> bool:
    """Read whether dual-lidar safety is currently enabled (fail-closed)."""
    return bool(load_dual_lidar_safety_config(path).get("enabled", False))


def observability_params(config: dict) -> dict[str, Any]:
    section = config.get("rotation_observability") or {}
    return {
        "footprint_radius_m": float(section.get("footprint_radius_m", 0.350)),
        "envelope_radius_m": float(section.get("envelope_radius_m", 0.511)),
        "requested_turn_range_deg": float(section.get("requested_turn_range_deg", 30.0)),
        "require_full_360": bool(section.get("require_full_360", True)),
    }
