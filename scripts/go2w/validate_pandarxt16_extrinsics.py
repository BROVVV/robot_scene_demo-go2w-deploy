#!/usr/bin/env python3
"""Validate a PandarXT-16 extrinsic config against captured scene residuals.

Reads the formal extrinsics slot
(``configs/go2w/hesai_pandarxt16_extrinsics.yaml``) plus the offline
calibration report, and re-states the acceptance decision. This validator
never authorises a TF publication, safety integration or motion.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import sys

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def _parse_bool(value) -> bool:
    return value is True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--extrinsics",
        type=Path,
        default=Path("configs/go2w/hesai_pandarxt16_extrinsics.yaml"),
    )
    parser.add_argument(
        "--calibration-report",
        type=Path,
        default=Path(
            "outputs/go2w_acceptance/dual_lidar_calibration/calibration/calibration_report.md"
        ),
    )
    parser.add_argument(
        "--scene-residuals",
        type=Path,
        default=Path(
            "outputs/go2w_acceptance/dual_lidar_calibration/calibration/scene_residuals.json"
        ),
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    config = yaml.safe_load(args.extrinsics.read_text(encoding="utf-8")) or {}
    confirmed = _parse_bool(config.get("confirmed"))
    authorizes_tf = _parse_bool(config.get("authorizes_tf_publication"))
    authorizes_safety = _parse_bool(config.get("authorizes_safety_integration"))
    authorizes_motion = _parse_bool(config.get("authorizes_motion"))
    candidate = (config.get("transform_candidate") or {})
    translation = candidate.get("translation_m") or {}
    rotation = candidate.get("rotation_rpy_deg") or {}

    thresholds = config.get("acceptance_thresholds") or {}
    translation_residual_limit = float(thresholds.get("translation_residual_m", 0.05))
    yaw_limit = float(thresholds.get("yaw_multi_scene_consistency_deg", 3.0))

    residuals = {}
    if args.scene_residuals.is_file():
        residuals = json.loads(args.scene_residuals.read_text(encoding="utf-8"))

    residual_values = [
        float(item["residual_m"])
        for item in residuals.values()
        if math.isfinite(float(item.get("residual_m", math.nan)))
    ]
    yaw_values = [
        float(item["yaw_deg"])
        for item in residuals.values()
        if math.isfinite(float(item.get("yaw_deg", math.nan)))
    ]
    median_residual = (
        float(math.inf) if not residual_values else float(statistics.median(residual_values))
    )
    yaw_consistency = (
        float(math.inf)
        if len(yaw_values) < 2
        else max(yaw_values) - min(yaw_values)
    )
    thresholds_pass = bool(
        median_residual < translation_residual_limit
        and yaw_consistency <= yaw_limit
        and len(residual_values) >= 3
    )
    # A config may only be confirmed when the thresholds pass AND the config
    # itself does not accidentally authorise anything.
    contract_ok = not (authorizes_tf or authorizes_safety or authorizes_motion)
    final_confirmed = bool(confirmed and thresholds_pass and contract_ok)

    checks = {
        "config_confirmed": confirmed,
        "thresholds_pass": thresholds_pass,
        "median_residual_below_limit": median_residual < translation_residual_limit,
        "yaw_consistency_within_limit": yaw_consistency <= yaw_limit,
        "at_least_three_scenes": len(residual_values) >= 3,
        "no_authorization_flags": contract_ok,
        "candidate_transform_present": bool(translation and rotation),
    }
    report = {
        "schema": "go2w.pandarxt16.extrinsics.validation.v1",
        "passed": all(checks.values()) and final_confirmed,
        "confirmed": final_confirmed,
        "authorizes_tf_publication": False,
        "authorizes_safety_integration": False,
        "authorizes_motion": False,
        "checks": checks,
        "candidate_translation_m": {
            "x": translation.get("x"),
            "y": translation.get("y"),
            "z": translation.get("z"),
        },
        "candidate_yaw_deg": rotation.get("yaw"),
        "median_residual_m": median_residual if math.isfinite(median_residual) else None,
        "yaw_consistency_deg": yaw_consistency if math.isfinite(yaw_consistency) else None,
        "scene_count": len(residual_values),
        "thresholds": {
            "translation_residual_m": translation_residual_limit,
            "yaw_multi_scene_consistency_deg": yaw_limit,
        },
    }
    encoded = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0 if report["passed"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
