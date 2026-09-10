#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
camera_config="${repo_root}/configs/go2w/camera_intrinsics.yaml"
extrinsics_config="${repo_root}/configs/go2w/sensor_extrinsics.yaml"

/usr/bin/python3 - "${camera_config}" "${extrinsics_config}" "${repo_root}" "$@" <<'PY'
import math
import pathlib
import sys
import yaml

camera = yaml.safe_load(pathlib.Path(sys.argv[1]).read_text()) or {}
extrinsics = yaml.safe_load(pathlib.Path(sys.argv[2]).read_text()) or {}
repo_root = pathlib.Path(sys.argv[3])
errors = []
if camera.get("calibration_status") != "calibrated":
    errors.append("camera intrinsics are not calibrated")
if extrinsics.get("calibration_status") != "calibrated" or not extrinsics.get("confirmed"):
    errors.append("RGB-LiDAR extrinsics are not calibrated and confirmed")
translation = extrinsics.get("translation_m") or {}
rotation = extrinsics.get("rotation_rpy_rad") or {}
try:
    transform_values = [
        float(translation[key]) for key in ("x", "y", "z")
    ] + [float(rotation[key]) for key in ("roll", "pitch", "yaw")]
    if not all(math.isfinite(value) for value in transform_values):
        raise ValueError
except (KeyError, TypeError, ValueError):
    errors.append("extrinsic transform is incomplete or non-finite")
validation = extrinsics.get("validation") or {}
required = max(5, int(validation.get("required_scene_count", 5)))
if int(validation.get("completed_scene_count", 0)) < required:
    errors.append("fewer than 5 overlay validation scenes")
mean_error = validation.get("mean_edge_error_px")
maximum_error = validation.get("maximum_allowed_mean_edge_error_px")
try:
    if (
        not math.isfinite(float(mean_error))
        or not math.isfinite(float(maximum_error))
        or float(mean_error) > float(maximum_error)
    ):
        raise ValueError
except (TypeError, ValueError):
    errors.append("overlay mean edge error is absent or above threshold")
bands = {str(item).strip().lower() for item in validation.get("distance_bands_tested", [])}
if not {"near", "medium", "far"}.issubset(bands):
    errors.append("near/medium/far distance bands have not all passed")
if not validation.get("moved_position_recheck_passed"):
    errors.append("moved-position overlay recheck has not passed")
report = str(validation.get("report_path") or "").strip()
if not report:
    errors.append("overlay validation report path is absent")
else:
    report_path = pathlib.Path(report)
    if not report_path.is_absolute():
        report_path = repo_root / report_path
    if not report_path.is_file():
        errors.append("overlay validation report does not exist")
if errors:
    print("BLOCKED: " + "; ".join(errors), file=sys.stderr)
    raise SystemExit(3)
print("PASS: recorded overlay validation gates are satisfied")
PY
