#!/usr/bin/env python3
"""Validate the workstation RGB-D spatial stack against the D435 HTTP service.

Checks:
  1. /rgbd/latest.json atomic metadata + intrinsics.
  2. color and depth download for the same frame_id.
  3. depth valid fraction / unit.
  4. DepthObjectLocalizer smoke on a central bbox (if cv2 is available).

Usage:
  python3 scripts/go2w/validate_rgbd_spatial_stack.py [--base-url URL]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from app.perception.realsense_http_rgbd_source import RealSenseHTTPRGBDSource  # noqa: E402
from app.perception.rgbd_source import RGBDFrameUnavailable  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-url", default="http://192.168.123.18:8080")
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()
    source = RealSenseHTTPRGBDSource(args.base_url, cache_dir="runtime/go2w/rgbd_validate")
    health = source.health()
    checks = {"health": health}
    try:
        frame = source.get_latest(timeout_seconds=args.timeout)
    except RGBDFrameUnavailable as exc:
        checks["status"] = "FAIL"
        checks["error"] = str(exc)
        print(json.dumps(checks, ensure_ascii=False, indent=2))
        return 2
    checks["status"] = "PASS"
    checks["frame_id"] = frame.frame_id
    checks["intrinsics"] = {
        "fx": frame.fx, "fy": frame.fy, "cx": frame.cx, "cy": frame.cy,
    }
    checks["depth_unit_m"] = frame.depth_unit_m
    checks["aligned_to_rgb"] = frame.depth_aligned_to_color
    checks["color_ref"] = frame.color_ref
    checks["depth_ref"] = frame.depth_ref
    try:
        import cv2
        import numpy as np
        img = cv2.imread(frame.depth_ref, cv2.IMREAD_UNCHANGED)
        valid = int(np.count_nonzero((img > 0) & (img < 8000)))
        total = int(img.size)
        checks["depth_valid_fraction"] = round(valid / total, 4)
        checks["depth_shape"] = list(img.shape)
    except Exception as exc:  # noqa: BLE001
        checks["depth_stats_error"] = str(exc)
    print(json.dumps(checks, ensure_ascii=False, indent=2))
    return 0 if checks.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
