"""Non-blocking reader for complete ROS-worker frame bundles."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


class FrameBundleUnavailable(RuntimeError):
    """No complete, valid latest bundle is currently available."""


@dataclass(frozen=True)
class FrameBundle:
    directory: Path
    image_path: Path
    payload: dict[str, Any]

    @property
    def frame_id(self) -> int:
        return int(self.payload["frame_id"])


class FrameBundleReader:
    def __init__(self, spool_root: str | Path) -> None:
        self.root = Path(spool_root).resolve()

    def read_latest(self, timeout_seconds: float = 0.0) -> FrameBundle:
        if timeout_seconds < 0.0:
            raise ValueError("timeout_seconds must be non-negative")
        deadline = time.monotonic() + timeout_seconds
        last_error = "latest bundle unavailable"
        while True:
            try:
                return self._read_once()
            except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
                last_error = str(exc)
            if time.monotonic() >= deadline:
                raise FrameBundleUnavailable(last_error)
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))

    def _read_once(self) -> FrameBundle:
        latest = self.root / "latest"
        directory = latest.resolve(strict=True)
        if self.root not in directory.parents:
            raise ValueError("latest bundle resolves outside spool root")
        if not (directory / "READY").is_file():
            raise ValueError("latest bundle has no READY marker")
        metadata_path = directory / "frame_bundle.json"
        payload = json.loads(metadata_path.read_text(encoding="utf-8"))
        self._validate(payload)
        image_name = payload["image_path"]
        if Path(image_name).name != image_name:
            raise ValueError("image_path must be a local filename")
        image_path = directory / image_name
        if not image_path.is_file() or image_path.stat().st_size == 0:
            raise ValueError("bundle image is missing or empty")
        return FrameBundle(directory=directory, image_path=image_path, payload=payload)

    @staticmethod
    def _validate(payload: dict[str, Any]) -> None:
        if payload.get("schema_version") != "1.0":
            raise ValueError("unsupported frame bundle schema")
        required = (
            "session_id",
            "frame_id",
            "image_path",
            "image_receive_time_ns",
            "image_capture_time_trusted",
            "camera_frame",
            "camera_info",
            "robot_pose",
            "clearance",
            "sensor_health",
        )
        missing = [key for key in required if key not in payload]
        if missing:
            raise ValueError(f"frame bundle missing: {', '.join(missing)}")
        if payload["image_capture_time_trusted"] is not False:
            raise ValueError("built-in camera capture time must remain untrusted")
        if not isinstance(payload["frame_id"], int) or payload["frame_id"] < 1:
            raise ValueError("frame_id must be a positive integer")
        for section in ("camera_info", "robot_pose", "clearance", "sensor_health"):
            if not isinstance(payload[section], dict):
                raise ValueError(f"{section} must be an object")
        for key in ("camera", "camera_info_calibrated", "lidar", "lio", "tf"):
            if not isinstance(payload["sensor_health"].get(key), bool):
                raise ValueError(f"sensor_health.{key} must be boolean")
        for key in (
            "rgb_lidar_overlay",
            "rgb_lidar_extrinsics",
            "rgb_lidar_fusion",
        ):
            if key in payload["sensor_health"] and not isinstance(
                payload["sensor_health"][key], bool
            ):
                raise ValueError(f"sensor_health.{key} must be boolean")
        # Optional Pandar / dual-LiDAR sections are forward-compatible: when
        # present they must be mappings, but old consumers must not crash when
        # they are absent.
        for optional in ("pandar", "dual_lidar"):
            if optional in payload and not isinstance(payload[optional], dict):
                raise ValueError(f"{optional} must be an object when present")


def pandar_status(payload: dict) -> dict:
    """Read the Pandar status section with fail-closed defaults when absent."""
    return {
        "raw_fresh": bool((payload.get("pandar") or {}).get("raw_fresh", False)),
        "raw_rate_ok": bool((payload.get("pandar") or {}).get("raw_rate_ok", False)),
        "zero_return_filter_ready": bool(
            (payload.get("pandar") or {}).get("zero_return_filter_ready", False)
        ),
        "preprocessor_ready": bool(
            (payload.get("pandar") or {}).get("preprocessor_ready", False)
        ),
        "clock_tier": str((payload.get("pandar") or {}).get("clock_tier", "unvalidated")),
        "extrinsics_validated": bool(
            (payload.get("pandar") or {}).get("extrinsics_validated", False)
        ),
        "self_occlusion_validated": bool(
            (payload.get("pandar") or {}).get("self_occlusion_validated", False)
        ),
        "safety_integration_ready": bool(
            (payload.get("pandar") or {}).get("safety_integration_ready", False)
        ),
    }


def dual_lidar_status(payload: dict) -> dict:
    """Read the dual-LiDAR status section with fail-closed defaults."""
    return {
        "diagnostic_ready": bool(
            (payload.get("dual_lidar") or {}).get("diagnostic_ready", False)
        ),
        "rotation_observability_valid": bool(
            (payload.get("dual_lidar") or {}).get("rotation_observability_valid", False)
        ),
        "rotation_clearance_valid": bool(
            (payload.get("dual_lidar") or {}).get("rotation_clearance_valid", False)
        ),
    }
