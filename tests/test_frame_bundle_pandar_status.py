"""Tests for Pandar / dual-LiDAR status in the frame bundle."""

from __future__ import annotations

from pathlib import Path

import pytest

from app.live_robot.frame_bundle_reader import (
    FrameBundle,
    FrameBundleReader,
    dual_lidar_status,
    pandar_status,
)


def _write_bundle(tmp_path: Path, *, payload: dict) -> Path:
    directory = tmp_path / "bundles" / "session-1"
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "image.jpg").write_bytes(b"jpeg")
    (directory / "frame_bundle.json").write_text(
        _json(payload), encoding="utf-8"
    )
    (directory / "READY").write_text("ok", encoding="utf-8")
    latest = tmp_path / "bundles" / "latest"
    try:
        latest.unlink()
    except FileNotFoundError:
        pass
    latest.symlink_to(directory.name)
    return latest


def _json(value: dict) -> str:
    import json

    return json.dumps(value)


def _base_payload() -> dict:
    return {
        "schema_version": "1.0",
        "session_id": "session-1",
        "frame_id": 7,
        "image_path": "image.jpg",
        "image_receive_time_ns": 1,
        "image_capture_time_trusted": False,
        "camera_frame": "front_camera_optical_frame",
        "camera_info": {"width": 1920, "height": 1080},
        "robot_pose": {"available": True},
        "clearance": {"front_status": "clear"},
        "sensor_health": {
            "camera": True,
            "camera_info_calibrated": True,
            "lidar": True,
            "lio": True,
            "tf": False,
        },
    }


def test_bundle_with_pandar_section_validates(tmp_path) -> None:
    payload = _base_payload()
    payload["pandar"] = {
        "raw_fresh": True,
        "raw_rate_ok": True,
        "zero_return_filter_ready": True,
        "preprocessor_ready": True,
        "clock_tier": "host_receive_time_only",
        "extrinsics_validated": False,
        "self_occlusion_validated": False,
        "safety_integration_ready": False,
    }
    payload["dual_lidar"] = {
        "diagnostic_ready": True,
        "rotation_observability_valid": False,
        "rotation_clearance_valid": False,
    }
    _write_bundle(tmp_path, payload=payload)
    bundle = FrameBundleReader(tmp_path / "bundles").read_latest()
    assert bundle.payload["pandar"]["raw_fresh"] is True


def test_pandar_status_fail_closed_when_absent() -> None:
    status = pandar_status(_base_payload())
    assert status["raw_fresh"] is False
    assert status["extrinsics_validated"] is False
    assert status["clock_tier"] == "unvalidated"
    assert status["safety_integration_ready"] is False


def test_dual_lidar_status_fail_closed_when_absent() -> None:
    status = dual_lidar_status(_base_payload())
    assert status["diagnostic_ready"] is False
    assert status["rotation_observability_valid"] is False
    assert status["rotation_clearance_valid"] is False


def test_pandar_status_reads_bundle(tmp_path) -> None:
    payload = _base_payload()
    payload["pandar"] = {
        "raw_fresh": True,
        "raw_rate_ok": True,
        "zero_return_filter_ready": True,
        "preprocessor_ready": True,
        "clock_tier": "host_receive_time_only",
        "extrinsics_validated": False,
        "self_occlusion_validated": True,
        "safety_integration_ready": False,
    }
    payload["dual_lidar"] = {
        "diagnostic_ready": True,
        "rotation_observability_valid": False,
        "rotation_clearance_valid": False,
    }
    _write_bundle(tmp_path, payload=payload)
    bundle = FrameBundleReader(tmp_path / "bundles").read_latest()
    p = pandar_status(bundle.payload)
    assert p["raw_fresh"] is True
    assert p["raw_rate_ok"] is True
    assert p["clock_tier"] == "host_receive_time_only"
    assert p["extrinsics_validated"] is False
    d = dual_lidar_status(bundle.payload)
    assert d["diagnostic_ready"] is True
    assert d["rotation_clearance_valid"] is False


def test_old_consumer_does_not_crash_without_pandar_section(tmp_path) -> None:
    _write_bundle(tmp_path, payload=_base_payload())
    bundle = FrameBundleReader(tmp_path / "bundles").read_latest()
    assert isinstance(bundle, FrameBundle)
    assert bundle.frame_id == 7
