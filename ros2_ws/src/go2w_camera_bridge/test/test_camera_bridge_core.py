from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from go2w_camera_bridge.camera_bridge_core import (
    FrameDecoder,
    decode_jpeg,
    image_content_metrics,
    load_calibration,
    select_topic_payload,
    strip_h264_vendor_prefix,
)


def test_decode_jpeg_and_payload_selection():
    image = np.zeros((12, 16, 3), dtype=np.uint8)
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok

    class Message:
        video720p = encoded.tobytes()
        video360p = []
        video180p = []

    payload, source = select_topic_payload(Message())
    decoded = decode_jpeg(payload)
    assert source == "video720p"
    assert decoded.shape == (12, 16, 3)


def test_placeholder_calibration_is_not_accepted(tmp_path):
    config = tmp_path / "placeholder_intrinsics.yaml"
    config.write_text(yaml.safe_dump({
        "calibration_status": "unmeasured",
        "image_width": 1920,
        "image_height": 1080,
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {"data": [0.0, 0.0, 0.0, 0.0, 0.0]},
        "camera_matrix": {"data": [0.0] * 9},
        "rectification_matrix": {
            "data": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        },
        "projection_matrix": {"data": [0.0] * 12},
    }), encoding="utf-8")
    calibration = load_calibration(config)
    assert not calibration.calibrated
    assert calibration.k[0] == 0.0


def test_measured_project_calibration_is_accepted():
    config = Path(__file__).resolve().parents[4] / "configs/go2w/camera_intrinsics.yaml"
    calibration = load_calibration(config)
    assert calibration.calibrated
    assert calibration.width == 1920
    assert calibration.height == 1080
    assert calibration.k[0] > 0.0
    assert calibration.k[4] > 0.0


def test_empty_payload_is_rejected():
    with pytest.raises(ValueError):
        decode_jpeg(b"")


def test_content_check_rejects_observed_solid_green_transport_failure():
    damaged = np.zeros((720, 1280, 3), dtype=np.uint8)
    damaged[:, :, 1] = 135
    metrics = image_content_metrics(damaged)
    assert not metrics["passed"]
    assert metrics["solid_green_fraction"] == pytest.approx(1.0)

    valid = np.zeros((40, 60, 3), dtype=np.uint8)
    valid[:, :30] = (20, 100, 180)
    valid[:, 30:] = (180, 80, 20)
    assert image_content_metrics(valid)["passed"]


def test_h264_vendor_prefix_is_removed_without_touching_invalid_fallback():
    payload = b"\xba\x85\x00\x00\x00\x00\x00\x01\x67\x64"
    assert strip_h264_vendor_prefix(payload) == b"\x00\x00\x00\x01\x67\x64"

    class Message:
        video720p = []
        video360p = [0] * 21
        video180p = b"\xff\xd8\xff\xd9"

    selected, field = select_topic_payload(Message(), max_payload_bytes=20)
    assert field == "video180p"
    assert selected == b"\xff\xd8\xff\xd9"
