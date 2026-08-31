"""ROS-independent decode and calibration helpers."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import threading
from typing import Any

import cv2
import numpy as np
import yaml


@dataclass(frozen=True)
class Calibration:
    calibrated: bool
    width: int
    height: int
    distortion_model: str
    d: tuple[float, ...]
    k: tuple[float, ...]
    r: tuple[float, ...]
    p: tuple[float, ...]
    source: str
    reason: str = ""


def decode_jpeg(payload: bytes) -> np.ndarray:
    if not payload:
        raise ValueError("empty JPEG payload")
    image = cv2.imdecode(
        np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR
    )
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("JPEG decode did not produce a BGR image")
    return image


def image_content_metrics(image: np.ndarray) -> dict[str, float | bool]:
    """Detect the solid-green failure signature produced by damaged H.264.

    This is deliberately a narrow fail-closed transport check, not a claim
    about scene quality.  Unitree topic corruption observed on this host fills
    almost the entire decoded frame with BGR (0, 135, 0).
    """
    if image.ndim != 3 or image.shape[2] != 3 or image.size == 0:
        raise ValueError("content check requires a non-empty BGR image")
    values = image.astype(np.int16, copy=False)
    blue, green, red = (values[:, :, index] for index in range(3))
    solid_green = (
        (green >= 80)
        & (green >= blue * 3)
        & (green >= red * 3)
        & (blue <= 12)
        & (red <= 12)
    )
    solid_green_fraction = float(np.mean(solid_green))
    channel_stddev_max = float(
        max(np.std(values[:, :, index]) for index in range(3))
    )
    passed = solid_green_fraction < 0.95
    return {
        "passed": passed,
        "solid_green_fraction": solid_green_fraction,
        "channel_stddev_max": channel_stddev_max,
    }


def strip_h264_vendor_prefix(payload: bytes) -> bytes:
    """Return Annex-B data after a small Unitree per-message prefix."""
    candidates = []
    for marker in (b"\x00\x00\x00\x01", b"\x00\x00\x01"):
        index = payload.find(marker, 0, 32)
        if index >= 0:
            candidates.append(index)
    if not candidates:
        raise ValueError("payload is neither JPEG nor Annex-B H.264")
    return payload[min(candidates) :]


class FrameDecoder:
    """Persistent H.264 decoder plus stateless JPEG decoding."""

    def __init__(self) -> None:
        self._h264 = None
        self._lock = threading.Lock()

    def decode(self, payload: bytes) -> tuple[np.ndarray | None, str]:
        if payload.startswith(b"\xff\xd8"):
            return decode_jpeg(payload), "jpeg"
        annex_b = strip_h264_vendor_prefix(payload)
        with self._lock:
            if self._h264 is None:
                try:
                    import av
                except ImportError as exc:
                    raise RuntimeError(
                        "H.264 input requires system package python3-av"
                    ) from exc
                self._h264 = av.CodecContext.create("h264", "r")
            frames = []
            for packet in self._h264.parse(annex_b):
                frames.extend(self._h264.decode(packet))
            if not frames:
                return None, "h264"
            return frames[-1].to_ndarray(format="bgr24"), "h264"


def _matrix(data: dict[str, Any], key: str, expected: int) -> tuple[float, ...]:
    value = data.get(key, {})
    raw = value.get("data", []) if isinstance(value, dict) else value
    numbers = tuple(float(item) for item in raw)
    if len(numbers) != expected:
        raise ValueError(f"{key} requires {expected} values, got {len(numbers)}")
    return numbers


def load_calibration(path: str | Path) -> Calibration:
    source = str(Path(path).expanduser().resolve())
    data = yaml.safe_load(Path(source).read_text(encoding="utf-8")) or {}
    status = str(data.get("calibration_status", "")).strip().lower()
    width = int(data.get("image_width", 0))
    height = int(data.get("image_height", 0))
    k = _matrix(data, "camera_matrix", 9)
    d_raw = data.get("distortion_coefficients", {})
    d_values = d_raw.get("data", []) if isinstance(d_raw, dict) else d_raw
    d = tuple(float(item) for item in d_values)
    r = _matrix(data, "rectification_matrix", 9)
    p = _matrix(data, "projection_matrix", 12)
    calibrated = (
        status == "calibrated"
        and width > 0
        and height > 0
        and len(d) > 0
        and k[0] > 0.0
        and k[4] > 0.0
        and p[0] > 0.0
        and p[5] > 0.0
    )
    reason = "" if calibrated else "calibration_status/K/P is not validated"
    return Calibration(
        calibrated=calibrated,
        width=width,
        height=height,
        distortion_model=str(data.get("distortion_model", "")),
        d=d,
        k=k,
        r=r,
        p=p,
        source=source,
        reason=reason,
    )


def select_topic_payload(
    message: Any, *, max_payload_bytes: int = 20_000_000
) -> tuple[bytes, str]:
    for field in ("video720p", "video360p", "video180p"):
        value = getattr(message, field, None)
        if value and len(value) <= max_payload_bytes:
            return bytes(value), field
        if value and len(value) > max_payload_bytes:
            # Some firmware/ROS IDL combinations expose invalid lengths for
            # unused resolutions. Never materialize such a sequence.
            continue
    raise ValueError("Go2FrontVideoData contains no safely bounded payload")
