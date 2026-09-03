#!/usr/bin/env python3
"""Persist the robot-local RealSense MJPEG stream for the WebUI.

The robot exposes the camera over HTTP. Keeping this small capture process
next to the WebUI avoids pushing 1280x720 JPEG frames through the cross-host
ROS graph solely for display/vision. It writes the same atomic ``latest.jpg``
and ``camera_status.json`` files used by the existing ROS worker.
"""

from __future__ import annotations

import argparse
import json
import time
import urllib.error
import urllib.request
from pathlib import Path


def _jpeg_dimensions(data: bytes) -> tuple[int | None, int | None]:
    """Read JPEG SOF dimensions without depending on an image library."""
    index = 2
    while index + 9 < len(data):
        if data[index] != 0xFF:
            index += 1
            continue
        marker = data[index + 1]
        if marker in (0xD8, 0xD9) or 0xD0 <= marker <= 0xD7:
            index += 2
            continue
        if index + 4 > len(data):
            return None, None
        segment_length = (data[index + 2] << 8) | data[index + 3]
        if 0xC0 <= marker <= 0xCF and marker not in (0xC4, 0xC8, 0xCC):
            if index + 9 <= len(data):
                height = (data[index + 5] << 8) | data[index + 6]
                width = (data[index + 7] << 8) | data[index + 8]
                return width, height
            return None, None
        index += 2 + segment_length
    return None, None


def _read_exact(stream, size: int) -> bytes:
    chunks: list[bytes] = []
    remaining = size
    while remaining > 0:
        chunk = stream.read(remaining)
        if not chunk:
            raise EOFError("MJPEG stream ended inside a frame")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def _frames(url: str):
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    request = urllib.request.Request(url, headers={"User-Agent": "go2w-webui/1"})
    with opener.open(request, timeout=10.0) as response:
        while True:
            line = response.readline()
            if not line:
                return
            if not line.startswith(b"--"):
                continue
            headers: dict[str, str] = {}
            while True:
                header = response.readline()
                if not header or header in (b"\r\n", b"\n"):
                    break
                name, _, value = header.decode("latin1").partition(":")
                headers[name.strip().lower()] = value.strip()
            content_length = int(headers.get("content-length", "0"))
            if content_length <= 0:
                continue
            yield _read_exact(response, content_length)
            response.readline()


def _write_frame(data: bytes, latest: Path, status_path: Path) -> None:
    temporary = latest.with_name(latest.name + ".http.tmp")
    temporary.write_bytes(data)
    temporary.replace(latest)
    width, height = _jpeg_dimensions(data)
    status = {
        "type": "camera_status",
        "available": True,
        "received_at": time.monotonic(),
        "captured_at": time.time(),
        "width": width,
        "height": height,
        "format": "jpeg",
        "source": "robot_http_mjpeg",
    }
    status_tmp = status_path.with_name(status_path.name + ".http.tmp")
    status_tmp.write_text(json.dumps(status, ensure_ascii=False), encoding="utf-8")
    status_tmp.replace(status_path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", required=True)
    parser.add_argument("--latest", required=True)
    parser.add_argument("--status", required=True)
    parser.add_argument("--retry-seconds", type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    latest = Path(args.latest)
    status_path = Path(args.status)
    latest.parent.mkdir(parents=True, exist_ok=True)
    status_path.parent.mkdir(parents=True, exist_ok=True)
    while True:
        try:
            count = 0
            for data in _frames(args.url):
                if data:
                    _write_frame(data, latest, status_path)
                    count += 1
                    if count == 1 or count % 30 == 0:
                        print(
                            f"camera_diag frames={count} bytes={len(data)} "
                            f"url={args.url}",
                            flush=True,
                        )
        except (EOFError, OSError, ValueError, urllib.error.URLError) as exc:
            print(f"camera reconnect: {type(exc).__name__}: {exc}", flush=True)
            time.sleep(max(0.2, float(args.retry_seconds)))


if __name__ == "__main__":
    raise SystemExit(main())
