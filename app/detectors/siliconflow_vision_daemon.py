"""Long-running SiliconFlow VLM daemon (Conda Python).

The main ROS 2 process (system Python) talks to this daemon over a Unix domain
socket using ``siliconflow_vision_protocol``.  It keeps the OpenAI/PIL runtime
in one process and separates realtime (Quick/Verify) from background (Full
Semantic) work with two small thread pools.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import socketserver
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


class ImageEncodingCache:
    """Small LRU for base64 image data URLs keyed by path+mtime+size."""

    def __init__(self, max_items: int = 8) -> None:
        self.max_items = max_items
        self._items: dict[tuple[str, int, int], dict[str, object]] = {}

    def get(self, image_path: str) -> dict[str, object] | None:
        try:
            stat = os.stat(image_path)
            key = (image_path, stat.st_mtime_ns, stat.st_size)
        except OSError:
            return None
        if key in self._items:
            value = self._items.pop(key)
            self._items[key] = value
            return value
        return None

    def put(self, image_path: str, value: dict[str, object]) -> None:
        try:
            stat = os.stat(image_path)
            key = (image_path, stat.st_mtime_ns, stat.st_size)
        except OSError:
            return
        self._items[key] = value
        if len(self._items) > self.max_items:
            oldest = next(iter(self._items))
            self._items.pop(oldest, None)


class VLMDaemon:
    def __init__(self, socket_path: str) -> None:
        self.socket_path = Path(socket_path)
        self.realtime_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vlm-realtime")
        self.semantic_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vlm-background")
        self.image_cache = ImageEncodingCache(max_items=8)
        self._shutdown = threading.Event()
        self._server: socketserver.UnixStreamServer | None = None
        self._settings = None
        self._realtime_client = None
        self._semantic_client = None
        self._client_lock = threading.Lock()

    def _ensure_clients(self) -> tuple[Any, Any]:
        """Return (realtime_client, semantic_client), creating them once."""
        with self._client_lock:
            if self._realtime_client is not None and self._semantic_client is not None:
                return self._realtime_client, self._semantic_client
            from openai import OpenAI

            from app.config import get_settings

            self._settings = get_settings()
            common = {
                "api_key": self._settings.siliconflow_api_key,
                "base_url": self._settings.siliconflow_base_url,
                "timeout": self._settings.siliconflow_timeout_seconds,
            }
            self._realtime_client = OpenAI(**common)
            self._semantic_client = OpenAI(**common)
            return self._realtime_client, self._semantic_client

    def start(self) -> None:
        self.socket_path.parent.mkdir(parents=True, exist_ok=True)
        if self.socket_path.exists():
            try:
                self.socket_path.unlink()
            except OSError:
                pass

        class Handler(socketserver.StreamRequestHandler):
            daemon = self

            def handle(self) -> None:
                try:
                    length_line = self.rfile.readline()
                    if not length_line:
                        return
                    request = json.loads(length_line.decode("utf-8"))
                except Exception:
                    return
                try:
                    if str(request.get("mode")) == "semantic":
                        future = self.daemon.semantic_executor.submit(
                            daemon_handle_sync, self.daemon, request
                        )
                    else:
                        future = self.daemon.realtime_executor.submit(
                            daemon_handle_sync, self.daemon, request
                        )
                    response = future.result(timeout=180)
                except Exception as exc:  # noqa: BLE001
                    response = {
                        "request_id": str(request.get("request_id", "")),
                        "ok": False,
                        "mode": str(request.get("mode", "")),
                        "payload": {},
                        "error": f"{type(exc).__name__}: {exc}",
                        "latency_ms": 0.0,
                        "frame_id": str(request.get("frame_id", "")),
                    }
                try:
                    self.wfile.write(
                        (json.dumps(response, ensure_ascii=False) + "\n").encode("utf-8")
                    )
                    self.wfile.flush()
                except OSError:
                    pass

        self._server = socketserver.UnixStreamServer(str(self.socket_path), Handler)
        threading.Thread(target=self._server.serve_forever, daemon=True).start()

    def stop(self) -> None:
        self._shutdown.set()
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
        self.realtime_executor.shutdown(wait=False)
        self.semantic_executor.shutdown(wait=False)
        try:
            self.socket_path.unlink()
        except OSError:
            pass


def daemon_handle_sync(daemon: VLMDaemon, request: dict) -> dict:
    """Execute the VLM request in the appropriate lane synchronously."""
    from app.detectors.siliconflow_vision_worker import (
        _quick_detect,
        _verify_detect,
    )
    from app.config import get_settings
    from app.llm_clients.siliconflow_client import SiliconFlowVisionClient

    start = time.perf_counter()
    realtime_client, semantic_client = daemon._ensure_clients()
    request_id = str(request.get("request_id", ""))
    mode = str(request.get("mode", "quick"))
    image_path = str(request.get("image_path", ""))
    target = str(request.get("target", ""))
    bbox = request.get("bbox")
    if not image_path:
        return _error_response(request_id, mode, "image_path is required")
    settings = get_settings()
    model = str(request.get("model") or "")
    extra = str(request.get("extra_instructions") or "")
    try:
        if mode == "quick":
            payload = _quick_detect(
                settings,
                image_path,
                target,
                extra,
                model,
                client=realtime_client,
            )
        elif mode == "semantic":
            client = SiliconFlowVisionClient(
                settings=settings, client=semantic_client
            )
            payload = client.analyze_scene(
                image_path,
                target,
                extra_instructions=extra or None,
            )
        elif mode == "verify":
            if not bbox:
                return _error_response(request_id, mode, "bbox is required for verify")
            bbox_text = ",".join(f"{float(v):.4f}" for v in bbox)
            payload = _verify_detect(
                settings,
                image_path,
                target,
                bbox_text,
                model,
                client=realtime_client,
            )
        else:
            return _error_response(request_id, mode, f"unknown mode: {mode}")
    except Exception as exc:  # noqa: BLE001
        return _error_response(
            request_id, mode, f"{type(exc).__name__}: {exc}"
        )
    latency_ms = (time.perf_counter() - start) * 1000.0
    return {
        "request_id": request_id,
        "ok": True,
        "mode": mode,
        "payload": payload,
        "error": None,
        "latency_ms": round(latency_ms, 3),
        "frame_id": str(request.get("frame_id", "")),
    }


def _error_response(request_id: str, mode: str, error: str) -> dict:
    return {
        "request_id": request_id,
        "ok": False,
        "mode": mode,
        "payload": {},
        "error": error,
        "latency_ms": 0.0,
        "frame_id": "",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--socket", default=str(PROJECT_ROOT / "runtime/go2w/siliconflow_vlm.sock"))
    args = parser.parse_args()
    daemon = VLMDaemon(args.socket)
    daemon.start()
    print(json.dumps({"status": "started", "socket": args.socket}), flush=True)

    def _stop(_sig=None, _frame=None):
        daemon.stop()
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        daemon.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
