"""JSON-line protocol for the SiliconFlow VLM daemon.

The daemon runs with the Conda Python (which has OpenAI/PIL).  The ROS 2 system
Python only needs stdlib ``socket`` / ``json`` to talk to it.

Protocol: each message is one JSON object terminated by ``\\n``.
"""

from __future__ import annotations

import json
from pathlib import Path
import socket
import time
from dataclasses import dataclass, field
from typing import Any


MODE_QUICK = "quick"
MODE_SEMANTIC = "semantic"
MODE_VERIFY = "verify"

PRIORITY_REALTIME = "realtime"
PRIORITY_BACKGROUND = "background"


@dataclass
class VLMRequest:
    request_id: str
    mode: str
    image_path: str
    frame_id: str = ""
    target: str = ""
    bbox: list[float] | None = None
    priority: str = PRIORITY_REALTIME
    extra_instructions: str = ""
    model: str = ""
    robot_pose: dict[str, Any] | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "request_id": self.request_id,
            "mode": self.mode,
            "image_path": self.image_path,
            "frame_id": self.frame_id,
            "target": self.target,
            "bbox": self.bbox,
            "priority": self.priority,
            "extra_instructions": self.extra_instructions,
            "model": self.model,
            "robot_pose": self.robot_pose,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "VLMRequest":
        return cls(
            request_id=str(data.get("request_id", "")),
            mode=str(data.get("mode", MODE_QUICK)),
            image_path=str(data.get("image_path", "")),
            frame_id=str(data.get("frame_id", "")),
            target=str(data.get("target", "")),
            bbox=data.get("bbox"),
            priority=str(data.get("priority", PRIORITY_REALTIME)),
            extra_instructions=str(data.get("extra_instructions", "")),
            model=str(data.get("model", "")),
            robot_pose=data.get("robot_pose"),
        )


@dataclass
class VLMResponse:
    request_id: str
    ok: bool
    mode: str
    payload: dict[str, Any] = field(default_factory=dict)
    error: str | None = None
    latency_ms: float = 0.0
    frame_id: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "request_id": self.request_id,
            "ok": self.ok,
            "mode": self.mode,
            "payload": self.payload,
            "error": self.error,
            "latency_ms": self.latency_ms,
            "frame_id": self.frame_id,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "VLMResponse":
        return cls(
            request_id=str(data.get("request_id", "")),
            ok=bool(data.get("ok", False)),
            mode=str(data.get("mode", "")),
            payload=dict(data.get("payload") or {}),
            error=data.get("error"),
            latency_ms=float(data.get("latency_ms", 0.0)),
            frame_id=str(data.get("frame_id", "")),
        )


def encode_message(obj: dict[str, Any]) -> bytes:
    return (json.dumps(obj, ensure_ascii=False) + "\n").encode("utf-8")


def read_message(conn: socket.socket, timeout: float = 5.0) -> dict[str, Any] | None:
    conn.settimeout(timeout)
    buffer = bytearray()
    while True:
        try:
            chunk = conn.recv(65536)
        except socket.timeout:
            return None
        except OSError:
            return None
        if not chunk:
            return None
        buffer.extend(chunk)
        if b"\n" in buffer:
            line, _, rest = buffer.partition(b"\n")
            # Keep any extra bytes for the next message (simple one-at-a-time client).
            if rest:
                # We do not buffer across calls in this minimal client.
                pass
            try:
                return json.loads(line.decode("utf-8"))
            except json.JSONDecodeError:
                return None


class SiliconFlowDaemonClient:
    """Minimal stdlib client for the VLM daemon Unix socket."""

    def __init__(self, socket_path: str, timeout: float = 120.0) -> None:
        self.socket_path = str(socket_path)
        self.timeout = timeout

    def available(self) -> bool:
        return Path(self.socket_path).exists()

    def request(self, request: VLMRequest) -> VLMResponse:
        import socket as socket_module

        sock = socket_module.socket(socket_module.AF_UNIX, socket_module.SOCK_STREAM)
        try:
            sock.settimeout(self.timeout)
            sock.connect(self.socket_path)
            sock.sendall(encode_message(request.to_dict()))
            response_line = b""
            while b"\n" not in response_line:
                chunk = sock.recv(65536)
                if not chunk:
                    break
                response_line += chunk
            if not response_line:
                raise ConnectionError("VLM daemon closed connection without response")
            line = response_line.split(b"\n", 1)[0]
            data = json.loads(line.decode("utf-8"))
            return VLMResponse.from_dict(data)
        finally:
            try:
                sock.close()
            except OSError:
                pass
