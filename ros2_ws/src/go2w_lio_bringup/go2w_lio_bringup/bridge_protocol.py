"""Small, allow-listed TCP protocol for the isolated ROS 1 Point-LIO process."""

from __future__ import annotations

import json
import socket
import struct
from typing import Any


PROTOCOL_VERSION = 1
MAX_HEADER_BYTES = 64 * 1024
MAX_PAYLOAD_BYTES = 64 * 1024 * 1024
_LENGTHS = struct.Struct("!II")


class ProtocolError(RuntimeError):
    pass


def _receive_exact(connection: socket.socket, length: int) -> bytes:
    chunks: list[bytes] = []
    remaining = length
    while remaining:
        chunk = connection.recv(remaining)
        if not chunk:
            raise EOFError("Point-LIO bridge peer disconnected")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def send_frame(
    connection: socket.socket,
    metadata: dict[str, Any],
    payload: bytes = b"",
) -> None:
    document = dict(metadata)
    document["protocol_version"] = PROTOCOL_VERSION
    header = json.dumps(
        document, allow_nan=False, separators=(",", ":"), sort_keys=True
    ).encode("utf-8")
    binary = bytes(payload)
    if not header or len(header) > MAX_HEADER_BYTES:
        raise ProtocolError("invalid bridge header size")
    if len(binary) > MAX_PAYLOAD_BYTES:
        raise ProtocolError("bridge payload exceeds safety limit")
    connection.sendall(_LENGTHS.pack(len(header), len(binary)) + header + binary)


def receive_frame(connection: socket.socket) -> tuple[dict[str, Any], bytes]:
    header_length, payload_length = _LENGTHS.unpack(
        _receive_exact(connection, _LENGTHS.size)
    )
    if not 0 < header_length <= MAX_HEADER_BYTES:
        raise ProtocolError("invalid bridge header size")
    if payload_length > MAX_PAYLOAD_BYTES:
        raise ProtocolError("bridge payload exceeds safety limit")
    try:
        metadata = json.loads(_receive_exact(connection, header_length))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError("invalid bridge JSON header") from exc
    if not isinstance(metadata, dict):
        raise ProtocolError("bridge header must be an object")
    if metadata.get("protocol_version") != PROTOCOL_VERSION:
        raise ProtocolError("unsupported Point-LIO bridge protocol version")
    return metadata, _receive_exact(connection, payload_length)


def require_message_type(metadata: dict[str, Any], allowed: set[str]) -> str:
    message_type = metadata.get("type")
    if not isinstance(message_type, str) or message_type not in allowed:
        raise ProtocolError("message type is not on the read-only allow list")
    return message_type
