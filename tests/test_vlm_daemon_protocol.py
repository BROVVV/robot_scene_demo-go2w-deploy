"""Tests for the VLM daemon JSON-line protocol and client fallback surface."""

from __future__ import annotations

import socket

from app.detectors.siliconflow_vision_protocol import (
    SiliconFlowDaemonClient,
    VLMRequest,
    read_message,
)


def test_request_roundtrip():
    request = VLMRequest(
        request_id="abc", mode="quick", image_path="/tmp/x.jpg",
        target="垃圾桶", bbox=[0.1, 0.2, 0.3, 0.4],
    )
    parsed = VLMRequest.from_dict(request.to_dict())
    assert parsed.request_id == "abc"
    assert parsed.mode == "quick"
    assert parsed.bbox == [0.1, 0.2, 0.3, 0.4]


def test_daemon_client_available_false_when_socket_missing(tmp_path):
    client = SiliconFlowDaemonClient(str(tmp_path / "missing.sock"), timeout=0.2)
    assert client.available() is False


def test_read_message_timeout_returns_none():
    left, right = socket.socketpair()
    try:
        result = read_message(right, timeout=0.1)
        assert result is None
    finally:
        left.close()
        right.close()
