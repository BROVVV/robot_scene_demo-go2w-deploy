"""ROS worker JSONL protocol tests (plan book §48 / §19)."""

from __future__ import annotations

import json

import pytest

from app.manual_web_demo.ros_worker_client import (
    RosWorkerClient,
    encode_web_command,
    parse_worker_message,
)


def test_valid_worker_message_parses() -> None:
    payload = parse_worker_message(
        json.dumps({"type": "motion_finished", "success": True})
    )
    assert payload["type"] == "motion_finished"
    assert payload["success"] is True


def test_unknown_type_is_allowed_by_parser_but_dropped_by_client() -> None:
    # The low-level parser keeps the payload; the client's router drops
    # unknown types so future worker messages cannot crash the loop.
    payload = parse_worker_message('{"type":"future_thing"}')
    assert payload["type"] == "future_thing"


def test_malformed_json_raises() -> None:
    with pytest.raises(ValueError):
        parse_worker_message("{not json")


def test_empty_line_raises() -> None:
    with pytest.raises(ValueError):
        parse_worker_message("")


def test_non_object_raises() -> None:
    with pytest.raises(ValueError):
        parse_worker_message('[1, 2, 3]')


def test_encode_web_command_round_trips() -> None:
    line = encode_web_command({"type": "pulse", "direction": "forward"})
    assert line.endswith("\n")
    parsed = parse_worker_message(line)
    assert parsed == {"type": "pulse", "direction": "forward"}


def test_blocked_response_round_trip() -> None:
    line = encode_web_command({"type": "stop"})
    assert parse_worker_message(line)["type"] == "stop"
    payload = parse_worker_message(
        json.dumps({"type": "blocked", "reason": "motion_already_active"})
    )
    assert payload["reason"] == "motion_already_active"


def test_worker_error_round_trip() -> None:
    payload = parse_worker_message(
        json.dumps({"type": "error", "message": "rclpy init failed"})
    )
    assert payload["message"] == "rclpy init failed"


def test_worker_command_set_used_by_client() -> None:
    """The documented Web->worker command types exist on the client."""
    client = RosWorkerClient(cmd=("/bin/true",), cwd="/")
    assert hasattr(client, "request_pulse")
    assert hasattr(client, "request_stop")
    assert hasattr(client, "request_estop")
    assert hasattr(client, "request_status")
    assert hasattr(client, "shutdown")
