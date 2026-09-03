#!/usr/bin/env python3
"""Validation and dispatch for the lease-owning SDK motion executor."""

from __future__ import annotations

import json
import math
from typing import Any


MOVE_API_ID = 1008
STOP_API_ID = 1003
MAX_REQUEST_BYTES = 64 * 1024
MAX_ABS_VX = 0.20
MAX_ABS_VY = 0.10
MAX_ABS_YAW_RATE = 0.25


class ProtocolError(ValueError):
    """A malformed or unsafe local executor request."""


def decode_request(raw: bytes) -> dict[str, Any]:
    if not raw or len(raw) > MAX_REQUEST_BYTES:
        raise ProtocolError("request is empty or too large")
    try:
        value = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError("request is not valid UTF-8 JSON") from exc
    if not isinstance(value, dict):
        raise ProtocolError("request must be a JSON object")
    return value


def _request_id(request: dict[str, Any]) -> int:
    value = request.get("request_id")
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ProtocolError("request_id must be a positive integer")
    return value


def _api_id(request: dict[str, Any]) -> int:
    value = request.get("api_id")
    if isinstance(value, bool) or not isinstance(value, int):
        raise ProtocolError("api_id must be an integer")
    if value not in (MOVE_API_ID, STOP_API_ID):
        raise ProtocolError(f"unsupported api_id: {value}")
    return value


def _parameter(request: dict[str, Any]) -> dict[str, Any]:
    value = request.get("parameter", {})
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except json.JSONDecodeError as exc:
            raise ProtocolError("parameter is not valid JSON") from exc
    if not isinstance(value, dict):
        raise ProtocolError("parameter must be a JSON object")
    return value


def _finite_number(parameter: dict[str, Any], name: str) -> float:
    value = parameter.get(name)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError(f"parameter.{name} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ProtocolError(f"parameter.{name} must be finite")
    return result


def _status_code(raw: Any) -> int:
    if isinstance(raw, tuple) and raw:
        raw = raw[0]
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise ProtocolError(f"SDK returned a non-numeric status: {raw!r}")
    return int(raw)


def execute_request(
    request: dict[str, Any], client: Any, lease_id: int
) -> dict[str, Any]:
    """Validate one request and execute it through the lease-owning client."""

    request_id = _request_id(request)
    api_id = _api_id(request)
    parameter = _parameter(request)
    if lease_id <= 0:
        return {
            "request_id": request_id,
            "api_id": api_id,
            "lease_id": 0,
            "status_code": -9998,
            "data": {"error": "lease unavailable"},
        }

    if api_id == MOVE_API_ID:
        vx = _finite_number(parameter, "x")
        vy = _finite_number(parameter, "y")
        yaw_rate = _finite_number(parameter, "z")
        if abs(vx) > MAX_ABS_VX:
            raise ProtocolError(f"abs(vx) exceeds {MAX_ABS_VX}")
        if abs(vy) > MAX_ABS_VY:
            raise ProtocolError(f"abs(vy) exceeds {MAX_ABS_VY}")
        if abs(yaw_rate) > MAX_ABS_YAW_RATE:
            raise ProtocolError(f"abs(yaw_rate) exceeds {MAX_ABS_YAW_RATE}")
        raw_status = client.Move(vx, vy, yaw_rate)
    else:
        raw_status = client.StopMove()

    return {
        "request_id": request_id,
        "api_id": api_id,
        "lease_id": int(lease_id),
        "status_code": _status_code(raw_status),
        "data": {"sdk_return": repr(raw_status), "transport": "sdk_direct"},
    }
