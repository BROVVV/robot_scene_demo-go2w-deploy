from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from sdk_motion_protocol import ProtocolError, decode_request, execute_request  # noqa: E402


class FakeSportClient:
    def __init__(self) -> None:
        self.calls: list[tuple[object, ...]] = []

    def Move(self, vx: float, vy: float, yaw_rate: float) -> int:
        self.calls.append(("move", vx, vy, yaw_rate))
        return 0

    def StopMove(self) -> int:
        self.calls.append(("stop",))
        return 0


def test_move_uses_the_lease_owning_client() -> None:
    client = FakeSportClient()
    response = execute_request(
        {
            "request_id": 7,
            "api_id": 1008,
            "parameter": {"x": 0.05, "y": 0.0, "z": -0.08},
        },
        client,
        lease_id=42,
    )

    assert response["status_code"] == 0
    assert response["lease_id"] == 42
    assert response["data"]["transport"] == "sdk_direct"
    assert client.calls == [("move", 0.05, 0.0, -0.08)]


def test_stop_uses_the_same_client() -> None:
    client = FakeSportClient()
    response = execute_request(
        {"request_id": 8, "api_id": 1003, "parameter": {}},
        client,
        lease_id=42,
    )

    assert response["status_code"] == 0
    assert client.calls == [("stop",)]


@pytest.mark.parametrize(
    "parameter",
    [
        {"x": 0.201, "y": 0.0, "z": 0.0},
        {"x": 0.0, "y": -0.101, "z": 0.0},
        {"x": 0.0, "y": 0.0, "z": 0.251},
        {"x": float("nan"), "y": 0.0, "z": 0.0},
    ],
)
def test_executor_rejects_unsafe_move(parameter: dict[str, float]) -> None:
    with pytest.raises(ProtocolError):
        execute_request(
            {"request_id": 9, "api_id": 1008, "parameter": parameter},
            FakeSportClient(),
            lease_id=42,
        )


def test_missing_lease_never_calls_sdk() -> None:
    client = FakeSportClient()
    response = execute_request(
        {
            "request_id": 10,
            "api_id": 1008,
            "parameter": {"x": 0.05, "y": 0.0, "z": 0.0},
        },
        client,
        lease_id=0,
    )

    assert response["status_code"] == -9998
    assert client.calls == []


def test_decode_request_requires_an_object() -> None:
    assert decode_request(json.dumps({"request_id": 1}).encode()) == {
        "request_id": 1
    }
    with pytest.raises(ProtocolError):
        decode_request(b"[]")
