#!/usr/bin/env python3
"""Independent StandDown/StandUp capability test; never gates Move testing."""

from __future__ import annotations

import argparse
import json
import signal
import sys
import threading
import time
from typing import Any

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
    MotionSwitcherClient,
)
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_


CONFIRMATION = "I_HAVE_CLEARED_THE_AREA"


class State:
    def __init__(self) -> None:
        self.mode: int | None = None
        self.velocity = [0.0, 0.0, 0.0]
        self.yaw_speed = 0.0
        self.lock = threading.Lock()

    def update(self, msg: SportModeState_) -> None:
        with self.lock:
            self.mode = int(msg.mode)
            self.velocity = [float(value) for value in msg.velocity]
            self.yaw_speed = float(msg.yaw_speed)

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "mode": self.mode,
                "velocity": list(self.velocity),
                "yaw_speed": self.yaw_speed,
            }


def emit(event: str, **fields: Any) -> None:
    print(json.dumps({"event": event, **fields}), flush=True)


def stationary(snapshot: dict[str, Any]) -> bool:
    velocity = snapshot["velocity"]
    return (
        abs(float(velocity[0])) < 0.02
        and abs(float(velocity[1])) < 0.02
        and abs(float(snapshot["yaw_speed"])) < 0.03
    )


def wait_mode(
    state: State,
    expected: int,
    timeout: float,
    interrupted: threading.Event | None = None,
) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if interrupted is not None and interrupted.is_set():
            raise InterruptedError("posture test interrupted")
        if state.snapshot()["mode"] == expected:
            return True
        time.sleep(0.1)
    return False


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", required=True)
    lease_group = parser.add_mutually_exclusive_group()
    lease_group.add_argument(
        "--enable-lease", dest="enable_lease", action="store_true"
    )
    lease_group.add_argument(
        "--disable-lease", dest="enable_lease", action="store_false"
    )
    parser.set_defaults(enable_lease=True)
    args = parser.parse_args()
    if not sys.stdin.isatty():
        raise SystemExit("safety confirmation requires an interactive terminal")

    interrupted = threading.Event()

    def on_signal(signum: int, _frame: object) -> None:
        interrupted.set()
        raise InterruptedError(f"received signal {signum}")

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    ChannelFactoryInitialize(0, args.interface)
    switcher = MotionSwitcherClient()
    switcher.SetTimeout(5.0)
    switcher.Init()
    mode_raw = switcher.CheckMode()
    emit("motion_mode", raw_return_repr=repr(mode_raw))
    if not (
        isinstance(mode_raw, tuple)
        and len(mode_raw) >= 2
        and mode_raw[0] == 0
        and isinstance(mode_raw[1], dict)
        and mode_raw[1].get("name") == "ai-w"
    ):
        raise SystemExit("MotionSwitcher is not in ai-w")

    state = State()
    subscriber = ChannelSubscriber("rt/lf/sportmodestate", SportModeState_)
    subscriber.Init(state.update, 20)
    if not wait_mode(state, 1, 5.0):
        subscriber.Close()
        raise SystemExit("initial balanceStand mode 1 was not observed")
    emit("state_before", **state.snapshot())
    confirmation = input(
        "平整地面和周围 2 米已清空，遥控器可立即急停。\n"
        f"Type {CONFIRMATION}: "
    ).strip()
    if confirmation != CONFIRMATION:
        subscriber.Close()
        raise SystemExit("safety confirmation rejected")

    sport = SportClient(enableLease=args.enable_lease)
    sport.SetTimeout(5.0)
    sport.Init()
    down_accepted = False
    up_reached = False
    stop_returns: list[int | None] = []
    result = 1
    try:
        if args.enable_lease:
            lease_deadline = time.monotonic() + 5.0
            while sport.GetLeaseId() == 0 and time.monotonic() < lease_deadline:
                if interrupted.is_set():
                    raise InterruptedError("posture test interrupted")
                time.sleep(0.05)
            lease_id = int(sport.GetLeaseId())
            emit("lease", enabled=True, lease_id=lease_id)
            if lease_id == 0:
                raise RuntimeError("sport lease was not acquired within 5 seconds")
        else:
            emit("lease", enabled=False, lease_id=None)
        down_raw = sport.StandDown()
        emit("stand_down_return", api_id=1005, raw_return_repr=repr(down_raw))
        if down_raw != 0:
            raise RuntimeError(f"StandDown rejected: {down_raw!r}")
        down_accepted = True
        if not wait_mode(state, 5, 15.0, interrupted):
            raise RuntimeError(f"lie-down mode 5 not reached: {state.snapshot()!r}")
        down_state = state.snapshot()
        emit("state_down", **down_state)
        if not stationary(down_state):
            raise RuntimeError(f"robot is not stationary after StandDown: {down_state!r}")
        time.sleep(2.0)
        up_raw = sport.StandUp()
        emit("stand_up_return", api_id=1004, raw_return_repr=repr(up_raw))
        if up_raw != 0:
            raise RuntimeError(f"StandUp rejected: {up_raw!r}")
        if not wait_mode(state, 1, 15.0, interrupted):
            raise RuntimeError(f"stand mode 1 not reached: {state.snapshot()!r}")
        up_reached = True
        up_state = state.snapshot()
        emit("state_up", **up_state)
        if not stationary(up_state):
            raise RuntimeError(f"robot is not stationary after StandUp: {up_state!r}")
        result = 0
    except Exception as exc:
        emit("error", error_type=type(exc).__name__, message=str(exc))
    finally:
        if down_accepted and not up_reached:
            try:
                raw = sport.StandUp()
                emit("safety_stand_up_return", api_id=1004, raw_return_repr=repr(raw))
                if raw != 0 or not wait_mode(state, 1, 15.0):
                    raise RuntimeError("safety StandUp did not restore mode 1")
                up_reached = True
            except Exception as exc:
                emit("safety_stand_up_error", message=str(exc))
        for attempt in range(1, 4):
            try:
                raw = sport.StopMove()
                stop_returns.append(int(raw))
                emit(
                    "stop_return",
                    attempt=attempt,
                    api_id=1003,
                    raw_return_repr=repr(raw),
                )
            except Exception as exc:
                stop_returns.append(None)
                emit("stop_error", attempt=attempt, message=str(exc))
            time.sleep(0.1)
        subscriber.Close()
    final_state = state.snapshot()
    stops_ok = stop_returns == [0, 0, 0]
    final_ok = final_state["mode"] == 1 and stationary(final_state)
    emit(
        "verification",
        stand_down_accepted=down_accepted,
        stand_up_reached=up_reached,
        stop_returns=stop_returns,
        stop_returns_ok=stops_ok,
        final_state=final_state,
        final_state_ok=final_ok,
    )
    if not stops_ok or not final_ok:
        result = 1
    emit("result", posture_test="PASS" if result == 0 else "FAIL")
    return result


if __name__ == "__main__":
    raise SystemExit(main())
