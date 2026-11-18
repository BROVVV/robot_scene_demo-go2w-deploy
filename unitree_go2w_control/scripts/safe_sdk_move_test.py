#!/usr/bin/env python3
"""Bounded Go2-W movement test using the official Unitree SDK2 Python API."""

from __future__ import annotations

import argparse
import json
import signal
import sys
import threading
import time
from dataclasses import asdict, dataclass
from typing import Any

import numpy as np

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
    MotionSwitcherClient,
)
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_


STATE_TOPIC = "rt/lf/sportmodestate"
LOW_STATE_TOPIC = "rt/lf/lowstate"
SAFETY_CONFIRMATION = "I_HAVE_CLEARED_THE_AREA"


@dataclass(frozen=True)
class StateSample:
    monotonic_time: float
    error_code: int
    mode: int
    position: list[float]
    velocity: list[float]
    yaw_speed: float
    range_obstacle: list[float]


class StateRecorder:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._samples: list[StateSample] = []

    def callback(self, msg: SportModeState_) -> None:
        sample = StateSample(
            monotonic_time=time.monotonic(),
            error_code=int(msg.error_code),
            mode=int(msg.mode),
            position=[float(value) for value in msg.position],
            velocity=[float(value) for value in msg.velocity],
            yaw_speed=float(msg.yaw_speed),
            range_obstacle=[float(value) for value in msg.range_obstacle],
        )
        with self._lock:
            self._samples.append(sample)

    def latest(self) -> StateSample | None:
        with self._lock:
            return self._samples[-1] if self._samples else None

    def since(self, start: float) -> list[StateSample]:
        with self._lock:
            return [sample for sample in self._samples if sample.monotonic_time >= start]


@dataclass(frozen=True)
class WheelSample:
    monotonic_time: float
    q: list[float]
    dq: list[float]


class WheelRecorder:
    """Read-only wheel encoder recorder for Go2-W motor indices 12..15."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._samples: list[WheelSample] = []

    def callback(self, msg: LowState_) -> None:
        sample = WheelSample(
            monotonic_time=time.monotonic(),
            q=[float(msg.motor_state[index].q) for index in range(12, 16)],
            dq=[float(msg.motor_state[index].dq) for index in range(12, 16)],
        )
        with self._lock:
            self._samples.append(sample)

    def latest(self) -> WheelSample | None:
        with self._lock:
            return self._samples[-1] if self._samples else None

    def since(self, start: float) -> list[WheelSample]:
        with self._lock:
            return [sample for sample in self._samples if sample.monotonic_time >= start]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", required=True)
    parser.add_argument("--vx", type=float, default=0.05)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--vyaw", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=0.5)
    lease_group = parser.add_mutually_exclusive_group()
    lease_group.add_argument(
        "--enable-lease", dest="enable_lease", action="store_true"
    )
    lease_group.add_argument(
        "--disable-lease", dest="enable_lease", action="store_false"
    )
    parser.set_defaults(enable_lease=True)
    return parser.parse_args()


def emit(event: str, **fields: Any) -> None:
    print(json.dumps({"event": event, **fields}, ensure_ascii=False), flush=True)


def validate_limits(args: argparse.Namespace) -> None:
    if abs(args.vx) > 0.05:
        raise ValueError("abs(vx) must be <= 0.05 m/s")
    if args.vy != 0.0:
        raise ValueError("vy must be exactly 0 for the first recovery test")
    if abs(args.vyaw) > 0.08:
        raise ValueError("abs(vyaw) must be <= 0.08 rad/s")
    if not 0.05 <= args.duration <= 0.6:
        raise ValueError("duration must be between 0.05 and 0.6 seconds")
    if args.vx == 0.0 and args.vyaw == 0.0:
        raise ValueError("at least one of vx or vyaw must be non-zero")


def check_ai_w() -> tuple[int, dict[str, Any], Any]:
    client = MotionSwitcherClient()
    client.SetTimeout(5.0)
    client.Init()
    raw = client.CheckMode()
    if isinstance(raw, tuple) and len(raw) >= 2:
        code, data = int(raw[0]), raw[1]
    else:
        code, data = int(raw), None
    data = data if isinstance(data, dict) else {}
    emit(
        "motion_mode",
        return_code=code,
        robot_form=data.get("form"),
        motion_name=data.get("name"),
        raw_return_repr=repr(raw),
    )
    return code, data, raw


def wait_for_state(recorder: StateRecorder, timeout: float) -> StateSample:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        sample = recorder.latest()
        if sample is not None:
            return sample
        time.sleep(0.05)
    raise RuntimeError(f"no {STATE_TOPIC} sample within {timeout:.1f}s")


def stationary(sample: StateSample) -> bool:
    return (
        abs(sample.velocity[0]) < 0.02
        and abs(sample.velocity[1]) < 0.02
        and abs(sample.yaw_speed) < 0.03
    )


def main() -> int:
    args = parse_args()
    validate_limits(args)
    interrupted = threading.Event()

    def on_signal(signum: int, _frame: Any) -> None:
        emit("signal", signum=signum)
        interrupted.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    ChannelFactoryInitialize(0, args.interface)
    recorder = StateRecorder()
    subscriber = ChannelSubscriber(STATE_TOPIC, SportModeState_)
    subscriber.Init(recorder.callback, 100)
    wheel_recorder = WheelRecorder()
    wheel_subscriber = ChannelSubscriber(LOW_STATE_TOPIC, LowState_)
    wheel_subscriber.Init(wheel_recorder.callback, 200)

    sport: SportClient | None = None
    move_started = 0.0
    move_return: Any = None
    stop_returns: list[Any] = []
    result = 1
    try:
        code, mode, _ = check_ai_w()
        if code != 0 or mode.get("name") != "ai-w":
            raise RuntimeError("MotionSwitcher is not in ai-w; movement refused")

        sport = SportClient(enableLease=args.enable_lease)
        sport.SetTimeout(5.0)
        sport.Init()
        if args.enable_lease:
            lease_deadline = time.monotonic() + 5.0
            while sport.GetLeaseId() == 0 and time.monotonic() < lease_deadline:
                time.sleep(0.05)
            lease_id = sport.GetLeaseId()
            emit("lease", enabled=True, lease_id=lease_id)
            if lease_id == 0:
                raise RuntimeError("sport lease was not acquired within 5 seconds")
        else:
            emit("lease", enabled=False, lease_id=None)

        initial = wait_for_state(recorder, 5.0)
        emit("state_before", **asdict(initial))
        wheel_deadline = time.monotonic() + 5.0
        while wheel_recorder.latest() is None and time.monotonic() < wheel_deadline:
            time.sleep(0.05)
        initial_wheels = wheel_recorder.latest()
        if initial_wheels is None:
            raise RuntimeError(f"no {LOW_STATE_TOPIC} wheel encoder sample")
        emit("wheels_before", **asdict(initial_wheels))
        if not stationary(initial):
            raise RuntimeError("robot is not stationary before movement")

        if not sys.stdin.isatty():
            raise RuntimeError("safety confirmation requires an interactive terminal")
        confirmation = input(
            "平整地面和周围 2 米已清空，遥控器可立即急停。\n"
            f"Type {SAFETY_CONFIRMATION}: "
        ).strip()
        if confirmation != SAFETY_CONFIRMATION:
            raise RuntimeError("safety confirmation rejected")

        emit(
            "move_request",
            api_id=1008,
            parameter={"x": args.vx, "y": args.vy, "z": args.vyaw},
            duration=args.duration,
            enable_lease=args.enable_lease,
        )
        move_started = time.monotonic()
        move_return = sport.Move(args.vx, args.vy, args.vyaw)
        emit("move_return", raw_return_repr=repr(move_return))

        deadline = move_started + args.duration
        while time.monotonic() < deadline and not interrupted.is_set():
            time.sleep(min(0.02, max(0.0, deadline - time.monotonic())))
    except Exception as exc:
        emit("error", error_type=type(exc).__name__, message=str(exc))
    finally:
        if sport is not None:
            for attempt in range(1, 4):
                try:
                    raw = sport.StopMove()
                    stop_returns.append(raw)
                    emit(
                        "stop_return",
                        attempt=attempt,
                        api_id=1003,
                        raw_return_repr=repr(raw),
                    )
                except Exception as exc:
                    stop_returns.append(exc)
                    emit(
                        "stop_error",
                        attempt=attempt,
                        error_type=type(exc).__name__,
                        message=str(exc),
                    )
                time.sleep(0.1)

        if move_started:
            time.sleep(2.0)
            movement_samples = recorder.since(move_started)
            wheel_samples = wheel_recorder.since(move_started)
            latest = recorder.latest()
            latest_wheels = wheel_recorder.latest()
            peak_vx = max((abs(sample.velocity[0]) for sample in movement_samples), default=0.0)
            peak_yaw = max((abs(sample.yaw_speed) for sample in movement_samples), default=0.0)
            modes = sorted({sample.mode for sample in movement_samples})
            position_delta = [0.0, 0.0, 0.0]
            if latest is not None:
                position_delta = [
                    latest.position[index] - initial.position[index] for index in range(3)
                ]
                emit("state_after", **asdict(latest))
            moved = 3 in modes or peak_vx > 0.02 or peak_yaw > 0.03
            peak_wheel_dq = max(
                (abs(value) for sample in wheel_samples for value in sample.dq),
                default=0.0,
            )
            wheel_q_values = np.asarray(
                [sample.q for sample in wheel_samples], dtype=float
            )
            wheel_dq_values = np.asarray(
                [sample.dq for sample in wheel_samples], dtype=float
            )
            wheel_q_peak_to_peak = (
                np.ptp(wheel_q_values, axis=0).tolist()
                if len(wheel_samples) > 1
                else [0.0, 0.0, 0.0, 0.0]
            )
            wheel_dq_p95_abs = (
                np.percentile(np.abs(wheel_dq_values), 95, axis=0).tolist()
                if len(wheel_samples) > 1
                else [0.0, 0.0, 0.0, 0.0]
            )
            wheel_q_delta = [0.0, 0.0, 0.0, 0.0]
            if latest_wheels is not None:
                wheel_q_delta = [
                    latest_wheels.q[index] - initial_wheels.q[index]
                    for index in range(4)
                ]
                emit("wheels_after", **asdict(latest_wheels))
            wheel_motion = (
                sum(value > 0.03 for value in wheel_q_peak_to_peak) >= 3
                and sum(value > 0.12 for value in wheel_dq_p95_abs) >= 3
            )
            wheels_stopped = latest_wheels is not None and max(
                abs(value) for value in latest_wheels.dq
            ) < 0.1
            stopped = latest is not None and stationary(latest)
            move_ok = move_return == 0
            stops_ok = len(stop_returns) == 3 and all(value == 0 for value in stop_returns)
            emit(
                "verification",
                move_return_ok=move_ok,
                motion_state_changed=moved,
                wheel_motion_observed=wheel_motion,
                peak_abs_wheel_dq=peak_wheel_dq,
                wheel_dq_p95_abs=wheel_dq_p95_abs,
                wheel_q_peak_to_peak=wheel_q_peak_to_peak,
                wheel_q_delta=wheel_q_delta,
                wheels_stopped_after_stop=wheels_stopped,
                peak_abs_vx=peak_vx,
                peak_abs_yaw_speed=peak_yaw,
                modes=modes,
                position_delta=position_delta,
                stop_returns_ok=stops_ok,
                stationary_after_stop=stopped,
                sample_count=len(movement_samples),
                wheel_sample_count=len(wheel_samples),
            )
            result = (
                0
                if move_ok
                and moved
                and wheel_motion
                and stops_ok
                and stopped
                and wheels_stopped
                else 1
            )

        subscriber.Close()
        wheel_subscriber.Close()

    emit("result", sdk_move_test="PASS" if result == 0 else "FAIL")
    return result


if __name__ == "__main__":
    raise SystemExit(main())
