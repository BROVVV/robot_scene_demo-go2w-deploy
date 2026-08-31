#!/usr/bin/env python3
"""Measure read-only Go2-W wheel encoder noise while no command is active."""

from __future__ import annotations

import argparse
import json
import threading
import time

import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", required=True)
    parser.add_argument("--duration", type=float, default=8.0)
    args = parser.parse_args()
    if not 2.0 <= args.duration <= 30.0:
        raise SystemExit("duration must be between 2 and 30 seconds")

    ChannelFactoryInitialize(0, args.interface)
    lock = threading.Lock()
    samples: list[tuple[float, list[float], list[float]]] = []
    sport_state: dict[str, object] = {}

    def on_low_state(msg: LowState_) -> None:
        with lock:
            samples.append(
                (
                    time.monotonic(),
                    [float(msg.motor_state[index].q) for index in range(12, 16)],
                    [float(msg.motor_state[index].dq) for index in range(12, 16)],
                )
            )

    def on_sport_state(msg: SportModeState_) -> None:
        with lock:
            sport_state.update(
                {
                    "error_code": int(msg.error_code),
                    "mode": int(msg.mode),
                    "velocity": [float(value) for value in msg.velocity],
                    "yaw_speed": float(msg.yaw_speed),
                    "range_obstacle": [float(value) for value in msg.range_obstacle],
                }
            )

    low_subscriber = ChannelSubscriber("rt/lf/lowstate", LowState_)
    low_subscriber.Init(on_low_state, 1000)
    sport_subscriber = ChannelSubscriber("rt/lf/sportmodestate", SportModeState_)
    sport_subscriber.Init(on_sport_state, 100)
    time.sleep(args.duration)
    low_subscriber.Close()
    sport_subscriber.Close()

    with lock:
        captured = list(samples)
        state = dict(sport_state)
    if len(captured) < 20:
        print(json.dumps({"event": "error", "sample_count": len(captured)}))
        return 1

    q = np.asarray([sample[1] for sample in captured], dtype=float)
    dq = np.asarray([sample[2] for sample in captured], dtype=float)
    wheels = []
    for index in range(4):
        abs_dq = np.abs(dq[:, index])
        wheels.append(
            {
                "motor_index": index + 12,
                "q_start": float(q[0, index]),
                "q_end": float(q[-1, index]),
                "q_net_delta": float(q[-1, index] - q[0, index]),
                "q_peak_to_peak": float(np.ptp(q[:, index])),
                "dq_max_abs": float(np.max(abs_dq)),
                "dq_p95_abs": float(np.percentile(abs_dq, 95)),
                "dq_rms": float(np.sqrt(np.mean(np.square(dq[:, index])))),
            }
        )
    print(
        json.dumps(
            {
                "event": "stationary_wheel_baseline",
                "duration": args.duration,
                "sample_count": len(captured),
                "sport_state": state,
                "wheels": wheels,
            },
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
