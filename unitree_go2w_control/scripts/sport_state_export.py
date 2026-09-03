#!/usr/bin/env python3
"""SDK -> JSON 状态导出：订阅狗主控 /lf/sportmodestate 与 /lf/lowstate。

独立进程（不依赖 rclpy）：用 python cyclonedds 0.10.2 + libddsc 0.10.2
多播直连狗主控 DDS 域，把状态原子写入 /tmp/go2w_sport_state.json 与
/tmp/go2w_low_state.json，供 ROS 层桥（sport_state_ros_bridge.py）转发。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import asdict
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "vendor", "unitree_sdk2_python"))

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_


def _atomic_write(path: Path, payload: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, default=str), encoding="utf-8"
    )
    os.replace(temporary, path)


def _as_json(msg) -> dict:
    try:
        return asdict(msg)
    except Exception:
        return {}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="eth0")
    parser.add_argument("--sport-out", default="/tmp/go2w_sport_state.json")
    parser.add_argument("--low-out", default="/tmp/go2w_low_state.json")
    args = parser.parse_args()

    ChannelFactoryInitialize(0, args.interface)
    sport_out = Path(args.sport_out)
    low_out = Path(args.low_out)
    counters = {"sport": 0, "low": 0}

    def on_sport(msg) -> None:
        counters["sport"] += 1
        _atomic_write(sport_out, {"stamp": time.time(), "msg": _as_json(msg)})

    def on_low(msg) -> None:
        counters["low"] += 1
        _atomic_write(low_out, {"stamp": time.time(), "msg": _as_json(msg)})

    sport_sub = ChannelSubscriber("lf/sportmodestate", SportModeState_)
    sport_sub.Init(on_sport, 10)
    low_sub = ChannelSubscriber("lf/lowstate", LowState_)
    low_sub.Init(on_low, 10)
    print("sport_state_export: subscribed /lf/sportmodestate + /lf/lowstate", flush=True)
    while True:
        time.sleep(5)
        print("counters=", counters, "sport_age=", _age(sport_out), "low_age=", _age(low_out), flush=True)


def _age(path: Path) -> float:
    try:
        return round(time.time() - path.stat().st_mtime, 2)
    except OSError:
        return -1.0


if __name__ == "__main__":
    raise SystemExit(main())
