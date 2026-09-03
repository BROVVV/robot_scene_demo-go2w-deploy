#!/usr/bin/env python3
"""Read the Unitree MotionSwitcher mode without changing robot state."""

from __future__ import annotations

import argparse
import inspect
import json
from typing import Any

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
    MotionSwitcherClient,
)
from unitree_sdk2py.core.channel import ChannelFactoryInitialize


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", required=True)
    parser.add_argument("--timeout", type=float, default=5.0)
    return parser.parse_args()


def normalize(result: Any) -> tuple[int, Any]:
    if isinstance(result, tuple) and len(result) >= 2:
        return int(result[0]), result[1]
    return int(result), None


def main() -> int:
    args = parse_args()
    ChannelFactoryInitialize(0, args.interface)
    client = MotionSwitcherClient()
    client.SetTimeout(args.timeout)
    client.Init()

    raw = client.CheckMode()
    code, data = normalize(raw)
    data = data if isinstance(data, dict) else {}
    output = {
        "check_mode_return_code": code,
        "robot_form": data.get("form"),
        "motion_name": data.get("name"),
        "raw_return_repr": repr(raw),
        "check_mode_signature": str(inspect.signature(client.CheckMode)),
        "release_mode_called": False,
    }
    print(json.dumps(output, ensure_ascii=False, sort_keys=True))
    return 0 if code == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
