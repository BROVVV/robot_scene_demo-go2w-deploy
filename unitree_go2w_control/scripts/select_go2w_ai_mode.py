#!/usr/bin/env python3
"""Select the official Go2-W wheeled_sport mode (ai-w), if necessary."""

from __future__ import annotations

import argparse
import json
import time
from typing import Any

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
    MotionSwitcherClient,
)
from unitree_sdk2py.core.channel import ChannelFactoryInitialize


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", required=True)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--settle", type=float, default=4.0)
    return parser.parse_args()


def normalize(result: Any) -> tuple[int, Any]:
    if isinstance(result, tuple) and len(result) >= 2:
        return int(result[0]), result[1]
    return int(result), None


def emit(event: str, raw: Any) -> tuple[int, dict[str, Any]]:
    code, data = normalize(raw)
    record = {
        "event": event,
        "return_code": code,
        "raw_return_repr": repr(raw),
        "robot_form": data.get("form") if isinstance(data, dict) else None,
        "motion_name": data.get("name") if isinstance(data, dict) else None,
    }
    print(json.dumps(record, ensure_ascii=False, sort_keys=True), flush=True)
    return code, data if isinstance(data, dict) else {}


def main() -> int:
    args = parse_args()
    if not 3.0 <= args.settle <= 5.0:
        raise SystemExit("--settle must be between 3 and 5 seconds")

    ChannelFactoryInitialize(0, args.interface)
    client = MotionSwitcherClient()
    client.SetTimeout(args.timeout)
    client.Init()

    code, current = emit("check_before", client.CheckMode())
    if code != 0:
        return 1
    if current.get("name") == "ai-w":
        print('{"event":"already_selected","motion_name":"ai-w"}', flush=True)
        return 0

    select_code, _ = emit("select_ai_w", client.SelectMode("ai-w"))
    if select_code != 0:
        return 1
    time.sleep(args.settle)
    final_code, final = emit("check_after", client.CheckMode())
    if final_code != 0 or final.get("name") != "ai-w":
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
