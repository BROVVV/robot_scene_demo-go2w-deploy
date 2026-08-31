#!/usr/bin/env python3
"""Send three bounded high-level StopMove requests through Unitree SDK2."""

from __future__ import annotations

import argparse
import json
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", required=True)
    args = parser.parse_args()
    ChannelFactoryInitialize(0, args.interface)
    client = SportClient()
    client.SetTimeout(3.0)
    client.Init()
    codes: list[int] = []
    for attempt in range(1, 4):
        try:
            raw = client.StopMove()
            code = int(raw)
            codes.append(code)
            print(
                json.dumps(
                    {
                        "event": "stop_return",
                        "attempt": attempt,
                        "api_id": 1003,
                        "raw_return_repr": repr(raw),
                    }
                ),
                flush=True,
            )
        except Exception as exc:
            codes.append(-9999)
            print(
                json.dumps(
                    {
                        "event": "stop_error",
                        "attempt": attempt,
                        "message": str(exc),
                    }
                ),
                flush=True,
            )
        time.sleep(0.1)
    return 0 if codes == [0, 0, 0] else 1


if __name__ == "__main__":
    raise SystemExit(main())
