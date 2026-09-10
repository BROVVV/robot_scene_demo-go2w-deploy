#!/usr/bin/env python3
"""Read-only Unitree videohub worker with a binary stdout frame protocol.

This process imports only communication and video client modules. It never
constructs SportClient, MotionSwitcherClient, or any command publisher.
"""

from __future__ import annotations

import argparse
import os
import signal
import struct
import sys
import time
from pathlib import Path


project_root = Path(__file__).resolve().parents[2]
control_root = Path(
    os.environ.get("GO2W_CONTROL_ROOT", str(project_root / "unitree_go2w_control"))
)
VENDOR_ROOT = Path(
    os.environ.get(
        "GO2W_UNITREE_SDK_ROOT",
        str(control_root / "vendor" / "unitree_sdk2_python"),
    )
)
sys.path.insert(0, str(VENDOR_ROOT))

from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # noqa: E402
from unitree_sdk2py.go2.video.video_client import VideoClient  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", required=True)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--max-fps", type=float, default=30.0)
    args = parser.parse_args()
    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    ChannelFactoryInitialize(0, args.interface)
    client = VideoClient()
    client.SetTimeout(args.timeout)
    client.Init()
    period = 1.0 / max(0.1, args.max_fps)
    output = sys.stdout.buffer
    while not stopping:
        cycle = time.monotonic()
        rpc_start_ns = time.time_ns()
        code, data = client.GetImageSample()
        rpc_end_ns = time.time_ns()
        if int(code) == 0 and data:
            payload = bytes(data)
            if len(payload) <= 20_000_000:
                output.write(
                    struct.pack(
                        "<QQI", rpc_start_ns, rpc_end_ns, len(payload)
                    )
                )
                output.write(payload)
                output.flush()
        remaining = period - (time.monotonic() - cycle)
        if remaining > 0:
            time.sleep(remaining)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
