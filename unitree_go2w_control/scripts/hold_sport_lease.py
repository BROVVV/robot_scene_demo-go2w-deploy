#!/usr/bin/env python3
"""Own the Unitree Sport lease and execute bounded commands through that client."""

from __future__ import annotations

import argparse
import json
import os
import signal
import threading
import time
from pathlib import Path

from sdk_motion_executor import SdkMotionExecutor, executor_timeouts
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
    MotionSwitcherClient,
)
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient


def _write_status(path: Path, value: str) -> None:
    temporary = path.with_name(f".{path.name}.tmp.{os.getpid()}")
    temporary.write_text(value, encoding="ascii")
    os.replace(temporary, path)

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", required=True)
    parser.add_argument("--ready-file")
    parser.add_argument('--command-timeout-seconds', type=float,
                        default=float(os.environ.get('GO2W_SDK_COMMAND_TIMEOUT_SECONDS', '0')))
    parser.add_argument('--rpc-timeout-seconds', type=float,
                        default=float(os.environ.get('GO2W_SDK_RPC_TIMEOUT_SECONDS', '5')))
    parser.add_argument(
        "--socket-path",
        default=os.environ.get(
            "GO2W_SDK_COMMAND_SOCKET", "/tmp/go2w_sdk_motion.sock"
        ),
    )
    parser.add_argument(
        "--status-dir",
        default=os.environ.get("GO2W_LEASE_STATUS_DIR", "/tmp/go2w_lease_status"),
    )
    # Accepted for compatibility with old launch files; ROS runs in the bridge.
    parser.add_argument("--ros-status", action="store_true")
    args = parser.parse_args()
    try:
        command_timeout, rpc_timeout = executor_timeouts(
            args.command_timeout_seconds, args.rpc_timeout_seconds)
    except ValueError as error:
        parser.error(str(error))

    stop_event = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    sdk_network = os.environ.get("GO2W_ROBOT_HOST_IP", args.interface)
    ChannelFactoryInitialize(0, sdk_network)
    client = SportClient(enableLease=True)
    client.SetTimeout(5.0)
    client.Init()
    deadline = time.monotonic() + 5.0
    while client.GetLeaseId() == 0 and time.monotonic() < deadline:
        time.sleep(0.05)
    lease_id = int(client.GetLeaseId())
    if lease_id == 0:
        print('{"event":"lease_error","message":"acquisition timeout"}', flush=True)
        return 1
    # Acquisition keeps its original timeout. Command calls must finish before
    # an enabled watchdog's deadline so it can acquire the shared SDK lock.
    client.SetTimeout(rpc_timeout)

    status_dir = Path(args.status_dir)
    status_dir.mkdir(parents=True, exist_ok=True)
    os.chmod(status_dir, 0o700)
    id_file = status_dir / "id"
    alive_file = status_dir / "alive"
    heartbeat_file = status_dir / "heartbeat"
    motion_name_file = status_dir / "motion_name"
    robot_form_file = status_dir / "robot_form"
    _write_status(id_file, f"{lease_id}\n")
    _write_status(alive_file, "1\n")
    _write_status(heartbeat_file, f"{time.time():.6f}\n")
    _write_status(motion_name_file, "")
    _write_status(robot_form_file, "")

    if args.ready_file:
        ready_path = Path(args.ready_file)
        _write_status(ready_path, f"{lease_id}\n")
        os.chmod(ready_path, 0o600)
    print(
        json.dumps(
            {
                "event": "lease_ready",
                "lease_id": lease_id,
                "transport": "sdk_direct",
            }
        ),
        flush=True,
    )

    client_lock = threading.Lock()
    executor = SdkMotionExecutor(
        client, client_lock, Path(args.socket_path), stop_event,
        command_timeout_seconds=command_timeout,
    )
    executor_thread = threading.Thread(
        target=executor.serve, name="sdk-motion-executor", daemon=True
    )
    executor_thread.start()

    switcher = MotionSwitcherClient()
    switcher.SetTimeout(1.0)
    switcher.Init()
    next_mode_check = 0.0
    try:
        while not stop_event.wait(0.2):
            if not executor_thread.is_alive():
                raise RuntimeError('SDK command executor exited; stopping lease holder')
            with client_lock:
                current_id = int(client.GetLeaseId())
            _write_status(id_file, f"{max(0, current_id)}\n")
            _write_status(alive_file, "1\n" if current_id else "0\n")
            _write_status(heartbeat_file, f"{time.time():.6f}\n")
            now = time.monotonic()
            if now >= next_mode_check:
                motion_name = ""
                robot_form = ""
                try:
                    raw_mode = switcher.CheckMode()
                    if (
                        isinstance(raw_mode, tuple)
                        and len(raw_mode) >= 2
                        and raw_mode[0] == 0
                        and isinstance(raw_mode[1], dict)
                    ):
                        motion_name = str(raw_mode[1].get("name", ""))
                        robot_form = str(raw_mode[1].get("form", ""))
                except Exception as exc:
                    print(
                        json.dumps(
                            {"event": "motion_mode_error", "message": str(exc)}
                        ),
                        flush=True,
                    )
                _write_status(motion_name_file, motion_name + "\n")
                _write_status(robot_form_file, robot_form + "\n")
                next_mode_check = now + 1.0
    finally:
        stop_event.set()
        executor.close()
        executor_thread.join(timeout=2.0)
        with client_lock:
            for attempt in range(1, 4):
                try:
                    raw = client.StopMove()
                    print(
                        json.dumps(
                            {
                                "event": "lease_holder_stop",
                                "attempt": attempt,
                                "raw_return_repr": repr(raw),
                            }
                        ),
                        flush=True,
                    )
                except Exception as exc:
                    print(
                        json.dumps(
                            {
                                "event": "lease_holder_stop_error",
                                "attempt": attempt,
                                "message": str(exc),
                            }
                        ),
                        flush=True,
                    )
                time.sleep(0.1)
        _write_status(alive_file, "0\n")
        _write_status(id_file, "0\n")
        _write_status(heartbeat_file, f"{time.time():.6f}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
