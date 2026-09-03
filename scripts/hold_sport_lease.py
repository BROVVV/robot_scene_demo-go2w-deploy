#!/usr/bin/env python3
"""Own the Unitree Sport lease and execute bounded commands through that client."""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import stat
import threading
import time
from pathlib import Path
from typing import Any

from sdk_motion_protocol import (
    MAX_REQUEST_BYTES,
    ProtocolError,
    decode_request,
    execute_request,
)
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
    MotionSwitcherClient,
)
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient


def _write_status(path: Path, value: str) -> None:
    temporary = path.with_name(f".{path.name}.tmp.{os.getpid()}")
    temporary.write_text(value, encoding="ascii")
    os.replace(temporary, path)


class SdkMotionExecutor:
    """Local Unix-socket facade around the exact client that owns the lease."""

    def __init__(
        self,
        client: SportClient,
        client_lock: threading.Lock,
        socket_path: Path,
        stop_event: threading.Event,
    ) -> None:
        self._client = client
        self._client_lock = client_lock
        self._socket_path = socket_path
        self._stop_event = stop_event
        self._server: socket.socket | None = None

    def _safe_unlink_stale_socket(self) -> None:
        try:
            mode = self._socket_path.lstat().st_mode
        except FileNotFoundError:
            return
        if not stat.S_ISSOCK(mode):
            raise RuntimeError(
                f"refusing to replace non-socket path: {self._socket_path}"
            )
        self._socket_path.unlink()

    def close(self) -> None:
        if self._server is not None:
            self._server.close()
            self._server = None
        try:
            self._socket_path.unlink()
        except FileNotFoundError:
            pass

    def _response_for(self, raw: bytes) -> dict[str, Any]:
        request_id = 0
        api_id = 0
        try:
            request = decode_request(raw)
            request_id = request.get("request_id", 0)
            api_id = request.get("api_id", 0)
            with self._client_lock:
                lease_id = int(self._client.GetLeaseId())
                response = execute_request(request, self._client, lease_id)
        except ProtocolError as exc:
            response = {
                "request_id": request_id,
                "api_id": api_id,
                "lease_id": 0,
                "status_code": -9996,
                "data": {"error": str(exc), "transport": "sdk_direct"},
            }
        except Exception as exc:  # SDK boundary: return a structured failure.
            response = {
                "request_id": request_id,
                "api_id": api_id,
                "lease_id": 0,
                "status_code": -9995,
                "data": {
                    "error": f"{type(exc).__name__}: {exc}",
                    "transport": "sdk_direct",
                },
            }
        print(
            json.dumps(
                {
                    "event": "sdk_motion_command",
                    "request_id": response.get("request_id", 0),
                    "api_id": response.get("api_id", 0),
                    "lease_id": response.get("lease_id", 0),
                    "status_code": response.get("status_code", -9999),
                }
            ),
            flush=True,
        )
        return response

    def _serve_connection(self, connection: socket.socket) -> None:
        connection.settimeout(2.5)
        chunks: list[bytes] = []
        size = 0
        while size <= MAX_REQUEST_BYTES:
            chunk = connection.recv(min(4096, MAX_REQUEST_BYTES + 1 - size))
            if not chunk:
                break
            if b"\n" in chunk:
                chunks.append(chunk.split(b"\n", 1)[0])
                break
            chunks.append(chunk)
            size += len(chunk)
        response = self._response_for(b"".join(chunks))
        connection.sendall(
            json.dumps(response, separators=(",", ":")).encode("utf-8") + b"\n"
        )

    def serve(self) -> None:
        self._socket_path.parent.mkdir(parents=True, exist_ok=True)
        self._safe_unlink_stale_socket()
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._server = server
        server.bind(str(self._socket_path))
        os.chmod(self._socket_path, 0o600)
        server.listen(8)
        server.settimeout(0.2)
        print(
            json.dumps(
                {
                    "event": "sdk_motion_executor_ready",
                    "socket": str(self._socket_path),
                }
            ),
            flush=True,
        )
        try:
            while not self._stop_event.is_set():
                try:
                    connection, _ = server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    if self._stop_event.is_set():
                        break
                    raise
                with connection:
                    try:
                        self._serve_connection(connection)
                    except (OSError, socket.timeout) as exc:
                        print(
                            json.dumps(
                                {
                                    "event": "sdk_motion_transport_error",
                                    "message": str(exc),
                                }
                            ),
                            flush=True,
                        )
        finally:
            self.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", required=True)
    parser.add_argument("--ready-file")
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
        client, client_lock, Path(args.socket_path), stop_event
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
