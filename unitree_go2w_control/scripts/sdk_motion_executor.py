#!/usr/bin/env python3
"""Pure local SDK socket executor; does not import ROS or initialize vendor DDS."""
from __future__ import annotations
import json
import math
import os
import socket
import stat
import threading
import time
from pathlib import Path
from typing import Any
from sdk_motion_protocol import (MAX_REQUEST_BYTES, MOVE_API_ID, STOP_API_ID,
                                 ProtocolError, decode_request, execute_request)


def executor_timeouts(command_timeout_seconds=0., rpc_timeout_seconds=5.):
    """Keep legacy defaults; a configured watchdog requires bounded SDK calls."""
    if isinstance(command_timeout_seconds, bool) or isinstance(rpc_timeout_seconds, bool):
        raise ValueError('SDK timeout values must be finite numbers')
    command, rpc = float(command_timeout_seconds), float(rpc_timeout_seconds)
    if not math.isfinite(command) or command < 0 or not math.isfinite(rpc) or rpc <= 0:
        raise ValueError('command timeout must be nonnegative and RPC timeout positive')
    if command and rpc > command:
        raise ValueError('SDK RPC timeout must not exceed command timeout')
    return command, rpc


class SdkMotionExecutor:
    """Local Unix-socket facade around the exact client that owns the lease."""

    def __init__(
        self,
        client: Any,
        client_lock: threading.Lock,
        socket_path: Path,
        stop_event: threading.Event,
        command_timeout_seconds: float = 0.,
    ) -> None:
        self._client = client
        self._client_lock = client_lock
        self._socket_path = socket_path
        self._stop_event = stop_event
        self._server: socket.socket | None = None
        if (isinstance(command_timeout_seconds, bool) or
                not math.isfinite(command_timeout_seconds) or command_timeout_seconds < 0):
            raise ValueError('command timeout must be finite and nonnegative')
        self._command_timeout = command_timeout_seconds
        self._watchdog_close = threading.Event()
        self._watchdog_thread = None
        # All command/deadline updates use the same SDK-client lock as dispatch.
        self._command_active = False
        self._command_deadline = 0.

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
        self._watchdog_close.set()
        if self._server is not None:
            self._server.close()
            self._server = None
        try:
            self._socket_path.unlink()
        except FileNotFoundError:
            pass
        if self._watchdog_thread is not None and self._watchdog_thread is not threading.current_thread():
            self._watchdog_thread.join(timeout=2.)

    def _watchdog_stop(self, reason: str) -> None:
        # Caller owns _client_lock. Recheck the deadline under that lock so an
        # intervening fresh command cannot be stopped by an old timeout.
        try:
            response = execute_request(
                {'request_id': time.time_ns(), 'api_id': STOP_API_ID, 'parameter': {}},
                self._client, int(self._client.GetLeaseId()))
            code = int(response['status_code'])
        except Exception:
            code = -9995
        if code == 0:
            self._command_active = False
        else:
            # Keep retrying a failed STOP, with bounded log/SDK traffic.
            self._command_deadline = time.monotonic()+min(.2, self._command_timeout)
        print(json.dumps({'event': 'sdk_command_watchdog_stop', 'reason': reason,
                          'status_code': code}), flush=True)

    def _watch_commands(self) -> None:
        try:
            while not self._watchdog_close.wait(min(.025, self._command_timeout/4)):
                if self._stop_event.is_set():
                    break
                with self._client_lock:
                    if self._command_active and time.monotonic() >= self._command_deadline:
                        self._watchdog_stop('command_timeout')
        finally:
            with self._client_lock:
                if self._command_active:
                    self._watchdog_stop('executor_shutdown')

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
                if response['status_code'] == 0:
                    parameter = request.get('parameter', {})
                    if isinstance(parameter, str):
                        parameter = json.loads(parameter)
                    active = (api_id == MOVE_API_ID and
                              any(float(parameter[key]) != 0. for key in ('x', 'y', 'z')))
                    self._command_active = active
                    if active:
                        self._command_deadline = time.monotonic()+self._command_timeout
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
        if self._command_timeout:
            self._watchdog_thread = threading.Thread(
                target=self._watch_commands, name='sdk-command-watchdog', daemon=True)
            self._watchdog_thread.start()
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
                    if self._stop_event.is_set() or self._watchdog_close.is_set():
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
