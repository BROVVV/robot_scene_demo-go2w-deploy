"""Web-side client for the ROS worker subprocess.

The ROS worker runs under ``/usr/bin/python3`` with the ROS environment
sourced; the Web/Conda process talks to it over JSON Lines on stdin/stdout
(plan book §19). This module owns spawning, keepalive, message parsing and a
small in-process watchdog that mirrors the worker-side motion watchdog.

Protocol (Web -> worker):

.. code-block:: json

   {"type":"status"}
   {"type":"pulse","direction":"forward"}
   {"type":"stop"}
   {"type":"estop"}
   {"type":"keepalive"}
   {"type":"shutdown"}

Protocol (worker -> Web):

.. code-block:: json

   {"type":"ready"}
   {"type":"motion_started","direction":"..."}
   {"type":"motion_finished", "success":true, "error_code":"none", ...}
   {"type":"blocked","reason":"..."}
   {"type":"camera_status", ...}
   {"type":"worker_status", ...}
   {"type":"error","message":"..."}
"""

from __future__ import annotations

import json
import os
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Callable

# Messages the worker may emit that we route to registered handlers.
_WORKER_MESSAGE_TYPES = (
    "ready",
    "motion_started",
    "motion_finished",
    "blocked",
    "camera_status",
    "worker_status",
    "error",
)


def parse_worker_message(line: str) -> dict[str, Any]:
    """Parse one JSONL line from the worker; raises ValueError on junk.

    A non-JSON or non-object line is dropped by callers, but the parser is
    strict so protocol tests can assert the failure modes.
    """
    text = line.strip()
    if not text:
        raise ValueError("empty worker line")
    try:
        payload = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ValueError(f"malformed JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError("worker line is not a JSON object")
    return payload


def encode_web_command(command: dict[str, Any]) -> str:
    """Encode one Web->worker command as a JSONL line."""
    return json.dumps(command, ensure_ascii=False, separators=(",", ":")) + "\n"


class RosWorkerClient:
    """Owns the ROS worker subprocess and routes its JSONL output."""

    def __init__(
        self,
        *,
        cmd: tuple[str, ...],
        cwd: str | Path,
        on_message: Callable[[str, dict[str, Any]], None] | None = None,
        log_path: str | Path | None = None,
        keepalive_interval_sec: float = 0.1,
    ) -> None:
        self._cmd = list(cmd)
        self._cwd = str(cwd)
        self._on_message = on_message
        self._log_path = Path(log_path) if log_path else None
        self._keepalive_interval_sec = keepalive_interval_sec
        self._proc: subprocess.Popen[str] | None = None
        self._reader: threading.Thread | None = None
        self._keepalive_thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._last_keepalive_sent = 0.0
        self._status: dict[str, Any] = {
            "state": "stopped",
            "motion_available": False,
            "camera_available": False,
            "robot_mode": None,
            "robot_error_code": None,
            "lease_alive": False,
            "state_fresh": False,
            "lidar_fresh": None,
            "front_clearance_m": None,
            "left_clearance_m": None,
            "right_clearance_m": None,
            "rotation_clearance_valid": None,
            "last_error": None,
        }

    def set_on_message(
        self, callback: Callable[[str, dict[str, Any]], None]
    ) -> None:
        self._on_message = callback

    # ------------------------------------------------------------------ #
    # lifecycle                                                           #
    # ------------------------------------------------------------------ #
    def start(self) -> None:
        if self._proc is not None and self._proc.poll() is None:
            return
        self._stop.clear()
        stderr_target = None
        if self._log_path is not None:
            self._log_path.parent.mkdir(parents=True, exist_ok=True)
            stderr_target = self._log_path.open("ab")
        env = dict(os.environ)
        env["PYTHONUNBUFFERED"] = "1"
        self._proc = subprocess.Popen(
            self._cmd,
            cwd=self._cwd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=stderr_target,
            env=env,
            text=True,
            bufsize=1,
        )
        self._status["state"] = "starting"
        self._reader = threading.Thread(
            target=self._read_loop, daemon=True, name="ros-worker-reader"
        )
        self._reader.start()
        self._keepalive_thread = threading.Thread(
            target=self._keepalive_loop, daemon=True, name="ros-worker-keepalive"
        )
        self._keepalive_thread.start()

    def stop(self, *, timeout_sec: float = 5.0) -> None:
        self._stop.set()
        self._send({"type": "shutdown"})
        proc = self._proc
        if proc is not None:
            try:
                proc.stdin.close()
            except OSError:
                pass
            try:
                proc.wait(timeout=timeout_sec)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
        self._proc = None
        self._status["state"] = "stopped"

    def alive(self) -> bool:
        proc = self._proc
        return proc is not None and proc.poll() is None

    # ------------------------------------------------------------------ #
    # commands                                                            #
    # ------------------------------------------------------------------ #
    def send(self, command: dict[str, Any]) -> None:
        self._send(command)

    def request_status(self) -> None:
        self._send({"type": "status"})

    def request_pulse(self, direction: str) -> None:
        self._send({"type": "pulse", "direction": direction})

    def request_stop(self) -> None:
        self._send({"type": "stop"})

    def request_estop(self) -> None:
        self._send({"type": "estop"})

    def shutdown(self) -> None:
        self._send({"type": "shutdown"})

    # ------------------------------------------------------------------ #
    # status                                                              #
    # ------------------------------------------------------------------ #
    def status(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._status)

    # ------------------------------------------------------------------ #
    # internals                                                           #
    # ------------------------------------------------------------------ #
    def _send(self, command: dict[str, Any]) -> None:
        proc = self._proc
        if proc is None or proc.poll() is not None:
            return
        try:
            proc.stdin.write(encode_web_command(command))
            proc.stdin.flush()
        except (BrokenPipeError, OSError):
            with self._lock:
                self._status["state"] = "broken"
                self._status["last_error"] = "worker pipe closed"

    def _read_loop(self) -> None:
        proc = self._proc
        if proc is None or proc.stdout is None:
            return
        for line in proc.stdout:
            if self._stop.is_set():
                break
            line = line.rstrip("\n")
            if not line.strip():
                continue
            try:
                payload = parse_worker_message(line)
            except ValueError:
                # Log to stderr of the web process, never crash the loop.
                print(f"[ros-worker] dropped non-JSON line: {line!r}", flush=True)
                continue
            msg_type = str(payload.get("type") or "")
            if msg_type not in _WORKER_MESSAGE_TYPES:
                continue
            self._handle_message(msg_type, payload)

    def _handle_message(self, msg_type: str, payload: dict[str, Any]) -> None:
        if msg_type == "camera_status":
            with self._lock:
                self._status["camera_available"] = bool(
                    payload.get("available", False)
                )
        elif msg_type == "worker_status":
            with self._lock:
                for key in (
                    "motion_available",
                    "robot_mode",
                    "robot_error_code",
                    "lease_alive",
                    "state_fresh",
                    "lidar_fresh",
                    "front_clearance_m",
                    "left_clearance_m",
                    "right_clearance_m",
                    "rotation_clearance_valid",
                ):
                    if key in payload:
                        self._status[key] = payload[key]
                self._status["state"] = "ready"
        elif msg_type == "error":
            with self._lock:
                self._status["state"] = "error"
                self._status["last_error"] = payload.get("message")
        elif msg_type == "ready":
            with self._lock:
                self._status["state"] = "ready"
        if self._on_message is not None:
            try:
                self._on_message(msg_type, payload)
            except Exception:  # noqa: BLE001 - handler errors must not kill reader
                pass

    def _keepalive_loop(self) -> None:
        while not self._stop.is_set() and self.alive():
            self._send({"type": "keepalive"})
            self._stop.wait(self._keepalive_interval_sec)
