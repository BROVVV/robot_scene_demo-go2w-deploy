"""Manual WASD+QE drive controller.

Implements the plan book's ManualDriveController (§14) as a pure, ROS-free
state machine:

* keyboard intent (key_down / key_up / heartbeat) into a single command;
* one short motion pulse in flight at a time — never goal stacking;
* last-pressed-key-wins with W/S, A/D, Q/E conflict pairs resolving to STOP;
* normal STOP vs emergency STOP distinction;
* a Web-side deadman (heartbeat timeout) and camera-freshness gate;
* per-direction safety gates evaluated BEFORE the pulse is armed.

The controller talks to the robot only through a small ``MotionExecutor``
interface (wired to the ROS worker over JSONL IPC by the web server), and
reads camera freshness / safety through injected providers so it can be unit
tested without ROS or hardware.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Protocol

from app.manual_web_demo.config import ManualDemoSettings
from app.manual_web_demo.models import ManualDriveState, SafetySnapshot

# Fixed key mapping (plan book §9.1) — A/D are strafes, never turns.
KEY_MAP: dict[str, str] = {
    "w": "forward",
    "s": "backward",
    "a": "strafe_left",
    "d": "strafe_right",
    "q": "turn_left",
    "e": "turn_right",
}

# Conflict pairs resolve to STOP (plan book §13).
_CONFLICT_PAIRS = (("w", "s"), ("a", "d"), ("q", "e"))

STATUS_DISABLED = "DISABLED"
STATUS_READY = "READY"
STATUS_MOVING = "MOVING"
STATUS_STOPPING = "STOPPING"
STATUS_ESTOP = "ESTOP"
STATUS_BLOCKED = "BLOCKED"
STATUS_ERROR = "ERROR"

# A hold goal must have run at least this long before the controller renews it.
# Guards against instantly-completing (fake / rejected) results causing an
# infinite renewal loop; a real hold goal runs for ``hold_duration_sec``.
_MIN_RENEW_ELAPSED_SEC = 1.0


class MotionExecutor(Protocol):
    """Minimal surface the controller drives to move the robot."""

    def available(self) -> bool:
        ...

    def send_pulse(self, direction: str, on_result: Callable[[dict[str, Any]], None]) -> None:
        ...

    def stop(self) -> None:
        ...

    def estop(self) -> None:
        ...


class _NoopExecutor:
    def available(self) -> bool:
        return False

    def send_pulse(self, direction: str, on_result: Callable[[dict[str, Any]], None]) -> None:
        on_result({"success": False, "message": "no motion executor"})

    def stop(self) -> None:
        pass

    def estop(self) -> None:
        pass


@dataclass
class ManualDriveController:
    """Keyboard-driven short-pulse controller for the manual web demo."""

    executor: MotionExecutor = field(default_factory=_NoopExecutor)
    config: ManualDemoSettings = field(default_factory=ManualDemoSettings)
    safety_provider: Callable[[], SafetySnapshot] = field(
        default_factory=lambda: (lambda: SafetySnapshot())
    )
    camera_fresh_provider: Callable[[], bool] = field(
        default_factory=lambda: (lambda: True)
    )
    clock: Callable[[], float] = field(default_factory=lambda: time.monotonic)
    on_event: Callable[[dict[str, Any]], None] = field(
        default_factory=lambda: (lambda event: None)
    )

    def __post_init__(self) -> None:
        self._state = ManualDriveState(status=STATUS_DISABLED)
        self._pressed: dict[str, float] = {}
        self._last_pulse_started_at: float = 0.0
        self._pending_stop: bool = False
        self._seq: int = 0
        # The controller is touched from the event loop (tick / WebSocket) and
        # from the ROS worker reader thread (motion_finished), so every public
        # entry point serializes through this reentrant lock.
        self._lock = threading.RLock()

    # ------------------------------------------------------------------ #
    # public API (wired to WebSocket / HTTP handlers)                     #
    # ------------------------------------------------------------------ #
    @property
    def state(self) -> ManualDriveState:
        return self._state

    def enable(self) -> bool:
        """Turn keyboard control on. Control stays disabled by default."""
        with self._lock:
            if self._state.status == STATUS_ESTOP:
                self._state.blocked_reason = "emergency_stop_latched"
                self._emit({"type": "motion_blocked", "direction": "enable", "reason": self._state.blocked_reason})
                return False
            if not self.executor.available():
                self._state.control_enabled = False
                self._state.status = STATUS_BLOCKED
                self._state.blocked_reason = "motion_action_offline"
                self._emit({"type": "motion_blocked", "direction": "enable", "reason": "motion_action_offline"})
                return False
            if not self.camera_fresh_provider():
                self._state.control_enabled = False
                self._state.status = STATUS_BLOCKED
                self._state.blocked_reason = "camera_stale"
                self._emit({"type": "motion_blocked", "direction": "enable", "reason": "camera_stale"})
                return False
            self._pressed.clear()
            self._pending_stop = False
            self._state.control_enabled = True
            self._state.status = STATUS_READY
            self._state.command = "stop"
            self._state.blocked_reason = None
            self._state.last_heartbeat_monotonic = self.clock()
            self._emit({"type": "state", "state": self._state.to_dict()})
            return True

    def disable(self, reason: str | None = None) -> None:
        with self._lock:
            self._stop_if_motion()
            self._pressed.clear()
            self._state.control_enabled = False
            self._state.status = STATUS_DISABLED
            self._state.blocked_reason = reason
            self._state.command = "stop"
            self._state.pressed_key = None
            self._emit({"type": "state", "state": self._state.to_dict()})

    def estop(self) -> None:
        """Emergency stop: latched until the user explicitly enables again."""
        with self._lock:
            self._pressed.clear()
            self._pending_stop = False
            self._state.control_enabled = False
            self._state.status = STATUS_ESTOP
            self._state.command = "stop"
            self._state.pressed_key = None
            self._state.blocked_reason = "emergency_stop"
            self.executor.estop()
            self._emit({"type": "state", "state": self._state.to_dict()})

    def on_key_down(self, key: str) -> None:
        with self._lock:
            if not self._state.control_enabled or self._state.status == STATUS_ESTOP:
                return
            key = key.lower()
            if key not in KEY_MAP:
                return
            self._pressed[key] = self.clock()
            self._reconcile(trigger_pulse=True)

    def on_key_up(self, key: str) -> None:
        with self._lock:
            self._pressed.pop(key.lower(), None)
            if not self._pressed:
                self._reconcile(trigger_pulse=False)
                self._stop_if_motion()
            else:
                self._reconcile(trigger_pulse=False)

    def on_heartbeat(self, pressed: list[str], seq: int | None = None) -> None:
        """Heartbeat every ~100ms.

        Only refreshes the deadman timestamp; it does NOT clear ``_pressed``
        from the heartbeat's ``pressed`` array. Multiple browser tabs each send
        heartbeats, and a stale tab's empty ``pressed`` would clear the active
        tab's held keys and make the robot ignore key presses (command falls
        back to STOP). Key state is managed by key_down / key_up / release_all;
        the deadman still stops motion if the browser dies.
        """
        with self._lock:
            self._state.last_heartbeat_monotonic = self.clock()
            if seq is not None:
                self._seq = int(seq)
            self._reconcile(trigger_pulse=False)

    def on_release_all(self) -> None:
        with self._lock:
            self._pressed.clear()
            self._stop_if_motion()
            self._state.command = "stop"
            self._state.pressed_key = None
            self._emit({"type": "state", "state": self._state.to_dict()})

    def on_ws_disconnect(self) -> None:
        with self._lock:
            self.on_release_all()
            if self._state.control_enabled:
                self.disable(reason="websocket_disconnected")

    def tick(self) -> None:
        """Periodic housekeeping (call ~10-20Hz): deadman, camera gate, hold."""
        with self._lock:
            if self._state.control_enabled:
                elapsed = self.clock() - self._state.last_heartbeat_monotonic
                if elapsed > self.config.deadman_sec:
                    self.disable(reason="heartbeat_timeout")
                    self._emit({"type": "deadman", "reason": "heartbeat_timeout"})
                    return
                if not self.camera_fresh_provider():
                    self._stop_if_motion()
                    self._pressed.clear()
                    self._state.control_enabled = False
                    self._state.status = STATUS_BLOCKED
                    self._state.blocked_reason = "camera_stale"
                    self._state.command = "stop"
                    self._state.pressed_key = None
                    self._emit({"type": "state", "state": self._state.to_dict()})
                    return
                self._maybe_pulse(force=False)

    def direction_availability(self) -> dict[str, dict[str, Any]]:
        """Per-direction live gate status for the UI (plan book §53)."""
        with self._lock:
            available = {}
            for direction in sorted(set(KEY_MAP.values())):
                ok, reason = self._check_gates(direction)
                available[direction] = {"allowed": ok, "reason": reason}
            return available

    # ------------------------------------------------------------------ #
    # internal state machine                                             #
    # ------------------------------------------------------------------ #
    def _active_command(self) -> tuple[str | None, str]:
        """Return (key, command) honoring conflict pairs -> STOP.

        ``last pressed wins``. When two keys share the same monotonic press
        time (e.g. two keydowns within the same event-loop tick), the more
        recently inserted key wins the tie.
        """
        keys = set(self._pressed)
        for left, right in _CONFLICT_PAIRS:
            if left in keys and right in keys:
                return None, "stop"
        if not self._pressed:
            return None, "stop"
        active_key: str | None = None
        active_time = float("-inf")
        for key in self._pressed:
            press_time = self._pressed[key]
            if press_time >= active_time:
                active_key = key
                active_time = press_time
        return active_key, KEY_MAP[active_key]

    def _reconcile(self, *, trigger_pulse: bool) -> None:
        key, command = self._active_command()
        previous = self._state.command
        self._state.command = command
        self._state.pressed_key = key
        if command == "stop":
            self._stop_if_motion()
            self._emit({"type": "state", "state": self._state.to_dict()})
            return
        if previous not in ("stop", command):
            # Direction switch: cancel the current pulse before the new one.
            self._stop_if_motion()
        if trigger_pulse:
            self._maybe_pulse(force=True)
        else:
            self._maybe_pulse(force=False)

    def _stop_if_motion(self) -> None:
        if self._state.motion_in_flight and self._state.status in (
            STATUS_MOVING,
            STATUS_STOPPING,
        ):
            self._state.status = STATUS_STOPPING
            self._pending_stop = True
            self.executor.stop()

    def _maybe_pulse(self, *, force: bool) -> None:
        if not self._state.control_enabled:
            return
        if self._state.status in (STATUS_ESTOP, STATUS_STOPPING):
            return
        if self._state.motion_in_flight or self._pending_stop:
            return
        key, command = self._active_command()
        if command == "stop" or key is None:
            return
        ok, reason = self._check_gates(command)
        if not ok:
            self._state.status = STATUS_BLOCKED
            self._state.blocked_reason = reason
            self._emit({"type": "motion_blocked", "direction": command, "reason": reason})
            return
        # Continuous hold mode: while a key is held, one long goal runs; it is
        # renewed on completion (see _on_motion_result). No repeat-interval gate.
        self._send_pulse(command)

    def _check_gates(self, command: str) -> tuple[bool, str]:
        """Gate the pulse BEFORE arming (plan book §16 / §17)."""
        safety = self.safety_provider()
        if command == "forward":
            if not self.config.allow_forward:
                return False, "forward disabled by configuration"
            if safety.lidar_fresh is not True:
                return False, "front clearance stale (LiDAR unavailable)"
            if (
                safety.front_clearance_m is None
                or safety.front_clearance_m < self.config.min_front_clearance_m
            ):
                return (
                    False,
                    f"front clearance "
                    f"{safety.front_clearance_m:.3f}m < "
                    f"{self.config.min_front_clearance_m:.3f}m",
                )
            return True, ""
        if command == "backward":
            if not self.config.allow_backward:
                return False, "backward not enabled: rear safety not validated"
            return True, ""
        if command in ("strafe_left", "strafe_right"):
            if not self.config.allow_strafe:
                return False, "lateral_motion_not_supported (wheeled robot)"
            return True, ""
        if command in ("turn_left", "turn_right"):
            if not self.config.allow_turn:
                return False, "turn disabled by configuration"
            if (
                not self.config.allow_turn_override
                and safety.rotation_clearance_valid is not True
            ):
                return False, "rotation_clearance_invalid"
            return True, ""
        return False, f"unknown command {command}"

    def _send_pulse(self, command: str) -> None:
        self._state.status = STATUS_MOVING
        self._state.motion_in_flight = True
        self._state.blocked_reason = None
        self._last_pulse_started_at = self.clock()
        self._emit({"type": "motion_started", "direction": command})
        self.executor.send_pulse(command, on_result=self._on_motion_result)

    def _on_motion_result(self, result: dict[str, Any]) -> None:
        with self._lock:
            self._state.motion_in_flight = False
            was_pending_stop = self._pending_stop
            self._pending_stop = False
            self._state.last_motion_result = result
            success = bool(result.get("success", False))
            error_code = str(result.get("error_code") or "none")
            if not success and error_code not in ("canceled", "none"):
                self._state.status = STATUS_BLOCKED
                self._state.blocked_reason = str(result.get("message") or "motion_failed")
            elif was_pending_stop:
                self._state.status = STATUS_READY
                self._state.blocked_reason = None
            elif success:
                self._state.status = STATUS_READY
                self._state.blocked_reason = None
            else:
                # A cancel produced by STOP (not a failure): return to READY.
                self._state.status = STATUS_READY
                self._state.blocked_reason = None
            self._emit({"type": "motion_finished", "result": result})
            self._emit({"type": "state", "state": self._state.to_dict()})
            # Continuous hold: if the key is still pressed, control enabled,
            # camera fresh and no stop pending, renew the long goal so motion
            # continues seamlessly across the goal boundary. Only renew a goal
            # that actually ran (elapsed >= threshold) to avoid renewing
            # instantly-completing/failed results.
            if (
                self._state.control_enabled
                and not self._pending_stop
                and self._state.status in (STATUS_READY,)
                and self.camera_fresh_provider()
                and bool(result.get("success"))
                and float(result.get("elapsed_sec") or 0.0)
                >= _MIN_RENEW_ELAPSED_SEC
            ):
                key, command = self._active_command()
                if command != "stop" and key is not None:
                    self._maybe_pulse(force=True)

    def _emit(self, event: dict[str, Any]) -> None:
        try:
            self.on_event(event)
        except Exception:  # noqa: BLE001 - event sinks must never break motion
            pass
