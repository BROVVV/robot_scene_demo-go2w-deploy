"""ControlOwner: manual / autonomous / estop ownership arbitration
(plan book §43, §100).

Exactly one owner may drive the robot at a time:

* NONE        – nobody owns motion;
* MANUAL      – the WASD+QE controller is enabled;
* AUTONOMOUS  – the autonomous search session owns motion;
* ESTOP       – emergency stop latched, everything else is refused until an
  explicit operator reset passes the runtime health checks.

The web-server routes consult the owner before enabling manual control or
starting a search; the estop path forces the owner to ESTOP and refuses any
further start/enable until explicitly reset by the WebUI.
"""

from __future__ import annotations

import threading
from enum import Enum
from typing import Any


class OwnerState(str, Enum):
    NONE = "NONE"
    MANUAL = "MANUAL"
    AUTONOMOUS = "AUTONOMOUS"
    ESTOP = "ESTOP"


class ControlOwner:
    """Thread-safe ownership tracker."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state = OwnerState.NONE
        self._owner_detail: str = ""

    # ------------------------------------------------------------------ #
    # queries                                                            #
    # ------------------------------------------------------------------ #
    def state(self) -> OwnerState:
        with self._lock:
            return self._state

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {"owner": self._state.value, "detail": self._owner_detail}

    def is_manual(self) -> bool:
        return self.state() == OwnerState.MANUAL

    def is_autonomous(self) -> bool:
        return self.state() == OwnerState.AUTONOMOUS

    def is_estop(self) -> bool:
        return self.state() == OwnerState.ESTOP

    # ------------------------------------------------------------------ #
    # transitions                                                        #
    # ------------------------------------------------------------------ #
    def try_manual(self, *, detail: str = "manual_control") -> tuple[bool, str]:
        """Manual takes ownership only when nobody else owns motion."""
        with self._lock:
            if self._state == OwnerState.ESTOP:
                return False, "emergency_stop_latched"
            if self._state == OwnerState.AUTONOMOUS:
                return False, "autonomous_search_running: stop or pause the search first"
            self._state = OwnerState.MANUAL
            self._owner_detail = detail
            return True, ""

    def try_autonomous(self, *, detail: str = "autonomous_search") -> tuple[bool, str]:
        """Autonomous takes ownership only when nobody else owns motion."""
        with self._lock:
            if self._state == OwnerState.ESTOP:
                return False, "emergency_stop_latched"
            if self._state == OwnerState.MANUAL:
                return False, "manual_control_active: disable manual control first"
            self._state = OwnerState.AUTONOMOUS
            self._owner_detail = detail
            return True, ""

    def release(self, owner: OwnerState) -> None:
        """Release ownership previously taken by ``owner`` (NONE if it still
        holds; ESTOP and other owners are never released by this call)."""
        with self._lock:
            if self._state == owner:
                self._state = OwnerState.NONE
                self._owner_detail = ""

    def estop(self) -> None:
        """Estop overrides every owner and latches until reset."""
        with self._lock:
            self._state = OwnerState.ESTOP
            self._owner_detail = "estop"

    def reset_estop(self, *, detail: str = "operator_estop_reset") -> tuple[bool, str]:
        """Clear the application latch after an explicit operator reset.

        Hardware stop/arm state is intentionally not changed here. The
        runtime performs health checks before calling this method, and the
        next motion request must still pass the normal arm/action gates.
        """
        with self._lock:
            if self._state != OwnerState.ESTOP:
                return True, ""
            self._state = OwnerState.NONE
            self._owner_detail = detail
            return True, ""
