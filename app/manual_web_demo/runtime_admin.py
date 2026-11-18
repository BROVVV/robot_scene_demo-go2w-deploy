"""Safe WebUI entry point for restarting the current mapping/search runtime.

The destructive work is deliberately delegated to a detached supervisor.  A
Web request must be able to return before that supervisor stops the WebUI that
accepted it.  The shell supervisor only receives validated enum values; no
user-provided command text is ever evaluated.
"""

from __future__ import annotations

import os
import subprocess
import time
from pathlib import Path
from typing import Any


RESET_CONFIRMATIONS = {
    "objects": "CLEAR_CURRENT_OBJECTS",
    "map": "CLEAR_CURRENT_MAP",
}
RUNTIME_MODES = {"autonomous_search", "teleop_mapping"}
DETECTORS = {"grounded_sam", "yoloe_seg"}


class RuntimeAdminService:
    """Validate and enqueue one current-runtime reset operation."""

    def __init__(
        self,
        *,
        project_root: str | Path,
        mode: str,
        detector: str,
        enabled: bool = True,
        supervisor: str | Path = "scripts/go2w/reset_webui_runtime.sh",
    ) -> None:
        self.project_root = Path(project_root).resolve()
        self.mode = str(mode).strip().lower()
        self.detector = str(detector).strip().lower()
        self.enabled = bool(enabled)
        candidate = Path(supervisor)
        self.supervisor = (
            candidate if candidate.is_absolute() else self.project_root / candidate
        ).resolve()
        self.runtime_dir = self.project_root / "runtime" / "go2w"
        self.lock_path = self.runtime_dir / "webui_reset.lock"
        self.log_path = self.runtime_dir / "webui_reset.log"

    def status(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "mode": self.mode,
            "detector": self.detector,
            "busy": self.lock_path.exists(),
            "log_path": str(self.log_path),
        }

    def request_reset(self, *, action: str, confirmation: str) -> dict[str, Any]:
        action = str(action).strip().lower()
        if not self.enabled:
            raise RuntimeError("runtime_admin_disabled")
        if self.mode not in RUNTIME_MODES:
            raise RuntimeError(f"unsupported_runtime_mode:{self.mode}")
        if self.detector not in DETECTORS:
            raise RuntimeError(f"unsupported_detector:{self.detector}")
        if action not in RESET_CONFIRMATIONS:
            raise ValueError(f"unsupported_reset_action:{action}")
        if confirmation != RESET_CONFIRMATIONS[action]:
            raise ValueError("confirmation_mismatch")
        if not self.supervisor.is_file():
            raise RuntimeError(f"reset_supervisor_missing:{self.supervisor}")

        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        try:
            descriptor = os.open(
                self.lock_path,
                os.O_CREAT | os.O_EXCL | os.O_WRONLY,
                0o600,
            )
        except FileExistsError as exc:
            raise RuntimeError("reset_already_running") from exc
        try:
            os.write(
                descriptor,
                f"requested_at={time.time():.6f}\naction={action}\n".encode(),
            )
        finally:
            os.close(descriptor)

        try:
            log_handle = self.log_path.open("ab", buffering=0)
            process = subprocess.Popen(
                [
                    "/usr/bin/env",
                    "bash",
                    str(self.supervisor),
                    "--action",
                    action,
                    "--mode",
                    self.mode,
                    "--detector",
                    self.detector,
                    "--lock-file",
                    str(self.lock_path),
                ],
                cwd=self.project_root,
                stdin=subprocess.DEVNULL,
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                close_fds=True,
            )
        except Exception:
            self.lock_path.unlink(missing_ok=True)
            raise
        finally:
            if "log_handle" in locals():
                log_handle.close()

        return {
            "ok": True,
            "accepted": True,
            "action": action,
            "mode": self.mode,
            "detector": self.detector,
            "supervisor_pid": process.pid,
            "reload_after_seconds": 4,
        }
