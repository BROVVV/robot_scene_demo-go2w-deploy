"""Durable rolling archive for WebUI search sessions.

Only directories carrying ``webui_session.json`` are managed/pruned here.
This deliberately preserves older CLI/experimental runs that may share the
``outputs/live_runs`` root but do not belong to the rolling WebUI history.
"""

from __future__ import annotations

import json
import os
import shutil
import threading
import time
from pathlib import Path
from typing import Any

from app.live_robot.search_event import SearchEvent
from app.live_robot.search_state_store import (
    BUDGET_COMPLETION_RESULTS,
    normalize_search_snapshot,
)

MARKER_FILE = "webui_session.json"
STATE_FILE = "webui_state.json"
EVENTS_FILE = "webui_events.jsonl"
ARCHIVE_SCHEMA = "webui_search_archive_v1"
TERMINAL_STATUSES = {
    "TARGET_FOUND", "SEARCH_EXHAUSTED", "FAILED", "OPERATOR_STOP",
    "FINISHED", "TASK_REJECTED", "MAX_STEPS_REACHED",
    "MAX_PLANNING_CYCLES_REACHED", "TIMEOUT",
}


class SearchSessionArchive:
    def __init__(self, root: str | Path, *, max_sessions: int = 10) -> None:
        self.root = Path(root)
        self.max_sessions = max(1, int(max_sessions))
        self._lock = threading.Lock()

    def begin(self, session_id: str, metadata: dict[str, Any]) -> None:
        now = time.time()
        marker = {
            "schema_version": ARCHIVE_SCHEMA,
            "session_id": session_id,
            "status": "STARTING",
            "created_at": now,
            "updated_at": now,
            **metadata,
        }
        with self._lock:
            directory = self._safe_session_dir(session_id)
            directory.mkdir(parents=True, exist_ok=True)
            self._atomic_json(directory / MARKER_FILE, marker)
            events = directory / EVENTS_FILE
            if not events.exists():
                events.touch()
            self._prune_locked()

    def record(self, snapshot: dict[str, Any], event: SearchEvent | None = None) -> None:
        session_id = str(snapshot.get("session_id") or "")
        if not session_id:
            return
        with self._lock:
            directory = self._safe_session_dir(session_id)
            marker_path = directory / MARKER_FILE
            if not marker_path.is_file():
                return
            self._atomic_json(directory / STATE_FILE, snapshot)
            if event is not None:
                with (directory / EVENTS_FILE).open("a", encoding="utf-8") as handle:
                    handle.write(json.dumps(event.to_dict(), ensure_ascii=False) + "\n")
                    handle.flush()
            marker = self._read_json(marker_path) or {}
            marker.update({
                "status": snapshot.get("status") or marker.get("status"),
                "result": snapshot.get("result") or "",
                "target": snapshot.get("target") or marker.get("target") or "",
                "finished_at": snapshot.get("finished_at"),
                "updated_at": time.time(),
                "error": snapshot.get("error"),
                "cycle": snapshot.get("cycle", 0),
                "elapsed_seconds": snapshot.get("elapsed_seconds", 0.0),
                "finish_reason": snapshot.get("finish_reason") or "",
            })
            self._atomic_json(marker_path, marker)
            if str(marker.get("status") or "") in TERMINAL_STATUSES:
                self._prune_locked()

    def list(self, limit: int = 10) -> list[dict[str, Any]]:
        with self._lock:
            records = self._markers_locked()
        return [
            self._normalize_marker(item)
            for item in records[: min(self.max_sessions, max(1, int(limit)))]
        ]

    def load(self, session_id: str) -> dict[str, Any] | None:
        with self._lock:
            directory = self._safe_session_dir(session_id)
            marker = self._read_json(directory / MARKER_FILE)
            state = self._read_json(directory / STATE_FILE)
            if marker is None or state is None:
                return None
            events: list[dict[str, Any]] = []
            events_path = directory / EVENTS_FILE
            if events_path.is_file():
                try:
                    for line in events_path.read_text(encoding="utf-8").splitlines():
                        if line.strip():
                            value = json.loads(line)
                            if isinstance(value, dict):
                                events.append(value)
                except (OSError, json.JSONDecodeError):
                    events = []
            artifacts = sorted(
                item.name for item in directory.iterdir()
                if item.is_file() and item.name not in {MARKER_FILE, STATE_FILE, EVENTS_FILE}
            )
            return {
                "session": self._normalize_marker(marker),
                "state": normalize_search_snapshot(state),
                "events": events,
                "artifacts": artifacts,
            }

    def latest(self) -> dict[str, Any] | None:
        records = self.list(1)
        return self.load(records[0]["session_id"]) if records else None

    def _markers_locked(self) -> list[dict[str, Any]]:
        if not self.root.is_dir():
            return []
        records: list[dict[str, Any]] = []
        for directory in self.root.iterdir():
            if not directory.is_dir():
                continue
            marker = self._read_json(directory / MARKER_FILE)
            if not marker:
                continue
            marker["updated_at"] = float(marker.get("updated_at") or 0.0)
            records.append(marker)
        records.sort(key=lambda item: item["updated_at"], reverse=True)
        return records

    def _prune_locked(self) -> None:
        records = self._markers_locked()
        for marker in records[self.max_sessions:]:
            # Never remove a task that still claims to be active.  There can be
            # at most one, and it will be finalized/reclassified on recovery.
            if str(marker.get("status") or "") not in TERMINAL_STATUSES:
                continue
            directory = self._safe_session_dir(str(marker.get("session_id") or ""))
            if directory.is_dir():
                shutil.rmtree(directory)

    def _safe_session_dir(self, session_id: str) -> Path:
        if not session_id or Path(session_id).name != session_id or not session_id.startswith("search_"):
            raise ValueError("invalid search session id")
        return self.root / session_id

    @staticmethod
    def _read_json(path: Path) -> dict[str, Any] | None:
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
            return value if isinstance(value, dict) else None
        except (OSError, json.JSONDecodeError):
            return None

    @staticmethod
    def _normalize_marker(marker: dict[str, Any]) -> dict[str, Any]:
        value = dict(marker or {})
        result = str(value.get("result") or value.get("finish_reason") or "")
        if result in BUDGET_COMPLETION_RESULTS:
            value["status"] = "FINISHED"
            value["error"] = None
        error = value.get("error") if isinstance(value.get("error"), dict) else {}
        if (
            str(value.get("status") or "") == "FAILED"
            and not str(value.get("finish_reason") or "")
            and str(error.get("source") or "") in {
                "observer_retry", "perception_retry",
            }
        ):
            value["status"] = "FINISHED"
            value["result"] = "INTERRUPTED_DURING_RETRY"
            value["finish_reason"] = "WEBUI_RESTART_DURING_RETRY"
            value["error"] = None
        return value

    @staticmethod
    def _atomic_json(path: Path, value: dict[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        temp = path.with_name(path.name + ".tmp")
        with temp.open("w", encoding="utf-8") as handle:
            # Compact encoding materially reduces write latency for topology
            # snapshots while os.replace still prevents half-written JSON.
            json.dump(value, handle, ensure_ascii=False, separators=(",", ":"))
            handle.flush()
        os.replace(temp, path)
