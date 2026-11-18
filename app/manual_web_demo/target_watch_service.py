"""Web-process service for the passive mapping target-watch file protocol."""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Any

from app.perception.passive_target_watch import (
    SCHEMA_VERSION,
    TargetWatchSpec,
    atomic_write_json,
    deterministic_spec,
    empty_state,
    load_json_object,
    spec_from_profile,
)


class TargetWatchService:
    def __init__(
        self,
        *,
        request_path: str | Path,
        state_path: str | Path,
        evidence_dir: str | Path,
        topology_path: str | Path,
    ) -> None:
        self.request_path = Path(request_path).resolve()
        self.state_path = Path(state_path).resolve()
        self.evidence_dir = Path(evidence_dir).resolve()
        self.topology_path = Path(topology_path).resolve()
        self._lock = threading.Lock()

    def mapping_session_id(self) -> Any:
        state = load_json_object(self.state_path) or {}
        if state.get("mapping_session_id") is not None:
            return state.get("mapping_session_id")
        topology = load_json_object(self.topology_path) or {}
        return topology.get("session_id")

    def available(self) -> bool:
        return self.mapping_session_id() is not None

    def get_state(self) -> dict[str, Any]:
        session_id = self.mapping_session_id()
        state = load_json_object(self.state_path)
        if state is None or str(state.get("mapping_session_id")) != str(session_id):
            return empty_state(session_id)
        return state

    def configure(self, target: str, *, use_llm: bool = True) -> dict[str, Any]:
        query = str(target).strip()
        if not query or len(query) > 500:
            raise ValueError("目标必须为 1 到 500 个字符")
        session_id = self.mapping_session_id()
        if session_id is None:
            raise RuntimeError("remote mapping session is not active")
        try:
            from app.reasoning.target_profile import TargetProfileResolver

            spec = spec_from_profile(
                query, TargetProfileResolver().resolve(query, use_llm=use_llm)
            )
        except Exception as exc:
            fallback = deterministic_spec(query)
            spec = TargetWatchSpec(
                **{**fallback.__dict__, "resolver_error": f"{type(exc).__name__}: {exc}"}
            )
        with self._lock:
            generation = self._next_generation()
            payload = {
                "schema_version": SCHEMA_VERSION,
                "mapping_session_id": session_id,
                "generation": generation,
                "enabled": True,
                "command": "set",
                "raw_query": query,
                "profile": spec.to_dict(),
                "submitted_at": time.time(),
            }
            atomic_write_json(self.request_path, payload)
        return payload

    def clear(self) -> dict[str, Any]:
        session_id = self.mapping_session_id()
        if session_id is None:
            raise RuntimeError("remote mapping session is not active")
        with self._lock:
            payload = {
                "schema_version": SCHEMA_VERSION,
                "mapping_session_id": session_id,
                "generation": self._next_generation(),
                "enabled": False,
                "command": "clear",
                "raw_query": "",
                "profile": None,
                "submitted_at": time.time(),
            }
            atomic_write_json(self.request_path, payload)
        return payload

    def evidence_path(self, verification_id: str) -> Path | None:
        safe_id = str(verification_id)
        if not safe_id or any(ch not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for ch in safe_id):
            return None
        state = self.get_state()
        records = list(state.get("confirmed_matches") or [])
        last = state.get("last_candidate")
        if isinstance(last, dict):
            records.append(last)
        verification = state.get("verification")
        if isinstance(verification, dict):
            records.append(verification)
        record = next(
            (item for item in records if str(item.get("verification_id") or "") == safe_id),
            None,
        )
        path_value = record.get("evidence_path") if record else None
        if not path_value:
            return None
        path = Path(str(path_value)).resolve()
        try:
            path.relative_to(self.evidence_dir)
        except ValueError:
            return None
        return path if path.is_file() else None

    def _next_generation(self) -> int:
        request = load_json_object(self.request_path) or {}
        state = load_json_object(self.state_path) or {}
        return max(int(request.get("generation") or 0), int(state.get("generation") or 0)) + 1
