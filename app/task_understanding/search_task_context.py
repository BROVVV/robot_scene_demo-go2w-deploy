"""Task context shared by the WebUI, worker and live explorer.

The raw sentence is retained for explainability, but navigation code receives
this structured object instead of treating user text as a target label.
"""

from __future__ import annotations

import time
import re
from dataclasses import dataclass, field
from typing import Any
from uuid import uuid4


@dataclass(frozen=True)
class SearchTaskContext:
    task_id: str
    raw_text: str
    intent: str
    canonical_target: str
    target_attributes: dict[str, Any] = field(default_factory=dict)
    target_relations: list[dict[str, Any] | str] = field(default_factory=list)
    constraints: list[str] = field(default_factory=list)
    subtasks: list[dict[str, Any]] = field(default_factory=list)
    executable: bool = False
    rejection_reason: str | None = None
    parsed_task: dict[str, Any] = field(default_factory=dict)
    created_at: float = field(default_factory=time.time)

    @classmethod
    def from_pipeline_result(
        cls,
        result: Any,
        *,
        task_id: str | None = None,
        raw_text: str | None = None,
    ) -> "SearchTaskContext":
        parsed = getattr(result, "parsed_task", None)
        actionability = getattr(result, "actionability", None)
        navigation = getattr(result, "navigation_task", None)
        raw = str(raw_text or getattr(parsed, "raw_task", "") or "").strip()
        target = getattr(navigation, "target", None) or getattr(parsed, "target", None)
        canonical = str(
            getattr(target, "name_zh", "")
            or getattr(target, "name_en", "")
            or getattr(navigation, "navigation_goal", "")
            or raw
        ).strip()
        intent = _value(
            getattr(navigation, "intent", None)
            or getattr(parsed, "primary_intent", "unknown")
        )
        attrs = {
            "category": _value(getattr(target, "category", "unknown")),
            "attributes": list(getattr(target, "attributes", []) or []),
        }
        relations = _jsonable(list(getattr(target, "relations", []) or []))
        subtasks = [
            _jsonable(item)
            for item in list(getattr(parsed, "requested_subtasks", []) or [])
        ]
        constraints = [
            str(item)
            for item in (getattr(actionability, "execution_constraints", {}) or {}).keys()
        ]
        executable = bool(getattr(navigation, "executable", False))
        blocked = list(getattr(actionability, "blocked_reasons", []) or [])
        rejection = None if executable else (
            "；".join(blocked)
            or getattr(navigation, "user_feedback_zh", None)
            or "任务不可执行"
        )
        return cls(
            task_id=task_id or f"task_{uuid4().hex[:12]}",
            raw_text=raw,
            intent=intent,
            canonical_target=canonical,
            target_attributes=attrs,
            target_relations=relations,
            constraints=constraints,
            subtasks=subtasks,
            executable=executable,
            rejection_reason=rejection,
            parsed_task=_jsonable(parsed) if parsed is not None else {},
        )

    @classmethod
    def mock_fallback(
        cls, raw_text: str, *, task_id: str | None = None
    ) -> "SearchTaskContext":
        """Explicit deterministic fallback for offline mock UI development."""
        text = str(raw_text or "").strip()
        canonical = _mock_canonical_target(text)
        return cls(
            task_id=task_id or f"task_{uuid4().hex[:12]}",
            raw_text=text,
            intent="search_semantic_target",
            canonical_target=canonical,
            target_attributes={"category": "semantic_target", "attributes": []},
            executable=bool(text),
            parsed_task={
                "raw_task": text,
                "canonical_target": canonical,
                "parser_source": "offline_mock_fallback",
            },
        )

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "SearchTaskContext":
        return cls(
            task_id=str(value.get("task_id") or f"task_{uuid4().hex[:12]}"),
            raw_text=str(value.get("raw_text") or value.get("task_text") or ""),
            intent=str(value.get("intent") or "unknown"),
            canonical_target=str(value.get("canonical_target") or value.get("target") or ""),
            target_attributes=dict(value.get("target_attributes") or {}),
            target_relations=list(value.get("target_relations") or []),
            constraints=list(value.get("constraints") or []),
            subtasks=list(value.get("subtasks") or []),
            executable=bool(value.get("executable", False)),
            rejection_reason=value.get("rejection_reason"),
            parsed_task=dict(value.get("parsed_task") or {}),
            created_at=float(value.get("created_at", time.time())),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "task_id": self.task_id,
            "raw_text": self.raw_text,
            "intent": self.intent,
            "canonical_target": self.canonical_target,
            "target_attributes": _jsonable(self.target_attributes),
            "target_relations": _jsonable(self.target_relations),
            "constraints": _jsonable(self.constraints),
            "subtasks": _jsonable(self.subtasks),
            "executable": self.executable,
            "rejection_reason": self.rejection_reason,
            "parsed_task": dict(self.parsed_task),
            "created_at": self.created_at,
        }


def _value(value: Any) -> str:
    return str(getattr(value, "value", value) or "unknown")


def _mock_canonical_target(text: str) -> str:
    """Extract a stable target for deterministic mock sessions only."""
    if not text:
        return ""
    first_clause = re.split(r"(?:然后|并且|接着|随后|[，,；;。！？])", text, maxsplit=1)[0]
    target = re.sub(
        r"^(?:请帮我|我需要|我想|麻烦|帮我|请)?\s*(?:找到|查找|搜索|寻找|定位|看看|观察|去找)\s*",
        "",
        first_clause,
    ).strip()
    return target or text


def _jsonable(value: Any) -> Any:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    enum_value = getattr(value, "value", None)
    if enum_value is not None and enum_value is not value:
        return _jsonable(enum_value)
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_jsonable(item) for item in value]
    if hasattr(value, "to_dict"):
        return _jsonable(value.to_dict())
    if hasattr(value, "__dataclass_fields__"):
        return {key: _jsonable(getattr(value, key)) for key in value.__dataclass_fields__}
    return str(value)
