"""Compatibility wrapper for LLM-first task understanding."""

from __future__ import annotations

from dataclasses import asdict
from typing import Any

from app.task_understanding.llm_task_interpreter import LLMTaskInterpreter, unavailable_task
from app.task_understanding.schemas import ParsedTask


def parse_natural_language_task(
    task_text: str,
    use_llm: bool = True,
    llm_client: Any | None = None,
    enable_verifier: bool = True,
) -> ParsedTask:
    """Deprecated entry point kept for callers that have not migrated yet."""
    if not use_llm and llm_client is None:
        return unavailable_task(
            task_text,
            reason="llm_task_understanding_unavailable",
        )
    return LLMTaskInterpreter(
        llm_client=llm_client,
        enable_verifier=enable_verifier,
    ).parse(task_text)


def parsed_task_debug_dict(task: ParsedTask) -> dict[str, Any]:
    return asdict(task)
