"""Helpers for extracting JSON objects from model text output."""

from __future__ import annotations

import json
import re
from typing import Any


_JSON_CODE_BLOCK_RE = re.compile(r"```(?:json|JSON)?\s*(.*?)\s*```", re.DOTALL)


def extract_json_from_text(text: str) -> dict[str, Any]:
    """Extract a JSON object from plain text or Markdown-wrapped model output."""

    if not isinstance(text, str) or text.strip() == "":
        raise ValueError("Cannot parse JSON from empty text.")

    raw_text = text.strip()
    candidates = [raw_text]
    candidates.extend(match.strip() for match in _JSON_CODE_BLOCK_RE.findall(raw_text))

    embedded = _extract_first_json_object(raw_text)
    if embedded is not None:
        candidates.append(embedded)

    errors: list[str] = []
    seen: set[str] = set()
    for candidate in candidates:
        if candidate in seen:
            continue
        seen.add(candidate)

        try:
            parsed = json.loads(candidate)
        except json.JSONDecodeError as exc:
            errors.append(str(exc))
            continue

        if not isinstance(parsed, dict):
            raise ValueError(
                "Parsed JSON must be an object. Raw text:\n"
                f"{raw_text}"
            )
        return parsed

    detail = errors[-1] if errors else "no JSON object candidate found"
    raise ValueError(
        "Failed to parse JSON object from text. "
        f"Last parser error: {detail}. Raw text:\n{raw_text}"
    )


def _extract_first_json_object(text: str) -> str | None:
    start = text.find("{")
    if start == -1:
        return None

    depth = 0
    in_string = False
    escape = False

    for index in range(start, len(text)):
        char = text[index]

        if in_string:
            if escape:
                escape = False
            elif char == "\\":
                escape = True
            elif char == '"':
                in_string = False
            continue

        if char == '"':
            in_string = True
        elif char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return text[start : index + 1]

    return None
