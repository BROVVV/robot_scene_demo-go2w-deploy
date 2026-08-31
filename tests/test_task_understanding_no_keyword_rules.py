from __future__ import annotations

from pathlib import Path


def test_intent_parser_does_not_use_keyword_harm_rules() -> None:
    files = [
        Path("app/task_understanding/intent_parser.py"),
        Path("app/task_understanding/llm_task_interpreter.py"),
        Path("app/task_understanding/capability_gate.py"),
    ]

    forbidden_snippets = [
        "打(?!印)",
        '"打" in',
        "'打' in",
        "re.search(r\"打",
        "re.search(r'打",
    ]

    for file in files:
        text = file.read_text(encoding="utf-8")
        for snippet in forbidden_snippets:
            assert snippet not in text
