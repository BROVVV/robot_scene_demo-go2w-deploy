#!/usr/bin/env python3
"""Lightweight static scan for suspicious handcrafted prior terms."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


KEYWORDS = [
    "usually",
    "typically",
    "common locations",
    "likely locations",
    "object_location_prior",
    "room_object_prior",
    "phone_table",
    "sofa_phone",
    "bedside_phone",
    "知识库",
    "先验",
    "常见位置",
    "通常在",
]

IGNORED_PARTS = {
    ".git",
    "__pycache__",
    "README.md",
    "tests",
    "docs",
    "examples",
    "outputs",
    "data/scene_kb",
    "app/schemas.py",
    "scripts/audit_handcrafted_priors.py",
    "scripts/query_scene_kb.py",
    "LLM_Generated_Prior_Modification_Plan.md",
}

ALLOWED_LINE_MARKERS = {
    "--disable-handwritten-priors",
    "--disable-static-kb",
    "--prior-audit",
    "禁用人工",
    "先验使用审计",
    "大模型运行时常识先验",
    "无人工先验",
    "Handcrafted prior control",
    "handcrafted_priors_used",
    "static_kb_used",
    "static_object_prompts_used",
    "llm_prior_can_confirm_target",
}


def scan(root: Path) -> dict:
    findings = []
    for path in root.rglob("*"):
        if not path.is_file():
            continue
        rel = path.relative_to(root).as_posix()
        if any(part in rel for part in IGNORED_PARTS):
            continue
        if path.suffix not in {".py", ".json", ".jsonl", ".md", ".txt"}:
            continue
        try:
            text = path.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        for line_no, line in enumerate(text.splitlines(), start=1):
            if any(marker in line for marker in ALLOWED_LINE_MARKERS):
                continue
            lowered = line.lower()
            for keyword in KEYWORDS:
                if keyword.lower() in lowered:
                    findings.append({"path": rel, "line": line_no, "keyword": keyword})
    return {"passed": not findings, "findings": findings}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=".")
    parser.add_argument("--output")
    args = parser.parse_args()
    payload = scan(Path(args.root).resolve())
    text = json.dumps(payload, ensure_ascii=False, indent=2)
    if args.output:
        Path(args.output).write_text(text + "\n", encoding="utf-8")
    else:
        print(text)
    return 0 if payload["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
