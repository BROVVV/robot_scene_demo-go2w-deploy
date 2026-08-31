"""Persistence helpers for video navigation planning artifacts."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def write_json(payload: Any, path: str | Path) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return output


def write_webui_manifest(output_dir: str | Path, paths: dict[str, Path], summary: dict[str, Any]) -> Path:
    manifest = {
        "summary": summary,
        "artifacts": {key: str(path) for key, path in paths.items()},
    }
    return write_json(manifest, Path(output_dir) / "webui_manifest.json")
