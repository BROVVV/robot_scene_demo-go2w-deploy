"""Integrity binding for a saved map/semantic/topology world bundle."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import time
from typing import Any

from app.spatial.world_state_snapshot import WorldRevisions


SCHEMA_VERSION = "go2w_world_bundle_v1"


def write_world_manifest(
    root: str | Path,
    *,
    revisions: WorldRevisions,
    components: dict[str, str | Path],
    mutable_components: dict[str, str | Path] | None = None,
    metadata: dict[str, Any] | None = None,
) -> Path:
    bundle_root = Path(root).resolve()
    bundle_root.mkdir(parents=True, exist_ok=True)
    files: dict[str, Any] = {}
    for name, raw_path in sorted(components.items()):
        path = Path(raw_path).resolve()
        if bundle_root not in path.parents:
            raise ValueError(f"component must be inside bundle root: {path}")
        if not path.is_file():
            raise FileNotFoundError(path)
        files[str(name)] = {
            "path": str(path.relative_to(bundle_root)),
            "sha256": _sha256(path),
            "bytes": path.stat().st_size,
        }
    mutable_files: dict[str, Any] = {}
    for name, raw_path in sorted((mutable_components or {}).items()):
        if name in files:
            raise ValueError(f"component cannot be both immutable and mutable: {name}")
        path = Path(raw_path).resolve()
        if bundle_root not in path.parents:
            raise ValueError(f"component must be inside bundle root: {path}")
        if not path.is_file():
            raise FileNotFoundError(path)
        # Replay/render manifests continue changing after the search worker
        # exits (collector finalization, video status and speed rerenders).
        # Record their location and mutability explicitly, but never pretend a
        # transient digest is part of the immutable world-state integrity set.
        mutable_files[str(name)] = {
            "path": str(path.relative_to(bundle_root)),
            "integrity_scope": "external_mutable",
        }
    value = {
        "schema_version": SCHEMA_VERSION,
        "created_at": time.time(),
        **revisions.to_dict(),
        "components": files,
        "mutable_components": mutable_files,
        "metadata": metadata or {},
        "motion_authorized": False,
    }
    target = bundle_root / "world_manifest.json"
    temporary = bundle_root / ".world_manifest.json.tmp"
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(target)
    return target


def verify_world_manifest(root: str | Path) -> dict[str, Any]:
    bundle_root = Path(root).resolve()
    manifest_path = bundle_root / "world_manifest.json"
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError) as exc:
        return {"valid": False, "errors": [f"manifest_unavailable:{exc}"]}
    errors: list[str] = []
    if manifest.get("schema_version") != SCHEMA_VERSION:
        errors.append("schema_version_mismatch")
    for required in ("map_id", "map_epoch", "frame_id", "sensor_profile_id"):
        if not str(manifest.get(required) or ""):
            errors.append(f"missing_identity:{required}")
    for name, item in (manifest.get("components") or {}).items():
        relative = Path(str(item.get("path") or ""))
        path = (bundle_root / relative).resolve()
        if bundle_root not in path.parents:
            errors.append(f"component_outside_bundle:{name}")
        elif not path.is_file():
            errors.append(f"component_missing:{name}")
        elif _sha256(path) != item.get("sha256"):
            errors.append(f"component_hash_mismatch:{name}")
    return {"valid": not errors, "errors": errors, "manifest": manifest}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
