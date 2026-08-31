"""Durable versioned bundle writer with an atomic latest pointer."""

from __future__ import annotations

import json
import os
import re
import shutil
import tempfile
from pathlib import Path


class AtomicBundleWriter:
    def __init__(
        self, spool_root: str | Path, *, max_bundles_per_session: int = 30
    ) -> None:
        self.root = Path(spool_root).resolve()
        self.bundles = self.root / "bundles"
        self.bundles.mkdir(parents=True, exist_ok=True)
        self.max_bundles_per_session = int(max_bundles_per_session)
        if self.max_bundles_per_session < 1:
            raise ValueError("max_bundles_per_session must be positive")

    def write(self, image_jpeg: bytes, payload: dict) -> Path:
        if payload.get("schema_version") != "1.0":
            raise ValueError("frame bundle schema_version must be 1.0")
        if not image_jpeg.startswith(b"\xff\xd8"):
            raise ValueError("image payload is not JPEG")
        session = str(payload["session_id"])
        if not re.fullmatch(r"[A-Za-z0-9_.-]{1,80}", session):
            raise ValueError("session_id contains unsafe characters")
        frame_id = int(payload["frame_id"])
        name = f"{session}-{frame_id:012d}"
        destination = self.bundles / name
        if destination.exists():
            raise FileExistsError(f"bundle already exists: {destination}")
        temporary = Path(tempfile.mkdtemp(prefix=".bundle-", dir=self.bundles))
        try:
            self._write_file(temporary / "image.jpg", image_jpeg)
            self._write_file(
                temporary / "frame_bundle.json",
                (
                    json.dumps(
                        payload,
                        ensure_ascii=False,
                        indent=2,
                        allow_nan=False,
                    )
                    + "\n"
                ).encode("utf-8"),
            )
            self._write_file(temporary / "READY", b"ready\n")
            os.replace(temporary, destination)
            self._fsync_directory(self.bundles)
            relative_target = Path("bundles") / name
            next_link = self.root / f".latest-{os.getpid()}-{frame_id}"
            os.symlink(relative_target, next_link)
            os.replace(next_link, self.root / "latest")
            self._fsync_directory(self.root)
            self._prune_session(session, keep=self.max_bundles_per_session)
            return destination
        except Exception:
            if temporary.exists():
                shutil.rmtree(temporary)
            raise

    def _prune_session(self, session: str, *, keep: int) -> None:
        """Bound the transient spool without touching another session."""
        pattern = re.compile(rf"{re.escape(session)}-[0-9]{{12}}")
        candidates = sorted(
            path
            for path in self.bundles.iterdir()
            if path.is_dir() and pattern.fullmatch(path.name)
        )
        for obsolete in candidates[:-keep]:
            shutil.rmtree(obsolete)
        if len(candidates) > keep:
            self._fsync_directory(self.bundles)

    @staticmethod
    def _write_file(path: Path, data: bytes) -> None:
        descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        try:
            with os.fdopen(descriptor, "wb") as stream:
                stream.write(data)
                stream.flush()
                os.fsync(stream.fileno())
        except Exception:
            try:
                os.close(descriptor)
            except OSError:
                pass
            raise

    @staticmethod
    def _fsync_directory(path: Path) -> None:
        descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
