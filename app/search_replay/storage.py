"""Atomic storage, file locking and path-ownership helpers for replay data.

所有写入都是"临时文件 + fsync + os.replace"，或者追加式 JSONL；未完成的
MP4 永远停在 ``*.partial.mp4``，只有通过 ffprobe/抽帧校验后才原子改名为
``replay.mp4``。读取端把损坏的末行计数而不静默丢弃，坏行也不会把别的
会话接进来。
"""

from __future__ import annotations

import errno
import fcntl
import hashlib
import json
import os
import re
import shutil
import threading
import time
from pathlib import Path
from typing import Any, Iterable, Iterator

from app.search_replay.models import REPLAY_DIRNAME, json_safe

SESSION_ID_RE = re.compile(r"^search_[A-Za-z0-9_]+$")
RECORDING_ID_RE = re.compile(r"^rec_[A-Za-z0-9]+$")
ARTIFACT_NAME_RE = re.compile(r"^[A-Za-z0-9_.-]+$")

MANIFEST = "manifest.json"
CAPTURE_STATUS = "capture_status.json"
JOB = "job.json"
TIMELINE = "timeline.jsonl"
POSES = "poses.jsonl"
FRAMES = "frames.jsonl"
EVENTS = "events.jsonl"
RENDER_LOG = "render.log"
LOCK = ".lock"
MAPS_DIR = "maps"
FRAMES_DIR = "frames"
ARTIFACTS_DIR = "artifacts"


def valid_session_id(session_id: str) -> bool:
    return bool(session_id) and bool(SESSION_ID_RE.match(session_id))


def valid_recording_id(recording_id: str) -> bool:
    return bool(recording_id) and bool(RECORDING_ID_RE.match(recording_id))


def replay_dir(session_dir: str | Path) -> Path:
    return Path(session_dir) / REPLAY_DIRNAME


def ensure_replay_layout(session_dir: str | Path) -> Path:
    directory = replay_dir(session_dir)
    for sub in (directory, directory / MAPS_DIR, directory / FRAMES_DIR,
                directory / ARTIFACTS_DIR):
        sub.mkdir(parents=True, exist_ok=True)
    return directory


def resolve_session_dir(root: str | Path, session_id: str) -> Path | None:
    """Resolve a session id to its directory without allowing escapes.

    Rejects ``..``, URL-encoded traversal that decoded into a separator, and
    symlinks that point outside the archive root.  Returns ``None`` when the
    id is invalid or the directory does not exist.
    """
    if not valid_session_id(session_id):
        return None
    root_path = Path(root).resolve()
    candidate = (root_path / session_id)
    try:
        resolved = candidate.resolve(strict=True)
    except (OSError, RuntimeError):
        return None
    if not resolved.is_dir():
        return None
    if not _is_within(resolved, root_path):
        return None
    return resolved


def resolve_artifact(
    session_dir: Path, *parts: str, must_exist: bool = True
) -> Path | None:
    """Resolve a replay artifact path and prove it stays inside the session.

    ``session_dir`` is the ownership root and must be the *unresolved* real
    path (``<sessions_root>/<session_id>`` or its ``replay`` directory).
    Symlinks are rejected at every component, including the root itself: a
    ``replay -> /somewhere/else`` link must not be accepted as a new root,
    otherwise ``resolve`` would happily prove the escape is "inside" the
    attacker-chosen directory.
    """
    root = Path(session_dir)
    if root.is_symlink():
        return None
    session_root = root.resolve()
    candidate = session_root
    for part in parts:
        text = str(part)
        if not text or text in {".", ".."} or "/" in text or "\\" in text:
            return None
        candidate = candidate / text
        if candidate.is_symlink():
            return None
    try:
        resolved = candidate.resolve(strict=must_exist)
    except (OSError, RuntimeError):
        return None
    if not _is_within(resolved, session_root):
        return None
    return resolved


def _is_within(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def _temporary_for(target: Path) -> Path:
    """Unique temp name: two threads may write the same file legitimately
    (the request thread writes job.json status while the render thread writes
    progress).  A shared ``.tmp`` name made one of them fail the rename."""
    suffix = f".{os.getpid()}.{threading.get_ident()}.{time.monotonic_ns()}.tmp"
    return target.with_name(target.name + suffix)


def atomic_write_json(path: str | Path, value: dict[str, Any]) -> None:
    atomic_write_text(
        path, json.dumps(json_safe(value), ensure_ascii=False,
                         separators=(",", ":"), allow_nan=False)
    )


def atomic_write_text(path: str | Path, text: str) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = _temporary_for(target)
    with temporary.open("w", encoding="utf-8") as handle:
        handle.write(text)
        handle.flush()
        try:
            os.fsync(handle.fileno())
        except OSError:
            pass
    os.replace(temporary, target)


def atomic_write_bytes(path: str | Path, data: bytes) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = _temporary_for(target)
    with temporary.open("wb") as handle:
        handle.write(data)
        handle.flush()
        try:
            os.fsync(handle.fileno())
        except OSError:
            pass
    os.replace(temporary, target)


def append_jsonl(path: str | Path, rows: Iterable[dict[str, Any]]) -> int:
    """Append rows, skipping (and counting) non-finite numbers."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    with target.open("a", encoding="utf-8") as handle:
        for row in rows:
            try:
                line = json.dumps(json_safe(row), ensure_ascii=False,
                                  allow_nan=False, separators=(",", ":"))
            except (TypeError, ValueError):
                continue
            handle.write(line + "\n")
            count += 1
        handle.flush()
    return count


def read_json(path: str | Path) -> dict[str, Any] | None:
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def iter_jsonl(path: str | Path) -> tuple[list[dict[str, Any]], int]:
    """Read a JSONL file; return (rows, bad_line_count).

    A truncated final line (process killed mid-write) is recoverable: it is
    counted as bad and every complete line before it is kept.
    """
    rows: list[dict[str, Any]] = []
    bad = 0
    target = Path(path)
    if not target.is_file():
        return rows, 0
    try:
        with target.open("r", encoding="utf-8", errors="replace") as handle:
            for line in handle:
                line = line.strip()
                if not line:
                    continue
                try:
                    value = json.loads(line)
                except json.JSONDecodeError:
                    bad += 1
                    continue
                if isinstance(value, dict):
                    rows.append(value)
                else:
                    bad += 1
    except OSError:
        return rows, bad + 1
    return rows, bad


def sha256_file(path: str | Path, chunk_size: int = 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        while True:
            chunk = handle.read(chunk_size)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def dir_size_bytes(path: str | Path) -> int:
    total = 0
    root = Path(path)
    if not root.exists():
        return 0
    for item in root.rglob("*"):
        try:
            if item.is_file() and not item.is_symlink():
                total += item.stat().st_size
        except OSError:
            continue
    return total


def free_bytes(path: str | Path) -> int:
    try:
        usage = shutil.disk_usage(str(path))
    except OSError:
        return 0
    return int(usage.free)


class FileLock:
    """``flock`` based lock; never blocks the caller unless asked to.

    Two uses:

    * ``replay/.lock`` — held exclusively by the process that is writing a
      session's replay (collector or renderer), held shared by HTTP streams.
      The archive prune checks it and never deletes a session someone uses.
    * ``<root>/encoder.lock`` — one encoder at a time globally.
    """

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self._handle = None
        self._shared = False

    def acquire(self, *, shared: bool = False, timeout: float = 0.0) -> bool:
        if self._handle is not None:
            return True
        self.path.parent.mkdir(parents=True, exist_ok=True)
        handle = self.path.open("a+")
        mode = fcntl.LOCK_SH if shared else fcntl.LOCK_EX
        deadline = time.monotonic() + max(0.0, timeout)
        while True:
            try:
                fcntl.flock(handle.fileno(), mode | fcntl.LOCK_NB)
                self._handle = handle
                self._shared = shared
                return True
            except OSError as exc:
                if exc.errno not in (errno.EACCES, errno.EAGAIN):
                    handle.close()
                    return False
                if time.monotonic() >= deadline:
                    handle.close()
                    return False
                time.sleep(0.05)

    def release(self) -> None:
        handle, self._handle = self._handle, None
        if handle is None:
            return
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        except OSError:
            pass
        handle.close()

    def __enter__(self) -> "FileLock":
        return self

    def __exit__(self, *exc_info: Any) -> None:
        self.release()


def lock_is_held(path: str | Path) -> bool:
    """True when another process holds a lock on this file.

    The check itself takes an exclusive non-blocking lock; success means
    nobody else holds it.  Used by prune protection and by the render
    single-flight check.
    """
    target = Path(path)
    if not target.exists():
        return False
    try:
        handle = target.open("a+")
    except OSError:
        return False
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        return True
    finally:
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        except OSError:
            pass
        handle.close()
    return False


def safe_artifact_name(name: str) -> bool:
    return bool(name) and bool(ARTIFACT_NAME_RE.match(name)) and ".." not in name
