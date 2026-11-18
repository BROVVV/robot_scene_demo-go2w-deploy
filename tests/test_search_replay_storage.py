"""Storage primitives for the search replay package.

These are the pieces the acceptance plan singles out because a failure here
looks exactly like a *content* bug later: two writers racing on the same
``job.json``/``manifest.json``, an artifact path escaping the replay
directory, or a lock that claims to be held when nobody holds it.
"""

from __future__ import annotations

import json
import threading
from pathlib import Path

from app.search_replay import storage


def test_atomic_json_survives_concurrent_writers(tmp_path: Path) -> None:
    target = tmp_path / "job.json"
    errors: list[BaseException] = []

    def writer(index: int) -> None:
        try:
            for _ in range(30):
                storage.atomic_write_json(target, {"index": index, "pad": "x" * 512})
        except BaseException as exc:  # noqa: BLE001 - surfaces in the assert
            errors.append(exc)

    threads = [threading.Thread(target=writer, args=(index,)) for index in range(8)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert not errors, errors
    payload = json.loads(target.read_text(encoding="utf-8"))
    assert payload["index"] in range(8)
    # Unique temp names mean no writer can leave a half-written sibling behind.
    assert list(tmp_path.glob("*.tmp")) == []


def test_resolve_artifact_rejects_traversal(tmp_path: Path) -> None:
    replay = tmp_path / "session" / "replay"
    artifacts = replay / "artifacts"
    artifacts.mkdir(parents=True)
    (artifacts / "replay.mp4").write_bytes(b"x")
    (tmp_path / "secret.txt").write_text("outside", encoding="utf-8")
    assert storage.resolve_artifact(replay, "artifacts", "replay.mp4") is not None
    assert storage.resolve_artifact(replay, "..", "secret.txt") is None
    assert storage.resolve_artifact(replay, "artifacts", "missing.mp4") is None
    assert storage.resolve_artifact(
        replay, "artifacts", "missing.mp4", must_exist=False
    ) == artifacts / "missing.mp4"


def test_session_and_recording_ids_are_validated() -> None:
    assert storage.valid_session_id("search_20261021_120000_abcdef")
    assert not storage.valid_session_id("../etc/passwd")
    assert not storage.valid_session_id("search_../x")
    assert storage.valid_recording_id("rec_0123456789ab")
    assert not storage.valid_recording_id("rec_../x")


def test_file_lock_excludes_other_processes(tmp_path: Path) -> None:
    path = tmp_path / "replay.lock"
    lock = storage.FileLock(path)
    assert lock.acquire(timeout=0.0)
    try:
        # fcntl locks are per-process, so the cross-process probe is the one
        # that matters for crash recovery (a dead Web process must release it).
        import subprocess
        import sys
        probe = (
            "import sys; sys.path.insert(0, %r);"
            "from app.search_replay.storage import FileLock;"
            "print(FileLock(%r).acquire(timeout=0.0))"
            % (str(Path(__file__).resolve().parents[1]), str(path))
        )
        result = subprocess.run([sys.executable, "-c", probe],
                                capture_output=True, text=True, timeout=30)
        assert result.stdout.strip() == "False", result.stderr
    finally:
        lock.release()
    assert storage.lock_is_held(path) is False


def test_append_jsonl_reports_bad_lines_without_losing_good_ones(
    tmp_path: Path,
) -> None:
    path = tmp_path / "events.jsonl"
    storage.append_jsonl(path, [{"a": 1}, {"a": 2}])
    with path.open("a", encoding="utf-8") as handle:
        handle.write("{not json\n")
    rows, bad = storage.iter_jsonl(path)
    assert [row["a"] for row in rows] == [1, 2]
    assert bad == 1
