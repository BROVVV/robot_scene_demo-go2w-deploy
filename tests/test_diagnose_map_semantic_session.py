import importlib.util
import json
from pathlib import Path

import numpy as np


SCRIPT = Path(__file__).parents[1] / "scripts/go2w/diagnose_map_semantic_session.py"
SPEC = importlib.util.spec_from_file_location("diagnose_map_semantic_session", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_diagnostic_is_read_only_and_counts_recorded_evidence(tmp_path):
    session = tmp_path / "session"
    maps = session / "replay" / "maps"
    maps.mkdir(parents=True)
    grid = np.array([[-1, 0], [100, 0]], dtype=np.int8)
    np.savez_compressed(maps / "abc.npz", grid=grid)
    (session / "replay" / "timeline.jsonl").write_text(
        json.dumps({"kind": "map", "t": 1.0, "frame_id": "map", "hash": "abc",
                    "resolution": 0.5, "origin": [1.0, 2.0], "new_content": True}) + "\n"
        + json.dumps({"kind": "localization", "t": 2.0, "valid": False}) + "\n",
        encoding="utf-8",
    )
    (session / "replay" / "poses.jsonl").write_text("", encoding="utf-8")
    (session / "replay" / "manifest.json").write_text(
        json.dumps({"session_id": "s1", "backend": "nav2"}), encoding="utf-8"
    )
    (session / "events.jsonl").write_text(
        json.dumps({"event": "localization_lost"}) + "\n", encoding="utf-8"
    )
    (session / "webui_graph.json").write_text(
        json.dumps({"objects": [{"status": "TENTATIVE", "map_xyz": [1, 2, 3]}],
                    "places": [], "frontiers": []}), encoding="utf-8"
    )
    (session / "webui_state.json").write_text("{}", encoding="utf-8")
    before = {path: path.stat().st_mtime_ns for path in session.rglob("*") if path.is_file()}
    result = MODULE.diagnose(session)
    after = {path: path.stat().st_mtime_ns for path in session.rglob("*") if path.is_file()}
    assert before == after
    assert result["read_only"] is True
    assert result["map"]["final"]["unknown"] == 1
    assert result["map"]["final"]["free"] == 2
    assert result["map"]["final"]["occupied"] == 1
    assert result["localization"]["invalid_samples"] == 1
    assert result["semantics"]["objects_with_map_xyz"] == 1


def test_cli_refuses_to_write_inside_source_session(tmp_path):
    session = tmp_path / "session"
    session.mkdir()
    try:
        MODULE.main([str(session), "--output", str(session / "report.json")])
    except SystemExit as exc:
        assert "refusing" in str(exc)
    else:
        raise AssertionError("expected source-session write refusal")
