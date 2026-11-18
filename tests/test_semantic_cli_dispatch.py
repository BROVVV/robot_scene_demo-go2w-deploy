"""The public CLI must route requests and preserve caller replay bounds."""

import json
from unittest.mock import patch

from scripts.go2w import run_semantic_exploration as runner


def test_nav2_cli_dispatches_to_real_runner() -> None:
    with patch.object(runner, "run_go2w", return_value=7) as real:
        with patch.object(runner, "_run_offline", return_value=8) as offline:
            assert runner.main(["--target", "chair", "--backend", "nav2"]) == 7
    real.assert_called_once()
    assert real.call_args.args[0].backend == "nav2"
    offline.assert_not_called()


def test_replay_cli_honors_cycle_budget_and_session_id(tmp_path) -> None:
    replay = tmp_path / "source.jsonl"
    replay.write_text(
        "".join(
            json.dumps({
                "event": "observation",
                "bundle_id": f"frame-{index}",
                "timestamp": float(index),
                "image_ref": f"mock://frame-{index}",
                "target_present": False,
                "target_match": {"target_present": False, "score": 0.0},
                "scene_objects": [],
                "scene_relations": [],
                "sensor_health": {"camera": True},
            }) + "\n"
            for index in (1, 2)
        ),
        encoding="utf-8",
    )
    session_root = tmp_path / "sessions"

    result = runner.main([
        "--target", "不存在的目标",
        "--backend", "mock",
        "--replay", str(replay),
        "--session-id", "bounded-replay",
        "--max-planning-cycles", "2",
        "--max-motion-steps", "0",
        "--session-dir", str(session_root),
        "--output", str(tmp_path / "events.jsonl"),
    ])

    assert result == 3
    summary = json.loads(
        (session_root / "bounded-replay" / "summary.json").read_text(
            encoding="utf-8"
        )
    )
    assert summary["session_id"] == "bounded-replay"
    assert summary["planning_cycles"] == 2
    assert summary["observations"] == 2
    assert summary["finish_reason"] == "MAX_PLANNING_CYCLES_REACHED"
