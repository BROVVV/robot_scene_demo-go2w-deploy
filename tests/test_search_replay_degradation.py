"""缺图/缺相机/缺定位时：不补造数据，但回放仍能出片并明确标注。"""

from __future__ import annotations

from pathlib import Path

import pytest

from app.search_replay.manager import ReplayManager
from app.search_replay.models import ReplayConfig
from app.search_replay.recording import load_recording
from app.search_replay.renderer import ReplayRenderer
from tests.search_replay_fixture import FIXTURE_SOURCE, make_recording

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SESSION_ID = "search_20261021_140000_degraded"


def _config(root: Path) -> ReplayConfig:
    return ReplayConfig(
        width=480, height=270, fps=10, end_hold_seconds=1.0,
        key_pad_seconds=1.0, default_speed=10, speeds=(10,),
        encoder_preset="ultrafast", encoder_crf=30, root=str(root),
    )


@pytest.fixture(scope="module")
def degraded(tmp_path_factory):
    root = tmp_path_factory.mktemp("degraded") / "sessions"
    make_recording(root, session_id=SESSION_ID, seconds=10.0, camera_fps=2.0,
                   map_hz=2.0, map_stall_after=3.0, camera_gap_after=3.0)
    manager = ReplayManager(_config(root), sessions_root=root,
                            project_root=PROJECT_ROOT, source=FIXTURE_SOURCE)
    assert manager.request_render(SESSION_ID, speed=10, force=True)["ok"]
    assert manager.wait_idle(180), "degraded fixture render did not finish"
    yield manager, root
    manager.shutdown()


def test_missing_sources_are_reported_not_fabricated(degraded) -> None:
    _manager, root = degraded
    recording = load_recording(root / SESSION_ID, config=_config(root))
    renderer = ReplayRenderer(recording, _config(root), speed=10,
                              width=480, height=270, fps=10)
    renderer.prepare()
    # 采集在 3 s 后断流，回放到 7 s 时三件事必须同时成立：
    after = min(pose.t for pose in recording.poses) + 9.0
    assert renderer._frame_at(after) is None, "断流后不该凭空造出相机帧"
    assert renderer._map_at(after) is not None, "最后一帧地图仍然可用"
    stalled = renderer._map_stalled(after)
    assert stalled is not None and stalled > 5.0
    assert renderer.pose_at(after) is not None, "位姿仍在，轨迹照常"


def test_degraded_recording_still_produces_a_playable_video(degraded) -> None:
    manager, _root = degraded
    status = manager.status(SESSION_ID)
    assert status["status"] in {"READY", "PARTIAL"}, status
    assert status["playable"] is True, status
    assert status["duration"] > 0
