"""Replay rendering acceptance: real ffmpeg encode, palette, fault calibration.

The recorder is synthetic on purpose (no motion permission), but everything
downstream of the recording package is the production path: the same
``ReplayManager`` → ``ReplayRenderer`` → ``VideoEncoder`` → HTTP artifact.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from app.search_replay.encoder import probe_video, verify_video
from app.search_replay.manager import ReplayManager
from app.search_replay.models import ReplayConfig
from app.search_replay.recording import load_recording
from app.search_replay.renderer import (MAP_FREE, MAP_OCCUPIED, MAP_UNKNOWN,
                                        ReplayRenderer, _yaw_from_quaternion)
from tests.search_replay_fixture import FIXTURE_SOURCE, make_recording

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SESSION_ID = "search_20261021_120000_fixture"


def _config(root: Path, **overrides) -> ReplayConfig:
    values = dict(
        width=480, height=270, fps=10, end_hold_seconds=1.0,
        key_pad_seconds=1.0, default_speed=10, speeds=(1, 10),
        encoder_preset="ultrafast", encoder_crf=28, root=str(root),
    )
    values.update(overrides)
    return ReplayConfig(**values)


@pytest.fixture(scope="module")
def rendered(tmp_path_factory):
    root = tmp_path_factory.mktemp("replay_render") / "sessions"
    manager = ReplayManager(
        _config(root), sessions_root=root, project_root=PROJECT_ROOT,
        source=FIXTURE_SOURCE,
    )
    make_recording(root, session_id=SESSION_ID, seconds=4.0, camera_fps=1.0)
    result = manager.request_render(SESSION_ID, speed=10, force=True)
    assert result.get("ok"), result
    assert manager.wait_idle(180), "render did not finish"
    yield manager, root
    manager.shutdown()


def test_render_produces_valid_mp4_and_artifacts(rendered) -> None:
    manager, root = rendered
    status = manager.status(SESSION_ID)
    assert status["status"] == "READY"
    assert status["playable"] is True
    assert status["width"] == 480 and status["height"] == 270
    assert status["fps"] == 10
    artifacts = root / SESSION_ID / "replay" / "artifacts"
    for name in ("replay.mp4", "final_map.png", "cover.jpg"):
        assert (artifacts / name).stat().st_size > 0, name
    assert status["artifacts"]["replay.mp4"]["sha256"]
    probe = probe_video(artifacts / "replay.mp4")
    assert probe["ok"] and probe["codec"] == "h264"
    assert probe["pix_fmt"] == "yuv420p"
    # first / middle / last frames must actually decode (not just a header).
    for offset in (0.05, probe["duration"] / 2, probe["duration"] - 0.2):
        result = subprocess.run(
            [shutil.which("ffmpeg") or "ffmpeg", "-v", "error", "-ss",
             f"{offset:.3f}", "-i", str(artifacts / "replay.mp4"),
             "-frames:v", "1", "-f", "null", "-"],
            capture_output=True, text=True, timeout=60,
        )
        assert result.returncode == 0, result.stderr


def test_final_map_keeps_unknown_free_and_occupied_distinct(rendered) -> None:
    _manager, root = rendered
    from PIL import Image

    path = root / SESSION_ID / "replay" / "artifacts" / "final_map.png"
    with Image.open(path) as image:
        rgb = np.asarray(image.convert("RGB"))
    counts = {
        "unknown": int(np.all(rgb == np.array(MAP_UNKNOWN, dtype=np.uint8),
                              axis=-1).sum()),
        "free": int(np.all(rgb == np.array(MAP_FREE, dtype=np.uint8),
                           axis=-1).sum()),
        "occupied": int(np.all(rgb == np.array(MAP_OCCUPIED, dtype=np.uint8),
                               axis=-1).sum()),
    }
    # Fault calibration: mapping -1 to white (the "unknown looks free" bug)
    # would zero the unknown count while free explodes.  Both must exist and
    # they must not be the same colour.
    assert counts["unknown"] > 1000, counts
    assert counts["free"] > 1000, counts
    assert counts["occupied"] > 50, counts
    assert len({MAP_UNKNOWN, MAP_FREE, MAP_OCCUPIED}) == 3


def test_truncated_artifact_is_not_reported_playable(rendered) -> None:
    manager, root = rendered
    video = root / SESSION_ID / "replay" / "artifacts" / "replay.mp4"
    original = video.read_bytes()
    try:
        video.write_bytes(original[: len(original) // 2])
        ok, reason, _probe = verify_video(
            video, width=480, height=270, fps=10,
            duration=manager._existing_job(root / SESSION_ID, 10).get("duration"),
        )
        assert ok is False and reason
        status = manager.status(SESSION_ID)
        assert status["playable"] is False
        assert any("无法解码" in item for item in status["reasons"]), status
    finally:
        video.write_bytes(original)


def test_recovery_rerenders_when_published_video_is_corrupt(rendered) -> None:
    manager, root = rendered
    video = root / SESSION_ID / "replay" / "artifacts" / "replay.mp4"
    original = video.read_bytes()
    try:
        video.write_bytes(original[: len(original) // 3])
        manager._housekeeping_once()
        assert manager.wait_idle(180)
        status = manager.status(SESSION_ID)
        assert status["playable"] is True, status
    finally:
        # The re-render replaced the file; only restore if it did not.
        if video.read_bytes() == original[: len(original) // 3]:
            video.write_bytes(original)


def test_origin_yaw_changes_the_drawn_map(rendered) -> None:
    """Ignoring the map origin yaw must visibly change the raster.

    A renderer that ignores ``origin[3:7]`` draws the same picture for a
    rotated map, which would put every obstacle in the wrong place.
    """
    import dataclasses

    _manager, root = rendered
    recording = load_recording(root / SESSION_ID, config=_config(root))
    renderer = ReplayRenderer(recording, _config(root), speed=10,
                              width=480, height=270, fps=10)
    renderer.prepare()
    snapshot = renderer.recording.maps[0]
    panel = renderer.map_panel
    viewport = renderer.viewports[renderer.trajectory.segments[0].segment_id]
    flat = renderer._build_map_layer(snapshot, panel, viewport)
    rotated = renderer._build_map_layer(
        dataclasses.replace(snapshot, origin=(snapshot.origin[0],
                                              snapshot.origin[1], 0.0,
                                              *[0.0, 0.0,
                                                np.sin(np.pi / 4),
                                                np.cos(np.pi / 4)])),
        panel, viewport,
    )
    assert flat.shape == rotated.shape
    difference = np.abs(flat.astype(np.int16) - rotated.astype(np.int16)).mean()
    assert difference > 1.0, difference
    assert abs(_yaw_from_quaternion((0, 0, np.sin(np.pi / 4),
                                     np.cos(np.pi / 4))) - np.pi / 2) < 1e-6


def test_invalid_span_splits_trajectory_instead_of_bridging(rendered) -> None:
    _manager, root = rendered
    sid = "search_20261021_120100_fixture"
    make_recording(root, session_id=sid, seconds=7.0, camera_fps=1.0,
                   with_invalid_span=True)
    recording = load_recording(root / sid, config=_config(root))
    trajectory = recording.trajectory()
    assert len(trajectory.segments) >= 2
    first, second = trajectory.segments[0], trajectory.segments[1]
    assert first.end_t < second.start_t
    # No points from the first segment may appear after the second starts;
    # a bridging line would also drag the distance across the gap.
    assert not [point for point in first.points if point.t > second.start_t]
    assert second.start_t - first.end_t >= 0


def test_negative_origin_and_coarse_resolution_place_the_cell(rendered,
                                                              tmp_path) -> None:
    """负 origin、0.5 m 分辨率的地图必须落在正确世界坐标上。"""
    import dataclasses

    from app.search_replay.trajectory import Viewport

    _manager, root = rendered
    recording = load_recording(root / SESSION_ID, config=_config(root))
    renderer = ReplayRenderer(recording, _config(root), speed=10,
                              width=480, height=270, fps=10)
    renderer.prepare()
    width, height, resolution = 40, 20, 0.5
    grid = np.full((height, width), -1, dtype=np.int8)
    grid[5, 10] = 100
    path = tmp_path / "negative_origin.npz"
    np.savez_compressed(path, grid=grid)
    snapshot = dataclasses.replace(
        recording.maps[0], content_hash="negative_origin", path=path,
        origin=(-10.0, -20.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        resolution=resolution, width=width, height=height,
    )
    cell_x = -10.0 + 10 * resolution
    cell_y = -20.0 + 5 * resolution
    viewport = Viewport(cell_x - 5.0, cell_y - 5.0,
                        cell_x + 5.0, cell_y + 5.0)
    panel = renderer.map_panel
    layer = renderer._build_map_layer(snapshot, panel, viewport)
    px, py, _scale = viewport.project(
        cell_x + resolution / 2.0, cell_y + resolution / 2.0,
        panel_width=panel.width, panel_height=panel.height,
    )
    occupied = np.all(layer == np.array(MAP_OCCUPIED, np.uint8), axis=-1)
    ys, xs = np.nonzero(occupied)
    assert len(xs) > 0, "负 origin 的地图没有画出占用格"
    assert abs(xs.mean() - px) <= 1.5 and abs(ys.mean() - py) <= 1.5
    # 栅格之外仍是未知，不是白色自由区。
    assert tuple(layer[0, 0]) == MAP_UNKNOWN
