"""HTTP contract for the replay endpoints (plan §19/§20).

Runs the real router + real ReplayManager over a fixture recording; only the
ROS collector is faked.  Range support, download naming, 404/416/202 statuses
are exactly what the history page depends on.
"""

from __future__ import annotations

import time
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from app.manual_web_demo.search_routes import create_search_router
from app.search_replay import storage
from app.search_replay.manager import ReplayManager
from app.search_replay.models import ReplayConfig
from tests.search_replay_fixture import FIXTURE_SOURCE, make_recording

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SESSION_ID = "search_20261021_141000_routes"


class FakeService:
    """Only what the router touches at import/request time."""

    def __init__(self, manager: ReplayManager | None) -> None:
        self.replay_manager = manager

    def subscribe_events(self, callback):
        return lambda: None


def _manager(root: Path, *, enabled: bool = True) -> ReplayManager:
    config = ReplayConfig(
        width=480, height=270, fps=8, end_hold_seconds=0.5,
        key_pad_seconds=0.5, default_speed=10, speeds=(1, 5, 10),
        encoder_preset="ultrafast", encoder_crf=30, root=str(root),
    )
    manager = ReplayManager(config, sessions_root=root,
                            project_root=PROJECT_ROOT, source=FIXTURE_SOURCE)
    if enabled:
        manager.start()
    return manager


def _client(manager: ReplayManager | None) -> TestClient:
    app = FastAPI()
    router, hub, ws_handler = create_search_router(FakeService(manager))
    app.include_router(router)
    return TestClient(app)


@pytest.fixture(scope="module")
def rendered(tmp_path_factory) -> Path:
    root = tmp_path_factory.mktemp("routes")
    make_recording(root, session_id=SESSION_ID, seconds=3.0, camera_fps=1.0)
    manager = _manager(root)
    try:
        # 后台恢复线程也会替终态会话排队同倍速任务；先同步跑一轮，
        # 免得 request_render 与它抢同一个 fingerprint。
        manager._housekeeping_once()
        assert manager.wait_idle(60)
        # No session was bound; render straight from the fixture package.
        result = manager.request_render(SESSION_ID, speed=10)
        assert result["ok"], result
        assert manager.wait_idle(180), "fixture render did not finish"
        assert manager.status(SESSION_ID)["status"] == "READY"
    finally:
        manager.shutdown()
    return root


def test_status_and_artifacts(rendered: Path) -> None:
    manager = _manager(rendered)
    client = _client(manager)
    try:
        payload = client.get(f"/api/search/history/{SESSION_ID}/replay").json()
        assert payload["ok"] is True
        assert payload["status"] == "READY"
        assert payload["playable"] is True

        video = client.get(f"/api/search/history/{SESSION_ID}/replay/video")
        assert video.status_code == 200
        assert video.headers["content-type"] == "video/mp4"
        assert video.headers["accept-ranges"] == "bytes"
        size = (rendered / SESSION_ID / "replay" / "artifacts" / "replay.mp4").stat().st_size
        assert int(video.headers["content-length"]) == size
        assert len(video.content) == size

        head = client.get(
            f"/api/search/history/{SESSION_ID}/replay/video",
            headers={"Range": "bytes=0-9"},
        )
        assert head.status_code == 206
        assert head.headers["content-range"] == f"bytes 0-9/{size}"
        assert len(head.content) == 10

        tail = client.get(
            f"/api/search/history/{SESSION_ID}/replay/video",
            headers={"Range": "bytes=-16"},
        )
        assert tail.status_code == 206
        assert tail.headers["content-range"] == f"bytes {size - 16}-{size - 1}/{size}"

        bad = client.get(
            f"/api/search/history/{SESSION_ID}/replay/video",
            headers={"Range": f"bytes={size + 10}-"},
        )
        assert bad.status_code == 416
        assert bad.headers["content-range"] == f"bytes */{size}"

        download = client.get(
            f"/api/search/history/{SESSION_ID}/replay/download"
        )
        assert download.status_code == 200
        assert download.headers["content-disposition"] == (
            f'attachment; filename="go2w_{SESSION_ID}_replay.mp4"'
        )

        final_map = client.get(
            f"/api/search/history/{SESSION_ID}/replay/final-map"
        )
        assert final_map.status_code == 200
        assert final_map.headers["content-type"] == "image/png"
        assert final_map.content.startswith(b"\x89PNG")

        cover = client.get(f"/api/search/history/{SESSION_ID}/replay/cover")
        assert cover.status_code == 200
        assert cover.headers["content-type"] == "image/jpeg"
        assert cover.content.startswith(b"\xff\xd8")
    finally:
        manager.shutdown()


def test_render_endpoint_reuses_and_queues(rendered: Path) -> None:
    manager = _manager(rendered)
    client = _client(manager)
    try:
        same = client.post(
            f"/api/search/history/{SESSION_ID}/replay/render",
            json={"speed": 10},
        )
        assert same.status_code == 200, same.text
        assert same.json()["queued"] is False

        other = client.post(
            f"/api/search/history/{SESSION_ID}/replay/render",
            json={"speed": 5},
        )
        assert other.status_code == 202, other.text
        assert other.json()["queued"] is True
        assert manager.wait_idle(180)
        assert manager.status(SESSION_ID)["speed"] == 5
    finally:
        manager.shutdown()


def test_unknown_session_is_404(rendered: Path) -> None:
    manager = _manager(rendered)
    client = _client(manager)
    missing = "search_20261021_000000_nothere"
    try:
        for path in (
            f"/api/search/history/{missing}/replay",
            f"/api/search/history/{missing}/replay/video",
            f"/api/search/history/{missing}/replay/download",
            f"/api/search/history/{missing}/replay/final-map",
            f"/api/search/history/{missing}/replay/cover",
        ):
            response = client.get(path)
            assert response.status_code == 404, path
            assert response.json()["error_detail"]["code"] == "REPLAY_NOT_FOUND"
    finally:
        manager.shutdown()


def test_disabled_replay_says_so(rendered: Path) -> None:
    client = _client(None)
    response = client.get(f"/api/search/history/{SESSION_ID}/replay")
    assert response.status_code == 404
    assert "回放" in response.json()["error"]


def test_unknown_session_render_refused(rendered: Path) -> None:
    manager = _manager(rendered)
    client = _client(manager)
    missing = "search_20261021_000000_nothere"
    try:
        response = client.post(
            f"/api/search/history/{missing}/replay/render", json={"speed": 10}
        )
        assert response.status_code == 404, response.text
    finally:
        manager.shutdown()


def test_partial_artifacts_404_until_rendered(tmp_path: Path) -> None:
    sid = "search_20261021_141100_raw"
    make_recording(tmp_path, session_id=sid, seconds=2.0, camera_fps=1.0)
    manager = _manager(tmp_path)
    client = _client(manager)
    try:
        status = client.get(f"/api/search/history/{sid}/replay").json()
        assert status["playable"] is False
        assert status["can_render"] is True
        video = client.get(f"/api/search/history/{sid}/replay/video")
        assert video.status_code == 404
        assert "尚未生成" in video.json()["error"]
        assert (tmp_path / sid / "replay" / storage.MANIFEST).is_file()
    finally:
        manager.shutdown()
