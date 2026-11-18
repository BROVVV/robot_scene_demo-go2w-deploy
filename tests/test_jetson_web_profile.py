"""The Jetson profile removes only the browser point-cloud surface."""

import asyncio
import json
import time
from io import BytesIO

import httpx
from PIL import Image

from app.manual_web_demo.config import ManualDemoSettings
from app.manual_web_demo.web_server import _web_video_frame, create_app


def test_jetson_page_has_2d_topology_without_3d_cloud():
    app = create_app(config=ManualDemoSettings(web_3d_map_enabled=False))

    async def check():
        async with httpx.AsyncClient(
            transport=httpx.ASGITransport(app=app), base_url="http://test"
        ) as client:
            page = await client.get("/")
            cloud = await client.get("/api/slam/map3d")
            status = await client.get("/api/status")
            assert page.status_code == 200
            assert 'id="search-map"' in page.text
            for forbidden in ("cell-slam3d", "slam-map-3d", "slam_map_3d.js", "/api/slam/map3d"):
                assert forbidden not in page.text
            assert cloud.status_code == 404
            assert status.json()["web_3d_map_enabled"] is False

    asyncio.run(check())


def test_jetson_browser_jpeg_is_reduced_without_changing_source():
    source = BytesIO()
    Image.new("RGB", (1280, 720), (25, 80, 120)).save(source, format="JPEG")
    raw = source.getvalue()
    reduced = _web_video_frame(raw, 640, 65)
    assert Image.open(BytesIO(reduced)).size == (640, 360)
    assert _web_video_frame(raw, 0, 65) == raw


def test_computer_default_keeps_existing_3d_page():
    app = create_app(config=ManualDemoSettings())

    async def check():
        async with httpx.AsyncClient(
            transport=httpx.ASGITransport(app=app), base_url="http://test"
        ) as client:
            page = await client.get("/")
            assert page.status_code == 200
            assert "slam_map_3d.js" in page.text
            assert 'id="slam-map-3d"' in page.text

    asyncio.run(check())


def test_robot_2d_page_and_api_do_not_expose_cloud(tmp_path):
    path = tmp_path / "map.json"
    grid = {"width": 2, "height": 2, "resolution": 0.05,
            "origin": {"x": -1, "y": -2, "yaw": 0}, "data": [-1, 0, 100, 0]}
    path.write_text(json.dumps({
        "map_kind": "occupancy_grid_2d", "frame_id": "map", "map_epoch": "test",
        "generated_at": time.time(), "grid": grid, "points": [[1, 2, 3]],
        "preview": {"points": [[4, 5, 6]]}, "localization_valid": True,
        "pose": {"x": 0, "y": 0, "yaw": 0},
    }))
    app = create_app(config=ManualDemoSettings(
        web_3d_map_enabled=False, web_2d_map_enabled=True,
        slam_map_snapshot=str(path), semantic_objects_snapshot=str(tmp_path / "missing.json")))

    async def check():
        async with httpx.AsyncClient(
            transport=httpx.ASGITransport(app=app), base_url="http://test"
        ) as client:
            page = await client.get("/")
            assert 'id="slam-map-2d"' in page.text
            assert "slam_map_2d.js" in page.text
            assert "slam_map_3d.js" not in page.text
            response = await client.get("/api/slam/map2d")
            data = response.json()
            assert data["available"] is True
            assert data["grid"] == grid
            assert "points" not in data and "preview" not in data
            assert (await client.get("/api/slam/map3d")).status_code == 404
            path.write_text(json.dumps({"points": [[1, 2, 3]], "map_kind": "cloud"}))
            assert (await client.get("/api/slam/map2d")).json() == {
                "available": False, "reason": "waiting_for_2d_map"}

    asyncio.run(check())
