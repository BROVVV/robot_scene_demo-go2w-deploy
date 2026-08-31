from __future__ import annotations

import json
import math
import struct
import time
from dataclasses import dataclass
from pathlib import Path

from app.manual_web_demo.search_models import SearchStartRequest
from app.manual_web_demo.slam_map_snapshot import load_slam_map_snapshot
from app.spatial.pointcloud_web_codec import BoundedVoxelCloud, extract_xyz_points
from scripts.go2w.autonomous_search_worker import build_argv


@dataclass
class _Field:
    name: str
    offset: int
    datatype: int = 7


@dataclass
class _Cloud:
    width: int
    height: int
    point_step: int
    row_step: int
    data: bytes
    fields: list[_Field]
    is_bigendian: bool = False


def test_pointcloud_decoder_handles_padding_and_rejects_nonfinite() -> None:
    point_step = 16
    row_step = point_step * 2 + 8
    raw = bytearray(row_step * 2)
    values = [(1.0, 2.0, 3.0), (math.nan, 0.0, 0.0), (-2.0, 4.0, 0.5), (3.0, -1.0, 1.5)]
    for index, point in enumerate(values):
        row, column = divmod(index, 2)
        struct.pack_into("<fff", raw, row * row_step + column * point_step, *point)
    cloud = _Cloud(
        width=2,
        height=2,
        point_step=point_step,
        row_step=row_step,
        data=bytes(raw),
        fields=[_Field("x", 0), _Field("y", 4), _Field("z", 8)],
    )
    assert extract_xyz_points(cloud) == [
        (1.0, 2.0, 3.0),
        (-2.0, 4.0, 0.5),
        (3.0, -1.0, 1.5),
    ]


def test_voxel_cloud_is_bounded_and_latest_value_wins() -> None:
    cloud = BoundedVoxelCloud(voxel_size_m=1.0, max_points=100)
    cloud.update((float(i), 0.0, 0.0) for i in range(140))
    assert len(cloud) == 100
    cloud.update([(139.2, 0.0, 0.0)])
    assert cloud.sampled(100)[-1] == (139.2, 0.0, 0.0)


def test_snapshot_reader_marks_fresh_and_preserves_safety_boundary(tmp_path: Path) -> None:
    path = tmp_path / "cloud.json"
    path.write_text(json.dumps({
        "available": True,
        "generated_at": time.time(),
        "source": "aligned_scan_accumulated",
        "points": [[1.0, 2.0, 3.0]],
        "motion_authorized": True,
        "safety_authorized": True,
    }), encoding="utf-8")
    value = load_slam_map_snapshot(path)
    assert value["available"] is True
    assert value["fresh"] is True
    assert value["point_count"] == 1
    assert value["mapping_mode"] == "mapping_assist"
    assert value["motion_authorized"] is False
    assert value["safety_authorized"] is False


def test_webui_search_defaults_to_plain_slam_and_worker_forwards_provider() -> None:
    request = SearchStartRequest.from_dict({"task_text": "寻找饮水机"})
    assert request.spatial_v2 is True
    assert request.spatial_provider == "plain_slam"
    argv = build_argv({
        "target": "饮水机",
        "spatial_v2": True,
        "spatial_provider": request.spatial_provider,
    })
    index = argv.index("--spatial-provider")
    assert argv[index + 1] == "plain_slam"


def test_webui_contains_live_3d_map_contract() -> None:
    root = Path(__file__).resolve().parents[1]
    html = (root / "app/manual_web_demo/templates/index.html").read_text(encoding="utf-8")
    javascript = (root / "app/manual_web_demo/static/slam_map_3d.js").read_text(encoding="utf-8")
    server = (root / "app/manual_web_demo/web_server.py").read_text(encoding="utf-8")
    assert 'id="slam-map-3d"' in html
    assert 'id="light-slam"' in html
    assert 'fetch("/api/slam/map3d"' in javascript
    assert '@app.get("/api/slam/map3d")' in server
