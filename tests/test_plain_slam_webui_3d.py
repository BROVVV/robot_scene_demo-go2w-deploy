from __future__ import annotations

import json
import math
import struct
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from app.manual_web_demo.search_models import SearchStartRequest
from app.manual_web_demo.slam_map_snapshot import load_slam_map_snapshot
from app.spatial.pointcloud_web_codec import (
    GlobalVoxelCloud,
    extract_xyz_array,
    extract_xyz_points,
)
from app.spatial.slam_web_map_state import (
    DEGRADED_LIO_DRIFT,
    HEALTHY,
    WAITING_FOR_MAP,
    MotionEpisodeGate,
    SlamWebMapState,
    drift_reason,
)
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


def _padded_cloud() -> tuple[_Cloud, list[tuple[float, float, float]]]:
    """Organized 2x2 cloud with per-point padding and per-row padding."""
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
    return cloud, [values[0], values[2], values[3]]


def _room_map(per_surface: int, size_m: float = 12.0) -> np.ndarray:
    """Synthetic room: floor plus two walls, so region survival is checkable."""
    rng = np.random.default_rng(7)
    half = size_m / 2.0
    x = rng.uniform(-half, half, per_surface).astype(np.float32)
    y = rng.uniform(-half, half, per_surface).astype(np.float32)
    z = rng.uniform(0.0, 2.5, per_surface).astype(np.float32)
    floor = np.stack([x, y, np.zeros_like(z)], axis=1)
    wall_x = np.stack([np.full_like(x, -half), y, z], axis=1)
    wall_y = np.stack([x, np.full_like(y, half), z], axis=1)
    return np.concatenate([floor, wall_x, wall_y], axis=0)


def _state(**kwargs) -> SlamWebMapState:
    options = {"voxel_size_m": 0.2, "max_global_voxels": 50_000,
               "max_web_points": 20_000}
    options.update(kwargs)
    return SlamWebMapState(**options)


def test_pointcloud_decoder_handles_padding_and_rejects_nonfinite() -> None:
    cloud, expected = _padded_cloud()
    assert extract_xyz_points(cloud) == expected


def test_numpy_decoder_matches_python_decoder_on_padded_cloud() -> None:
    cloud, expected = _padded_cloud()
    array = extract_xyz_array(cloud)
    assert [tuple(row) for row in array.tolist()] == expected


# ---------------------------------------------------------------------------
# 计划书 §11.3：地图坐标系六种情况
# ---------------------------------------------------------------------------


def test_map_accepts_canonical_frame_only_and_never_rewrites_frame_id() -> None:
    state = _state()
    cloud = np.array([[1.0, 0.0, 0.0], [1.5, 0.0, 0.0]], dtype=np.float32)
    assert state.accept_map(cloud, frame_id="pslam_odom", stamp=10.0,
                            wall_time=10.0) is False
    assert state.rejected_counts["map_frame_mismatch"] == 1
    assert state.health == WAITING_FOR_MAP
    assert len(state.cloud) == 0
    assert state.accept_map(cloud, frame_id="pslam_map", stamp=11.0,
                            wall_time=11.0) is True
    assert state.health == HEALTHY
    snapshot = state.snapshot(now=11.0, generated_at=11.0)
    assert snapshot["frame_id"] == "pslam_map"
    assert snapshot["canonical_frame"] == "pslam_map"
    assert snapshot["source"] == "/go2w/slam/map_3d"


def test_map_revision_replaces_previous_geometry_wholesale() -> None:
    state = _state()
    first = np.array([[5.0, 5.0, 0.0], [5.6, 5.0, 0.0]], dtype=np.float32)
    state.accept_map(first, frame_id="pslam_map", stamp=1.0, wall_time=1.0)
    assert state.cloud.bounds()["max"][0] >= 5.0
    second = np.array([[-8.0, -8.0, 0.0], [-8.6, -8.0, 0.0]], dtype=np.float32)
    state.accept_map(second, frame_id="pslam_map", stamp=2.0, wall_time=2.0)
    # 回环优化后的新版地图整张替换，旧关键帧不许残留成重影。
    assert state.cloud.bounds()["max"][0] < 0.0
    assert state.map_revision == 2
    assert state.accepted_maps == 2


def test_capacity_overflow_coarsens_globally_and_keeps_every_region() -> None:
    points = _room_map(200_000)
    assert points.shape[0] > 500_000
    state = _state(voxel_size_m=0.05, max_global_voxels=60_000,
                   max_web_points=20_000)
    assert state.accept_map(points, frame_id="pslam_map", stamp=5.0,
                            wall_time=5.0) is True
    assert len(state.cloud) <= 60_000
    assert state.cloud.capacity_limited is True
    assert state.cloud.effective_voxel_size_m > 0.05
    assert state.source_map_points == points.shape[0]
    box = state.cloud.bounds()
    assert box["min"][0] < -5.5 and box["max"][0] > 5.5
    assert box["min"][1] < -5.5 and box["max"][1] > 5.5
    kept, _ = state.cloud.sample(20_000)
    corners = {(x > 0.0, y > 0.0) for x, y, _ in kept
               if abs(x) > 5.0 and abs(y) > 5.0}
    assert len(corners) == 4


def test_web_sampling_is_spatially_uniform_not_insertion_ordered() -> None:
    rng = np.random.default_rng(11)
    early = np.stack([rng.uniform(-41.0, -39.0, 150_000),
                      rng.uniform(-1.0, 1.0, 150_000),
                      rng.uniform(0.0, 2.0, 150_000)], axis=1)
    late = np.stack([rng.uniform(39.0, 41.0, 150_000),
                     rng.uniform(-1.0, 1.0, 150_000),
                     rng.uniform(0.0, 2.0, 150_000)], axis=1)
    cloud = GlobalVoxelCloud(voxel_size_m=0.05, max_voxels=400_000)
    cloud.replace(np.concatenate([early, late], axis=0))
    points, info = cloud.sample(5_000)
    xs = [x for x, _, _ in points]
    # 先建的房间和后建的房间都必须留在网页显示里，不能按插入顺序砍掉前半段。
    assert min(xs) < -39.0
    assert max(xs) > 39.0
    assert info["mode"] == "uniform_voxel"
    assert info["truncated"] is True
    assert 2_500 <= len(points) <= 5_000


def test_session_reset_clears_cache_and_drops_stale_session_maps() -> None:
    state = _state()
    good = np.array([[1.0, 1.0, 0.0], [2.0, 1.0, 0.0]], dtype=np.float32)
    assert state.accept_map(good, frame_id="pslam_map", stamp=100.0,
                            wall_time=100.0) is True
    state.reset_session(stamp=200.0, reason="reset_marker")
    assert state.session_id == 2
    assert len(state.cloud) == 0
    assert state.health == WAITING_FOR_MAP
    assert state.snapshot(now=200.0, generated_at=200.0)["available"] is False
    # 旧 session 的迟到地图必须丢掉，否则新会话第一帧就是错位的历史。
    assert state.accept_map(good, frame_id="pslam_map", stamp=150.0,
                            wall_time=201.0) is False
    assert state.last_rejected_reason == "stale_session_map"
    assert state.accept_map(good, frame_id="pslam_map", stamp=201.0,
                            wall_time=201.0) is True
    assert state.health == HEALTHY


def test_preview_scan_never_enters_permanent_map() -> None:
    state = _state()
    state.accept_map(np.array([[1.0, 1.0, 0.0]], dtype=np.float32),
                     frame_id="pslam_map", stamp=1.0, wall_time=1.0)
    before = len(state.cloud)
    state.set_preview([(9.0, 9.0, 0.5)], frame_id="pslam_odom", stamp=2.0,
                      wall_time=2.0)
    snapshot = state.snapshot(now=2.0, generated_at=2.0)
    assert len(state.cloud) == before
    assert state.map_revision == 1
    assert snapshot["preview"]["frame_id"] == "pslam_odom"
    assert snapshot["preview"]["point_count"] == 1
    payload = json.loads(state.serialize(snapshot))
    assert payload["points"] == [[1.0, 1.0, 0.0]]


# ---------------------------------------------------------------------------
# 计划书 §10：运动段漂移判定
# ---------------------------------------------------------------------------


def test_drift_reason_covers_rotation_and_translation_mismatch() -> None:
    assert "原地转向中产生3.24米假平移" in drift_reason((0.03, 30.0), (3.24, 29.0))
    assert "轮式里程计只有0.40米" in drift_reason((0.40, 2.0), (2.10, 2.0))
    assert drift_reason((1.00, 3.0), (1.05, 3.0)) == ""


def test_motion_episode_gate_latches_drift_until_new_session() -> None:
    state = _state(gate=MotionEpisodeGate(settle_seconds=0.5))
    fixed = np.array([[1.0, 1.0, 0.0]], dtype=np.float32)
    assert state.accept_map(fixed, frame_id="pslam_map", stamp=1.0,
                            wall_time=1.0) is True
    state.note_lio(0.0, 0.0, 0.0)
    assert state.note_wheel(0.0, 0.0, 0.0, speed=0.0, yaw_rate=0.0, now=0.0) == ""
    assert state.note_wheel(0.0, 0.0, 0.0, speed=0.0, yaw_rate=0.4,
                            now=0.2) == "started"
    state.note_lio(2.9, 0.4, 0.5)
    assert state.note_wheel(0.02, 0.01, 0.52, speed=0.0, yaw_rate=0.0, now=1.0) == ""
    assert state.note_wheel(0.02, 0.01, 0.52, speed=0.0, yaw_rate=0.0,
                            now=2.0) == "degraded"
    assert state.health == DEGRADED_LIO_DRIFT
    assert "假平移" in state.health_reason
    assert state.accept_map(fixed, frame_id="pslam_map", stamp=3.0,
                            wall_time=3.0) is False
    assert state.last_rejected_reason == "lio_drift_frozen"
    # §10.2：安静的 scan 不许让冻结自己恢复。
    for tick in range(5):
        state.note_wheel(0.02, 0.01, 0.52, speed=0.0, yaw_rate=0.0, now=3.0 + tick)
    assert state.health == DEGRADED_LIO_DRIFT
    assert state.snapshot(now=8.0, generated_at=8.0)["lio_pose_valid"] is False
    state.reset_session(stamp=50.0, reason="lio_reset")
    assert state.health == WAITING_FOR_MAP
    assert state.accept_map(fixed, frame_id="pslam_map", stamp=51.0,
                            wall_time=51.0) is True
    assert state.health == HEALTHY


def test_snapshot_json_reports_three_capacity_layers_and_splices_points() -> None:
    state = _state(voxel_size_m=0.05, max_global_voxels=60_000,
                   max_web_points=8_000)
    points = _room_map(60_000)
    assert state.accept_map(points, frame_id="pslam_map", stamp=3.0,
                            wall_time=3.0, source_points=1_234_567) is True
    payload = json.loads(state.snapshot_json(now=3.0, generated_at=3.0))
    assert payload["schema_version"] == "go2w_slam_web_cloud_v2"
    # §14：网页采样点数不许冒充 SLAM 真实点数。
    assert payload["source_map_points"] == 1_234_567
    assert payload["global_cached_voxels"] == len(state.cloud)
    assert payload["web_display_points"] == len(payload["points"]) <= 8_000
    assert payload["global_cached_voxels"] > payload["web_display_points"]
    assert payload["web_sampling"]["mode"] == "uniform_voxel"
    assert payload["map_extent_m"][0] > 10.0
    assert payload["mapping_mode"] == "mapping_assist"
    assert payload["motion_authorized"] is False
    assert payload["safety_authorized"] is False


def _corridor(count: int) -> np.ndarray:
    rng = np.random.default_rng(5)
    return np.stack([rng.uniform(-10.0, 10.0, count),
                     rng.uniform(-1.0, 1.0, count),
                     rng.uniform(0.0, 2.4, count)], axis=1).astype(np.float32)


def _building_map(per_region: int) -> tuple[np.ndarray, dict[str, tuple[float, float]]]:
    """长走廊 + 起点/中段/终点三个区域，总点数超过 50 万。"""
    rng = np.random.default_rng(3)
    regions = {"start_room": (-30.0, 0.0), "corridor_mid": (0.0, 0.0),
               "end_room": (30.0, 0.0)}
    blocks = []
    for center_x, center_y in regions.values():
        blocks.append(np.stack([
            rng.uniform(center_x - 4.0, center_x + 4.0, per_region),
            rng.uniform(center_y - 4.0, center_y + 4.0, per_region),
            rng.uniform(0.0, 2.4, per_region)], axis=1))
    blocks.append(np.stack([rng.uniform(-30.0, 30.0, per_region),
                            rng.uniform(-0.9, 0.9, per_region),
                            rng.uniform(0.0, 2.4, per_region)], axis=1))
    return np.concatenate(blocks, axis=0).astype(np.float32), regions


def test_large_building_map_keeps_start_middle_and_end_regions() -> None:
    points, regions = _building_map(130_000)
    assert points.shape[0] > 500_000
    state = _state(voxel_size_m=0.08, max_global_voxels=80_000,
                   max_web_points=20_000)
    assert state.accept_map(points, frame_id="pslam_map", stamp=9.0,
                            wall_time=9.0) is True
    assert len(state.cloud) <= 80_000
    assert state.cloud.capacity_limited is True
    web, _ = state.cloud.sample(state.max_web_points)
    assert len(web) <= 20_000
    for name, (center_x, center_y) in regions.items():
        near = [1 for x, y, _ in web
                if abs(x - center_x) <= 4.0 and abs(y - center_y) <= 4.0]
        assert len(near) > 50, name
    # 显示预算有限，但显示出来的范围必须仍然覆盖整张地图的边界。
    box = state.cloud.bounds()
    assert min(x for x, _, _ in web) <= box["min"][0] + 1.0
    assert max(x for x, _, _ in web) >= box["max"][0] - 1.0


def test_repeated_identical_map_does_not_duplicate_or_re_encode() -> None:
    state = _state(voxel_size_m=0.1, max_global_voxels=50_000,
                   max_web_points=5_000)
    corridor = _corridor(20_000)
    assert state.accept_map(corridor, frame_id="pslam_map", stamp=1.0,
                            wall_time=1.0) is True
    first_size = len(state.cloud)
    first_bounds = state.cloud.bounds()
    payload = state.points_json()
    # §11.3：没有新的 map revision 就不许重新生成、重新发送同一份大点云。
    assert state.points_json() is payload
    # 机器人原地转过 90° 之后 SLAM 又发一遍同一张世界地图：世界坐标没变，
    # 不许出现旋转副本，也不许点数翻倍。
    assert state.accept_map(corridor, frame_id="pslam_map", stamp=2.0,
                            wall_time=2.0) is True
    assert len(state.cloud) == first_size
    assert state.cloud.bounds() == first_bounds
    assert state.points_json() == payload
    rotated = (corridor[:, [1, 0, 2]] *
               np.array([1.0, -1.0, 1.0], dtype=np.float32))
    assert state.accept_map(rotated, frame_id="pslam_map", stamp=3.0,
                            wall_time=3.0) is True
    # 整张替换：真正旋转过的世界不会和旧世界叠成双份走廊。
    assert len(state.cloud) <= first_size + 5
    assert state.cloud.bounds() != first_bounds


def test_growing_map_keeps_the_first_room_when_new_rooms_arrive() -> None:
    rng = np.random.default_rng(13)

    def room(center_x: float) -> np.ndarray:
        return np.stack([rng.uniform(center_x - 3.0, center_x + 3.0, 120_000),
                         rng.uniform(-3.0, 3.0, 120_000),
                         rng.uniform(0.0, 2.4, 120_000)], axis=1).astype(np.float32)

    first, second, third = room(0.0), room(20.0), room(40.0)
    state = _state(voxel_size_m=0.06, max_global_voxels=40_000,
                   max_web_points=8_000)
    grown = [first, np.concatenate([first, second], axis=0),
             np.concatenate([first, second, third], axis=0)]
    for index, chunk in enumerate(grown, start=1):
        assert state.accept_map(chunk, frame_id="pslam_map", stamp=float(index),
                                wall_time=float(index)) is True
        web, _ = state.cloud.sample(8_000)
        # 地图长大之后，最早那个房间仍然要有代表点，不许按时间先后淘汰。
        assert any(abs(x) <= 3.0 for x, _, _ in web), index
    assert state.cloud.bounds()["max"][0] > 36.0


def test_correct_rotation_episode_keeps_world_map_healthy() -> None:
    state = _state(gate=MotionEpisodeGate(settle_seconds=0.5))
    corridor = _corridor(5_000)
    assert state.accept_map(corridor, frame_id="pslam_map", stamp=1.0,
                            wall_time=1.0) is True
    state.note_lio(0.0, 0.0, 0.0)
    assert state.note_wheel(0.0, 0.0, 0.0, speed=0.0, yaw_rate=0.0, now=0.0) == ""
    assert state.note_wheel(0.0, 0.0, 0.0, speed=0.0, yaw_rate=0.5,
                            now=0.2) == "started"
    # 正确的原地转向：LIO 认为自己几乎没有平移，只是转了 60°。
    state.note_lio(0.04, 0.02, 1.05)
    assert state.note_wheel(0.02, 0.005, 1.04, speed=0.0, yaw_rate=0.0,
                            now=1.2) == ""
    assert state.note_wheel(0.02, 0.005, 1.04, speed=0.0, yaw_rate=0.0,
                            now=2.0) == "finished"
    assert state.health == HEALTHY
    snapshot = state.snapshot(now=2.0, generated_at=2.0)
    assert snapshot["pslam_episode_delta_yaw_deg"] > 55.0
    assert snapshot["pslam_episode_delta_xy_m"] < 0.1
    assert state.accept_map(corridor, frame_id="pslam_map", stamp=3.0,
                            wall_time=3.0) is True
    assert state.health == HEALTHY


def test_snapshot_reader_marks_fresh_and_preserves_safety_boundary(tmp_path: Path) -> None:
    now = time.time()
    path = tmp_path / "cloud.json"
    path.write_text(json.dumps({
        "available": True,
        "generated_at": now,
        "map_updated_at": now - 1.5,
        "source": "/go2w/slam/map_3d",
        "canonical_frame": "pslam_map",
        "points": [[1.0, 2.0, 3.0]],
        "preview": {"frame_id": "pslam_odom", "wall_time": now - 0.4,
                    "point_count": 1, "points": [[0.0, 0.0, 0.0]]},
        "motion_authorized": True,
        "safety_authorized": True,
    }), encoding="utf-8")
    value = load_slam_map_snapshot(path)
    assert value["available"] is True
    assert value["fresh"] is True
    assert value["point_count"] == 1
    assert value["web_display_points"] == 1
    # 全局地图和当前 scan 是两个时间轴，网页要分别显示。
    assert 1.0 <= value["map_age_seconds"] <= 3.0
    assert value["preview_age_seconds"] < 1.0
    assert value["mapping_mode"] == "mapping_assist"
    assert value["motion_authorized"] is False
    assert value["safety_authorized"] is False


def test_missing_snapshot_reports_v2_unavailable_contract(tmp_path: Path) -> None:
    value = load_slam_map_snapshot(tmp_path / "missing.json")
    assert value["schema_version"] == "go2w_slam_web_cloud_v2"
    assert value["available"] is False
    assert value["reason"] == "snapshot_not_ready"
    assert value["canonical_frame"] == "pslam_map"
    assert value["permanent_source"] == "/go2w/slam/map_3d"
    assert value["mapping_health"] == "UNAVAILABLE"
    assert value["points"] == []


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
    spool_index = argv.index("--spool-root")
    assert argv[spool_index + 1].endswith("/runtime/go2w/spool")
    assert argv[spool_index + 1].startswith("/")


def test_webui_contains_live_3d_map_contract() -> None:
    root = Path(__file__).resolve().parents[1]
    html = (root / "app/manual_web_demo/templates/index.html").read_text(encoding="utf-8")
    javascript = (root / "app/manual_web_demo/static/slam_map_3d.js").read_text(encoding="utf-8")
    server = (root / "app/manual_web_demo/web_server.py").read_text(encoding="utf-8")
    assert 'id="slam-map-3d"' in html
    assert 'id="light-slam"' in html
    assert 'id="slam3d-capacity"' in html
    assert 'id="slam3d-preview-toggle"' in html
    assert 'fetch("/api/slam/map3d"' in javascript
    for key in ("source_map_points", "global_cached_voxels", "web_display_points",
                "canonical_frame", "health_reason", "capacity_limited"):
        assert key in javascript
    assert '@app.get("/api/slam/map3d")' in server
