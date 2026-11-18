"""故障注入校准：把已知的"坏版本"注入进去，证明验收判据抓得住它。

计划 §9 要求至少 6 项：未知画成自由、用规划路线当实走、忽略 origin yaw、
跨段连线、未完成文件标 READY、prune 删除正在生成的会话。

注入一律走"依赖注入/猴补丁"，不改运行中的生产源码；每个用例都先证明
正常路径成立，再证明坏版本会被同一条判据判负。
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np
import pytest

from app.manual_web_demo.search_session_archive import SearchSessionArchive
from app.search_replay import renderer as renderer_mod
from app.search_replay import storage
from app.search_replay.manager import ReplayManager
from app.search_replay.models import ReplayConfig
from app.search_replay.recording import load_recording
from app.search_replay.renderer import (MAP_FREE, MAP_OCCUPIED, MAP_UNKNOWN,
                                        ReplayRenderer)
from app.search_replay.trajectory import (Trajectory, TrajectoryPoint,
                                          TrajectorySegment, build_trajectory)
from tests.search_replay_fixture import FIXTURE_SOURCE, make_recording

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SESSION_ID = "search_20261021_120000_calibration"


def _config(root: Path) -> ReplayConfig:
    return ReplayConfig(
        width=480, height=270, fps=10, end_hold_seconds=1.0,
        key_pad_seconds=1.0, default_speed=10, speeds=(10,),
        encoder_preset="ultrafast", encoder_crf=30, root=str(root),
    )


@pytest.fixture(scope="module")
def rendered(tmp_path_factory):
    root = tmp_path_factory.mktemp("calibration") / "sessions"
    manager = ReplayManager(
        _config(root), sessions_root=root, project_root=PROJECT_ROOT,
        source=FIXTURE_SOURCE,
    )
    make_recording(root, session_id=SESSION_ID, seconds=4.0, camera_fps=1.0)
    assert manager.request_render(SESSION_ID, speed=10, force=True)["ok"]
    assert manager.wait_idle(180), "calibration fixture render did not finish"
    yield manager, root
    manager.shutdown()


def _renderer(root: Path) -> ReplayRenderer:
    recording = load_recording(root / SESSION_ID, config=_config(root))
    renderer = ReplayRenderer(recording, _config(root), speed=10,
                              width=480, height=270, fps=10)
    renderer.prepare()
    return renderer


def _pixel_counts(layer: np.ndarray) -> dict[str, int]:
    return {
        "unknown": int(np.all(layer == np.array(MAP_UNKNOWN, np.uint8), axis=-1).sum()),
        "free": int(np.all(layer == np.array(MAP_FREE, np.uint8), axis=-1).sum()),
        "occupied": int(np.all(layer == np.array(MAP_OCCUPIED, np.uint8), axis=-1).sum()),
    }


def test_calibration_unknown_painted_free_is_caught(rendered, monkeypatch) -> None:
    """未知画成白色（"未知当自由"）时，未知像素数必须归零并被判负。"""
    _manager, root = rendered
    renderer = _renderer(root)
    panel = renderer.map_panel
    viewport = renderer.viewports[renderer.trajectory.segments[0].segment_id]
    # 用最后一张图：此时地图已展开，自由区足够大，计数才有意义。
    snapshot = renderer.recording.maps[-1]

    good = _pixel_counts(renderer._build_map_layer(snapshot, panel, viewport))
    assert good["unknown"] > 500 and good["free"] > 500, good
    assert MAP_UNKNOWN != MAP_FREE

    # 注入：把"未知"的底板色换成自由色。
    monkeypatch.setattr(renderer_mod, "MAP_UNKNOWN", MAP_FREE)
    bad = _pixel_counts(renderer._build_map_layer(snapshot, panel, viewport))
    assert bad["unknown"] == 0, bad
    # 验收判据（未知 >1000 且与自由不同色）在坏版本上必然失败。
    assert not (bad["unknown"] > 1000 and MAP_UNKNOWN != MAP_FREE)


def test_calibration_ignoring_origin_yaw_is_caught(rendered, monkeypatch) -> None:
    """忽略 origin 四元数里的 yaw，旋转地图会与未旋转画得一模一样。"""
    import dataclasses

    _manager, root = rendered
    renderer = _renderer(root)
    panel = renderer.map_panel
    viewport = renderer.viewports[renderer.trajectory.segments[0].segment_id]
    snapshot = renderer.recording.maps[0]
    rotated = dataclasses.replace(snapshot, origin=(
        snapshot.origin[0], snapshot.origin[1], 0.0,
        0.0, 0.0, float(np.sin(np.pi / 4)), float(np.cos(np.pi / 4)),
    ))

    flat = renderer._build_map_layer(snapshot, panel, viewport)
    yawed = renderer._build_map_layer(rotated, panel, viewport)
    difference = np.abs(flat.astype(np.int16) - yawed.astype(np.int16)).mean()
    assert difference > 1.0, difference

    # 注入：yaw 恒为 0（等于忽略 origin 朝向）。
    monkeypatch.setattr(renderer_mod, "_yaw_from_quaternion", lambda _q: 0.0)
    flat_bad = renderer._build_map_layer(snapshot, panel, viewport)
    yawed_bad = renderer._build_map_layer(rotated, panel, viewport)
    difference_bad = np.abs(
        flat_bad.astype(np.int16) - yawed_bad.astype(np.int16)
    ).mean()
    assert difference_bad == 0.0, difference_bad
    assert not difference_bad > 1.0


def test_calibration_bridging_segments_is_caught(tmp_path: Path) -> None:
    """定位中断后把两段连成一条线会跨过空洞、并把跳跃算进行程。"""
    root = tmp_path / "sessions"
    sid = "search_20261021_120010_calibration_gap"
    make_recording(root, session_id=sid, seconds=7.0, camera_fps=1.0,
                   with_invalid_span=True, invalid_span=(4.0, 5.5))
    recording = load_recording(root / sid, config=_config(root))

    good = recording.trajectory()
    assert len(good.segments) >= 2, [s.segment_id for s in good.segments]
    first, second = good.segments[0], good.segments[1]
    assert first.end_t < second.start_t
    # 恢复后的位姿要留在同一段里，不能被逐点拆成散点。
    assert len(second.points) > 1, "定位恢复后的轨迹被切碎了"
    gap_lo, gap_hi = first.end_t, second.start_t
    # 间隔刻意选在 2 s 以内：隐式"间隔过大"启发式不会命中，分段只能靠
    # 采集器写的标记/无效区间 —— 这样去掉它们才能复现"连成一条线"。
    assert 1.0 < gap_hi - gap_lo <= 2.0

    def bridges(trajectory) -> list:
        """一个段里同时出现空洞两侧的点 = 跨段连线。"""
        return [
            segment for segment in trajectory.segments
            if any(point.t <= gap_lo for point in segment.points)
            and any(point.t >= gap_hi for point in segment.points)
        ]

    # 生产判据在正常版本上成立。
    assert bridges(good) == []

    # 注入：把分段标记、无效区间和所有隐式分段手段（跳跃/间隔/里程速度）
    # 全部关掉，得到"一条线连到底"的坏版本。
    bad = build_trajectory(
        recording.poses, recording.odom,
        segment_markers=(), invalid_spans=(),
        max_jump_m=1e9, max_jump_window_s=1e9, max_odom_speed_mps=1e9,
    )
    assert len(bad.segments) == 1, [s.segment_id for s in bad.segments]
    assert bridges(bad), "坏版本应当把两段连成一条线"
    # 连线还会把空洞两端的跳跃算进行程：这正是"行程虚高"的来源。
    assert bad.distance_m > good.distance_m + 1.0, (
        bad.distance_m, good.distance_m)


def test_calibration_planned_route_cannot_pass_as_actual(tmp_path: Path) -> None:
    """规划路线不是实测轨迹：写一份 plan.jsonl 不能改变轨迹。"""
    root = tmp_path / "sessions"
    sid = "search_20261021_120020_calibration_plan"
    make_recording(root, session_id=sid, seconds=4.0, camera_fps=1.0)
    before = load_recording(root / sid, config=_config(root)).trajectory()

    # 往会话里塞一条"很漂亮"的规划路线（如果渲染器会读它，就会画出来）。
    plan = [
        {"kind": "plan", "t": before.segments[0].start_t + index * 0.5,
         "x": 10.0 + index, "y": -5.0, "yaw": 0.0}
        for index in range(8)
    ]
    (root / sid / "replay" / "plan.jsonl").write_text(
        "\n".join(json.dumps(row) for row in plan) + "\n", encoding="utf-8",
    )
    after = load_recording(root / sid, config=_config(root)).trajectory()

    assert len(after.segments) == len(before.segments)

    def actual_only(trajectory) -> bool:
        """验收判据：轨迹来自实测位姿，而不是 plan.jsonl。"""
        xs = [point.x for segment in trajectory.segments
              for point in segment.points]
        return max(xs) < 9.0 and abs(
            trajectory.distance_m - before.distance_m) < 1e-9

    assert actual_only(after), "规划路线的点混进了实测轨迹"

    # 坏版本：渲染器改成读 plan.jsonl（这里直接构造它会产生的那条轨迹）。
    points = [TrajectoryPoint(t=row["t"], x=row["x"], y=row["y"],
                              yaw=row["yaw"]) for row in plan]
    planned = Trajectory(
        segments=[TrajectorySegment(
            segment_id="planned", reason="plan", points=points,
            distance_m=sum(math.hypot(b.x - a.x, b.y - a.y)
                           for a, b in zip(points, points[1:])),
            net_displacement_m=math.hypot(
                points[-1].x - points[0].x, points[-1].y - points[0].y),
            distance_source="plan")],
        distance_m=sum(math.hypot(b.x - a.x, b.y - a.y)
                       for a, b in zip(points, points[1:])),
        net_displacement_m=math.hypot(
            points[-1].x - points[0].x, points[-1].y - points[0].y),
        distance_source="plan",
        distances_by_segment={"planned": 7.0},
    )
    assert not actual_only(planned), "坏版本应当被判负"


def test_calibration_truncated_video_cannot_claim_ready(rendered,
                                                        monkeypatch) -> None:
    """没解码校验时，半个 MP4 也会报 READY；校验必须在，且要真的判负。"""
    manager, root = rendered
    video = (root / SESSION_ID / "replay" / "artifacts" / "replay.mp4")
    original = video.read_bytes()
    try:
        video.write_bytes(original[: max(1, len(original) // 2)])
        # 生产判据：截断文件不可播放。
        assert manager.status(SESSION_ID)["playable"] is False

        # 注入：把"能解码"换成"文件存在就算好"。
        monkeypatch.setattr(manager, "_video_ok", lambda _session_dir: True)
        assert manager.status(SESSION_ID)["playable"] is True
        # 也就是说：上面那条 playable is False 完全靠 _video_ok 撑着。
    finally:
        video.write_bytes(original)
        monkeypatch.undo()
    assert manager.status(SESSION_ID)["playable"] is True


def _archive_session(archive: SearchSessionArchive, session_id: str,
                     updated_at: float) -> None:
    archive.begin(session_id, {"target": "垃圾桶"})
    archive.record({"session_id": session_id, "status": "TARGET_FOUND",
                    "finished_at": time.time()})
    marker = archive.root / session_id / "webui_session.json"
    payload = json.loads(marker.read_text(encoding="utf-8"))
    payload["updated_at"] = updated_at
    marker.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")


def test_calibration_prune_protection_is_load_bearing(tmp_path: Path) -> None:
    """去掉回放保护后，正在录制的会话会被历史滚动清理删掉。"""
    root = tmp_path / "live_runs"
    active = "search_20261021_120030_calibration_active"
    make_recording(root, session_id=active, seconds=2.0, camera_fps=1.0)
    capture = root / active / "replay" / storage.CAPTURE_STATUS
    payload = storage.read_json(capture) or {}
    payload.update({"status": "RECORDING", "ready": True,
                    "updated_t": time.monotonic()})
    storage.atomic_write_json(capture, payload)

    manager = ReplayManager(_config(root), sessions_root=root,
                            project_root=PROJECT_ROOT, source=FIXTURE_SOURCE)
    try:
        now = time.time()
        archive = SearchSessionArchive(
            root, max_sessions=2,
            protect=lambda sid: manager.session_protected(sid),
        )
        # 录制中的会话也进归档索引，并且是最旧的一条：一旦超过 2 条，
        # 它就该落在清理范围内，只剩保护钩子能挡住。
        _archive_session(archive, active, now - 300)
        _archive_session(archive, "search_20261021_120032_new", now - 100)
        assert (root / active).is_dir(), "正常路径：录制中的会话必须被保护"

        # 注入：保护钩子恒假（等于没接保护）。
        archive_no_guard = SearchSessionArchive(
            root, max_sessions=2, protect=lambda _sid: False,
        )
        _archive_session(archive_no_guard, "search_20261021_120033_newer", now)
        assert not (root / active).exists(), "没有保护时应当被删（证明它真的在挡）"
    finally:
        manager.shutdown()
