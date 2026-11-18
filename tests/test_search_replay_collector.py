"""只读采集器的地图去重口径（不导入 rclpy，不需要 ROS）。"""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "scripts" / "go2w"))

from record_search_replay_ros import Recorder  # noqa: E402


def _message(data, *, x=1.0, y=2.0, resolution=0.05, width=4, height=2,
             q=(0.0, 0.0, 0.0, 1.0), stamp=1.0):
    return SimpleNamespace(
        data=list(data),
        header=SimpleNamespace(frame_id="map",
                               stamp=SimpleNamespace(sec=stamp, nanosec=0)),
        info=SimpleNamespace(
            width=width, height=height, resolution=resolution,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y, z=0.0),
                orientation=SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3]),
            ),
        ),
    )


def test_same_content_with_a_new_timestamp_hashes_the_same() -> None:
    inside = [0] * 8
    first, _ = Recorder.map_hash(_message(inside, stamp=1.0))
    later, _ = Recorder.map_hash(_message(inside, stamp=99.0))
    assert first == later


def test_metadata_change_is_not_deduplicated() -> None:
    inside = [0] * 8
    base, _ = Recorder.map_hash(_message(inside))
    moved, _ = Recorder.map_hash(_message(inside, x=1.05))
    rotated, _ = Recorder.map_hash(_message(inside, q=(0.0, 0.0, 0.3826834,
                                                       0.9238795)))
    coarser, _ = Recorder.map_hash(_message(inside, resolution=0.1))
    assert len({base, moved, rotated, coarser}) == 4


def test_content_change_is_not_deduplicated() -> None:
    first, _ = Recorder.map_hash(_message([0] * 8))
    second, _ = Recorder.map_hash(_message([100] + [0] * 7))
    assert first != second


def test_grid_is_reshaped_rows_by_width() -> None:
    data = list(range(8))
    _digest, grid = Recorder.map_hash(_message(data, width=4, height=2))
    assert grid.shape == (2, 4)
    assert grid.tolist() == [[0, 1, 2, 3], [4, 5, 6, 7]]


# --------------------------------------------------------------------------- #
# 落盘语义（审查 R01）：stored 标记只由真正写盘的版本产生                   #
# --------------------------------------------------------------------------- #
def _recorder(tmp_path: Path, **overrides) -> Recorder:
    from app.search_replay.models import ReplayConfig

    kwargs = dict(root=str(tmp_path), map_max_hz=1.0)
    kwargs.update(overrides)
    config = ReplayConfig(**kwargs)
    return Recorder(tmp_path / "search_test", config, "", 1.0)


def _maps(recorder: Recorder) -> list[dict]:
    return [row for row in recorder._timeline_buffer if row.get("kind") == "map"]


def _markers(recorder: Recorder) -> list[dict]:
    return [row for row in recorder._timeline_buffer
            if row.get("kind") == "map_stored"]


def test_first_map_is_persisted_and_carries_a_stored_marker(tmp_path: Path) -> None:
    recorder = _recorder(tmp_path)
    recorder.on_map(_message([0] * 8))
    rows, markers = _maps(recorder), _markers(recorder)
    assert len(rows) == 1 and rows[0]["stored"] is True
    assert len(markers) == 1
    assert markers[0]["hash"] == rows[0]["hash"]
    assert markers[0]["t"] == rows[0]["t"]
    assert recorder.counts["maps_stored"] == 1
    assert (recorder.replay_dir / "maps" / f"{rows[0]['hash']}.npz").is_file()


def test_rate_limited_map_is_only_playable_after_it_is_written(
    tmp_path: Path,
) -> None:
    recorder = _recorder(tmp_path)
    recorder.on_map(_message([0] * 8))
    recorder._last_map_save = recorder._last_map_t  # 立刻到期算了刚刚的首图
    # 内容变化但还没到 1 Hz 落盘窗口：行进时间线，但不该被当成可播放版本。
    recorder.on_map(_message([100] + [0] * 7))
    rows = _maps(recorder)
    assert len(rows) == 2
    assert rows[1]["stored"] is False
    assert rows[1]["new_content"] is True
    assert len(_markers(recorder)) == 1
    assert recorder.counts["maps_merged"] == 1

    recorder.store_pending_map()
    assert rows[1]["stored"] is True
    assert len(_markers(recorder)) == 2
    assert _markers(recorder)[1]["hash"] == rows[1]["hash"]
    assert recorder.counts["maps_stored"] == 2


def test_map_odom_discontinuity_creates_a_segment_boundary(
    tmp_path: Path,
) -> None:
    import sys as _sys

    _sys.path.insert(0, str(PROJECT_ROOT / "scripts" / "go2w"))
    import record_search_replay_ros as collector

    recorder = _recorder(tmp_path)

    def transform(x: float):
        return SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=x, y=0.0, z=0.0),
                rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )

    class Buffer:
        def __init__(self, x):
            self.x = x

        def lookup_transform(self, *args, **kwargs):
            return transform(self.x)

    class Boom(Exception):
        pass

    kwargs = dict(Time=lambda: 0, Duration=lambda seconds: seconds,
                  LookupException=Boom, ConnectivityException=Boom,
                  ExtrapolationException=Boom)
    collector._sample_evidence(recorder, Buffer(0.0), stamp=10.0, wall=0.0, **kwargs)
    collector._sample_evidence(recorder, Buffer(0.1), stamp=11.0, wall=0.0, **kwargs)
    assert recorder.counts["map_odom_discontinuities"] == 0
    collector._sample_evidence(recorder, Buffer(5.0), stamp=12.0, wall=0.0, **kwargs)
    assert recorder.counts["map_odom_discontinuities"] == 1
    segments = [row for row in recorder._timeline_buffer
                if row.get("kind") == "segment"]
    assert [row["reason"] for row in segments] == ["map_odom_discontinuity"]
    assert recorder.degraded
