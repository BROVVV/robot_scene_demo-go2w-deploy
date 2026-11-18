"""段边界来源：显式标记（重启/时钟阶跃）与坐标系变化都不能被硬连。"""

from __future__ import annotations

from app.search_replay.models import OdomRow, PoseRow
from app.search_replay.trajectory import build_trajectory


def _pose(t: float, x: float, *, frame: str = "map", yaw: float = 0.0) -> PoseRow:
    return PoseRow(t=t, wall=t, x=x, y=0.0, yaw=yaw, frame_id=frame)


def _odom(t: float, x: float) -> OdomRow:
    return OdomRow(t=t, wall=t, x=x, y=0.0, yaw=0.0, vx=0.2)


def _bridges(trajectory, cut_t: float) -> list:
    return [
        segment for segment in trajectory.segments
        if any(point.t < cut_t for point in segment.points)
        and any(point.t > cut_t for point in segment.points)
    ]


def test_clock_step_marker_splits_and_is_not_bridged() -> None:
    poses = [_pose(i * 0.1, i * 0.02) for i in range(20)]
    odom = [_odom(i * 0.1, i * 0.02) for i in range(20)]
    cut = 1.0
    trajectory = build_trajectory(
        poses, odom,
        segment_markers=[{"t": cut, "reason": "clock_step"}],
    )
    assert len(trajectory.segments) == 2
    assert _bridges(trajectory, cut) == []
    assert trajectory.segments[1].reason.startswith("marker:clock_step")


def test_collector_restart_marker_splits_with_a_pose_jump() -> None:
    before = [_pose(i * 0.1, i * 0.02) for i in range(10)]
    # 重启后坐标系回到原点：如果不看标记，1.5 s 内 0.18 m 的跳变又低于
    # 0.6 m 的隐式门限，会被画成一条连续的线。
    after = [_pose(1.0 + i * 0.1, i * 0.01) for i in range(10)]
    odom = ([_odom(i * 0.1, i * 0.02) for i in range(10)]
            + [_odom(1.0 + i * 0.1, i * 0.01) for i in range(10)])
    trajectory = build_trajectory(
        before + after, odom,
        segment_markers=[{"t": 1.0, "reason": "collector_restart"}],
    )
    assert len(trajectory.segments) == 2
    assert _bridges(trajectory, 1.0) == []
    assert trajectory.segments[1].reason == "marker:collector_restart"


def test_frame_change_splits_map_from_odom() -> None:
    poses = [_pose(i * 0.1, i * 0.02, frame="map") for i in range(10)]
    poses += [_pose(1.0 + i * 0.1, 0.18 + i * 0.02, frame="odom")
              for i in range(10)]
    trajectory = build_trajectory(poses, [])
    assert len(trajectory.segments) == 2
    assert _bridges(trajectory, 1.0) == []
    assert trajectory.segments[1].reason == "frame_change"


def test_odom_speed_ceiling_rejects_jump_distance() -> None:
    poses = [_pose(i * 0.1, i * 0.02) for i in range(10)]
    odom = [_odom(i * 0.1, i * 0.02) for i in range(10)]
    # 里程里混入一次 50 m 的瞬时跳变：物理上不可能，不能算进行程。
    odom[5] = _odom(0.5, 50.0)
    trajectory = build_trajectory(poses, odom)
    assert trajectory.distance_m < 5.0, trajectory.distance_m
