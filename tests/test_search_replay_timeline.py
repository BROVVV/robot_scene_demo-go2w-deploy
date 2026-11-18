"""时间对齐与事件语义：倍速后相机/箭头/事件仍对应同一源时刻。"""

from __future__ import annotations

import pytest

from app.search_replay.timeline import (KeyMoment, PlaybackTimemap,
                                        SpeedSegment, build_speed_plan,
                                        key_moments_from_events)


def test_timemap_roundtrips_source_time_at_every_speed() -> None:
    segments = [
        SpeedSegment(0.0, 10.0, 10.0),
        SpeedSegment(10.0, 12.0, 1.0),
        SpeedSegment(12.0, 20.0, 5.0),
    ]
    timemap = PlaybackTimemap(segments, end_hold_seconds=2.0)
    for source_t in (0.0, 3.3, 9.99, 10.0, 11.5, 12.0, 17.25, 20.0):
        video_t = timemap.source_to_video(source_t)
        assert timemap.video_to_source(video_t) == pytest.approx(
            source_t, abs=1e-6)
    assert timemap.duration == pytest.approx(
        timemap.content_duration + 2.0)
    assert timemap.is_end_hold(timemap.content_duration + 0.1)
    # 关键窗口只覆盖 1× 区间，边界用视频时间给出。
    windows = timemap.key_windows()
    assert len(windows) == 1
    assert windows[0]["source_start"] == 10.0
    assert windows[0]["video_end"] == pytest.approx(1.0 + 2.0)


def test_speed_plan_covers_the_interval_exactly_once() -> None:
    moments = [KeyMoment(30.0, "target_confirmed"),
               KeyMoment(31.0, "search_finished")]
    segments = build_speed_plan(0.0, 60.0, moments,
                                default_speed=10.0, key_speed=1.0,
                                pad_seconds=6.0)
    assert segments[0].src_start == 0.0
    assert segments[-1].src_end == 60.0
    for first, second in zip(segments, segments[1:]):
        assert first.src_end == second.src_start
    assert min(segment.speed for segment in segments) == 1.0


def test_suspected_target_is_never_promoted_to_confirmed() -> None:
    events = [
        {"t": 5.0, "event_type": "TARGET_CANDIDATE"},
        {"t": 9.0, "event_type": "SEARCH_FINISHED"},
    ]
    labels = [moment.label for moment in key_moments_from_events(events)]
    assert "target_suspected" in labels
    assert "target_confirmed" not in labels

    confirmed = key_moments_from_events(
        events + [{"t": 7.0, "event_type": "TARGET_CONFIRMED"}]
    )
    labels = [moment.label for moment in confirmed]
    assert "target_confirmed" in labels and "target_suspected" in labels


def test_result_labels_never_turn_stop_or_failure_into_success() -> None:
    from app.search_replay.renderer import EVENT_STYLE, RESULT_LABELS

    success = RESULT_LABELS["TARGET_FOUND"]
    assert success == "找到目标"
    for result in ("OPERATOR_STOP", "SEARCH_EXHAUSTED", "FAILED", "TIMEOUT"):
        assert RESULT_LABELS[result] != success
    assert EVENT_STYLE["TARGET_CANDIDATE"][0] != EVENT_STYLE["TARGET_CONFIRMED"][0]
    assert EVENT_STYLE["TARGET_CANDIDATE"][0] == "疑似目标"
