from types import SimpleNamespace

from app.spatial.object_mobility import ObjectMobilityTracker


def _observation(label: str, xyz=(1.0, 2.0, 0.5)):
    return SimpleNamespace(
        label=label,
        label_zh=None,
        map_xyz=xyz,
        bbox=[0.1, 0.1, 0.3, 0.4],
    )


def test_open_vocabulary_aliases_continue_the_same_stationary_track():
    tracker = ObjectMobilityTracker({
        "stable_observations": 2,
        "stable_span_s": 1.0,
        "stable_radius_m": 0.35,
    })

    first = tracker.update(
        [_observation("computer chair")], frame_id="f1", timestamp=10.0, map_epoch="e1"
    )[0]
    second = tracker.update(
        [_observation("chair", (1.08, 2.02, 0.5))],
        frame_id="f2", timestamp=12.2, map_epoch="e1",
    )[0]

    assert second.track_id == first.track_id
    assert second.mobility_state == "stationary"
    assert second.mobility_class == "semi_static"
    assert second.fields()["topology_eligible"] is True


def test_unrelated_frame_labels_are_not_merged_by_a_generic_word():
    tracker = ObjectMobilityTracker()

    door = tracker.update(
        [_observation("door frame")], frame_id="f1", timestamp=10.0, map_epoch="e1"
    )[0]
    picture = tracker.update(
        [_observation("picture frame")], frame_id="f2", timestamp=11.0, map_epoch="e1"
    )[0]

    assert picture.track_id != door.track_id
