from app.spatial.world_state_snapshot import (
    WorldRevisions,
    build_world_state_snapshot,
    compatible_revisions,
)


def test_snapshot_id_binds_world_content_but_not_capture_wall_time():
    revisions = WorldRevisions("map-a", "epoch-1", 3, 4, 5, sensor_profile_id="profile-a")
    first = build_world_state_snapshot(
        revisions=revisions, task={"target": "bag"}, robot={"x": 1},
        map_summary={"unknown": 4}, objects=[{"object_id": "o1"}], topology={},
        created_at=10,
    )
    second = build_world_state_snapshot(
        revisions=revisions, task={"target": "bag"}, robot={"x": 1},
        map_summary={"unknown": 4}, objects=[{"object_id": "o1"}], topology={},
        created_at=20,
    )
    assert first.snapshot_id == second.snapshot_id
    assert first.created_at != second.created_at
    assert first.to_dict()["grid_revision"] == 4


def test_epoch_and_sensor_profile_are_part_of_coordinate_compatibility():
    left = WorldRevisions("map-a", "epoch-1", 1, 1, 1, sensor_profile_id="p1")
    assert compatible_revisions(left, left)
    assert not compatible_revisions(left, WorldRevisions("map-a", "epoch-2", 1, 1, 1,
                                                          sensor_profile_id="p1"))
    assert not compatible_revisions(left, WorldRevisions("map-a", "epoch-1", 1, 1, 1,
                                                          sensor_profile_id="p2"))


def test_snapshot_defensively_copies_nested_values():
    task = {"attrs": ["red"]}
    snapshot = build_world_state_snapshot(
        revisions=WorldRevisions("m", "e", 0, 0, 0), task=task, robot={},
        map_summary={}, objects=[], topology={}, created_at=1,
    )
    task["attrs"].append("large")
    assert snapshot.task == {"attrs": ["red"]}


def test_snapshot_changes_when_candidate_evidence_changes():
    revisions = WorldRevisions("m", "e", 1, 2, 3)
    first = build_world_state_snapshot(
        revisions=revisions, task={}, robot={}, map_summary={}, objects=[], topology={},
        candidates=[{"id": "f1", "semantic": 0.1}], created_at=1,
    )
    second = build_world_state_snapshot(
        revisions=revisions, task={}, robot={}, map_summary={}, objects=[], topology={},
        candidates=[{"id": "f1", "semantic": 0.9}], created_at=1,
    )
    assert first.snapshot_id != second.snapshot_id
