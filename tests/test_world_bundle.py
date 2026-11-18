import json

import pytest

from app.spatial.world_bundle import verify_world_manifest, write_world_manifest
from app.spatial.world_state_snapshot import WorldRevisions


def test_manifest_binds_map_epoch_semantics_and_topology(tmp_path):
    grid = tmp_path / "grid.json"
    objects = tmp_path / "objects.json"
    topology = tmp_path / "topology.json"
    for path, value in ((grid, {"grid": 1}), (objects, {"objects": []}),
                        (topology, {"places": []})):
        path.write_text(json.dumps(value), encoding="utf-8")
    write_world_manifest(
        tmp_path,
        revisions=WorldRevisions("map-a", "epoch-a", 2, 3, 4,
                                 sensor_profile_id="profile-a"),
        components={"grid": grid, "semantic_objects": objects, "topology": topology},
    )
    result = verify_world_manifest(tmp_path)
    assert result["valid"] is True
    assert result["manifest"]["map_epoch"] == "epoch-a"
    assert result["manifest"]["motion_authorized"] is False


def test_tampered_component_is_rejected(tmp_path):
    component = tmp_path / "objects.json"
    component.write_text("{}", encoding="utf-8")
    write_world_manifest(
        tmp_path,
        revisions=WorldRevisions("m", "e", 0, 0, 0, sensor_profile_id="p"),
        components={"objects": component},
    )
    component.write_text('{"injected":true}', encoding="utf-8")
    result = verify_world_manifest(tmp_path)
    assert result["valid"] is False
    assert "component_hash_mismatch:objects" in result["errors"]


def test_external_component_is_refused(tmp_path):
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    external = tmp_path / "external.json"
    external.write_text("{}", encoding="utf-8")
    with pytest.raises(ValueError, match="inside bundle"):
        write_world_manifest(
            bundle,
            revisions=WorldRevisions("m", "e", 0, 0, 0, sensor_profile_id="p"),
            components={"objects": external},
        )


def test_mutable_replay_manifest_is_declared_but_not_bound_as_world_state(tmp_path):
    world = tmp_path / "objects.json"
    replay = tmp_path / "replay" / "manifest.json"
    replay.parent.mkdir()
    world.write_text('{"objects":[]}', encoding="utf-8")
    replay.write_text('{"status":"CAPTURING"}', encoding="utf-8")
    write_world_manifest(
        tmp_path,
        revisions=WorldRevisions("m", "e", 1, 2, 3, sensor_profile_id="p"),
        components={"objects": world},
        mutable_components={"replay_manifest": replay},
    )
    replay.write_text('{"status":"READY","speed":20}', encoding="utf-8")
    result = verify_world_manifest(tmp_path)
    assert result["valid"] is True
    assert "replay_manifest" not in result["manifest"]["components"]
    assert result["manifest"]["mutable_components"]["replay_manifest"] == {
        "path": "replay/manifest.json",
        "integrity_scope": "external_mutable",
    }
