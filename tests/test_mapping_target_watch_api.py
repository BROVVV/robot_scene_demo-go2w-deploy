from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from fastapi.testclient import TestClient

from app.manual_web_demo.config import ManualDemoSettings
from app.manual_web_demo.web_server import DemoRuntime, create_app
from app.perception.passive_target_watch import (
    atomic_write_json,
    deterministic_spec,
    empty_state,
    target_profile_from_spec,
)


class _Worker:
    def __init__(self):
        self.cb = None

    def set_on_message(self, cb): self.cb = cb
    def status(self):
        return {
            "state": "ready", "motion_available": False, "state_fresh": True,
            "lease_alive": True, "lidar_fresh": True,
            "rotation_clearance_valid": True, "last_error": None,
        }
    def alive(self): return True
    def start(self): pass
    def stop(self): pass
    def shutdown(self): pass
    def request_status(self): pass
    def request_stop(self): raise AssertionError("display-only must not send stop")
    def request_estop(self): raise AssertionError("display-only must not send estop")
    def request_pulse(self, _direction): raise AssertionError("display-only must not move")


def _client(tmp_path: Path, monkeypatch) -> TestClient:
    runtime_dir = tmp_path / "runtime"
    topology = runtime_dir / "semantic_topology.json"
    request = runtime_dir / "target_watch_request.json"
    state = runtime_dir / "target_watch_state.json"
    evidence = runtime_dir / "target_watch_evidence"
    slam_map = runtime_dir / "slam_map_3d.json"
    objects = runtime_dir / "semantic_objects.json"
    runtime_dir.mkdir(parents=True)
    atomic_write_json(topology, {"session_id": "teleop_0", "revision": 0})
    atomic_write_json(state, empty_state("teleop_0", detector_backend="grounded_sam"))
    atomic_write_json(slam_map, {
        "schema_version": "go2w_slam_web_cloud_v2",
        "map_epoch": "epoch_test", "frame_id": "map",
        "points": [[0.0, 0.0, 0.0]], "available": True,
        "generated_at": 1.0,
    })
    monkeypatch.setattr(
        "app.reasoning.target_profile.TargetProfileResolver.resolve",
        lambda self, query, use_llm=True: target_profile_from_spec(
            deterministic_spec(query)
        ),
    )
    config = replace(
        ManualDemoSettings(),
        display_only=True,
        runtime_dir=str(runtime_dir / "web"),
        logs_dir=str(tmp_path / "logs"),
        slam_map_snapshot=str(slam_map),
        semantic_objects_snapshot=str(objects),
        mapping_topology_snapshot=str(topology),
        target_watch_request=str(request),
        target_watch_state=str(state),
        target_watch_evidence_dir=str(evidence),
    )
    runtime = DemoRuntime(config, worker=_Worker(), camera_fresh=lambda: True)
    return TestClient(create_app(config=config, runtime=runtime))


def test_display_only_allows_perception_config_but_blocks_motion(tmp_path, monkeypatch):
    client = _client(tmp_path, monkeypatch)
    with client:
        response = client.post("/api/mapping/target-watch", json={"target": "黑椅子"})
        assert response.status_code == 202
        request = response.json()["request"]
        assert request["profile"]["verification_mode"] == "attribute"
        assert client.post("/api/control/enable").status_code == 403
        assert client.post("/api/search/start", json={"target": "chair"}).status_code == 403
        assert client.get("/api/mapping/target-watch").status_code == 200
        cleared = client.delete("/api/mapping/target-watch")
        assert cleared.status_code == 202
        assert cleared.json()["request"]["generation"] > request["generation"]


def test_evidence_endpoint_rejects_unregistered_and_traversal(tmp_path, monkeypatch):
    client = _client(tmp_path, monkeypatch)
    with client:
        assert client.get("/api/mapping/target-watch/evidence/not_registered").status_code == 404
        assert client.get("/api/mapping/target-watch/evidence/..%2Fsecret").status_code in {404, 422}


def test_confirmed_metric_target_is_projected_as_separate_map_marker(tmp_path, monkeypatch):
    client = _client(tmp_path, monkeypatch)
    state_path = tmp_path / "runtime" / "target_watch_state.json"
    state = empty_state("teleop_0", detector_backend="yoloe_seg")
    state.update({
        "enabled": True,
        "state": "CONFIRMED",
        "raw_query": "黑椅子",
        "generation": 1,
        "confirmed_matches": [{
            "verification_id": "twv_test",
            "map_xyz": [1.0, 2.0, 0.5],
            "spatial_quality": "METRIC_LIDAR",
            "provenance": {"map_epoch": "epoch_test"},
        }],
    })
    atomic_write_json(state_path, state)
    with client:
        response = client.get("/api/slam/map3d")
        assert response.status_code == 200
        markers = [
            item for item in response.json()["objects"]
            if item.get("target_watch_only")
        ]
        assert markers == [{
            "object_id": "twv_test",
            "label_zh": "黑椅子",
            "x": 1.0, "y": 2.0, "z": 0.5, "size": None,
            "decision": "TARGET_VERIFIED",
            "is_target_confirmed": True,
            "target_watch_only": True,
            "is_hazard": False,
            "hazard_confirmation_state": None,
            "spatial_quality": "METRIC_LIDAR",
        }]


def test_auto_hazard_confirmed_marker_uses_hazard_name(tmp_path, monkeypatch):
    client = _client(tmp_path, monkeypatch)
    state_path = tmp_path / "runtime" / "target_watch_state.json"
    state = empty_state("teleop_0", detector_backend="yoloe_seg")
    state.update({
        "state": "CONFIRMED",
        "generation": 0,
        "automatic_hazard_enabled": True,
        "confirmed_matches": [{
            "verification_id": "twv_auto_hazard",
            "automatic_hazard": True,
            "canonical_name_zh": "疑似管道史莱姆泥泄漏污染源",
            "map_xyz": [1.5, -0.5, 0.4],
            "spatial_quality": "METRIC_LIDAR",
            "provenance": {"map_epoch": "epoch_test"},
        }],
    })
    atomic_write_json(state_path, state)

    with client:
        marker = next(
            item for item in client.get("/api/slam/map3d").json()["objects"]
            if item.get("object_id") == "twv_auto_hazard"
        )

    assert marker["label_zh"] == "疑似管道史莱姆泥泄漏污染源"
    assert marker["is_hazard"] is True
    assert marker["hazard_confirmation_state"] == "vlm_confirmed"
