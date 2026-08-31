import json

import pytest

from robot_scene_live_bridge.bundle_writer import AtomicBundleWriter
from robot_scene_live_bridge.live_bridge_node import (
    clearance_status,
    scheduled_bundle_deadline,
)


def payload(frame_id):
    return {
        "schema_version": "1.0",
        "session_id": "test_session",
        "frame_id": frame_id,
        "image_path": "image.jpg",
    }


def test_bundle_is_complete_before_atomic_latest_switch(tmp_path):
    writer = AtomicBundleWriter(tmp_path)
    first = writer.write(b"\xff\xd8first\xff\xd9", payload(1))
    assert (tmp_path / "latest").resolve() == first
    assert (first / "READY").is_file()
    assert json.loads((first / "frame_bundle.json").read_text())["frame_id"] == 1

    second = writer.write(b"\xff\xd8second\xff\xd9", payload(2))
    assert (tmp_path / "latest").resolve() == second
    assert first.is_dir()
    assert second.is_dir()


def test_nonfinite_metadata_is_rejected_without_switching_latest(tmp_path):
    writer = AtomicBundleWriter(tmp_path)
    first = writer.write(b"\xff\xd8first\xff\xd9", payload(1))
    invalid = payload(2)
    invalid["clearance"] = {"front_m": float("inf")}
    with pytest.raises(ValueError, match="Out of range float values"):
        writer.write(b"\xff\xd8invalid\xff\xd9", invalid)
    assert (tmp_path / "latest").resolve() == first
    assert not (tmp_path / "bundles/test_session-000000000002").exists()


def test_spool_retention_is_bounded_per_session(tmp_path):
    writer = AtomicBundleWriter(tmp_path, max_bundles_per_session=2)
    first = writer.write(b"\xff\xd8first\xff\xd9", payload(1))
    second = writer.write(b"\xff\xd8second\xff\xd9", payload(2))
    third = writer.write(b"\xff\xd8third\xff\xd9", payload(3))
    assert not first.exists()
    assert second.is_dir()
    assert third.is_dir()
    assert (tmp_path / "latest").resolve() == third


def test_spool_retention_never_deletes_another_session(tmp_path):
    writer = AtomicBundleWriter(tmp_path, max_bundles_per_session=1)
    other_payload = payload(1)
    other_payload["session_id"] = "other_session"
    other = writer.write(b"\xff\xd8other\xff\xd9", other_payload)
    writer.write(b"\xff\xd8first\xff\xd9", payload(1))
    writer.write(b"\xff\xd8second\xff\xd9", payload(2))
    assert other.is_dir()


def test_scheduled_rate_limit_does_not_accumulate_frame_quantization_drift():
    period = 1_000_000_000
    next_deadline = None
    accepted = []
    # A roughly 4 Hz camera whose stamps never land exactly on a one-second
    # boundary. Anchoring each deadline to the accepted frame decays to about
    # 0.8 Hz; the fixed schedule remains at 1 Hz.
    for stamp in range(70_000_000, 10_000_000_000, 260_000_000):
        allowed, candidate = scheduled_bundle_deadline(next_deadline, stamp, period)
        if allowed:
            accepted.append(stamp)
            next_deadline = candidate
    assert len(accepted) == 10
    assert accepted[-1] - accepted[0] < 9.5 * period


def test_clearance_status_distinguishes_no_return_from_unknown():
    assert clearance_status(1.2, source_fresh=True) == "measured"
    assert clearance_status(float("inf"), source_fresh=True) == "no_return"
    assert clearance_status(float("nan"), source_fresh=True) == "unknown"
    assert clearance_status(1.2, source_fresh=False) == "unknown"
    assert clearance_status(float("inf"), source_fresh=True, valid=False) == "unknown"
