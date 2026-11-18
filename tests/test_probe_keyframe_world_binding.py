from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_probe_is_read_only_and_rejects_unknown_sensor_profile():
    source = (ROOT / 'scripts/go2w/probe_keyframe_world_binding.py').read_text()
    assert '"read_only": True' in source
    assert '"motion_sent": False' in source
    assert 'graph.sensor_profile_id not in {"", "unknown"}' in source
    assert 'and identity_matches and quality_compared' in source
    assert '"acquisition_pose_anchor_matches"' in source
    assert 'and result["acquisition_pose_anchor_matches"]' in source
