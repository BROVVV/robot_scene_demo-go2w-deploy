from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_production_mapping_binds_sensor_profile_hash_to_keyframe_graph():
    launcher = (ROOT / 'scripts/go2w/start_2d_localization.sh').read_text()
    launch = (ROOT / 'ros2_ws/src/robot_scene_nav_bringup/launch/localization_2d.launch.py').read_text()
    assert 'sensor_profile_id="sha256:$(sha256sum "$sensor_profile"' in launcher
    assert '"sensor_profile_id:=$sensor_profile_id"' in launcher
    assert "DeclareLaunchArgument('sensor_profile_id', default_value='unknown')" in launch
    assert "{'sensor_profile_id': LaunchConfiguration('sensor_profile_id')}" in launch
