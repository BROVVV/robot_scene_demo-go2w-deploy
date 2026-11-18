import copy
import pytest
from scripts.go2w.localization_epoch import (require_fresh_wheel_after_robot_reboot,
                                              require_matching_epoch)


def epoch():
    return dict(schema_version=1, robot_boot_id='robot-a', host_boot_id='host-a',
                localization={'pid': 42, 'start_ticks': 10},
                wheel_odometry={'pid': 43, 'start_ticks': 11},
                sensor_profile_sha256='geometry-a')


@pytest.mark.parametrize('key', list(epoch()))
def test_reuse_rejects_changed_or_missing_epoch(key):
    saved=epoch(); current=copy.deepcopy(saved)
    current[key]={'pid': 42, 'start_ticks': 999} if isinstance(current[key], dict) else 'changed'
    with pytest.raises(ValueError, match=key):
        require_matching_epoch(saved, current)
    del saved[key]
    with pytest.raises(ValueError, match=key):
        require_matching_epoch(saved, epoch())


def test_same_process_boot_and_geometry_can_reuse_map():
    require_matching_epoch(epoch(), epoch())


def test_new_map_rejects_wheel_process_that_crossed_robot_reboot():
    saved=epoch();current=copy.deepcopy(saved);current['robot_boot_id']='robot-b'
    current['localization']={'pid':99,'start_ticks':100}
    with pytest.raises(ValueError,match='wheel odometry process crosses robot reboot'):
        require_fresh_wheel_after_robot_reboot(saved,current)


def test_new_map_accepts_restarted_wheel_after_robot_reboot():
    saved=epoch();current=copy.deepcopy(saved);current['robot_boot_id']='robot-b'
    current['wheel_odometry']={'pid':100,'start_ticks':101}
    require_fresh_wheel_after_robot_reboot(saved,current)
