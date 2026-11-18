"""The offline entry must not fall through to physical DDS domain zero."""
import os
from pathlib import Path
import subprocess

import pytest

ROOT=Path(__file__).resolve().parents[1]


def test_offline_entry_refuses_the_physical_domain():
    env=os.environ.copy();env['ROS_DOMAIN_ID']='0'
    result=subprocess.run(['bash','-c','source scripts/go2w/setup_offline_environment.sh'],
        cwd=ROOT,env=env,capture_output=True,text=True)
    assert result.returncode==2 and 'domain 0 is refused' in result.stderr


@pytest.mark.skipif(not (ROOT/'external/unitree_ros2/cyclonedds_ws/install_system/local_setup.bash').exists(),
    reason='requires the local ROS vendor message install, no robot required')
def test_offline_entry_imports_ros2_and_unitree_without_a_robot_interface():
    env=os.environ.copy();env['ROS_DOMAIN_ID']='89'
    result=subprocess.run(['bash','-c',
        'source scripts/go2w/setup_offline_environment.sh >/dev/null && '
        '/usr/bin/python3 -c "import os,rclpy; from unitree_go.msg import LowState; '
        'from nav_msgs.msg import Odometry; assert os.environ[\'ROS_VERSION\']==\'2\'; '
        'assert os.environ[\'ROS_LOCALHOST_ONLY\']==\'1\'; '
        'assert os.environ[\'ROS_DOMAIN_ID\']==\'89\'"'],
        cwd=ROOT,env=env,capture_output=True,text=True,timeout=15)
    assert result.returncode==0,result.stderr
