"""Cached true inputs and late arm responses cannot retain motion authority."""
from go2w_cmd_vel_bridge.bridge_node import CmdVelBridge


def test_stale_positive_inputs_expire_and_missing_estop_fails_closed():
    node=CmdVelBridge.__new__(CmdVelBridge)
    node._safety_received={'_lidar_fresh':10.};node._lidar_fresh=True
    assert node._live_flag('_lidar_fresh',10.1,timeout=.3)
    assert not node._live_flag('_lidar_fresh',10.31,timeout=.3)
    assert node._live_flag('_emergency',10.1,missing=True)
