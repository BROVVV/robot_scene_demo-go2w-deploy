from types import SimpleNamespace
from unittest.mock import Mock
from go2w_cmd_vel_bridge.bridge_node import CmdVelBridge
from go2w_cmd_vel_bridge.bridge_core import Velocity
from go2w_motion_interfaces.action import MotionCommand


def test_dispatch_opens_one_bounded_stream_and_refreshes_that_token():
    client=Mock();client.server_is_ready.return_value=True
    bridge=SimpleNamespace(_client=client,_stream_token=None,_goal_pending=False,
        _goal_handle=None,_goal_response=Mock(),_publish_stream=Mock(),
        get_parameter=lambda name:SimpleNamespace(value=.25))
    CmdVelBridge._dispatch(bridge,Velocity(.05,0.))
    goal=client.send_goal_async.call_args.args[0]
    assert goal.mode==MotionCommand.Goal.MODE_STREAMING_VELOCITY
    assert goal.duration_sec==.25 and goal.timeout_sec==30
    assert len(goal.velocity_stream_token)==32
    assert bridge._stream_token==goal.velocity_stream_token
    CmdVelBridge._dispatch(bridge,Velocity(.03,.01))
    assert client.send_goal_async.call_count==1
    assert bridge._publish_stream.call_count==2


def test_failed_motion_latches_until_disarm_but_cancel_is_expected():
    bridge=SimpleNamespace(_stream_token='a',_goal_handle=object(),_cancel_pending=True,
        _motion_fault='',_safety_received={},get_logger=lambda:Mock())
    result=SimpleNamespace(success=False,error_code=6,message='Move failed')
    future=SimpleNamespace(result=lambda:SimpleNamespace(result=result))
    CmdVelBridge._goal_result(bridge,future)
    assert bridge._motion_fault=='leased_action_failed_6'
    assert bridge._stream_token is None and bridge._goal_handle is None
    CmdVelBridge._safety_update(bridge,'_operator_armed',SimpleNamespace(data=False))
    assert not bridge._motion_fault
    result.error_code=MotionCommand.Result.ERROR_CANCELED
    CmdVelBridge._goal_result(bridge,future)
    assert not bridge._motion_fault
