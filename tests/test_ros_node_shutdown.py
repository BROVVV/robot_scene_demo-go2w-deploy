"""Operator shutdown is clean; unrelated node errors still propagate."""
import ast
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import pytest


class ContextShutdown(Exception):
    pass


@pytest.mark.parametrize('package,factory',[
    ('go2w_cmd_vel_bridge','CmdVelBridge'),('go2w_control_arbiter','ControlArbiter'),
    ('go2w_lio_bringup','WheelOdomNode')])
@pytest.mark.parametrize('error,context_alive',[
    (KeyboardInterrupt(),True),(ContextShutdown(),False),(ValueError('unexpected'),True)])
def test_shutdown_cleans_up_without_hiding_other_errors(package,factory,error,context_alive):
    root=Path(__file__).resolve().parents[1]
    filename={'CmdVelBridge':'bridge_node.py','ControlArbiter':'arbiter_node.py',
              'WheelOdomNode':'wheel_odom.py'}[factory]
    path=root/'ros2_ws/src'/package/package/filename
    tree=ast.parse(path.read_text());fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    node=Mock()
    ros=SimpleNamespace(init=Mock(),spin=Mock(side_effect=error),ok=Mock(return_value=context_alive),shutdown=Mock())
    namespace={'rclpy':ros,'ExternalShutdownException':ContextShutdown,factory:lambda:node}
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(path),'exec'),namespace)
    if isinstance(error,ValueError):
        with pytest.raises(ValueError,match='unexpected'):namespace['main']()
    else:namespace['main']()
    node.destroy_node.assert_called_once()
    assert ros.shutdown.call_count==int(context_alive)
