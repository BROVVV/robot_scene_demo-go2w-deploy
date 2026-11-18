"""Check the actual ROS listener wiring without initializing ROS."""
import ast
import inspect
import textwrap
from types import SimpleNamespace as NS
from pathlib import Path

import pytest

from app.spatial.nav2_spatial_provider import Nav2SpatialProvider


def test_dynamic_tf_uses_bounded_sensor_qos_and_preserves_static_defaults():
    tree = ast.parse(textwrap.dedent(inspect.getsource(Nav2SpatialProvider._enable_ros)))
    sensor_assignment = next(n for n in ast.walk(tree) if isinstance(n, ast.Assign)
                             and any(isinstance(t, ast.Name) and t.id == 'sensor' for t in n.targets))
    listener_assignment = next(n for n in ast.walk(tree) if isinstance(n, ast.Assign)
                               and any(isinstance(t, ast.Attribute) and t.attr == '_tf_listener'
                                       for t in n.targets))
    calls = []
    def listener(*args, **kwargs):
        calls.append((args, kwargs))
        return object()
    scope = dict(self=NS(_tf_buffer=object(), _node=object()),
                 QoSProfile=lambda **kwargs: NS(**kwargs),
                 ReliabilityPolicy=NS(BEST_EFFORT='best_effort'), TransformListener=listener)
    module = ast.Module(body=[sensor_assignment, listener_assignment], type_ignores=[])
    exec(compile(module, '<actual_nav2_listener_wiring>', 'exec'), scope)
    assert len(calls) == 1
    assert calls[0][1] == {'qos': scope['sensor']}
    assert scope['sensor'].depth == 10
    assert scope['sensor'].reliability == 'best_effort'
    assert calls[0][0] == (scope['self']._tf_buffer, scope['self']._node)


@pytest.mark.parametrize('relative', [
    'ros2_ws/src/go2w_lidar_preprocessor/go2w_lidar_preprocessor/preprocessor_node.py',
    'ros2_ws/src/robot_scene_nav_bringup/scripts/localization_health_monitor.py',
    'ros2_ws/src/robot_scene_nav_bringup/scripts/observed_floor_map.py',
    'app/navigation/nav2_search_backend.py',
    'scripts/go2w/record_search_replay_ros.py',
])
def test_live_tf_consumers_bound_dynamic_history_without_changing_static_qos(relative):
    source = Path(__file__).resolve().parents[1] / relative
    tree = ast.parse(source.read_text())
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Name) and n.func.id == 'TransformListener']
    assert len(calls) == 1
    keywords = {k.arg: k.value for k in calls[0].keywords}
    assert 'static_qos' not in keywords
    qos = eval(compile(ast.Expression(keywords['qos']), str(source), 'eval'),
               dict(QoSProfile=lambda **kwargs: NS(**kwargs),
                    ReliabilityPolicy=NS(BEST_EFFORT='best_effort')))
    assert qos.depth == 10 and qos.reliability == 'best_effort'
