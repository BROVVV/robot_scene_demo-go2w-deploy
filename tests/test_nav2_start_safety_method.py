from types import SimpleNamespace
import time
import ast
from pathlib import Path
import pytest
from app.live_robot.motion_bounds import evaluate_lidar_motion_readiness

# Exercise the real method without importing ROS/application dependencies.
source=ast.parse(Path('scripts/go2w/run_autonomous_loop.py').read_text())
cls=next(n for n in source.body if isinstance(n,ast.ClassDef) and n.name=='AutonomousLoop')
method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_safety_ok')
module=ast.Module(body=[method],type_ignores=[])
namespace={'time':time,'evaluate_lidar_motion_readiness':evaluate_lidar_motion_readiness}
exec(compile(ast.fix_missing_locations(module),'<actual runner safety method>','exec'),namespace)
AutonomousLoop=SimpleNamespace(_safety_ok=namespace['_safety_ok'])


def node():
    return SimpleNamespace(_odom_discontinuity_monotonic=None,
        _sport=SimpleNamespace(mode=1,error_code=0), _lidar_fresh=True,
        _clearance=.004, _min_clearance=.3)


def test_nav2_can_plan_beside_obstacle_but_legacy_forward_stays_blocked():
    n=node()
    assert AutonomousLoop._safety_ok(n,require_front_clearance=False)==(True,'')
    assert AutonomousLoop._safety_ok(n)[0] is False


@pytest.mark.parametrize('problem',['stale','mode','error','odom_jump'])
def test_nav2_keeps_live_robot_and_sensor_gates(problem):
    n=node()
    if problem=='stale':n._lidar_fresh=False
    elif problem=='mode':n._sport.mode=0
    elif problem=='error':n._sport.error_code=1
    else:
        n._odom_discontinuity_monotonic=time.monotonic()
        n._odom_discontinuity_reason='wheel reset'
    assert AutonomousLoop._safety_ok(n,require_front_clearance=False)[0] is False
