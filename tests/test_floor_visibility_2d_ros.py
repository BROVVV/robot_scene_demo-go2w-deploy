"""The planar transport preserves measured floor rays and their freshness gate."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import time

import numpy as np
import pytest

pytest.importorskip('rclpy')
FloorVisibility2D = pytest.importorskip('go2w_mapping_interfaces.msg').FloorVisibility2D
from app.navigation.floor_visibility import measured_floor_endpoints

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('observed_floor_map_2d_test',
    ROOT/'ros2_ws/src/robot_scene_nav_bringup/scripts/observed_floor_map.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def message():
    result=FloorVisibility2D()
    ns=time.time_ns()
    result.header.stamp.sec=ns//10**9
    result.header.stamp.nanosec=ns%10**9
    result.header.frame_id='odom_fused'
    result.origin_x=.1;result.origin_y=.2
    result.endpoints_x=[.6,.9,-.4,3.]
    result.endpoints_y=[.2,.2,.5,.2]
    return result


def observer():
    recorded=[]
    node=SimpleNamespace(
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=time.time_ns())),
        rays=SimpleNamespace(add=lambda origin,ends:recorded.append((origin,ends))))
    return node,recorded


def test_planar_message_preserves_the_horizontal_measured_endpoints():
    msg=message();node,recorded=observer()
    module.ObservedFloorMap.on_floor_2d(node,msg)
    origin,ends=recorded[0]
    old_points=np.column_stack((msg.endpoints_x,msg.endpoints_y,[-.4,-.5,-.3,-.4]))
    expected=measured_floor_endpoints(old_points,[msg.origin_x,msg.origin_y,.3])
    np.testing.assert_allclose(ends[:,:2],expected[:,:2])
    np.testing.assert_allclose(origin,[msg.origin_x,msg.origin_y,0.])
    assert len(ends)==2  # Closest duplicate and the >1.25m return are excluded.


@pytest.mark.parametrize('invalid',['stale','wrong_frame','different_lengths','nonfinite_origin'])
def test_invalid_planar_observations_cannot_add_free_space(invalid):
    msg=message();node,recorded=observer()
    if invalid=='stale':msg.header.stamp.sec-=1
    elif invalid=='wrong_frame':msg.header.frame_id='base_link'
    elif invalid=='different_lengths':msg.endpoints_y=[.2]
    else:msg.origin_x=float('nan')
    module.ObservedFloorMap.on_floor_2d(node,msg)
    assert recorded==[]


@pytest.mark.parametrize('transforms', ['valid', 'missing_map', 'missing_base', 'stale_base'])
def test_floor_map_keeps_source_grid_and_obstacle_rules(transforms):
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import TransformStamped

    grid = OccupancyGrid()
    grid.header.frame_id = 'map'; grid.header.stamp.sec = 100
    grid.info.width = 20; grid.info.height = 5; grid.info.resolution = .1
    grid.info.origin.position.x = -.5; grid.info.origin.position.y = -.25
    grid.info.origin.orientation.w = 1.
    data = [-1]*100
    data[2*20+10] = 100  # Outside the physical body, blocks measured rays.
    if transforms in ('valid', 'stale_base'):
        data[2*20+5] = 100  # Only a current, fresh physical body can clear it.
    grid.data = data
    original = list(grid.data)

    def lookup(target, source, stamp):
        if transforms == 'missing_map' or (transforms == 'missing_base' and source == 'base_link'):
            raise LookupError('test transform unavailable')
        result = TransformStamped()
        result.transform.rotation.w = 1.
        result.header.stamp.sec = 97 if transforms == 'stale_base' and source == 'base_link' else 100
        return result

    published, statuses = [], []
    node = SimpleNamespace(
        rays=SimpleNamespace(rays=lambda:[(np.array([-.35, 0., 0.]), np.array([.85, 0., 0.]))], origin_count=1),
        buffer=SimpleNamespace(lookup_transform=lookup), floor_input_mode='rays_2d',
        pub=SimpleNamespace(publish=published.append),
        status=SimpleNamespace(publish=lambda m: statuses.append(json.loads(m.data))))
    module.ObservedFloorMap.on_map(node, grid)
    assert list(grid.data) == original  # No mutation of SLAM's authoritative input.
    result = published[0]
    assert result.data.typecode == 'b'
    assert result.header.stamp.sec == 100
    assert result.data[2*20+10] == 100
    assert list(result.data[2*20+11:2*20+20]) == [-1]*9
    assert statuses[0]['occupied_cells_removed'] == 0
    if transforms == 'missing_map':
        assert list(result.data) == original
    else:
        assert result.data[2*20+2] == 0
    if transforms == 'valid':
        assert result.data[2*20+5] == 0
        assert statuses[0]['current_physical_footprint']['published_occupied_cells_cleared'] == 1
    if transforms == 'stale_base':
        assert result.data[2*20+5] == 100


@pytest.mark.parametrize('mode,selection,expected', [
    ('rays_2d','single','SingleThreadedExecutor'),
    ('rays_2d','multi','MultiThreadedExecutor'),
    ('pointcloud','multi','MultiThreadedExecutor'),
])
def test_floor_executor_selects_only_supported_modes(mode,selection,expected):
    assert module.floor_executor_class(mode,selection).__name__==expected


@pytest.mark.parametrize('mode,selection', [('pointcloud','single'),('rays_2d','typo')])
def test_floor_executor_rejects_waiting_callbacks_or_unknown_selection(mode,selection):
    with pytest.raises(ValueError):module.floor_executor_class(mode,selection)
