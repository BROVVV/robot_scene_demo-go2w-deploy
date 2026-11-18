"""Web occupancy thresholds stay distinct from localization validation."""
from array import array
import ast
from pathlib import Path
import math,time
from types import SimpleNamespace as NS

import pytest

from app.navigation.localization_health import map_statistics
from app.navigation.occupancy_values import int8_grid_view


def callback():
    p=Path(__file__).resolve().parents[1]/'scripts/go2w/nav2_web_map_bridge.py'
    tree=ast.parse(p.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Nav2WebMapBridge')
    fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='on_map')
    scope=dict(math=math,time=time,map_statistics=map_statistics,int8_grid_view=int8_grid_view)
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(p),'exec'),scope)
    return scope['on_map']


def message(data):
    return NS(info=NS(width=len(data),height=1,resolution=.05,
        origin=NS(position=NS(x=-1.,y=.25),orientation=NS(x=0.,y=0.,z=0.,w=1.))),
        header=NS(frame_id='map',stamp=NS(sec=123,nanosec=250000000)),data=data)


@pytest.mark.parametrize('data',[array('b',[-1,0,1,49,50,65,100]),[-1,0,1,49,50,65,100]])
def test_full_map_probabilities_and_distinct_display_thresholds_are_preserved(data):
    node=NS(grid=None,grid_revision=0,grid_wall_time=0.)
    callback()(node,message(data))
    assert node.grid==dict(width=7,height=1,resolution=.05,
        origin=dict(x=-1.,y=.25,yaw=0.),data=[-1,0,1,49,50,65,100],stamp=123.25,
        counts=dict(unknown=1,free=3,occupied=3))
    assert node.grid_revision==1 and node.grid_wall_time>0
    data[0]=100
    assert node.grid['data'][0]==-1, 'cached Web snapshot must own its cells'


@pytest.mark.parametrize('data',[array('b',[-1,0,127]),array('B',[255,0,100]),array('h',[-257,0,100])])
def test_invalid_cells_never_replace_the_last_web_grid(data):
    node=NS(grid={'last':'valid'},grid_revision=4,grid_wall_time=42.)
    callback()(node,message(data))
    assert node.grid=={'last':'valid'} and node.grid_revision==4 and node.grid_wall_time==42.


def test_real_ros_signed_map_keeps_its_wire_bytes():
    pytest.importorskip('rclpy')
    from nav_msgs.msg import OccupancyGrid
    from rclpy.serialization import serialize_message
    msg=OccupancyGrid();msg.header.frame_id='map';msg.header.stamp.sec=123
    msg.info.width=7;msg.info.height=1;msg.info.resolution=.05
    msg.info.origin.orientation.w=1.
    msg.data=array('b',[-1,0,1,49,50,65,100])
    before=serialize_message(msg)
    assert int8_grid_view(msg.data) is not None
    node=NS(grid=None,grid_revision=0,grid_wall_time=0.)
    callback()(node,msg)
    assert node.grid['data']==[-1,0,1,49,50,65,100]
    assert node.grid['counts']==dict(unknown=1,free=3,occupied=3)
    assert serialize_message(msg)==before
