"""PointCloud2 byte decoding must preserve sensor values and reject bad layouts."""
import ast
import copy
import importlib.util
import math
from pathlib import Path
import struct
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from go2w_lidar_preprocessor import preprocess_core as core

ROOT=Path(__file__).resolve().parents[1]
PACKAGE=ROOT/'ros2_ws/src/go2w_lidar_preprocessor/go2w_lidar_preprocessor'
spec=importlib.util.spec_from_file_location('pointcloud_xyz_under_test',PACKAGE/'pointcloud_xyz.py')
codec=importlib.util.module_from_spec(spec);spec.loader.exec_module(codec)
FORMATS={1:'b',2:'B',3:'h',4:'H',5:'i',6:'I',7:'f',8:'d'}


def cloud(datatype=7,big=False,padding=0):
    size=struct.calcsize(FORMATS[datatype]);step=3*size+4;width,height=4,3
    row_step=width*step+padding;buffer=bytearray([165]*(height*row_step))
    fields=[SimpleNamespace(name=name,offset=i*size,datatype=datatype,count=1)
            for i,name in enumerate(('x','y','z'))]
    expected=[]
    for row in range(height):
        for column in range(width):
            values=[]
            for axis,field in enumerate(fields):
                value=(10 if datatype in (2,4,6) else -20)+row*12+column*3+axis
                if datatype in (7,8):value=value/4.
                struct.pack_into(('>' if big else '<')+FORMATS[datatype],buffer,
                    row*row_step+column*step+field.offset,value)
                values.append(value)
            expected.append(values)
    msg=SimpleNamespace(width=width,height=height,point_step=step,row_step=row_step,
                        data=buffer,fields=fields,is_bigendian=big)
    return msg,np.asarray(expected,dtype=np.float64)


@pytest.mark.parametrize('datatype',list(FORMATS))
@pytest.mark.parametrize('big',[False,True])
@pytest.mark.parametrize('padding',[0,7])
def test_scalar_types_byte_order_and_padded_rows(datatype,big,padding):
    msg,expected=cloud(datatype,big,padding);before=bytes(msg.data)
    actual=codec.read_xyz(msg)
    np.testing.assert_array_equal(actual,expected)
    assert actual.dtype==np.float64 and actual.shape==(12,3)
    actual[0,0]=12345.
    assert bytes(msg.data)==before
    np.testing.assert_array_equal(codec.read_xyz(msg),expected)


def test_field_names_override_storage_and_descriptor_order():
    msg=SimpleNamespace(width=2,height=1,point_step=24,row_step=48,
        is_bigendian=False,data=bytearray(48),fields=[
        SimpleNamespace(name='z',offset=4,datatype=7,count=1),
        SimpleNamespace(name='x',offset=10,datatype=7,count=1),
        SimpleNamespace(name='intensity',offset=18,datatype=4,count=2),
        SimpleNamespace(name='y',offset=0,datatype=7,count=1)])
    for i in range(2):
        for name,offset in [('x',10),('y',0),('z',4)]:
            struct.pack_into('<f',msg.data,i*24+offset,{'x':1.,'y':2.,'z':3.}[name]+i*10)
    np.testing.assert_array_equal(codec.read_xyz(msg),[[1.,2.,3.],[11.,12.,13.]])


def test_nan_infinity_and_signed_zero_are_retained():
    msg,_=cloud();values=[float('nan'),float('inf'),float('-inf'),-0.]
    for i,value in enumerate(values):struct.pack_into('<f',msg.data,i*msg.point_step,value)
    result=codec.read_xyz(msg)
    assert len(result)==12 and np.isnan(result[0,0])
    assert result[1,0]==float('inf') and result[2,0]==float('-inf')
    assert np.signbit(result[3,0])


@pytest.mark.parametrize('width,height',[(0,0),(0,2),(3,0)])
def test_empty_cloud_does_not_invent_points(width,height):
    msg=SimpleNamespace(width=width,height=height,point_step=0,row_step=0,
                        data=b'',fields=[],is_bigendian=False)
    actual=codec.read_xyz(msg);assert actual.shape==(0,3) and actual.dtype==np.float64


@pytest.mark.parametrize('kind',[
    'truncated','row_overlap','zero_step','missing_z','duplicate_x','array_x',
    'unknown_type','field_past_record','negative_width','fractional_width','noncontiguous'])
def test_malformed_layout_is_rejected(kind):
    msg,_=cloud(padding=7)
    if kind=='truncated':msg.data=msg.data[:-1]
    elif kind=='row_overlap':msg.row_step=msg.width*msg.point_step-1
    elif kind=='zero_step':msg.point_step=0
    elif kind=='missing_z':msg.fields=msg.fields[:2]
    elif kind=='duplicate_x':msg.fields.append(msg.fields[0])
    elif kind=='array_x':msg.fields[0].count=2
    elif kind=='unknown_type':msg.fields[0].datatype=99
    elif kind=='field_past_record':msg.fields[0].offset=msg.point_step
    elif kind=='negative_width':msg.width=-1
    elif kind=='fractional_width':msg.width=4.5
    elif kind=='noncontiguous':msg.data=memoryview(msg.data)[::2]
    with pytest.raises(ValueError):codec.read_xyz(msg)


def test_invalid_packet_revokes_freshness_and_does_not_escape_sensor_callback():
    tree=ast.parse((PACKAGE/'preprocessor_node.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='LidarPreprocessor')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_cloud')
    namespace={'PointCloud2':object,'sensor_header_fresh':lambda **kwargs:True,
        'Time':SimpleNamespace(from_msg=lambda stamp:stamp),'Duration':lambda **kwargs:None,
        'TransformException':LookupError,'Bool':lambda **kwargs:SimpleNamespace(**kwargs),
        'read_xyz':codec.read_xyz}
    exec(compile(ast.Module(body=[method],type_ignores=[]),'sensor_callback','exec'),namespace)
    published=[];logger=Mock()
    node=SimpleNamespace(_ready=True,_config={'sensor_profile':None,'safety':{'lidar_timeout_seconds':.3}},
        _last_valid_ns=999,_timestamp_rejected=False,_layout_rejected=False,
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10**9)),
        _tf_buffer=SimpleNamespace(lookup_transform=lambda *a,**k:object()),
        _fresh_pub=SimpleNamespace(publish=published.append),get_logger=lambda:logger)
    msg,_=cloud();msg.point_step=0
    msg.header=SimpleNamespace(frame_id='sensor',stamp=SimpleNamespace(sec=1,nanosec=0))
    namespace['_cloud'](node,msg);namespace['_cloud'](node,msg)
    assert node._last_valid_ns is None and node._layout_rejected
    assert len(published)==2 and all(message.data is False for message in published)
    logger.warning.assert_called_once()


@pytest.mark.parametrize('big', [False, True])
@pytest.mark.parametrize('padding', [0, 7])
def test_sensor_callback_keeps_planar_scans_and_clearance_after_bad_packet(big, padding):
    """Exercise the actual callback and scan method without creating a ROS node."""
    tree = ast.parse((PACKAGE/'preprocessor_node.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == 'LidarPreprocessor')
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef)
               and n.name in ('_cloud', '_publish_scan')]
    cloud_output = Mock()
    namespace = {
        'np': np, 'copy': copy, 'math': math, 'PointCloud2': object,
        'Time': SimpleNamespace(from_msg=lambda stamp: stamp),
        'Duration': lambda **kwargs: None, 'TransformException': LookupError,
        'read_xyz': codec.read_xyz,
        'point_cloud2': SimpleNamespace(create_cloud_xyz32=cloud_output),
        'Vector3Stamped': lambda: SimpleNamespace(vector=SimpleNamespace()),
        'Bool': lambda **kwargs: SimpleNamespace(**kwargs),
        'Float32': lambda **kwargs: SimpleNamespace(**kwargs),
        'LaserScan': SimpleNamespace,
    }
    for name in ('sensor_header_fresh', 'transform_points', 'filter_points_base_link',
                 'collision_obstacles', 'directional_clearance', 'laser_scan_ranges'):
        namespace[name] = getattr(core, name)
    exec(compile(ast.Module(body=methods, type_ignores=[]),
                 'actual_planar_callback', 'exec'), namespace)

    msg, _ = cloud(big=big, padding=padding)
    points = [
        [1.2, .1, .2], [.2, 0., .2], [1.5, -.6, .4], [2., .4, .9],
        [7., 0., .2], [math.nan, 0., .2], [math.inf, 0., .2], [1., 0., 0.],
        [-1.4, -.2, .3], [0., 1.2, .3], [1.2, -.1, .2], [2., 0., 2.],
    ]
    original_xyz = []
    for i, point in enumerate(points):
        row, column = divmod(i, msg.width)
        offset = row*msg.row_step + column*msg.point_step
        struct.pack_into(('>' if big else '<')+'fff', msg.data, offset, *point)
        original_xyz.append(struct.unpack_from(
            ('>' if big else '<')+'fff', msg.data, offset))
    msg.header = SimpleNamespace(frame_id='sensor', stamp=SimpleNamespace(sec=1, nanosec=0))
    original_bytes = bytes(msg.data)
    geometry = core.PreprocessParameters(
        minimum_range=.3, maximum_range=6., minimum_height=-.2, maximum_height=1.5,
        ground_height=.05, collision_maximum_height=.55,
        self_half_length=.5, self_half_width=.3, self_filter_margin=.05,
        front_corridor_half_width=.4, rotation_envelope_radius=1.)
    transform = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=.1, y=-.05, z=0.),
        rotation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    node = SimpleNamespace(
        _ready=True, _config={'sensor_profile': None,
            'safety': {'lidar_timeout_seconds': .3},
            'scan': {'angle_min_rad': -math.pi, 'angle_max_rad': math.pi,
                     'angle_increment_rad': math.pi/180}},
        _geometry=geometry, _last_valid_ns=999,
        _timestamp_rejected=False, _layout_rejected=False,
        _raw_self_filter=None, _floor_compensation=False,
        _floor_pub=None, _floor_2d_pub=None, _filtered_pub=None,
        _obstacles_pub=None, _collision_obstacles_pub=None,
        _mapping_accumulator=None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10**9)),
        _tf_buffer=SimpleNamespace(lookup_transform=lambda *a, **k: transform),
        get_logger=lambda: Mock())
    captured = {}
    for name in ('_fresh_pub', '_clearance_pub', '_diagnostic_clearance_pub',
                 '_front_pub', '_left_pub', '_right_pub', '_rotation_clearance_valid_pub',
                 '_scan_pub', '_navigation_scan_pub', '_mapping_scan_pub'):
        captured[name] = []
        setattr(node, name, SimpleNamespace(publish=captured[name].append))
    node._publish_scan = MethodType(namespace['_publish_scan'], node)

    bad = copy.copy(msg)
    bad.point_step = 0
    namespace['_cloud'](node, bad)
    assert node._last_valid_ns is None and node._layout_rejected
    namespace['_cloud'](node, msg)
    assert not node._layout_rejected and node._last_valid_ns == 10**9
    assert [m.data for m in captured['_fresh_pub']] == [False, True]
    assert bytes(msg.data) == original_bytes
    cloud_output.assert_not_called()

    xyz = core.transform_points(np.asarray(original_xyz), (.1, -.05, 0.), (0., 0., 0., 1.))
    _, obstacles = core.filter_points_base_link(xyz, geometry)
    collision = core.collision_obstacles(obstacles, geometry)
    for name, expected_points in (('_scan_pub', collision),
                                  ('_navigation_scan_pub', obstacles),
                                  ('_mapping_scan_pub', obstacles)):
        scans = captured[name]
        assert len(scans) == 1 and scans[0].header.frame_id == 'base_link'
        expected = core.laser_scan_ranges(
            expected_points, angle_min=-math.pi, angle_max=math.pi,
            angle_increment=math.pi/180, range_min=.3, range_max=6.)
        np.testing.assert_array_equal(scans[0].ranges, expected)
    clearance = core.directional_clearance(collision, geometry)
    diagnostic = captured['_diagnostic_clearance_pub'][0].vector
    np.testing.assert_array_equal(
        [diagnostic.x, diagnostic.y, diagnostic.z],
        [clearance.front, clearance.left, clearance.right])
    assert captured['_front_pub'][0].data == clearance.front
    assert math.isnan(captured['_left_pub'][0].data)
    assert math.isnan(captured['_right_pub'][0].data)
    assert captured['_rotation_clearance_valid_pub'][0].data is False
