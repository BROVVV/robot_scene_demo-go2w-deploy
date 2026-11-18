"""Exercise the production cloud callback, including the missing-floor stop."""
import copy
from pathlib import Path
import time
from types import SimpleNamespace
import numpy as np
import pytest
import yaml
from geometry_msgs.msg import TransformStamped
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from unitree_go.msg import LowState
from go2w_description.description_config import _rpy_to_quaternion_xyzw
from go2w_lidar_preprocessor.config import load_safety_ready_config
from go2w_lidar_preprocessor.floor_attitude import rpy_matrix,standing_height
from go2w_lidar_preprocessor.preprocessor_node import LidarPreprocessor


class Recorder:
    def __init__(self):self.messages=[]
    def publish(self,message):self.messages.append(message)


@pytest.mark.parametrize('publish_clouds', [True, False])
@pytest.mark.parametrize('floor_visibility', [True, False])
def test_cloud_callback_requires_independent_floor_and_keeps_real_obstacles(publish_clouds,floor_visibility):
    root=Path(__file__).resolve().parents[4]
    config,geometry=load_safety_ready_config(str(root/'configs/go2w/lidar_preprocess.yaml'),
        str(root/'configs/go2w/official_reference.yaml'),str(root/'configs/go2w/navigation_sensor_profile.yaml'))
    profile=config['sensor_profile'];urdf=(root/'configs/go2w/calibration_20261019/official_go2w.urdf').read_bytes()
    compensation={'minimum_obstacle_height_m':.08};profile['floor_attitude_compensation']=compensation
    extr=profile['frames']['base_to_cloud'];R=rpy_matrix(*extr['rotation_rpy_rad']);t=np.array(extr['translation_m']);post=profile['posture']
    node=SimpleNamespace(_ready=True,_config=config,_geometry=geometry,_last_valid_ns=None,
        _raw_self_filter=None,_self_removed=0,_mapping_self_removed=0,
        _floor_compensation=compensation,_floor_urdf=urdf,
        _reference_height=standing_height(urdf,post['joint_q'],post['body_roll_pitch_rad']),
        _floor_check_at=0.,_floor_check_error='not_checked',_timestamp_rejected=False)
    for name in ('fresh','floor','filtered','obstacles','collision_obstacles','clearance','diagnostic_clearance','front','left','right','rotation_clearance_valid'):
        setattr(node,'_'+name+'_pub',Recorder())
    if not publish_clouds:
        for name in ('floor','filtered','obstacles','collision_obstacles'):
            setattr(node,'_'+name+'_pub',None)
    scans=[];node._navigation_scan_pub=object();node._mapping_scan_pub=object()
    node._publish_scan=lambda header,points,**kw:scans.append((kw.get('publisher'),points))
    floor_calls=[]
    node._floor_2d_pub=Recorder() if floor_visibility else None
    def floor_lookup(header,points):
        # An odometry lookup here can wait up to 40ms. The collision and
        # navigation outputs must already exist before entering that work.
        assert len(scans)==2, 'floor TF lookup delayed safety scans'
        assert node._last_valid_ns is not None
        assert node._fresh_pub.messages[-1].data is True
        floor_calls.append((copy.deepcopy(header),points.copy()))
    node._publish_floor_visibility_2d=floor_lookup
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=time.time_ns()))
    transform=TransformStamped();transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z=t
    quat=_rpy_to_quaternion_xyzw(tuple(extr['rotation_rpy_rad']))
    for name,value in zip(('x','y','z','w'),quat):setattr(transform.transform.rotation,name,value)
    node._tf_buffer=SimpleNamespace(lookup_transform=lambda *a,**kw:transform)
    low=LowState()
    for i,q in enumerate(post['joint_q']):low.motor_state[i].q=q
    rp=np.array(post['body_roll_pitch_rad'])+[.012,-.022]
    low.imu_state.rpy[:]=[float(rp[0]),float(rp[1]),0.]
    LidarPreprocessor._posture(node,low)
    assert node._posture_valid
    base_ref=R@np.array(profile['raw_floor_plane']['normal']);normal=rpy_matrix(*rp).T@rpy_matrix(*post['body_roll_pitch_rad'])@base_ref
    distance=profile['raw_floor_plane']['distance_m']+base_ref@t+node._height_delta
    x,y=np.meshgrid(np.linspace(-2,2,35),np.linspace(-2,2,35));z=(distance-normal[0]*x.ravel()-normal[1]*y.ravel())/normal[2]
    floor=np.c_[x.ravel(),y.ravel(),z]
    # Same forward location, measured heights 5cm and 15cm above the floor.
    ground=(distance-normal[0]*1.2)/normal[2]
    # The third point is in the measured low wheel/leg band.  It remains in
    # the live collision scan but must not enter the structural mapping scan.
    additions=np.array([[1.2,0.,ground+.05],[1.2,0.,ground+.15],
                        [0.,-.30,-.28]])
    points=np.vstack((floor,additions));raw=(points-t)@R
    def message(values):
        header=Header();header.frame_id=extr['child'];ns=time.time_ns();header.stamp.sec=ns//10**9;header.stamp.nanosec=ns%10**9
        return point_cloud2.create_cloud_xyz32(header,values.tolist())
    LidarPreprocessor._cloud(node,message(raw))
    if publish_clouds:
        assert len(node._floor_pub.messages)==1
    assert len(floor_calls)==int(floor_visibility)
    assert len(scans)==3
    assert len(scans[0][1])==2  # Hard collision scan retains the low-band point.
    assert scans[1][0] is node._navigation_scan_pub
    assert len(scans[1][1])==1  # Nav2 excludes the measured self band, without accumulation.
    assert scans[2][0] is node._mapping_scan_pub
    assert len(scans[2][1])==1  # Structural map excludes the same measured self band.
    assert node._last_valid_ns is not None
    LidarPreprocessor._cloud(node,message(raw[-2:]))
    assert node._last_valid_ns is None
    assert node._floor_check_error
    assert node._fresh_pub.messages[-1].data is False
    assert len(scans)==3


def test_geometry_status_explains_joint_rejection_after_stable_pose_tolerance():
    """Without live floor/FK validation, leaving the static envelope closes scans."""
    root=Path(__file__).resolve().parents[4]
    config,geometry=load_safety_ready_config(str(root/'configs/go2w/lidar_preprocess.yaml'),
        str(root/'configs/go2w/official_reference.yaml'),str(root/'configs/go2w/navigation_sensor_profile.yaml'))
    profile=config['sensor_profile'];post=profile['posture']
    node=SimpleNamespace(_ready=True,_config=config,_geometry=geometry,_last_valid_ns=None,
        _floor_compensation=None,_self_removed=0,_posture_at=0.,_posture_valid=False,
        _geometry_status_pub=Recorder(),_fresh_pub=Recorder())
    low=LowState()
    for i,q in enumerate(post['joint_q']):low.motor_state[i].q=q
    low.imu_state.rpy[:]=[float(v) for v in post['body_roll_pitch_rad']]+[0.]
    LidarPreprocessor._posture(node,low)
    assert node._posture_valid
    low.motor_state[5].q=post['joint_q'][5]+.0818
    LidarPreprocessor._posture(node,low)
    assert node._posture_valid
    assert node._static_self_model_eligible
    low.motor_state[5].q=post['joint_q'][5]+.0918
    LidarPreprocessor._posture(node,low)
    assert node._posture_valid is False
    assert node._static_self_model_eligible is False
    LidarPreprocessor._publish_freshness(node)
    import json
    status=json.loads(node._geometry_status_pub.messages[-1].data)
    assert status['posture_valid'] is False
    assert status['posture_blocking']==['joint_reference_deviation']
    assert status['static_self_model_eligible'] is False
    assert status['maximum_joint_deviation_index']==5
    assert abs(status['maximum_joint_deviation_rad']-.0918)<1e-6
    assert status['allowed_joint_deviation_rad']==.09
    assert node._fresh_pub.messages[-1].data is False
    low.motor_state[5].q=post['joint_q'][5]
    low.motor_state[11].q=post['joint_q'][11]+.292
    LidarPreprocessor._posture(node,low)
    assert not node._posture_valid
    assert not node._static_self_model_eligible
    low.motor_state[5].q=float('nan')
    LidarPreprocessor._posture(node,low)
    LidarPreprocessor._publish_freshness(node)
    status=json.loads(node._geometry_status_pub.messages[-1].data)
    assert status['maximum_joint_deviation_rad'] is None
    assert 'nonfinite_joint_state' in status['posture_blocking']


def test_observed_floor_profile_uses_dynamic_fallback_outside_static_model_envelope():
    """A captured valid wheel-gait pose keeps geometry but disables the learned model."""
    root=Path(__file__).resolve().parents[4]
    config,geometry=load_safety_ready_config(str(root/'configs/go2w/lidar_preprocess.yaml'),
        str(root/'configs/go2w/official_reference.yaml'),str(root/'configs/go2w/navigation_sensor_profile.yaml'))
    profile=config['sensor_profile'];post=profile['posture']
    urdf=(root/'configs/go2w/calibration_20261019/official_go2w.urdf').read_bytes()
    node=SimpleNamespace(_ready=True,_config=config,_geometry=geometry,_last_valid_ns=None,
        _floor_compensation=profile['floor_attitude_compensation'],_floor_urdf=urdf,
        _reference_height=standing_height(urdf,post['joint_q'],post['body_roll_pitch_rad']),
        _self_removed=0,_posture_at=0.,_posture_valid=False,
        _geometry_status_pub=Recorder(),_fresh_pub=Recorder(),_floor_check_at=time.monotonic(),
        _floor_check_error='')
    # Synchronized with raw cloud frame 179 in the 2026-10-23 bounded trial.
    dynamic_q=[0.004049777984619141,0.8190297484397888,-1.6064518690109253,
        -0.05839794874191284,0.7588379383087158,-1.4272668361663818,
        0.031052231788635254,0.7317282557487488,-1.3765742778778076,
        -0.015217065811157227,0.8144766092300415,-1.574700117111206]
    dynamic_rpy=[0.004661593586206436,0.00826340913772583,-0.13851630687713623]
    low=LowState()
    for i,q in enumerate(dynamic_q):low.motor_state[i].q=q
    low.imu_state.rpy[:]=dynamic_rpy
    LidarPreprocessor._posture(node,low)
    assert node._posture_valid is True
    assert node._static_self_model_eligible is False
    assert node._posture_blocking==[]
    assert node._posture_warnings==['static_self_model_outside_envelope']
    LidarPreprocessor._publish_freshness(node)
    import json
    status=json.loads(node._geometry_status_pub.messages[-1].data)
    assert status['posture_valid'] is True
    assert status['raw_self_filter_active'] is False
    assert status['geometry_mode']=='dynamic_measured_self_surface'


def test_dynamic_fallback_does_not_require_parked_four_wheel_height_agreement():
    root=Path(__file__).resolve().parents[4]
    config,geometry=load_safety_ready_config(str(root/'configs/go2w/lidar_preprocess.yaml'),
        str(root/'configs/go2w/official_reference.yaml'),str(root/'configs/go2w/navigation_sensor_profile.yaml'))
    profile=config['sensor_profile'];post=profile['posture']
    urdf=(root/'configs/go2w/calibration_20261019/official_go2w.urdf').read_bytes()
    node=SimpleNamespace(_ready=True,_config=config,_geometry=geometry,_last_valid_ns=None,
        _floor_compensation=profile['floor_attitude_compensation'],_floor_urdf=urdf,
        _reference_height=standing_height(urdf,post['joint_q'],post['body_roll_pitch_rad']),
        _self_removed=0,_posture_at=0.,_posture_valid=False)
    # Frame 186: official joints and mild body tilt, but wheel-height spread is
    # 5.4 cm because this is an active rolling gait rather than a parked pose.
    dynamic_q=[0.01561582088470459,0.7223839163780212,-1.3580286502838135,
        0.008363991975784302,0.851456880569458,-1.6358184814453125,
        -0.004584416747093201,0.8718223571777344,-1.6983428001403809,
        -0.011735200881958008,0.715590238571167,-1.3376349210739136]
    low=LowState()
    for i,q in enumerate(dynamic_q):low.motor_state[i].q=q
    low.imu_state.rpy[:]=[0.00611078878864646,0.002055632881820202,-0.06027056649327278]
    LidarPreprocessor._posture(node,low)
    assert node._posture_valid is True
    assert node._static_self_model_eligible is False
    assert node._posture_blocking==[]
    assert node._height_delta==0.
