"""A previous boot/future file must not poison a relay's deduplication clock."""
from pathlib import Path
import importlib.util,json,sys,types
from types import SimpleNamespace
import pytest

ROOT=Path(__file__).resolve().parents[1]

class Low:
    def __init__(self):
        self.motor_state=[SimpleNamespace(q=0.,dq=0.) for _ in range(16)]
        self.imu_state=SimpleNamespace();self.bms_state=SimpleNamespace()

class Sport:
    def __init__(self):self.stamp=SimpleNamespace(sec=0,nanosec=0);self.imu_state=SimpleNamespace()

@pytest.fixture(params=['low','sport'])
def relay(request,tmp_path,monkeypatch):
    for name in ('rclpy','rclpy.node','rclpy.qos','unitree_go','unitree_go.msg'):
        monkeypatch.setitem(sys.modules,name,types.ModuleType(name))
    sys.modules['rclpy.node'].Node=object
    sys.modules['rclpy.qos'].QoSProfile=object
    sys.modules['rclpy.qos'].QoSReliabilityPolicy=SimpleNamespace(RELIABLE=1)
    sys.modules['unitree_go.msg'].LowState=Low;sys.modules['unitree_go.msg'].SportModeState=Sport
    relative=('scripts/go2w/local_low_state_ros_relay.py' if request.param=='low'
              else 'unitree_go2w_control/scripts/local_sport_state_ros_relay.py')
    spec=importlib.util.spec_from_file_location('clock_guard_'+request.param,ROOT/relative)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:10.,time=lambda:1000.))
    cls=module.LocalLowStateRelay if request.param=='low' else module.LocalSportStateRelay
    node=cls.__new__(cls);node._input_path=tmp_path/'state.json';node._max_age=.3
    node._last_capture_monotonic=0.;node._boot_id='new-boot';messages=[]
    node._publisher=SimpleNamespace(publish=messages.append)
    payload=dict(capture_monotonic=9.9,capture_wall_time=999.9,capture_boot_id='new-boot',
                 motor_q=[i*.1 for i in range(16)],motor_dq=[0.]*16,
                 error_code=0,mode=1,position=[1.,2.,3.],velocity=[0.]*3,yaw_speed=0.,imu_rpy=[0.]*3)
    return node,payload,messages,request.param

@pytest.mark.parametrize('patch',[
    {'capture_monotonic':10.01},{'capture_wall_time':1000.01},
    {'capture_monotonic':9.},{'capture_wall_time':999.},
    {'capture_monotonic':float('nan')},{'capture_monotonic':float('inf')},
    {'capture_monotonic':-float('inf')},{'capture_wall_time':float('nan')},
    {'capture_wall_time':float('inf')},{'capture_monotonic':True},
    {'capture_monotonic':'9.9'},{'capture_wall_time':True},
    {'capture_wall_time':'999.9'},{'capture_boot_id':'old-boot'},
    {'capture_boot_id':None},
])
def test_rejected_file_cannot_latch_future_sequence(relay,patch):
    node,payload,messages,_=relay
    node._input_path.write_text(json.dumps(dict(payload,**patch)))
    node._publish_latest();assert not messages and node._last_capture_monotonic==0.
    node._input_path.write_text(json.dumps(payload));node._publish_latest()
    assert len(messages)==1 and node._last_capture_monotonic==9.9

@pytest.mark.parametrize('missing',['capture_wall_time','capture_monotonic'])
def test_incomplete_clock_evidence_fails_closed(relay,missing):
    node,payload,messages,_=relay;del payload[missing]
    node._input_path.write_text(json.dumps(payload));node._publish_latest();assert not messages

def test_current_python_producer_without_boot_metadata_still_works(relay):
    node,payload,messages,kind=relay;del payload['capture_boot_id']
    node._input_path.write_text(json.dumps(payload));node._publish_latest();node._publish_latest()
    assert len(messages)==1
    if kind=='low':assert [m.q for m in messages[0].motor_state]==payload['motor_q']
    else:assert messages[0].position==payload['position']
