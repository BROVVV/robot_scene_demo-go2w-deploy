"""Foxy integration tests must never select the physical graph/output."""
import ast
import os
from pathlib import Path

import pytest


class Configuration:
    def __init__(self, name): self.name = name
    def perform(self, context): return context.get(self.name, '')


def gate():
    path = Path(__file__).resolve().parents[1]/'ros2_ws/src/robot_scene_nav_bringup/launch/robot_scene_nav2.launch.py'
    tree = ast.parse(path.read_text())
    fn = next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='_foxy_execution_gate')
    namespace = {'os':os,'Path':Path,'LaunchConfiguration':Configuration}
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(path),'exec'),namespace)
    return namespace[fn.name]


def isolated(monkeypatch,tmp_path):
    path = tmp_path/'dds.xml'
    path.write_text('<CycloneDDS><Domain><General><NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress>'
        '<AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer Address="127.0.0.1"/>'
        '</Peers></Discovery></Domain></CycloneDDS>')
    for name,value in {'ROS_DISTRO':'foxy','ROS_DOMAIN_ID':'99','ROS_LOCALHOST_ONLY':'1',
            'RMW_IMPLEMENTATION':'rmw_cyclonedds_cpp','CYCLONEDDS_URI':'file://'+str(path)}.items():
        monkeypatch.setenv(name,value)
    return {'execution_enabled':'true','foxy_isolated_validation':'true',
            'nav2_cmd_vel_topic':'/go2w/acceptance/nav2_cmd_vel'}, path


def test_production_foxy_execution_remains_closed(monkeypatch,tmp_path):
    context,_ = isolated(monkeypatch,tmp_path)
    context['foxy_isolated_validation'] = 'false'
    with pytest.raises(RuntimeError,match='Foxy execution is disabled'):gate()(context)


def test_explicit_isolated_validation_is_allowed(monkeypatch,tmp_path):
    context,_ = isolated(monkeypatch,tmp_path)
    assert gate()(context)==[]


@pytest.mark.parametrize('key,value', [('ROS_DOMAIN_ID','0'),('ROS_LOCALHOST_ONLY','0'),
    ('CYCLONEDDS_URI',''),('RMW_IMPLEMENTATION','rmw_fastrtps_cpp')])
def test_unchecked_or_production_transport_is_rejected(monkeypatch,tmp_path,key,value):
    context,_ = isolated(monkeypatch,tmp_path);monkeypatch.setenv(key,value)
    with pytest.raises(RuntimeError):gate()(context)


@pytest.mark.parametrize('topic', ['/go2w/nav2_cmd_vel','/go2w/cmd_vel_selected','/cmd_vel_nav_raw'])
def test_isolated_flag_cannot_enable_a_physical_output(monkeypatch,tmp_path,topic):
    context,_ = isolated(monkeypatch,tmp_path);context['nav2_cmd_vel_topic']=topic
    with pytest.raises(RuntimeError):gate()(context)


@pytest.mark.parametrize('before,after', [('127.0.0.1','192.168.123.18'),('false','true')])
def test_dds_content_is_checked(monkeypatch,tmp_path,before,after):
    context,path = isolated(monkeypatch,tmp_path)
    path.write_text(path.read_text().replace(before,after))
    with pytest.raises(RuntimeError):gate()(context)


def test_desktop_execution_and_foxy_plan_only_keep_their_existing_gate(monkeypatch,tmp_path):
    context,_=isolated(monkeypatch,tmp_path)
    monkeypatch.setenv('ROS_DISTRO','humble');assert gate()(context)==[]
    monkeypatch.setenv('ROS_DISTRO','foxy');context['execution_enabled']='false'
    assert gate()(context)==[]


def test_supervised_path_still_rejects_the_workstation(monkeypatch,tmp_path):
    context,_=isolated(monkeypatch,tmp_path)
    context.update(foxy_isolated_validation='false',foxy_supervised_trial_ledger=str(tmp_path/'ledger.json'))
    with pytest.raises(RuntimeError,match='native robot'):gate()(context)


def test_native_dispatch_checks_the_same_one_use_ledger(monkeypatch,tmp_path):
    import platform
    import sys
    from app.navigation.trial_budget import NavigationTrialBudget
    import app.navigation.foxy_supervised_trial as policy
    boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    real=tmp_path/'ledger.json'
    NavigationTrialBudget.initialize(real,window_id='one',reviewed_robot_boot_id=boot,
        maximum_speed_mps=.12,maximum_steps=1)
    selected='/home/unitree/robotscene_runtime/tests/ledger.json'
    original=policy.validate_supervised_trial
    calls=[]
    def validate(path,**kwargs):
        assert str(path)==selected;calls.append(kwargs['boot_id'])
        return original(real,**kwargs)
    monkeypatch.setattr(policy,'validate_supervised_trial',validate)
    monkeypatch.setattr(platform,'machine',lambda:'aarch64')
    is_file=Path.is_file
    monkeypatch.setattr(Path,'is_file',lambda p:True if str(p)=='/etc/nv_tegra_release' else is_file(p))
    monkeypatch.setattr(sys,'path',list(sys.path))
    for key,value in dict(ROS_DISTRO='foxy',ROS_DOMAIN_ID='0',ROS_LOCALHOST_ONLY='0',
        RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',GO2W_AREA_CLEARED='I_HAVE_CLEARED_THE_AREA',
        GO2W_RELEASE_ROOT='/home/unitree/robotscene_releases/reviewed_candidate').items():
        monkeypatch.setenv(key,value)
    context=dict(execution_enabled='true',foxy_isolated_validation='false',
        foxy_supervised_trial_ledger=selected,nav2_cmd_vel_topic='/go2w/nav2_cmd_vel')
    assert gate()(context)==[]
    assert calls==[boot]
    import json
    data=json.loads(real.read_text());data['steps']=[{'consumed':True}];real.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='steps'):gate()(context)


def test_collision_output_is_merged_before_node_specific_yaml_can_override_it():
    import yaml
    root=Path(__file__).resolve().parents[1]/'ros2_ws/src/robot_scene_nav_bringup'
    path=root/'launch/collision_monitor.launch.py'
    tree=ast.parse(path.read_text())
    fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='_parameters')
    namespace={}
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(path),'exec'),namespace)
    config=root/'config/collision_monitor.yaml'
    original=yaml.safe_load(config.read_text())['collision_monitor']['ros__parameters']
    actual=namespace['_parameters'](config,False,'/go2w/acceptance/nav2_cmd_vel')
    assert actual['cmd_vel_out_topic']=='/go2w/acceptance/nav2_cmd_vel'
    assert actual['Stop']==original['Stop'] and actual['scan']==original['scan']
    assert yaml.safe_load(config.read_text())['collision_monitor']['ros__parameters']==original
