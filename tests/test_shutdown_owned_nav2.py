"""Lifecycle shutdown must use the verified launcher's DDS and manager."""
from pathlib import Path
import importlib.util
import pytest

path=Path(__file__).resolve().parents[1]/'scripts/go2w/shutdown_owned_nav2.py'
spec=importlib.util.spec_from_file_location('shutdown_owned_nav2',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def process(proc,pid,parent,argv,environment=None):
    p=proc/str(pid);p.mkdir()
    (p/'cmdline').write_bytes(b'\0'.join(a.encode() for a in argv)+b'\0')
    (p/'status').write_text('Name:\ttest\nPPid:\t%d\n'%parent)
    if environment is not None:
        (p/'environ').write_bytes(b'\0'.join((k+'='+v).encode() for k,v in environment.items()))


def owner(proc,distro='foxy'):
    process(proc,100,1,['/usr/bin/python3','/opt/ros/'+distro+'/bin/ros2',
        'launch','/release/robot_scene_nav2.launch.py'],
        {'ROS_DISTRO':distro,'ROS_DOMAIN_ID':'99','CYCLONEDDS_URI':'file:///owner/loopback.xml'})


def manager(proc,pid=110,parent=100,name=module.MANAGERS[0]):
    process(proc,pid,parent,['/opt/ros/foxy/lib/nav2_lifecycle_manager/lifecycle_manager',
        '--ros-args','-r','__node:='+name])


@pytest.mark.parametrize('name',module.MANAGERS)
def test_uses_actual_owner_environment_and_child_manager(tmp_path,name):
    owner(tmp_path);manager(tmp_path,name=name)
    env,child=module.inspect_launch(100,tmp_path)
    assert child==(110,name)
    assert env['ROS_DOMAIN_ID']=='99' and env['CYCLONEDDS_URI']=='file:///owner/loopback.xml'


def test_manager_can_be_a_grandchild(tmp_path):
    owner(tmp_path);process(tmp_path,105,100,['intermediate']);manager(tmp_path,parent=105)
    assert module.inspect_launch(100,tmp_path)[1][0]==110


@pytest.mark.parametrize('mode',['foreign','duplicate','missing','cycle'])
def test_ambiguous_or_foreign_owner_is_rejected(tmp_path,mode):
    owner(tmp_path)
    if mode=='foreign':manager(tmp_path,parent=999)
    elif mode=='duplicate':manager(tmp_path);manager(tmp_path,pid=111,parent=999)
    elif mode=='cycle':manager(tmp_path,parent=110)
    with pytest.raises(ValueError,match='exactly one'):module.inspect_launch(100,tmp_path)


def test_unrelated_localizer_manager_does_not_grant_or_block_ownership(tmp_path):
    owner(tmp_path);manager(tmp_path)
    manager(tmp_path,pid=111,parent=999,name='lifecycle_manager_localization_2d')
    assert module.inspect_launch(100,tmp_path)[1][0]==110


def test_foreign_launch_script_is_rejected(tmp_path):
    process(tmp_path,100,1,['ros2','launch','vendor.launch.py'],{'ROS_DISTRO':'foxy'})
    with pytest.raises(ValueError,match='not the project'):module.inspect_launch(100,tmp_path)


def test_program_merely_mentioning_ros2_launch_is_rejected(tmp_path):
    process(tmp_path,100,1,['/usr/bin/printf','ros2','launch','robot_scene_nav2.launch.py'],
            {'ROS_DISTRO':'foxy'})
    with pytest.raises(ValueError,match='ros2 launch owner'):module.inspect_launch(100,tmp_path)


def test_humble_stop_behavior_is_preserved(tmp_path):
    owner(tmp_path,'humble')
    env,child=module.inspect_launch(100,tmp_path)
    assert env['ROS_DISTRO']=='humble' and child is None


@pytest.mark.parametrize('verb',['launch','run'])
def test_background_launcher_does_not_inherit_ignored_sigint(tmp_path,verb):
    import json
    import os
    import subprocess
    import sys
    executable=tmp_path/'ros2'
    executable.write_text('#!'+sys.executable+'\nimport json,signal\n'
        'print(json.dumps({"ignored":signal.getsignal(signal.SIGINT)==signal.SIG_IGN}))\n')
    executable.chmod(0o755)
    env=dict(os.environ,PATH=str(tmp_path)+os.pathsep+os.environ['PATH'])
    baseline=subprocess.run(['bash','-c','setsid ros2 "$1" robot_scene_nav_bringup robot_scene_nav2.launch.py & wait','_',verb],
        env=env,capture_output=True,text=True,timeout=5,check=True)
    assert json.loads(baseline.stdout)['ignored'] is True
    helper=path.parent/'exec_ros2.py'
    fixed=subprocess.run(['bash','-c','setsid /usr/bin/python3 "$1" "$2" robot_scene_nav_bringup robot_scene_nav2.launch.py & wait',
        '_',str(helper),verb],env=env,capture_output=True,text=True,timeout=5,check=True)
    assert json.loads(fixed.stdout)['ignored'] is False


@pytest.mark.parametrize('verb', ['launch', 'run'])
def test_perception_entry_restores_sigint_and_preserves_owned_pid(tmp_path, verb):
    import json
    import os
    import subprocess
    import sys
    script_dir = path.parent
    source = (script_dir/'start_live_perception.sh').read_text()
    function = 'start_read_only_node() {' + source.split(
        'start_read_only_node() {', 1)[1].split('\n}\n', 1)[0] + '\n}\n'
    executable = tmp_path/'ros2'
    executable.write_text('#!'+sys.executable+'\nimport json,os,signal\n'
        'print(json.dumps({"ignored":signal.getsignal(signal.SIGINT)==signal.SIG_IGN,'
        '"pid":os.getpid(),"pgid":os.getpgrp()}))\n')
    executable.chmod(0o755)
    environment = dict(os.environ, PATH=str(tmp_path)+os.pathsep+os.environ['PATH'])
    script = ('set -e\nscript_dir="$1"\nlog_root="$2"\npid_root="$2"\n'
              'process_ids=()\n'+function+
              'start_read_only_node fixture ros2 "$3" package entry\n'
              'wait "${process_ids[0]}"\n')
    subprocess.run(['bash', '-c', script, '_', str(script_dir), str(tmp_path), verb],
                   env=environment, capture_output=True, text=True, timeout=5, check=True)
    result = json.loads((tmp_path/'fixture.log').read_text())
    assert result['ignored'] is False
    assert result['pid'] == result['pgid'] == int((tmp_path/'fixture.pid').read_text())
