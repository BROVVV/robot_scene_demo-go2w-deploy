"""Exercise process ownership and orphan cleanup without ROS or robot access."""
import os
from pathlib import Path
import signal
import subprocess
import sys


def stop_code():
    source=(Path(__file__).resolve().parents[1]/'scripts/go2w/stop_2d_localization.sh').read_text()
    return source.split("<<'PY'\n",1)[1].rsplit('\nPY',1)[0]


def test_full_stack_stop_invokes_owned_localization_stop_and_propagates_failure():
    source=(Path(__file__).resolve().parents[1]/'scripts/go2w/stop_all.sh').read_text()
    assert 'localization_stop="${script_dir}/stop_2d_localization.sh"' in source
    assert 'bash "${localization_stop}" || {' in source
    assert 'full-stack stop is incomplete' in source


def pidfile(root,pid):
    p=root/'outputs/autonomous_search/runtime/localization_2d.pid'
    p.parent.mkdir(parents=True);p.write_text(str(pid));return p


def test_mapper_orphan_group_is_stopped_after_launch_parent_exits(tmp_path):
    script=tmp_path/'ros2_ws/install/robot_scene_nav_bringup/lib/robot_scene_nav_bringup/fake_mapper.py'
    script.parent.mkdir(parents=True)
    script.write_text('import subprocess,sys,time\n'
        'if "--child" in sys.argv: time.sleep(60)\n'
        'else: subprocess.Popen([sys.executable,__file__,"--child"],stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)\n')
    parent=subprocess.Popen([sys.executable,str(script)],start_new_session=True)
    pgid=parent.pid;p=pidfile(tmp_path,pgid);parent.wait(timeout=5)
    try:
        result=subprocess.run([sys.executable,'-c',stop_code(),str(tmp_path)],capture_output=True,text=True,timeout=12)
        assert result.returncode==0,result.stderr
        assert not p.exists()
    finally:
        try:os.killpg(pgid,signal.SIGKILL)
        except ProcessLookupError:pass


def test_foreign_process_is_refused_and_pidfile_is_retained(tmp_path):
    child=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'],start_new_session=True)
    p=pidfile(tmp_path,child.pid)
    try:
        result=subprocess.run([sys.executable,'-c',stop_code(),str(tmp_path)],capture_output=True,text=True,timeout=5)
        assert result.returncode!=0 and 'refusing foreign process' in result.stderr
        assert child.poll() is None and p.exists()
    finally:child.kill();child.wait(timeout=5)


def test_separately_owned_floor_overlay_stops_even_without_main_pidfile(tmp_path):
    script=tmp_path/'ros2_ws/install/robot_scene_nav_bringup/lib/robot_scene_nav_bringup/observed_floor_map.py'
    script.parent.mkdir(parents=True);script.write_text('import time;time.sleep(60)\n')
    child=subprocess.Popen([sys.executable,str(script)],start_new_session=True)
    p=tmp_path/'outputs/autonomous_search/runtime/localization_floor_overlay.pid'
    p.parent.mkdir(parents=True);p.write_text(str(child.pid))
    try:
        result=subprocess.run([sys.executable,'-c',stop_code(),str(tmp_path)],capture_output=True,text=True,timeout=12)
        assert result.returncode==0,result.stderr
        child.wait(timeout=5)
        assert not p.exists()
    finally:
        if child.poll() is None:child.kill();child.wait(timeout=5)
