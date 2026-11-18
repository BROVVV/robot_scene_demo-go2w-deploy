"""Native six-node Nav2 plus real Python consumers; loopback/synthetic only."""
from pathlib import Path
import argparse,json,os,time,signal,subprocess,re,sys,traceback
import psutil
ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT),str(ROOT/'compat/foxy_nav2/probes')]
from velocity_chain import isolated_environment
from stop_trace import evaluate_stop
# Frozen02 predates stop_trace; caller supplies the independently verified probe helper.
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav2_msgs.action import ComputePathToPose,NavigateToPose
from lifecycle_msgs.srv import GetState

isolated_environment()
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output-dir',type=Path,required=True)
args=parser.parse_args()
OUT=args.output_dir.resolve();OUT.mkdir(parents=True,exist_ok=False)
report={'scope':'six native Nav2 nodes, real health monitor and search pose reader; synthetic source only',
    'physical_motion_sent':False,'sdk_or_robot_bridge_started':False,'source_release':ROOT.name,
    'domain':99,'started_wall':time.time(),'phases':[],'nav_runs':[],'passed':False}
children={};nav=None;navlog=None;launch_tag=None
rclpy.init();node=rclpy.create_node('native_full_tf_rebuild_supervisor')
commands=[];physical=[];upstream=[];guard_status=[]
node.create_subscription(Twist,'/go2w/acceptance/nav2_cmd_vel',lambda m:commands.append(
    (time.monotonic(),m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z)),100)
for topic in ['/cmd_vel_nav_raw','/cmd_vel_nav_smoothed','/cmd_vel_nav_collision_checked']:
    node.create_subscription(Twist,topic,lambda m,t=topic:upstream.append((t,time.monotonic(),m.linear.x,m.angular.z)),100)
node.create_subscription(String,'/go2w_navigation_velocity_guard/status',lambda m:guard_status.append((time.monotonic(),m.data)),100)
for topic in ['/cmd_vel','/go2w/nav2_cmd_vel','/go2w/cmd_vel_selected']:
    node.create_subscription(Twist,topic,lambda m,t=topic:physical.append((t,time.monotonic())),qos_profile_sensor_data)
planner=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
navigator=ActionClient(node,NavigateToPose,'/navigate_to_pose')
def read(name):
    p=OUT/name
    return json.loads(p.read_text()) if p.exists() else {}
def write(name,value):
    p=OUT/name;t=p.with_suffix('.tmp');t.write_text(json.dumps(value,indent=2)+'\n');t.replace(p)
def pump(duration=.03):
    end=time.monotonic()+duration
    while time.monotonic()<end:
        assert time.time()-report['started_wall']<170,'short test deadline'
        assert all(p.poll() is None or p.returncode==0 for p in children.values()),'role exited abnormally'
        if nav is not None:assert nav.poll() is None,'Nav2 exited unexpectedly'
        rclpy.spin_once(node,timeout_sec=.01)
def wait(future,timeout=10.):
    end=time.monotonic()+timeout
    while not future.done() and time.monotonic()<end:pump()
    assert future.done(),'native action/service timeout'
    return future.result()
def start_role(role,tag):
    assert tag not in children
    with (OUT/(tag+'.log')).open('x') as log:
        children[tag]=subprocess.Popen(['/usr/bin/python3',str(HERE/'tf_rebuild_roles.py'),role,str(OUT),'--tag',tag],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
def stop_role(tag):
    p=children[tag]
    if p.poll() is None:p.send_signal(signal.SIGINT)
    try:p.wait(timeout=6)
    except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=3)
    assert p.returncode==0,(tag,p.returncode)
def phase(tag,predicate,timeout=10.,count=3):
    after=time.monotonic();seen=set();samples=[]
    end=after+timeout
    while time.monotonic()<end:
        pump();row=read(tag+'_state.json');stamp=row.get('status_received',0.)
        if stamp>after and row.get('boolean_received',0.)>after and stamp not in seen and row.get('received',0.)-stamp<.3 and predicate(row):
            seen.add(stamp);samples.append(row)
            if len(samples)>=count:
                report['phases'].append({'tag':tag,'samples':samples});return samples[-1]
    raise TimeoutError((tag,read(tag+'_state.json')))
def healthy(row):return row.get('boolean') is True and row.get('status',{}).get('valid') is True
def correct(row):return healthy(row) and row.get('pose') is not None and abs(row['pose']['x']-1.2)<1e-5
def poisoned(row):return row.get('raw_pose') is not None and abs(row['raw_pose']['x']-3.2)<1e-5
def closed(row):return row.get('boolean') is False and row.get('status',{}).get('valid') is False and 'tf_future_cache_recovered' in row.get('status',{}).get('blocking',[])
def start_nav(tag):
    global nav,navlog,launch_tag
    launch_tag=tag;navlog=(OUT/(tag+'_launch.log')).open('x')
    nav=subprocess.Popen(['ros2','launch',str(ROOT/'ros2_ws/src/robot_scene_nav_bringup/launch/robot_scene_nav2.launch.py'),
        'execution_enabled:=true','foxy_isolated_validation:=true','nav2_cmd_vel_topic:=/go2w/acceptance/nav2_cmd_vel',
        'params_file:='+str(ROOT/'ros2_ws/src/robot_scene_nav_bringup/config/nav2_params_foxy_execution.yaml')],stdout=navlog,stderr=subprocess.STDOUT,start_new_session=True)
    report['nav_runs'].append({'tag':tag,'pid':nav.pid,'start_time':psutil.Process(nav.pid).create_time()})
    for name in ['planner_server','controller_server','behavior_server','bt_navigator','velocity_smoother','collision_monitor']:
        client=node.create_client(GetState,'/'+name+'/get_state')
        try:
            end=time.monotonic()+25
            while time.monotonic()<end:
                if client.service_is_ready():
                    state=wait(client.call_async(GetState.Request())).current_state
                    if state.id==3:break
                pump(.1)
            else:raise TimeoutError('lifecycle not active: '+name)
        finally:node.destroy_client(client)
    end=time.monotonic()+8
    while not planner.server_is_ready() or not navigator.server_is_ready():
        assert time.monotonic()<end,'Nav2 action discovery';pump()
    for topic in ['/go2w/nav2_cmd_vel','/go2w/cmd_vel_selected','/go2w/software_arm']:
        assert node.count_publishers(topic)==0,'unexpected physical publisher '+topic
def stop_nav():
    global nav,navlog
    if nav is None:return
    row=report['nav_runs'][-1];owned=[]
    if nav.poll() is None:
        owned=psutil.Process(nav.pid).children(recursive=True)
        with (OUT/(launch_tag+'_shutdown.log')).open('x') as log:
            helper=subprocess.Popen(['/usr/bin/python3',str(ROOT/'scripts/go2w/shutdown_owned_nav2.py'),'--launch-pid',str(nav.pid)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                end=time.monotonic()+30
                while helper.poll() is None and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
                assert helper.poll() is not None,'lifecycle helper timed out'
            finally:
                if helper.poll() is None:helper.kill();helper.wait(timeout=3)
        lines=(OUT/(launch_tag+'_shutdown.log')).read_text().splitlines();rows=[json.loads(s) for s in lines if s.startswith('{')]
        row['lifecycle_shutdown']=rows[-1] if rows else {}
        row['helper_exit']=helper.returncode
        nav.send_signal(signal.SIGINT)
        try:nav.wait(timeout=12)
        except subprocess.TimeoutExpired:
            row['forced_shutdown']=True;os.killpg(nav.pid,signal.SIGTERM)
            try:nav.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(nav.pid,signal.SIGKILL);nav.wait(timeout=3)
    row['remaining_children']=[]
    for child in owned:
        try:
            if child.is_running() and child.status()!=psutil.STATUS_ZOMBIE:row['remaining_children'].append(child.pid);child.terminate()
        except psutil.Error:pass
    row['launch_exit']=nav.returncode;navlog.close()
    text=(OUT/(launch_tag+'_launch.log')).read_text();exits={pid:0 for pid in re.findall(r'process has finished cleanly \[pid (\d+)\]',text)}
    exits.update({pid:int(code) for pid,code in re.findall(r'process has died \[pid (\d+), exit code (-?\d+)',text)})
    row['child_exit_codes']=exits
    row['cleanup_passed']=row.get('helper_exit')==0 and row.get('lifecycle_shutdown',{}).get('success') and row['launch_exit']==0 and not row['remaining_children'] and not row.get('forced_shutdown') and len(exits)==8 and all(v==0 for v in exits.values())
    nav=None;navlog=None
    assert row['cleanup_passed'],row
def plan(tag,expected):
    goal=ComputePathToPose.Goal();p=goal.goal if hasattr(goal,'goal') else goal.pose
    p.header.frame_id='map';p.header.stamp=node.get_clock().now().to_msg();p.pose.position.x=2.5;p.pose.position.y=1.5;p.pose.orientation.w=1.
    goal.planner_id='GridBased';handle=wait(planner.send_goal_async(goal));assert handle and handle.accepted
    result=wait(handle.get_result_async());path=result.result.path.poses
    sample={'tag':tag,'status':result.status,'poses':len(path),'start_x':path[0].pose.position.x if path else None,'wall':time.time()}
    report['phases'].append(sample);assert result.status==4 and path and abs(sample['start_x']-expected)<.15,sample
    return sample
try:
    write('control.json',{'seq':0,'fault':'normal'});start_role('source','source');start_role('monitor','monitor_01');start_role('consumer','reader_01');start_nav('nav_01')
    phase('reader_01',correct);plan('baseline',1.2)
    goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=node.get_clock().now().to_msg();goal.pose.pose.position.x=2.5;goal.pose.pose.position.y=1.5;goal.pose.pose.orientation.w=1.
    handle=wait(navigator.send_goal_async(goal));assert handle.accepted
    after=time.monotonic();end=after+4
    while not any(r[0]>after and r[1]>.05 for r in commands):
        assert time.monotonic()<end,'no isolated motion before fault';pump()
    write('control.json',{'seq':1,'fault':'future'})
    phase('reader_01',lambda r:closed(r) and poisoned(r))
    ack=read('first_control_ack_1.json');report['future_stamp_wall']=ack['future_stamp'];fault_at=ack['published_at']
    first_invalid=report['phases'][-1]['samples'][0]['status_received']
    pump(.4);report['fault_stop_from_first_invalid_status']=evaluate_stop(commands,first_invalid,deadline=.3)
    report['fault_stop_from_first_bad_tf']=evaluate_stop(commands,fault_at,deadline=.3)
    wait(handle.cancel_goal_async());wait(handle.get_result_async())
    write('control.json',{'seq':2,'fault':'normal'})
    phase('reader_01',lambda r:closed(r) and poisoned(r) and r['status']['blocking']==['tf_future_cache_recovered'])
    plan('normal_source_but_cpp_still_poisoned',3.2)
    stop_role('monitor_01');start_role('monitor','monitor_02')
    phase('reader_01',lambda r:healthy(r) and poisoned(r) and r.get('pose') is None)
    plan('monitor_only_restarted_cpp_still_poisoned',3.2)
    report['monitor_only_restart_incomplete_reproduced']=True
    stop_nav();stop_role('reader_01');stop_role('monitor_02')
    start_role('monitor','monitor_03');start_role('consumer','reader_02');start_nav('nav_02')
    phase('reader_02',correct);restored=plan('all_owned_consumers_rebuilt_normal_source',1.2)
    report['recovered_before_future_stamp']=restored['wall']<report['future_stamp_wall']-5;assert report['recovered_before_future_stamp']
    # Do not reopen by rebuilding while the producer is still faulty.
    write('control.json',{'seq':3,'fault':'future'});phase('reader_02',lambda r:closed(r) and poisoned(r))
    stop_nav();stop_role('reader_02');stop_role('monitor_03')
    start_role('monitor','monitor_04');start_role('consumer','reader_03');start_nav('nav_03')
    phase('reader_03',lambda r:closed(r) and poisoned(r) and r.get('pose') is None)
    report['full_rebuild_with_bad_source_stays_closed']=True
    assert not physical,'physical velocity topic observed';report['passed']=True
except BaseException as exc:report.update(error=repr(exc),traceback=traceback.format_exc())
finally:
    try:stop_nav()
    except BaseException as exc:report.update(passed=False,cleanup_error=repr(exc))
    for tag in reversed(list(children)):
        try:stop_role(tag)
        except BaseException as exc:report.setdefault('role_cleanup_errors',[]).append([tag,repr(exc)]);report['passed']=False
    report['role_exit_codes']={tag:p.returncode for tag,p in children.items()}
    report['role_cleanup']={tag:read(tag+'_cleanup.json') for tag in children if tag!='source'}
    report['physical_velocity_messages']=physical
    report['cleanup_passed']=all(p.poll()==0 for p in children.values()) and all(x['cleanup_passed'] for x in report['nav_runs']) and all(c and not c.get('errors') and c.get('action_sends')==0 and c.get('twist_messages')==0 for c in report['role_cleanup'].values())
    report['passed']=report['passed'] and report['cleanup_passed']
    write('upstream_commands.json',upstream);write('guard_status.json',guard_status);write('commands_full.json',commands);planner.destroy();navigator.destroy();node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
    report['ended_wall']=time.time();write('result.json',report)
print(json.dumps(report));raise SystemExit(0 if report['passed'] else 2)
