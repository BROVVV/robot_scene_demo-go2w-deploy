#!/usr/bin/python3
"""Real sensor/TF shadow Nav2 plus optional native vision load, output isolated."""
from pathlib import Path
import argparse,json,math,os,signal,subprocess,time,re
from velocity_chain import isolated_environment
from stop_trace import evaluate_stop


def record_cancel_case(report, output, *, cycle, active_started, triggered,
                       action_status, commands, statuses):
    """Persist the trigger and failed trace before propagating a stop failure."""
    case = dict(kind='cancel', cycle=cycle, active_started=active_started,
                triggered=triggered, action_status=action_status, passed=False)
    case['freshness_blocks'] = [s for s in statuses
        if active_started + .25 <= s['monotonic'] < triggered and s['reason'].split('_waiting_controller_stop',1)[0] in
        {'controller_command_stale', 'smoothed_command_stale', 'scan_stale', 'localization_stale'}]
    report['cases'].append(case)
    try:
        assert action_status == 5, action_status
        case.update(evaluate_stop(commands, triggered, deadline=.3))
        case['passed'] = True
    except BaseException as exc:
        case['error'] = repr(exc)
        raise
    finally:
        output.write_text(json.dumps(report, indent=2) + '\n')
    return case


def main():
    isolated_environment();parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True);parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--sink-pid',type=int,required=True);parser.add_argument('--ml-script',type=Path)
    parser.add_argument('--sink-start-ticks',type=int,required=True)
    parser.add_argument('--collect-vision-after-cancel-failure',action='store_true',
                        help='Keep the cancelled, isolated chain idle for a bounded vision diagnostic; never retry a goal')
    args=parser.parse_args();root=args.root.resolve();out=args.output_dir.resolve();out.mkdir(exist_ok=False)
    import rclpy,psutil
    sink=psutil.Process(args.sink_pid);sink_args=sink.cmdline()
    assert len(sink_args)>2 and Path(sink_args[1]).name=='live_inputs.py' and sink_args[2]=='sink'
    assert sink.environ().get('ROS_DOMAIN_ID')=='99'
    assert sink.environ().get('CYCLONEDDS_URI')==os.environ['CYCLONEDDS_URI']
    def sink_same():
        return int(Path('/proc',str(args.sink_pid),'stat').read_text().rsplit(')',1)[1].split()[19])==args.sink_start_ticks
    assert sink_same()
    from rclpy.action import ActionClient
    from rclpy.time import Time
    from geometry_msgs.msg import Twist
    from nav2_msgs.action import NavigateToPose
    from lifecycle_msgs.srv import GetState
    from tf2_ros import Buffer,TransformListener
    isolated_output='/go2w/acceptance/live_nav2_cmd_vel'
    log=(out/'launch.log').open('x');process=subprocess.Popen(['ros2','launch',str(root/'ros2_ws/src/robot_scene_nav_bringup/launch/robot_scene_nav2.launch.py'),
        'execution_enabled:=true','foxy_isolated_validation:=true','nav2_cmd_vel_topic:='+isolated_output],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    report={'started_wall':time.time(),'scope':'real live map/scan/odom/TF in loopback domain99, shadow Nav2 only',
        'physical_motion_enabled':False,'sensor_stamps_changed':False,'cases':[],'passed':False}
    rclpy.init();node=rclpy.create_node('acceptance_live_navigation_observer');buffer=Buffer();listener=TransformListener(buffer,node)
    commands=[];raw=[];statuses=[];ml=None;handle=None;sink_paused=False
    def command(m):commands.append((time.monotonic(),m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z))
    node.create_subscription(Twist,isolated_output,command,100)
    node.create_subscription(Twist,'/cmd_vel_nav_raw',lambda m:raw.append((time.monotonic(),m.linear.x,m.angular.z)),100)
    from std_msgs.msg import String
    node.create_subscription(String,'/go2w_navigation_velocity_guard/status',lambda m:statuses.append({'monotonic':time.monotonic(),'reason':m.data}),100)
    action=ActionClient(node,NavigateToPose,'/navigate_to_pose')
    def pump(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            assert process.poll() is None,'shadow Nav2 exited'
    def wait(future,timeout=8.):
        end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end:pump(.02)
        assert future.done(),'shadow Nav2 future timeout'
        return future.result()
    def moving(timeout=4.):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if commands and time.monotonic()-commands[-1][0]<.15 and commands[-1][1]>.05:return
            pump(.02)
        raise AssertionError('no fresh forward shadow output before stop case')
    def start():
        pose=buffer.lookup_transform('map','base_link',Time()).transform
        q=pose.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=node.get_clock().now().to_msg()
        goal.pose.pose.position.x=pose.translation.x+.6*math.cos(yaw);goal.pose.pose.position.y=pose.translation.y+.6*math.sin(yaw)
        goal.pose.pose.orientation=q
        h=wait(action.send_goal_async(goal));assert h and h.accepted
        return h
    try:
        deadline=time.monotonic()+45
        while not action.server_is_ready() and time.monotonic()<deadline:pump(.05)
        assert action.server_is_ready()
        for name in ['planner_server','controller_server','behavior_server','bt_navigator','velocity_smoother','collision_monitor']:
            c=node.create_client(GetState,'/'+name+'/get_state');assert c.wait_for_service(timeout_sec=3)
            deadline=time.monotonic()+20
            while time.monotonic()<deadline:
                state=wait(c.call_async(GetState.Request())).current_state
                if state.id==3:break
                pump(.1)
            else:raise AssertionError((name,state.label))
            node.destroy_client(c)
        deadline=time.monotonic()+10
        while (node.count_publishers(isolated_output)!=1 or not buffer.can_transform('map','base_link',Time())) and time.monotonic()<deadline:pump(.05)
        assert node.count_publishers(isolated_output)==1
        assert all(node.count_publishers(t)==0 for t in ['/go2w/nav2_cmd_vel','/go2w/cmd_vel_selected','/go2w/software_arm'])
        if args.ml_script:
            ml_log=(out/'ml_stdout.log').open('x');ml=subprocess.Popen(['/usr/bin/python3',str(args.ml_script)],stdout=ml_log,stderr=subprocess.STDOUT,start_new_session=True)
        end=time.monotonic()+70;cycles=0
        while time.monotonic()<end and (cycles<3 or (ml is not None and ml.poll() is None)):
            handle=start();moving();active=time.monotonic();pump(1.2)
            # Cancellation must also work after the guard has fault-stopped
            # this goal. Requiring another moving output here would reject
            # the deliberate latch before its reset/cancel can be exercised.
            triggered=time.monotonic()
            report['pending_cancel']=dict(cycle=cycles,active_started=active,triggered=triggered)
            (out/'live.json').write_text(json.dumps(report,indent=2)+'\n')
            wait(handle.cancel_goal_async());result=wait(handle.get_result_async());pump(.45)
            record_cancel_case(report,out/'live.json',cycle=cycles,active_started=active,
                               triggered=triggered,action_status=result.status,
                               commands=commands,statuses=statuses)
            report.pop('pending_cancel',None);handle=None;cycles+=1
            (out/'live.json').write_text(json.dumps(report,indent=2)+'\n')
        if ml is not None:assert ml.poll()==0,'native vision load failed or exceeded shadow window'
        handle=start();moving();assert sink_same();frozen=time.monotonic();os.kill(args.sink_pid,signal.SIGSTOP);sink_paused=True
        pump(.9);report['sensor_delivery_pause']=evaluate_stop(commands,frozen,deadline=.3)
        os.kill(args.sink_pid,signal.SIGCONT);sink_paused=False
        wait(handle.cancel_goal_async());wait(handle.get_result_async());pump(.4);handle=None
        report['concurrency_clean']=not any(c['freshness_blocks'] for c in report['cases'])
        report['passed']=report['concurrency_clean']
    except BaseException as exc:
        report.update(error=repr(exc))
        # The failed stop remains a failure. Only finish the already-owned ML
        # request after an acknowledged cancellation, with no new goal issued.
        if (args.collect_vision_after_cancel_failure and report['cases']
                and report['cases'][-1].get('action_status')==5
                and not report['cases'][-1].get('passed') and ml is not None):
            handle=None;begin=time.monotonic();end=begin+60
            try:
                while ml.poll() is None and time.monotonic()<end:pump(.02)
                report['vision_after_failed_cancel']={
                    'scope':'cancelled shadow Nav2, no new goal; not active-navigation concurrency',
                    'elapsed_s':time.monotonic()-begin,'exit_code':ml.poll(),
                    'nonzero_samples':[row for row in commands if row[0]>=begin and any(abs(v)>=1e-8 for v in row[1:])]}
            except BaseException as diagnostic_exc:report['vision_after_failed_cancel_error']=repr(diagnostic_exc)
        raise
    finally:
        if sink_paused:os.kill(args.sink_pid,signal.SIGCONT)
        if handle is not None:
            try:wait(handle.cancel_goal_async(),3.)
            except Exception as exc:report['cancel_cleanup_error']=repr(exc)
        if ml is not None:
            if ml.poll() is None:
                os.killpg(ml.pid,signal.SIGTERM)
                try:ml.wait(timeout=6)
                except subprocess.TimeoutExpired:os.killpg(ml.pid,signal.SIGKILL);ml.wait(timeout=3)
            report['ml_exit_code']=ml.returncode;ml_log.close()
        helper=None
        try:
            with (out/'lifecycle_shutdown.log').open('x') as f:
                helper=subprocess.Popen(['/usr/bin/python3',str(root/'scripts/go2w/shutdown_owned_nav2.py'),'--launch-pid',str(process.pid)],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
                end=time.monotonic()+35
                while helper.poll() is None and time.monotonic()<end:pump(.02)
                assert helper.poll()==0,'lifecycle shutdown failed'
            report['lifecycle_shutdown']=json.loads([s for s in (out/'lifecycle_shutdown.log').read_text().splitlines() if s.startswith('{')][-1])
        except BaseException as exc:report['shutdown_error']=repr(exc)
        finally:
            if helper is not None and helper.poll() is None:
                os.killpg(helper.pid,signal.SIGTERM)
                try:helper.wait(timeout=3.)
                except subprocess.TimeoutExpired:os.killpg(helper.pid,signal.SIGKILL);helper.wait(timeout=3.)
        children=[]
        if process.poll() is None:
            children=psutil.Process(process.pid).children(recursive=True);process.send_signal(signal.SIGINT)
        try:process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3)
        report['launch_exit_code']=process.returncode
        report['remaining_children']=[]
        for c in children:
            try:
                if c.is_running() and c.status()!=psutil.STATUS_ZOMBIE:
                    report['remaining_children'].append(c.pid);c.terminate()
            except psutil.Error:pass
        log.close();text=(out/'launch.log').read_text();exits={pid:0 for pid in re.findall(r'process has finished cleanly \[pid (\d+)\]',text)};exits.update({pid:int(code) for pid,code in re.findall(r'process has died \[pid (\d+), exit code (-?\d+)',text)})
        report['child_exit_codes']=exits
        report['cleanup_passed']=bool(report.get('lifecycle_shutdown',{}).get('success') and process.returncode==0 and not report['remaining_children'] and len(exits)==8 and all(v==0 for v in exits.values()))
        report['passed']=report['passed'] and report['cleanup_passed'];report['ended_wall']=time.time()
        (out/'commands.json').write_text(json.dumps(commands));(out/'raw_commands.json').write_text(json.dumps(raw));(out/'guard_statuses.json').write_text(json.dumps(statuses));(out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        action.destroy();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0 if report['passed'] else 2


if __name__=='__main__':raise SystemExit(main())
