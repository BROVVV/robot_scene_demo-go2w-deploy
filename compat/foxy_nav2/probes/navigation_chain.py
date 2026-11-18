#!/usr/bin/python3
"""Full Nav2/BT compatibility acceptance with synthetic 2D sensors only."""
from pathlib import Path
import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import time

from velocity_chain import isolated_environment
from stop_trace import evaluate_stop


def main():
    isolated_environment()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True)
    args = parser.parse_args();out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).resolve().parents[3];sys.path.insert(0,str(root))
    import psutil
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from geometry_msgs.msg import Twist,TransformStamped
    from nav_msgs.msg import Odometry,OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster
    from nav2_msgs.action import NavigateToPose
    from rcl_interfaces.srv import GetParameters,SetParameters
    from rclpy.parameter import Parameter
    from app.navigation.nav2_search_backend import create_nav2_action_clients,SpinGoalRequest

    launch=root/'ros2_ws/src/robot_scene_nav_bringup/launch/robot_scene_nav2.launch.py'
    params=root/'ros2_ws/src/robot_scene_nav_bringup/config/nav2_params_foxy_execution.yaml'
    log=(out/'launch.log').open('w')
    process=subprocess.Popen(['ros2','launch',str(launch),'execution_enabled:=true',
        'foxy_isolated_validation:=true','nav2_cmd_vel_topic:=/go2w/acceptance/nav2_cmd_vel',
        'params_file:='+str(params)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=rclpy.create_node('acceptance_full_nav2_fixture')
    tf=TransformBroadcaster(node);pose=[0.,0.,0.];commands=[];commands_full=[];raw=[];last=time.monotonic();velocity=[0.,0.];last_scan=last_map=0.
    odom=node.create_publisher(Odometry,'/go2w/odom/fused',10)
    scans=node.create_publisher(LaserScan,'/go2w/lidar/navigation_scan',10)
    raw_scans=node.create_publisher(LaserScan,'/go2w/lidar/scan',10)
    health=node.create_publisher(String,'/go2w/nav/localization_status',10)
    maps=node.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def command(message):
        velocity[:]=[float(message.linear.x),float(message.angular.z)]
        received=time.monotonic()
        commands.append((received,*velocity))
        commands_full.append((received,float(message.linear.x),float(message.linear.y),
            float(message.linear.z),float(message.angular.x),float(message.angular.y),float(message.angular.z)))
    node.create_subscription(Twist,'/go2w/acceptance/nav2_cmd_vel',command,100)
    node.create_subscription(Twist,'/cmd_vel_nav_raw',lambda m:raw.append(
        (time.monotonic(),float(m.linear.x),float(m.angular.z))),100)
    report={'scope':'full Nav2/BT, synthetic pose/sensors only','domain_id':99,
            'started_wall':time.time(),
            'physical_motion_enabled':False,'cases':{},'passed':False}
    action=None;ports=();last_goal_started=0.
    def pump(duration,*,moving=False,valid=True,obstacle=None):
        nonlocal last,last_scan,last_map
        end=time.monotonic()+duration
        while time.monotonic()<end:
            now=time.monotonic();dt=min(.05,now-last);last=now
            if moving:
                pose[0]+=velocity[0]*math.cos(pose[2])*dt
                pose[1]+=velocity[0]*math.sin(pose[2])*dt
                pose[2]+=velocity[1]*dt
            stamp=node.get_clock().now().to_msg()
            transforms=[]
            for parent,child,x,y,yaw in [('map','odom_fused',0.,0.,0.),
                    ('odom_fused','base_link',*pose)]:
                t=TransformStamped();t.header.stamp=stamp;t.header.frame_id=parent;t.child_frame_id=child
                t.transform.translation.x=x;t.transform.translation.y=y
                t.transform.rotation.z=math.sin(yaw/2);t.transform.rotation.w=math.cos(yaw/2)
                transforms.append(t)
            tf.sendTransform(transforms)
            state=Odometry();state.header.stamp=stamp;state.header.frame_id='odom_fused';state.child_frame_id='base_link'
            state.pose.pose.position.x=pose[0];state.pose.pose.position.y=pose[1]
            state.pose.pose.orientation.z=math.sin(pose[2]/2);state.pose.pose.orientation.w=math.cos(pose[2]/2)
            state.twist.twist.linear.x=velocity[0];state.twist.twist.angular.z=velocity[1];odom.publish(state)
            if now-last_scan>=.05:
                scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='base_link'
                scan.angle_min=-math.pi;scan.angle_increment=math.pi/360;scan.angle_max=scan.angle_min+719*scan.angle_increment
                scan.range_min=.1;scan.range_max=6.2;scan.ranges=[6.]*720
                if obstacle is not None:
                    for i in range(352,369):scan.ranges[i]=obstacle
                scans.publish(scan)
                raw_scans.publish(scan)
                last_scan=now
            health.publish(String(data=json.dumps({'valid':valid,'wall_time':time.time()})))
            if now-last_map>=1.:
                grid=OccupancyGrid();grid.header.stamp=stamp;grid.header.frame_id='map'
                grid.info.resolution=.05;grid.info.width=120;grid.info.height=120
                grid.info.origin.position.x=-3.;grid.info.origin.position.y=-3.;grid.info.origin.orientation.w=1.
                grid.data=[0]*14400;maps.publish(grid)
                last_map=now
            rclpy.spin_once(node,timeout_sec=.01)
            assert process.poll() is None,'Nav2 launch exited'
    def wait(future,timeout=8.,**kwargs):
        end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end:pump(.03,**kwargs)
        assert future.done(),'Nav2 response timeout'
        return future.result()
    def start(x):
        nonlocal last_goal_started
        last_goal_started=time.monotonic()
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=node.get_clock().now().to_msg()
        goal.pose.pose.position.x=x;goal.pose.pose.orientation.w=1.
        handle=wait(action.send_goal_async(goal));assert handle and handle.accepted
        return handle
    def wait_for_motion(name, timeout=2.):
        # Action discovery/acceptance does not imply the first costmap and
        # guarded command have arrived. Require actual fresh nonzero output
        # before the fault, within a bounded startup window.
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if (commands and commands[-1][0]>=last_goal_started
                    and time.monotonic()-commands[-1][0]<.15
                    and velocity[0]>.05):
                return
            pump(.03,moving=True)
        raise AssertionError(('Not moving before '+name,list(velocity)))
    def stopped(name,started,*,deadline=.3):
        report['cases'][name]=evaluate_stop(commands_full,started,deadline=deadline)
    try:
        action=ActionClient(node,NavigateToPose,'/navigate_to_pose')
        deadline=time.monotonic()+40
        while not action.server_is_ready() and time.monotonic()<deadline:pump(.05)
        assert action.server_is_ready(),'NavigateToPose unavailable'
        # Server discovery can precede lifecycle activation. Wait for all
        # lifecycle states rather than sending a goal to an inactive server.
        from lifecycle_msgs.srv import GetState
        for name in ('planner_server','controller_server','behavior_server','bt_navigator','velocity_smoother','collision_monitor'):
            client=node.create_client(GetState,'/'+name+'/get_state')
            assert client.wait_for_service(timeout_sec=3),name
            deadline=time.monotonic()+20
            while time.monotonic()<deadline:
                state=wait(client.call_async(GetState.Request())).current_state
                if state.id==3:break
                pump(.1)
            else:raise AssertionError((name,state.label))
            node.destroy_client(client)
        deadline=time.monotonic()+10
        while (node.count_publishers('/cmd_vel_nav_collision_checked')!=1
               or node.count_publishers('/go2w/acceptance/nav2_cmd_vel')!=1) and time.monotonic()<deadline:pump(.05)
        assert node.count_publishers('/cmd_vel_nav_collision_checked')==1,'Collision output unavailable'
        assert node.count_publishers('/go2w/acceptance/nav2_cmd_vel')==1,'Final velocity guard unavailable'
        for topic in ('/go2w/nav2_cmd_vel','/go2w/cmd_vel_selected','/go2w/software_arm'):
            assert node.count_publishers(topic)==0,'Unexpected physical topic: '+topic
        setter=node.create_client(SetParameters,'/controller_server/set_parameters')
        getter=node.create_client(GetParameters,'/controller_server/get_parameters')
        assert setter.wait_for_service(timeout_sec=3) and getter.wait_for_service(timeout_sec=3)
        def set_double(name,value):
            request=SetParameters.Request()
            request.parameters=[Parameter(name,value=value).to_parameter_msg()]
            response=wait(setter.call_async(request))
            assert len(response.results)==1
            return response.results[0]
        def get_double(name):
            request=GetParameters.Request();request.names=[name]
            response=wait(getter.call_async(request))
            assert len(response.values)==1 and response.values[0].type==3
            return response.values[0].double_value
        assert abs(get_double('failure_tolerance')-.3)<1e-9
        assert set_double('failure_tolerance',.25).successful
        assert abs(get_double('failure_tolerance')-.25)<1e-9
        assert set_double('failure_tolerance',.3).successful
        started=time.monotonic();handle=start(.8)
        wait_for_motion('parameter gate test')
        rejected=set_double('failure_tolerance',.4)
        assert not rejected.successful and 'currently running' in rejected.reason,rejected
        assert abs(get_double('failure_tolerance')-.3)<1e-9
        assert set_double('FollowPath.desired_linear_vel',.12).successful
        assert abs(get_double('FollowPath.desired_linear_vel')-.12)<1e-9
        speed_changed=time.monotonic();pump(.45,moving=True)
        speeds=[abs(row[1]) for row in raw if row[0]>=speed_changed+.15]
        assert speeds and max(speeds)<=.12+1e-6 and max(speeds)>.05,speeds
        report['cases']['controller_parameter_rpc']={
            'idle_failure_tolerance_update_verified':True,
            'active_core_update_rejected':True,'rejection_reason':rejected.reason,
            'plugin_speed_update_accepted':True,
            'actual_raw_speed_min':min(speeds),'actual_raw_speed_max':max(speeds)}
        assert set_double('FollowPath.desired_linear_vel',.3).successful
        result=wait(handle.get_result_async(),timeout=10.,moving=True);pump(.2)
        assert result.status==4,(result.status,pose)
        assert .59<pose[0]<.85 and abs(pose[1])<.03,pose
        report['cases']['navigate_success']={'status':result.status,'simulated_pose':list(pose),
            'samples':sum(r[0]>=started for r in commands)}
        assert set_double('failure_tolerance',.25).successful
        assert abs(get_double('failure_tolerance')-.25)<1e-9
        assert set_double('failure_tolerance',.3).successful
        report['cases']['controller_parameter_rpc']['core_updates_reenabled_after_goal']=True
        handle=start(1.6);wait_for_motion('cancel')
        canceled=time.monotonic();wait(handle.cancel_goal_async());result=wait(handle.get_result_async());pump(.4)
        assert result.status==5,result.status
        stopped('navigate_cancel',canceled)
        handle=start(1.8);wait_for_motion('obstacle injection')
        blocked=time.monotonic();pump(.6,obstacle=.45)
        stopped('obstacle_stop',blocked,deadline=.2)
        wait(handle.cancel_goal_async(),obstacle=.45);wait(handle.get_result_async(),obstacle=.45);pump(.3)
        handle=start(1.8);wait_for_motion('localization fault')
        invalid=time.monotonic();pump(.6,valid=False)
        stopped('localization_stop',invalid,deadline=.2)
        wait(handle.cancel_goal_async(),valid=False);wait(handle.get_result_async(),valid=False);pump(.3)
        ports=create_nav2_action_clients(node);spin=ports[-1]
        assert spin.server_ready()
        initial=pose[2];spin.send(SpinGoalRequest(.15,3.))
        deadline=time.monotonic()+4
        while spin.poll().status not in ('SUCCEEDED','ABORTED','CANCELED') and time.monotonic()<deadline:
            pump(.03,moving=True)
        report['cases']['bounded_spin']={'status':spin.poll().status,
            'initial_yaw':initial,'final_yaw':pose[2],'simulated_yaw_delta':pose[2]-initial}
        assert spin.poll().status=='SUCCEEDED',(spin.poll(),pose)
        assert abs(pose[2]-initial-.15)<.04,report['cases']['bounded_spin']
        pump(.3)
        assert all(abs(r[1])<=.3+1e-6 and abs(r[2])<=.5+1e-6 for r in commands)
        assert all(all(abs(r[i])<1e-8 for i in (2,3,4,5)) for r in commands_full), 'unsupported velocity axis'
        report['passed']=True
    finally:
        if action is not None:action.destroy()
        for port in ports:port._client.destroy()
        # Foxy's native costmap owners must release active plugins while the
        # ROS context and their executors are still alive. SIGINT alone tears
        # down the context before that lifecycle cleanup can run.
        shutdown={'success':False,'final_states':{}}
        helper=None
        try:
            with (out/'lifecycle_shutdown.log').open('w') as shutdown_log:
                helper=subprocess.Popen(['/usr/bin/python3',
                    str(root/'scripts/go2w/shutdown_owned_nav2.py'),
                    '--launch-pid',str(process.pid)],stdout=shutdown_log,
                    stderr=subprocess.STDOUT,start_new_session=True)
                deadline=time.monotonic()+35.
                while helper.poll() is None and time.monotonic()<deadline:pump(.03)
                assert helper.poll() is not None,'Owned lifecycle shutdown timed out'
            lines=(out/'lifecycle_shutdown.log').read_text().splitlines()
            reports=[json.loads(line) for line in lines if line.startswith('{')]
            assert reports,'Owned lifecycle shutdown produced no report'
            shutdown=reports[-1]
            assert helper.returncode==0 and shutdown['success'],shutdown
        except Exception as error:
            shutdown['success']=False;shutdown['error']=str(error)
        finally:
            if helper is not None and helper.poll() is None:
                os.killpg(helper.pid,signal.SIGKILL);helper.wait(timeout=3)
        report['lifecycle_shutdown']=shutdown
        children=[]
        if process.poll() is None:
            try:children=psutil.Process(process.pid).children(recursive=True)
            except psutil.Error:pass
            # launch forwards SIGINT to its children. Signaling the entire
            # group would interrupt them twice, including during cleanup.
            process.send_signal(signal.SIGINT)
        try:process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM)
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3)
        remaining=[]
        for child in children:
            try:
                if child.is_running() and child.status()!=psutil.STATUS_ZOMBIE:
                    remaining.append(child.pid);child.terminate()
            except psutil.Error:pass
        report['launch_exit_code']=process.returncode;report['remaining_owned_children']=remaining
        log.close()
        text=(out/'launch.log').read_text()
        exits={pid:0 for pid in re.findall(r'process has finished cleanly \[pid (\d+)\]',text)}
        exits.update({pid:int(code) for pid,code in re.findall(
            r'process has died \[pid (\d+), exit code (-?\d+)',text)})
        report['child_exit_codes']=exits
        report['cleanup_passed']=shutdown['success'] and process.returncode==0 and not remaining and len(exits)==8 and all(v==0 for v in exits.values())
        report['passed']=report['passed'] and report['cleanup_passed']
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        (out/'commands.json').write_text(json.dumps(commands));(out/'raw_commands.json').write_text(json.dumps(raw))
        (out/'commands_full.json').write_text(json.dumps(commands_full))
        report['ended_wall']=time.time()
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report));return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
