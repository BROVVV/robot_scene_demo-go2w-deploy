#!/usr/bin/python3
"""Real Foxy action/lifecycle faults, synthetic inputs on domain99/loopback only."""
from pathlib import Path
import argparse,json,os,signal,subprocess,sys,time
root=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(root/'compat/foxy_nav2/probes'))
from velocity_chain import isolated_environment

def main():
 isolated_environment();parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output-dir',type=Path,required=True);args=parser.parse_args();out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=False)
 import yaml,rclpy
 from rclpy.action import ActionClient
 from lifecycle_msgs.srv import ChangeState
 from lifecycle_msgs.msg import Transition
 from geometry_msgs.msg import TransformStamped,Twist,PoseStamped
 from nav_msgs.msg import Path as NavPath,Odometry
 from nav2_msgs.action import FollowPath
 from sensor_msgs.msg import LaserScan
 from tf2_ros import TransformBroadcaster
 config=yaml.safe_load((root/'ros2_ws/src/robot_scene_nav_bringup/config/nav2_params_foxy_execution.yaml').read_text())
 config['controller_server']['ros__parameters']['progress_checker']['movement_time_allowance']=.7
 config['controller_server']['ros__parameters']['odom_topic']='/go2w/acceptance/controller_odom'
 scanconfig=config['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']['scan']
 scanconfig['topic']='/go2w/acceptance/controller_scan';scanconfig['expected_update_rate']=.12
 (out/'parameters.yaml').write_text(yaml.safe_dump(config))
 binary=Path('/home/unitree/robotscene_runtime/install')/(root.name+'_controller_compat')/'go2w_nav2_controller_compat/lib/go2w_nav2_controller_compat/controller_server'
 assert binary.is_file()
 log=(out/'server.log').open('w');process=subprocess.Popen([str(binary),'--ros-args','--params-file',str(out/'parameters.yaml'),'-r','cmd_vel:=/go2w/acceptance/controller_raw'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 rclpy.init();node=rclpy.create_node('acceptance_controller_lifecycle_fixture');tf=TransformBroadcaster(node)
 scans=node.create_publisher(LaserScan,'/go2w/acceptance/controller_scan',10);odom=node.create_publisher(Odometry,'/go2w/acceptance/controller_odom',10)
 rows=[];node.create_subscription(Twist,'/go2w/acceptance/controller_raw',lambda m:rows.append((time.monotonic(),float(m.linear.x),float(m.linear.y),float(m.linear.z),float(m.angular.x),float(m.angular.y),float(m.angular.z))),100)
 last_state=1
 flags={'tf':True,'scan':True};state=node.create_client(ChangeState,'/controller_server/change_state');action=ActionClient(node,FollowPath,'/follow_path')
 def inputs():
  stamp=node.get_clock().now().to_msg()
  if flags['tf']:
   t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='odom_fused';t.child_frame_id='base_link';t.transform.rotation.w=1.;tf.sendTransform(t)
  o=Odometry();o.header.stamp=stamp;o.header.frame_id='odom_fused';o.child_frame_id='base_link';o.pose.pose.orientation.w=1.;odom.publish(o)
  if flags['scan']:
   s=LaserScan();s.header.stamp=stamp;s.header.frame_id='base_link';s.angle_min=-3.14;s.angle_increment=.01;s.angle_max=3.14;s.range_min=.1;s.range_max=6.2;s.ranges=[6.]*629;scans.publish(s)
 node.create_timer(.02,inputs)
 def pump(seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   assert process.poll() is None,'controller process exited';rclpy.spin_once(node,timeout_sec=.01)
 def wait(future,timeout=5):
  end=time.monotonic()+timeout
  while not future.done() and time.monotonic()<end:pump(.02)
  assert future.done(),'controller response timeout'
  return future.result()
 def transition(identifier):
  nonlocal last_state
  r=wait(state.call_async(ChangeState.Request(transition=Transition(id=identifier))),10);assert r.success,identifier
  last_state={1:2,3:3,4:2,2:1,5:4,6:4,7:4}[identifier]
 def goal(controller='FollowPath',frame='odom_fused'):
  g=FollowPath.Goal();g.controller_id=controller;g.path=NavPath();g.path.header.frame_id=frame;g.path.header.stamp=node.get_clock().now().to_msg()
  for x in [0.,.5,1.,1.5,2.]:
   p=PoseStamped();p.header=g.path.header;p.pose.position.x=x;p.pose.orientation.w=1.;g.path.poses.append(p)
  return wait(action.send_goal_async(g))
 def moving(start):
  end=time.monotonic()+3
  while not any(r[0]>=start and r[1]>.05 for r in rows) and time.monotonic()<end:pump(.02)
  assert any(r[0]>=start and r[1]>.05 for r in rows),'no computed nonzero command before fault'
 def zero_after(start,deadline=.5):
  selected=[r for r in rows if r[0]>=start];zeros=[r[0]-start for r in selected if all(abs(v)<1e-8 for v in r[1:])]
  assert zeros and min(zeros)<=deadline,(deadline,selected[-5:]);first=min(r[0] for r in selected if all(abs(v)<1e-8 for v in r[1:]));assert all(all(abs(v)<1e-8 for v in r[1:]) for r in selected if r[0]>=first),selected[-5:]
  return min(zeros)
 report={'scope':'actual native controller action/lifecycle with synthetic TF/scans; no SDK or physical bridge','domain_id':99,'cases':{},'passed':False,'physical_command_sent':False}
 try:
  end=time.monotonic()+15
  while not state.service_is_ready() and time.monotonic()<end:pump(.05)
  assert state.service_is_ready();transition(1);pump(.3)
  end=time.monotonic()+5
  while not action.server_is_ready() and time.monotonic()<end:pump(.02)
  assert action.server_is_ready();inactive=goal();assert not inactive.accepted;report['cases']['inactive_goal_rejected']=True
  transition(3);pump(.3)
  start=time.monotonic();old=goal();assert old.accepted;moving(start)
  fault=time.monotonic();invalid=goal('MISSING_CONTROLLER');assert invalid.accepted
  assert wait(invalid.get_result_async()).status==6
  assert wait(old.get_result_async()).status==6;pump(.2)
  report['cases']['invalid_preemption']={'old_and_new_aborted':True,'zero_s':zero_after(fault)}
  start=time.monotonic();g=goal();assert g.accepted;moving(start);fault=time.monotonic()
  result=wait(g.get_result_async(),3);assert result.status==6;pump(.15)
  report['cases']['progress_failure']={'status':result.status,'zero_s':zero_after(fault,1.5)}
  start=time.monotonic();g=goal();assert g.accepted;moving(start);fault=time.monotonic()
  broken=goal(frame='missing_path_frame');assert broken.accepted
  result=wait(broken.get_result_async(),3);assert result.status==6;assert wait(g.get_result_async()).status==6;pump(.15)
  report['cases']['missing_path_transform']={'status':result.status,'zero_s':zero_after(fault,1.5)}
  pump(.5)
  start=time.monotonic();g=goal();assert g.accepted;moving(start);flags['scan']=False;fault=time.monotonic();pump(.6)
  assert not g.get_result_async().done();first=zero_after(fault,.5)
  cancel=time.monotonic();wait(g.cancel_goal_async());result=wait(g.get_result_async(),1);assert result.status==5
  report['cases']['stale_costmap_cancel']={'first_zero_s':first,'cancel_result_s':time.monotonic()-cancel,'status':result.status}
  flags['scan']=True;pump(.5)
  transition(4);transition(2)
  # Repeat configure/activate in the same process. Plugins and their callbacks
  # must be recreated after the ownership cycle has been broken.
  for index in range(2):
   transition(1);transition(3);pump(.3);start=time.monotonic();g=goal();assert g.accepted;moving(start);wait(g.cancel_goal_async());assert wait(g.get_result_async()).status==5;transition(4);transition(2)
  report['cases']['reconfigure_cycles']=2
  transition(1);transition(3);pump(.3);start=time.monotonic();g=goal();assert g.accepted;moving(start);fault=time.monotonic();transition(7);pump(.2)
  report['cases']['active_shutdown']={'response_s':time.monotonic()-fault,'zero_s':zero_after(fault,.6)}
  # The finalized process must actually release its DDS participant and exit.
  process.send_signal(signal.SIGINT);process.wait(timeout=8);assert process.returncode==0,process.returncode
  report['cases']['normal_exit_after_active_shutdown']=True;report['passed']=True
 finally:
  if process.poll() is None:
   if last_state!=4:
    try:transition({1:5,2:6,3:7}[last_state])
    except Exception as error:report['cleanup_error']=str(error)
   process.send_signal(signal.SIGINT)
   try:process.wait(timeout=8)
   except subprocess.TimeoutExpired:
    process.terminate()
    try:process.wait(timeout=3)
    except subprocess.TimeoutExpired:process.kill();process.wait()
  report['exit_code']=process.returncode
  action.destroy();node.destroy_client(state);node.destroy_node();rclpy.shutdown();log.close()
  (out/'commands.json').write_text(json.dumps(rows));(out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps(report));return 0 if report['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
