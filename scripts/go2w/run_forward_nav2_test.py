"""One observed bounded Nav2 translation or small-turn test.

Default is read-only planning. Execution requires --execute, a fresh reviewed
robot boot, field confirmation, and the independent bridge travel/direction gate.
This command never creates or resets a movement allowance.
"""
import argparse,json,math,os,signal,sys,threading,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scripts.go2w.prepare_nav2_trial import live_robot_boot_id
from app.navigation.trial_budget import NavigationTrialBudget
from app.navigation.motion_authorization import arm_with_arbiter_reset,motion_preflight_blockers
from app.navigation.nav2_search_backend import create_nav2_search_backend
from app.navigation.nav2_candidate_geometry import bounded_heading_target
from app.navigation.models import ExplorationGoal
from app.navigation.robot_backend import TERMINAL_NAVIGATION_STATUSES
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import SetBool,Trigger
from std_msgs.msg import Bool,String
from unitree_go.msg import SportModeState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from app.ros_compat import init_preserving_python_signals
p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true')
motion=p.add_mutually_exclusive_group()
motion.add_argument('--distance',type=float)
motion.add_argument('--turn-deg',type=float,default=0.,help='bounded in-place turn (-20..20 deg)')
p.add_argument('--heading-offset-deg',type=float,default=0.,help='bounded target bearing relative to the current heading (-20..20 deg)');p.add_argument('--ledger',required=True);p.add_argument('--json',required=True);args=p.parse_args()
if args.distance is None:
 args.distance=0. if args.turn_deg else 1.
if not args.turn_deg and (not math.isfinite(args.distance) or not .03<=args.distance<=2.5):
 p.error('--distance must be finite and in 0.03..2.5m')
if not math.isfinite(args.turn_deg) or not -20.<=args.turn_deg<=20.:
 p.error('--turn-deg must be finite and in -20..20deg')
if not math.isfinite(args.heading_offset_deg) or not -20.<=args.heading_offset_deg<=20.:
 p.error('--heading-offset-deg must be finite and in -20..20deg')
if args.turn_deg and args.heading_offset_deg:
 p.error('--turn-deg and --heading-offset-deg cannot be combined')
ledger=Path(args.ledger).resolve()
budget=NavigationTrialBudget(ledger)
single_short=(budget.MAX_STEPS==1 and (
    budget.MAX_TRANSLATION_M==.2 and .10<args.distance<=.12
    or (getattr(budget,'collision_diagnostic',False)
        and .10<args.distance<=budget.MAX_TRANSLATION_M-budget.STOP_RESERVE_M)
    or getattr(budget,'speed_calibration_0_30',False) and .20<args.distance<=.30))
bounded_search=(budget.bounded_search and args.distance<=.45)
assert (budget.forward_only and budget.MAX_STEPS==1 and budget.MAX_TRANSLATION_M==3.) or single_short or bounded_search, 'requires a bounded forward/recheck/search window'
if args.execute:
 assert os.getenv('GO2W_AREA_CLEARED')=='I_HAVE_CLEARED_THE_AREA'
 budget.require_reviewed_boot(live_robot_boot_id())
interrupted=threading.Event()
# Keep ROS alive until cancel/disarm and evidence flushing finish. rclpy's
# default SIGINT handler shuts down the context before those service calls.
init_preserving_python_signals()
for signum in (signal.SIGINT,signal.SIGTERM):
 signal.signal(signum,lambda *_:interrupted.set())
node=rclpy.create_node('go2w_bounded_nav2_acceptance');executor=SingleThreadedExecutor();executor.add_node(node)
state={};events=[];armed=False
trace=[]
odometry_trace=[]
def odometry(m):
 p=m.pose.pose.position;q=m.pose.pose.orientation
 sample=dict(received=time.time(),stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,x=float(p.x),y=float(p.y),yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
 state["odometry"]=sample
 if len(odometry_trace)<2000:odometry_trace.append(sample)
node.create_subscription(Odometry,"/go2w/odom/fused",odometry,qos_profile_sensor_data)
def record(topic,value):
 if len(trace)<10000:trace.append({'at':time.time(),'topic':topic,'value':value})
for topic in ['/cmd_vel_nav_raw','/cmd_vel_nav_smoothed','/cmd_vel_nav_collision_checked','/go2w/nav2_cmd_vel','/go2w/cmd_vel_selected']:
 node.create_subscription(Twist,topic,lambda m,t=topic:record(t,[m.linear.x,m.angular.z]),10)
for topic in ['/go2w/cmd_vel_bridge/status','/go2w_navigation_velocity_guard/status',
              '/go2w/nav/localization_status','/go2w/lidar/geometry_status',
              '/go2w/control_source']:
 node.create_subscription(String,topic,lambda m,t=topic:record(t,m.data),10)

def flag(key,msg):state[key]=(bool(msg.data),time.monotonic())
for topic,key in [('software_arm','software_arm'),('sport_lease/alive','lease_alive'),
                  ('safety/lidar_fresh','lidar_fresh'),
                  ('nav/localization_valid','localization_valid'),
                  ('robot_error_zero','robot_error_zero'),
                  ('emergency_stop','emergency_stop'),('remote_override','remote_override')]:
 node.create_subscription(Bool,'/go2w/'+topic,lambda m,k=key:flag(k,m),10)
def sport_state(message):
 state['sport']=(int(message.mode),int(message.error_code),time.monotonic())

# The robot DDS topic can disappear while the local SDK capture and relay stay
# healthy.  The motion server itself consumes the relay, so monitor both real
# state sources instead of rejecting a safe run solely because the redundant
# network topic is silent.
node.create_subscription(
 SportModeState,'/lf/sportmodestate',sport_state,qos_profile_sensor_data)
node.create_subscription(
 SportModeState,'/go2w/motion/local_sportmodestate',sport_state,10)
node.create_subscription(String,'/go2w/cmd_vel_bridge/status',lambda m:state.update(bridge=m.data),10)
arm_client=node.create_client(SetBool,'/go2w/arm');reset_client=node.create_client(Trigger,'/go2w/control_arbiter/reset_emergency_stop')
stop_client=node.create_client(Trigger,'/go2w/emergency_stop')
def call(client,req):
 if not client.service_is_ready():raise RuntimeError('service unavailable')
 f=client.call_async(req);end=time.monotonic()+2.
 while not f.done() and time.monotonic()<end:time.sleep(.01)
 if not f.done():f.cancel();raise RuntimeError('service response timeout')
 response=f.result()
 if not response or not response.success:raise RuntimeError(getattr(response,'message','no service response'))
 return response

def fresh(key,value=True):
 v,at=state.get(key,(None,0));return v is value and 0<=time.monotonic()-at<=.5

def set_arm(value):
 global armed
 q=SetBool.Request();q.data=value;call(arm_client,q);armed=value
 events.append(dict(event='arm' if value else 'disarm',wall_time=time.time()))

def reset():
 end=time.monotonic()+.8
 while not fresh('software_arm') and time.monotonic()<end:time.sleep(.02)
 if not fresh('software_arm'):return False
 end=time.monotonic()+1.5
 while time.monotonic()<end:
  try:
   call(reset_client,Trigger.Request());events.append(dict(event='arbiter_reset',wall_time=time.time()));return True
  except RuntimeError:
   time.sleep(.05)
 return False

def authorize():
 if interrupted.is_set():return False
 if motion_preflight_blockers(state,now=time.monotonic()):return False
 return arm_with_arbiter_reset(arm=lambda:set_arm(True),is_armed=lambda:armed,reset=reset,disarm=lambda:set_arm(False))

def release():
 if armed:set_arm(False)

def wait_for_stable_backend(timeout_s,stable_s=.6):
 """Return a pose only after readiness stayed continuously true.

 A single good sample is not sufficient on the low-power robot: scan freshness
 can briefly alternate true/false while the localization monitor catches up.
 Waiting for a continuous window preserves the fail-closed health checks and
 avoids racing the very next health read against a transient false sample.
 """
 deadline=time.monotonic()+timeout_s;stable_since=None;last_health=None
 while time.monotonic()<deadline:
  if interrupted.is_set():raise InterruptedError('operator cancelled before planning')
  last_health=backend.health();pose=backend.get_pose()
  if last_health.ready and pose is not None:
   now=time.monotonic()
   if stable_since is None:stable_since=now
   if now-stable_since>=stable_s:return pose
  else:
   stable_since=None
  time.sleep(.03)
 raise AssertionError(last_health)

backend=create_nav2_search_backend(node,dry_run=not args.execute,
 motion_authorizer=authorize,motion_releaser=release,
 trial_budget=budget,
 max_path_length_m=budget.MAX_TRANSLATION_M-budget.STOP_RESERVE_M)
thread=threading.Thread(target=executor.spin,daemon=True);thread.start();output={'execute':args.execute,'events':events,'requested_translation_m':0 if args.turn_deg else args.distance}
try:
 before=wait_for_stable_backend(15)
 if args.execute:
  end=time.monotonic()+6
  blockers=motion_preflight_blockers(state,now=time.monotonic())
  while blockers and time.monotonic()<end:
   time.sleep(.03);blockers=motion_preflight_blockers(state,now=time.monotonic())
  output['motion_preflight']={'ready':not blockers,'blockers':blockers,'checked_at':time.time()}
  if blockers:raise RuntimeError('motion preflight incomplete: '+','.join(blockers))
  # Preflight can take several seconds.  Re-establish a continuous ready
  # window immediately before expressing the map-frame target; execute_goal
  # still performs its own independent fail-closed readiness check.
  before=wait_for_stable_backend(8)
 distance=0 if args.turn_deg else args.distance
 target_position=bounded_heading_target(
  (before.x,before.y),before.yaw,distance,args.heading_offset_deg)
 goal=ExplorationGoal(goal_id='bounded_acceptance_'+str(time.time_ns()),goal_type='NAVIGATE_POSE',frame='map',
  position=target_position,
  yaw=math.degrees(before.yaw)+args.heading_offset_deg+args.turn_deg)
 output.update(goal=goal.to_dict(),before=before.to_dict(),heading_offset_deg=args.heading_offset_deg,wheel_odometry_before=state.get('odometry'));handle=backend.execute_goal(goal);end=time.monotonic()+(20 if args.turn_deg else 60)
 while time.monotonic()<end:
  result=backend.get_navigation_status(handle)
  if result.status in TERMINAL_NAVIGATION_STATUSES:break
  bridge_reasons=set(str(state.get('bridge','')).split(','))
  fatal={r for r in bridge_reasons if r.startswith('trial_') and r!='trial_step_not_reserved'}
  if interrupted.is_set() or fatal:
   output['cancel_reason']='operator_signal' if interrupted.is_set() else ','.join(sorted(fatal))
   backend.cancel(handle);result=backend.get_navigation_status(handle);break
  time.sleep(.025)
 else:
  backend.cancel(handle);result=backend.get_navigation_status(handle);output['timed_out']=True
 after=backend.get_pose()
 output.update(result=result.to_dict(),after=after.to_dict() if after else None,bridge=state.get('bridge'),armed=armed,
  planned_path=list(backend._path_client._path),diagnostic_only=True,authorizes_unrestricted_search=False)
except BaseException as exc:
 output['error']=f'{type(exc).__name__}: {exc}'
 raise
finally:
 try:
  try:backend.stop()
  except Exception as exc:output['stop_error']=str(exc)
  if armed:
   try:release()
   except Exception as exc:
    output['disarm_error']=str(exc)
    try:call(stop_client,Trigger.Request())
    except Exception as stop_exc:output['emergency_stop_error']=str(stop_exc)
  time.sleep(.4)
  output['settled_flags']={k:state.get(k) for k in ['software_arm','remote_override','bridge']}
  output['velocity_trace']=trace
  output['wheel_odometry_after']=state.get('odometry')
  output['wheel_odometry_trace']=odometry_trace
  Path(args.json).write_text(json.dumps(output,ensure_ascii=False,indent=2,allow_nan=False))
  print(json.dumps({k:v for k,v in output.items() if k not in ('velocity_trace','wheel_odometry_trace')},ensure_ascii=False,allow_nan=False),flush=True)
 finally:
  executor.shutdown();thread.join(2);backend._path_client._client.destroy();backend._navigate_client._client.destroy();backend._spin_client._client.destroy();node.destroy_node();rclpy.shutdown()
status=output.get('result',{})
passed=status.get('status')=='succeeded' if args.execute else (
 status.get('provenance',{}).get('dry_run') is True and
 status.get('provenance',{}).get('candidate_decision',{}).get('accepted') is True)
raise SystemExit(0 if passed and not output.get('timed_out') else 2)
