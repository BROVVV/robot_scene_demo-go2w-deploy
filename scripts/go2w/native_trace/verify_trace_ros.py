"""Isolated topic/clock/protocol fixture; never connects to a robot or SDK."""
from pathlib import Path
import argparse,json,math,os,subprocess,time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool,Float32,String
from nav_msgs.msg import Odometry

def main():
 p=argparse.ArgumentParser();p.add_argument('--executable',required=True);p.add_argument('--output-dir',type=Path,required=True);a=p.parse_args()
 assert os.environ.get('ROS_DOMAIN_ID')=='99' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
 out=a.output_dir;out.mkdir(exist_ok=False);(out/'phase.json').write_text('{"phase":"fixture"}')
 boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip();low=out/'low.json'
 def state(tag,**override):
  d=dict(capture_boot_id=boot,capture_monotonic=time.monotonic(),capture_wall_time=time.time(),marker=tag);d.update(override);tmp=low.with_suffix('.tmp');tmp.write_text(json.dumps(d));tmp.replace(low)
 state('fresh')
 rclpy.init();n=rclpy.create_node('native_trace_isolated_fixture')
 pubs=[n.create_publisher(Float32,'/go2w/safety/front_clearance',qos_profile_sensor_data),
 n.create_publisher(Bool,'/go2w/safety/lidar_fresh',10),n.create_publisher(String,'/go2w/nav/localization_status',10),
 n.create_publisher(String,'/go2w/lidar/geometry_status',10),n.create_publisher(Odometry,'/go2w/odom/fused',qos_profile_sensor_data)]
 child=None;report={'domain':99,'localhost_only':True,'physical_command_sent':False}
 def pump(seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
 try:
  with (out/'native.log').open('x') as log:child=subprocess.Popen([a.executable,'--output-dir',str(out),'--low-state',str(low),'--duration','12'],stdout=log,stderr=subprocess.STDOUT)
  end=time.monotonic()+8
  while not all(x.get_subscription_count()==1 for x in pubs):
   assert child.poll() is None and time.monotonic()<end;state('fresh');pump(.05)
  for value in [float('inf'),float('-inf'),float('nan'),.125]*3:
   state('fresh');pubs[0].publish(Float32(data=value));pubs[1].publish(Bool(data=False))
   pubs[2].publish(String(data=json.dumps({'valid':False,'blocking':['scan_fresh'],'unicode':'二维'})))
   pubs[3].publish(String(data='{"posture_valid":true}'))
   odom=Odometry();odom.pose.pose.position.x=.125;odom.pose.pose.position.y=-.25;pubs[4].publish(odom);pump(.1)
  for tag,extra in [('stale',{'capture_monotonic':time.monotonic()-10}),('future_wall',{'capture_wall_time':time.time()+10}),('wrong_boot',{'capture_boot_id':'invalid-boot'})]:
   state(tag,**extra);pump(.6)
  pubs[2].publish(String(data='invalid-json'));pump(.2)
  (out/'phase.json').write_text('{"phase":"finished"}');child.wait(timeout=3);assert child.returncode==0
  rows=[json.loads(s) for s in (out/'safety_trace.jsonl').read_text().splitlines()]
  fronts=[x['value'] for x in rows if x['topic']=='front']
  assert any(math.isnan(x) for x in fronts) and float('inf') in fronts and float('-inf') in fronts and .125 in fronts
  assert all(x['value']=={'valid':False,'blocking':['scan_fresh'],'unicode':'二维'} for x in rows if x['topic']=='health')
  assert any(x['topic']=='health' for x in rows) and any(x['topic']=='geometry' for x in rows)
  assert any(x['topic']=='invalid_health' for x in rows)
  assert all(x['value']=={'x':.125,'y':-.25} for x in rows if x['topic']=='odom')
  low_rows=[x for x in rows if x['topic']=='low'];assert low_rows and all(x['value']['marker']=='fresh' for x in low_rows)
  assert all(x['wall']>0 and x['mono']>0 and x['phase']=='fixture' for x in rows)
  terminal=json.loads((out/'collector_terminal.json').read_text());assert terminal['parse_errors']==1 and not terminal['motion_sent']
  again=subprocess.run([a.executable,'--output-dir',str(out),'--low-state',str(low)],capture_output=True,text=True,timeout=3)
  assert again.returncode==2 and 'refusing existing trace file' in again.stderr
  report.update(passed=True,rows=len(rows),front_samples=len(fronts),low_samples=len(low_rows),nonfinite_preserved=True,clock_boot_rejections_passed=True,existing_output_refused=True,terminal=terminal)
 finally:
  if child and child.poll() is None:child.terminate();child.wait(timeout=3)
  n.destroy_node();rclpy.shutdown();(out/'fixture_result.json').write_text(json.dumps(report,indent=2))
 print(json.dumps(report))
if __name__=='__main__':main()
