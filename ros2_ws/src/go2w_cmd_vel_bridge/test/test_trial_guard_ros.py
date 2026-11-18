"""Real ROS odometry subscription reaches the final dispatch gate, isolated."""
import os
from pathlib import Path
import subprocess
import sys

import pytest

pytest.importorskip('rclpy')


def test_real_odometry_and_bridge_tick_cancel_at_trial_limit(tmp_path):
    environment = dict(os.environ, ROS_DOMAIN_ID='82')
    code = r'''
import json,sys,time
from pathlib import Path
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.qos import QoSProfile,ReliabilityPolicy
from rclpy.executors import SingleThreadedExecutor
from go2w_cmd_vel_bridge import bridge_node
from go2w_cmd_vel_bridge.bridge_core import BridgeDecision,Velocity
ledger=Path(sys.argv[1])
rclpy.init(args=['--ros-args','-p','trial_ledger:='+str(ledger)])
bridge=bridge_node.CmdVelBridge()
assert not bridge._execution_enabled and bridge._client is None
observer=rclpy.create_node('trial_guard_isolated_observer')
publisher=observer.create_publisher(Odometry,'/go2w/odom/fused',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
statuses=[]
observer.create_subscription(String,'/go2w/cmd_vel_bridge/status',lambda m:statuses.append(m.data),10)
executor=SingleThreadedExecutor();executor.add_node(bridge);executor.add_node(observer)
# Only replace upstream classification in this isolated test. There is no
# ActionClient and no physical motion interface anywhere in this ROS domain.
bridge_node.decide_velocity=lambda *a,**k:BridgeDecision(Velocity(.05,0),True,'isolated_test',False)
time.sleep(.01)
ledger.write_text(json.dumps(dict(version=1,maximum_steps=3,maximum_translation_m=.2,
    stop_reserve_m=.05,active_token='t',steps=[dict(token='t',state='reserved',reserved_at=time.time())])))
def feed(x):
    end=time.monotonic()+.35
    while time.monotonic()<end:
        m=Odometry();m.header.frame_id='odom_fused';m.header.stamp=bridge.get_clock().now().to_msg()
        m.pose.pose.position.x=x;m.pose.pose.orientation.w=1.
        publisher.publish(m);executor.spin_once(timeout_sec=.01)
feed(0.)
assert bridge._trial_guard.token=='t' and not bridge._trial_guard.reason
feed(.16)
assert any('trial_travel_limit' in s for s in statuses),statuses
assert not bridge._dispatch_permitted and bridge._client is None
executor.shutdown();bridge.destroy_node();observer.destroy_node();rclpy.shutdown()
'''
    result=subprocess.run([sys.executable,'-c',code,str(tmp_path/'trial.json')],
        env=environment,text=True,capture_output=True,timeout=12)
    assert result.returncode==0, result.stdout+result.stderr
