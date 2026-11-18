#!/usr/bin/env python3
"""Real ROS topic/bridge/factory handshake with synthetic wheel data only."""
import argparse
import json
import os
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--json',required=True)
    args=parser.parse_args()
    if (os.environ.get('ROS_DOMAIN_ID','0')=='0'
            or os.environ.get('ROS_LOCALHOST_ONLY')!='1'):
        raise SystemExit('Requires an isolated nonzero ROS domain and ROS_LOCALHOST_ONLY=1')
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from go2w_cmd_vel_bridge import bridge_node
    from go2w_cmd_vel_bridge.bridge_core import BridgeDecision,Velocity
    from app.navigation.nav2_search_backend import create_nav2_search_backend,ActionPoll
    from app.navigation.corridor_analyzer import CorridorGrid
    from app.navigation.robot_backend import RobotPose,NavigationStatus
    from app.navigation.models import ExplorationGoal

    rclpy.init(args=[])
    bridge=bridge_node.CmdVelBridge()
    io=rclpy.create_node('goal_travel_isolated_search')
    executor=SingleThreadedExecutor()
    executor.add_node(bridge);executor.add_node(io)
    original_decide=bridge_node.decide_velocity
    try:
        assert bridge._client is None and not bridge._execution_enabled
        # The sole upstream substitute. No physical MotionCommand client exists.
        bridge_node.decide_velocity=lambda *a,**k:BridgeDecision(Velocity(.1,0),True,'synthetic',False)
        events=[]
        def arm():
            assert bridge._goal_travel_guard.phase=='prepared'
            events.append('arm_stub');return True
        backend=create_nav2_search_backend(io,max_path_length_m=.3,
            motion_authorizer=arm,motion_releaser=lambda:events.append('disarm_stub'))
        backend._pose_provider=lambda:RobotPose(0,0,0)
        backend._grid_provider=lambda:CorridorGrid(.05,-3.,-3.,120,120,(0,)*14400)
        backend._localization_valid=lambda:True
        backend._speed_limit_setter=lambda _:True
        backend._stop_publisher=lambda:events.append('stop_stub')
        class Port:
            def __init__(self,path=False):self.path=path;self.sent=0;self.canceled=0
            def server_ready(self):return True
            def send(self,request):
                if not self.path:assert bridge._goal_travel_guard.phase=='running'
                self.sent+=1
            def poll(self):return ActionPoll(status='SUCCEEDED' if self.path else 'EXECUTING',path=((0.,0.),(.12,0.)))
            def cancel(self):self.canceled+=1;return True
        backend._path_client=Port(path=True);backend._navigate_client=Port()
        publisher=io.create_publisher(Odometry,'/go2w/odom/fused',
            QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
        def feed(x,seconds=.08):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                message=Odometry()
                message.header.frame_id='odom_fused';message.child_frame_id='base_link'
                message.header.stamp=io.get_clock().now().to_msg()
                message.pose.pose.position.x=x;message.pose.pose.orientation.w=1.
                publisher.publish(message);executor.spin_once(timeout_sec=.01)
        feed(0.,.9)
        assert backend._travel_budget is not None
        handle=backend.execute_goal(ExplorationGoal('synthetic','NAVIGATE_POSE',position=(.12,0),yaw=0.,frame='map'))
        for _ in range(20):
            backend.get_navigation_status(handle);feed(0.)
            if backend._navigate_client.sent:break
        assert backend._navigate_client.sent==1 and bridge._dispatch_permitted
        x=0.
        for _ in range(70):
            x+=.004
            feed(x)
            if bridge._goal_travel_guard.reason:
                assert not bridge._dispatch_permitted, 'bridge must cut velocity before application polling'
            result=backend.get_navigation_status(handle)
            if result.status!=NavigationStatus.RUNNING:break
        assert result.status==NavigationStatus.REJECTED,result
        assert result.message=='goal_travel_limit',result
        measured=result.provenance['wheel_travel']['traveled_m']
        feed(x)
        assert not bridge._dispatch_permitted
        assert events==['arm_stub','stop_stub','disarm_stub'],events
        assert bridge._client is None
        report=dict(synthetic_inputs=True,isolated_domain=os.environ['ROS_DOMAIN_ID'],
            robot_connected=False,physical_action_client=False,
            initial_plan_m=.12,cumulative_wheel_m=measured,limit_m=.3,
            stopped_by=result.message,goal_status=result.status.value,
            events=events,physical_commands=0)
        report['independent_bridge_cutoff_before_application_poll']=True
        Path(args.json).write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report))
    finally:
        bridge_node.decide_velocity=original_decide
        executor.shutdown();bridge.destroy_node();io.destroy_node();rclpy.shutdown()
    return 0


if __name__=='__main__':
    raise SystemExit(main())
