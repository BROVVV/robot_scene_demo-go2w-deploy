#!/usr/bin/python3
"""Exercise the real arbiter reset service in isolated DDS; no robot motion."""
import argparse
import json
import os
from pathlib import Path
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--json',required=True)
    args=p.parse_args()
    os.environ['ROS_DOMAIN_ID']='89'
    import rclpy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool
    from std_srvs.srv import Trigger
    from rclpy.executors import SingleThreadedExecutor
    from go2w_control_arbiter.arbiter_node import ControlArbiter
    rclpy.init();arbiter=ControlArbiter();probe=rclpy.create_node('isolated_arbiter_reset_probe')
    executor=SingleThreadedExecutor();executor.add_node(arbiter);executor.add_node(probe)
    pubs={name:probe.create_publisher(Bool,'/go2w/'+name,10)
        for name in ('emergency_stop','software_arm','remote_override')}
    nav=probe.create_publisher(Twist,'/go2w/nav2_cmd_vel',10)
    output=[];probe.create_subscription(Twist,'/go2w/cmd_vel_selected',lambda m:output.append(m.linear.x),10)
    client=probe.create_client(Trigger,'/go2w/control_arbiter/reset_emergency_stop')
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:executor.spin_once(timeout_sec=.01)
    def flags(estop,armed):
        for name,value in [('emergency_stop',estop),('software_arm',armed),('remote_override',False)]:
            pubs[name].publish(Bool(data=value))
        spin(.05)
    def reset():
        f=client.call_async(Trigger.Request());end=time.monotonic()+1
        while not f.done() and time.monotonic()<end:executor.spin_once(timeout_sec=.01)
        assert f.done();return f.result().success
    result=dict(domain_id=89,motion_dispatch=False)
    try:
        spin(.4)
        assert sorted(probe.get_node_names())==['go2w_control_arbiter','isolated_arbiter_reset_probe']
        flags(True,False);flags(False,False)
        assert arbiter._emergency
        result['false_level_does_not_clear_latch']=True
        assert not reset();result['unarmed_reset_rejected']=True
        old=Twist();old.linear.x=.1;nav.publish(old);spin(.05)
        flags(False,True)
        assert reset();assert not arbiter._commands
        spin(.05);assert output[-1]==0.
        result['explicit_armed_reset_clears_old_commands']=True
        nav.publish(old);spin(.05);assert output[-1]==.1
        result['new_intent_only_after_reset']=True
        flags(True,True);flags(False,True);assert arbiter._emergency
        spin(.35);assert not reset();result['stale_reset_rejected']=True
        Path(args.json).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    finally:
        executor.shutdown();probe.destroy_node();arbiter.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
