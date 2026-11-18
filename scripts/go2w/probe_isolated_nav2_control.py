#!/usr/bin/python3
"""Run/cancel a Nav2 goal with collision output isolated from the physical arbiter."""
import argparse,json,math,time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile,ReliabilityPolicy
from tf2_ros import Buffer,TransformListener


def main():
    p=argparse.ArgumentParser();p.add_argument('--json',required=True);args=p.parse_args()
    rclpy.init();n=rclpy.create_node('go2w_isolated_control_probe');b=Buffer();listener=TransformListener(b,n)
    client=ActionClient(n,NavigateToPose,'/navigate_to_pose')
    stamps={};odom=[]
    for key,topic in [('raw','/cmd_vel_nav_raw'),('smoothed','/cmd_vel_nav_smoothed'),('filtered','/go2w/acceptance/nav2_cmd_vel')]:
        n.create_subscription(Twist,topic,lambda m,k=key:stamps.setdefault(k,[]).append([time.monotonic(),m.linear.x,m.angular.z]),10)
    n.create_subscription(Odometry,'/go2w/odom/fused',lambda m:odom.append([m.pose.pose.position.x,m.pose.pose.position.y]),QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
    spin(2)
    assert n.count_publishers('/go2w/nav2_cmd_vel')==0,'physical Nav2 output must have no publisher'
    for name,param,expected in [('collision_monitor','cmd_vel_out_topic','/go2w/acceptance/nav2_cmd_vel'),('go2w_cmd_vel_bridge','execution_enabled',False)]:
        c=n.create_client(GetParameters,f'/{name}/get_parameters');assert c.wait_for_service(timeout_sec=3)
        request=GetParameters.Request();request.names=[param];f=c.call_async(request);rclpy.spin_until_future_complete(n,f,timeout_sec=3)
        value=f.result().values[0]
        assert (value.string_value if isinstance(expected,str) else value.bool_value)==expected
    transform=b.lookup_transform('map','base_link',rclpy.time.Time());start=transform.transform.translation
    goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=n.get_clock().now().to_msg()
    goal.pose.pose.position.x=start.x+.5;goal.pose.pose.position.y=start.y;goal.pose.pose.orientation=transform.transform.rotation
    f=client.send_goal_async(goal);rclpy.spin_until_future_complete(n,f,timeout_sec=5);handle=f.result();assert handle and handle.accepted
    spin(2)
    canceled=time.monotonic();f=handle.cancel_goal_async();rclpy.spin_until_future_complete(n,f,timeout_sec=3);spin(1)
    result=handle.get_result_async();rclpy.spin_until_future_complete(n,result,timeout_sec=3)
    delta=math.hypot(odom[-1][0]-odom[0][0],odom[-1][1]-odom[0][1]) if odom else None
    output=stamps.get('filtered',[])
    zeros=[v[0]-canceled for v in output if v[0]>=canceled and abs(v[1])+abs(v[2])<1e-8]
    maxima={k:max((abs(v[1]) for v in values),default=0) for k,values in stamps.items()}
    report={'physical_motion_enabled':False,'goal_sent':True,'cancel_status':result.result().status if result.done() else None,
            'odom_delta_m':delta,'maximum_linear_mps':maxima,'cancel_to_zero_s':min(zeros) if zeros else None,
            'samples':{k:len(v) for k,v in stamps.items()},
            'passed':all(maxima.get(k,0)>0 for k in ('raw','smoothed','filtered')) and bool(zeros) and min(zeros)<=.3 and delta is not None and delta<.005}
    Path(args.json).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report));n.destroy_node();rclpy.shutdown()
    return 0 if report['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
