#!/usr/bin/python3
"""Read-only planner probe: ComputePathToPose only, never NavigateToPose."""
import argparse
import json
import math
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def main():
    p=argparse.ArgumentParser();p.add_argument('--json',required=True);args=p.parse_args()
    rclpy.init();node=Node('go2w_readonly_plan_probe');buffer=Buffer();listener=TransformListener(buffer,node)
    client=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
    transform=None;end=time.monotonic()+10
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.05)
        try:
            transform=buffer.lookup_transform('map','base_link',rclpy.time.Time())
            if client.server_is_ready():break
        except Exception:pass
    if transform is None or not client.server_is_ready():raise RuntimeError('planner/TF unavailable')
    x,y=transform.transform.translation.x,transform.transform.translation.y
    q=transform.transform.rotation
    def pose(px,py):
        m=PoseStamped();m.header.frame_id='map';m.header.stamp=node.get_clock().now().to_msg()
        m.pose.position.x=px;m.pose.position.y=py;m.pose.orientation=q
        return m
    results=[]
    explicit_start=hasattr(ComputePathToPose.Goal(),'start')
    for dx,dy in ((.2,0),(.5,0),(0,.5),(0,-.5),(-.5,0),(1000,1000)):
        goal=ComputePathToPose.Goal()
        if explicit_start:
            goal.start=pose(x,y);goal.use_start=True
        target=pose(x+dx,y+dy)
        if hasattr(goal,'goal'):goal.goal=target
        else:goal.pose=target
        goal.planner_id='GridBased'
        future=client.send_goal_async(goal);rclpy.spin_until_future_complete(node,future,timeout_sec=10)
        handle=future.result()
        item={'offset':[dx,dy],'accepted':bool(handle and handle.accepted)}
        if item['accepted']:
            result=handle.get_result_async();rclpy.spin_until_future_complete(node,result,timeout_sec=10)
            if result.done() and result.result():
                response=result.result();poses=response.result.path.poses
                length=sum(math.hypot(b.pose.position.x-a.pose.position.x,b.pose.position.y-a.pose.position.y) for a,b in zip(poses,poses[1:]))
                item.update(status=response.status,poses=len(poses),length_m=length,
                            path=[[p.pose.position.x,p.pose.position.y] for p in poses])
            else:
                handle.cancel_goal_async();item['error']='planner_timeout'
        results.append(item)
    payload={'motion_sent':False,'action':'ComputePathToPose','start':[x,y],'results':results,
             'start_source':'explicit_start' if explicit_start else 'live_robot_tf',
             'positive_plans':sum(i.get('status')==4 and i.get('poses',0)>0 for i in results[:-1]),
             'invalid_goal_rejected':results[-1].get('status')!=4}
    Path(args.json).write_text(json.dumps(payload,indent=2)+'\n')
    print(json.dumps({k:v for k,v in payload.items() if k!='results'}))
    client.destroy();node.destroy_node();rclpy.shutdown()
    return 0 if payload['positive_plans']==5 and payload['invalid_goal_rejected'] else 1

if __name__=='__main__':raise SystemExit(main())
