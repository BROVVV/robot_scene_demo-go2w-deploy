#!/usr/bin/python3
"""Inject synthetic scans only into acceptance topics; no physical command output."""
import argparse,json,math,time
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def main():
    p=argparse.ArgumentParser();p.add_argument('--json',required=True);args=p.parse_args()
    rclpy.init();n=rclpy.create_node('go2w_isolated_collision_probe')
    command=n.create_publisher(Twist,'/go2w/acceptance/raw',10)
    scanpub=n.create_publisher(LaserScan,'/go2w/acceptance/synthetic_scan',10)
    received=[]
    n.create_subscription(Twist,'/go2w/acceptance/filtered',lambda m:received.append((time.monotonic(),m.linear.x,m.angular.z)),10)
    def phase(duration, obstacle=None, scans=True):
        start=time.monotonic();next_send=start
        while time.monotonic()-start<duration:
            if time.monotonic()>=next_send:
                m=Twist();m.linear.x=.12;command.publish(m)
                if scans:
                    scan=LaserScan();scan.header.frame_id='base_link';scan.header.stamp=n.get_clock().now().to_msg()
                    scan.angle_min=-math.pi;scan.angle_increment=math.pi/360;scan.angle_max=scan.angle_min+719*scan.angle_increment
                    scan.range_min=.1;scan.range_max=6.2;scan.ranges=[6.]*720
                    if obstacle is not None:
                        for i in range(352,369):scan.ranges[i]=obstacle
                    scanpub.publish(scan)
                next_send=time.monotonic()+.04
            rclpy.spin_once(n,timeout_sec=.005)
        return start,[v for v in received if v[0]>start+max(0,duration-.3)]
    phase(1)
    _,free=phase(1)
    _,caution=phase(1,obstacle=.8)
    stopped,stop=phase(1,obstacle=.45)
    phase(1)
    dropped,drop=phase(.8,scans=False)
    zeros=[v[0]-dropped for v in received if v[0]>=dropped and abs(v[1])+abs(v[2])<1e-8]
    def speeds(values):return [round(v[1],4) for v in values]
    report={'physical_motion_enabled':False,'topics_prefix':'/go2w/acceptance/',
            'free_mps':speeds(free),'caution_mps':speeds(caution),'stop_mps':speeds(stop),
            'dropout_mps':speeds(drop),'dropout_to_zero_s':min(zeros) if zeros else None,
            'passed':bool(free and caution and stop and drop) and min(v[1] for v in free)>.11
              and all(v[1]>=.11 for v in caution) and all(abs(v[1])<1e-8 for v in stop+drop)
              and bool(zeros) and min(zeros)<=.4}
    command.publish(Twist())
    Path(args.json).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report));n.destroy_node();rclpy.shutdown()
    return 0 if report['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
