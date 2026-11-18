#!/usr/bin/python3
"""Pause one verified local preprocessor, measure fail-closed latency, always resume."""
import argparse,json,os,signal,time
from pathlib import Path
import rclpy
from std_msgs.msg import Bool


def main():
    p=argparse.ArgumentParser();p.add_argument('--pid',type=int,required=True);p.add_argument('--json',required=True);args=p.parse_args()
    command=Path(f'/proc/{args.pid}/cmdline').read_bytes().replace(b'\0',b' ').decode()
    if '/lib/go2w_lidar_preprocessor/lidar_preprocessor' not in command:raise ValueError('not a verified preprocessor')
    rclpy.init();node=rclpy.create_node('go2w_dropout_probe');seen={};events=[]
    def receive(key,msg):
        seen[key]=bool(msg.data);events.append({'key':key,'value':bool(msg.data),'t':time.monotonic()})
    for key,topic in [('lidar','/go2w/safety/lidar_fresh'),('localization','/go2w/nav/localization_valid')]:
        node.create_subscription(Bool,topic,lambda m,k=key:receive(k,m),10)
    end=time.monotonic()+5
    while time.monotonic()<end and not all(seen.get(k) for k in ('lidar','localization')):rclpy.spin_once(node,timeout_sec=.02)
    if not all(seen.get(k) for k in ('lidar','localization')):raise RuntimeError('baseline must be healthy')
    paused=time.monotonic();os.kill(args.pid,signal.SIGSTOP)
    try:
        while time.monotonic()-paused<.8:rclpy.spin_once(node,timeout_sec=.01)
    finally:os.kill(args.pid,signal.SIGCONT)
    end=time.monotonic()+5
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.02)
        if all(seen.get(k) for k in ('lidar','localization')):break
    latency={key:min([e['t']-paused for e in events if e['t']>=paused and e['key']==key and not e['value']],default=None) for key in ('lidar','localization')}
    report={'motion_sent':False,'pause_s':.8,'false_latency_s':latency,'recovered':all(seen.get(k) for k in ('lidar','localization')),
            'passed':all(v is not None and v<=.3 for v in latency.values())}
    Path(args.json).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report));node.destroy_node();rclpy.shutdown()
    return 0 if report['passed'] and report['recovered'] else 1
if __name__=='__main__':raise SystemExit(main())
