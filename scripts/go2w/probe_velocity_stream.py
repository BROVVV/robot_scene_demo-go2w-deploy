#!/usr/bin/python3
"""Isolated real bridge/proxy/gateway/controller protocol; SDK is DRY RUN.

Domain 0 is refused. Synthetic robot state is restricted to this isolated
domain. Tests continuous updates, token isolation, and expiry STOP requests.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import threading


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--domain-id',type=int,default=79)
    p.add_argument('--output-dir',required=True)
    p.add_argument('--proxy-script',help='isolated fault-injection copy; defaults to production proxy')
    args=p.parse_args()
    if not 1<=args.domain_id<=100:raise ValueError('isolated nonzero domain required')
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id),ROS_LOCALHOST_ONLY='1')
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from std_msgs.msg import String,Bool,UInt64
    from unitree_go.msg import SportModeState,LowState
    from go2w_motion_interfaces.action import MotionCommand
    from go2w_cmd_vel_bridge.bridge_node import CmdVelBridge
    from go2w_cmd_vel_bridge.bridge_core import Velocity
    root=Path(__file__).resolve().parents[2]
    out=Path(args.output_dir).resolve();out.mkdir(parents=True,exist_ok=True)
    sock=out/'gateway.sock'
    from ament_index_python.packages import get_package_prefix
    exe=Path(get_package_prefix('go2w_motion_control'))/'lib/go2w_motion_control/go2w_motion_action_server'
    children=[];logs=[]
    def launch(cmd,name):
        log=(out/(name+'.log')).open('w');logs.append(log)
        children.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT))
    launch([str(exe),'--ros-args','--params-file',str(root/'unitree_go2w_control/ros2_ws/src/go2w_motion_control/config/motion_control.yaml'),
        '-p','dry_run:=true','-p','require_arm:=false','-p','max_abs_vx:=0.06',
        '-p','post_turn_rollback_max_vx:=0.06','-p','log_root:='+str(out/'motion_logs'),
        '-p','sdk_command_socket:=/isolated_probe_no_physical_sdk_socket'], 'controller')
    launch([sys.executable,str(root/'scripts/go2w/motion_local_gateway.py'),'--socket',str(sock)],'gateway')
    launch([sys.executable,str(Path(args.proxy_script).resolve() if args.proxy_script else
        root/'scripts/go2w/motion_network_proxy.py'),'--socket',str(sock)],'proxy')
    rclpy.init();node=rclpy.create_node('isolated_velocity_stream_fixture');bridge=CmdVelBridge()
    for timer in bridge.timers:timer.cancel()
    assert not bridge._execution_enabled and bridge._client is None
    bridge._client=ActionClient(bridge,MotionCommand,'/go2w/motion')
    bridge._dispatch_permitted=True
    executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(bridge)
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pubs=[node.create_publisher(t,topic,q) for t,topic,q in [
        (SportModeState,'/go2w/motion/local_sportmodestate',10),
        (LowState,'/go2w/motion/local_lowstate',10),
        (String,'/go2w/motion_mode/name',latched),
        (UInt64,'/go2w/sport_lease/id',latched),(Bool,'/go2w/sport_lease/alive',10)]]
    results=[];original=bridge._goal_result
    def completed(f):
        response=f.result();r=response.result
        results.append({'success':r.success,'error_code':r.error_code,'message':r.message,'action_status':response.status})
        original(f)
    bridge._goal_result=completed
    def spin(seconds,refresh=None,foreign=False,over_limit=False):
        end=time.monotonic()+seconds;next_pub=0
        while time.monotonic()<end:
            now=time.monotonic()
            if now>=next_pub:
                m=SportModeState();m.mode=1
                for pub,msg in zip(pubs,[m,LowState(),String(data='ai-w'),UInt64(data=123),Bool(data=True)]):pub.publish(msg)
                if refresh is not None:bridge._dispatch(Velocity(refresh,0.))
                if foreign:bridge._stream_pub.publish(String(data=json.dumps(dict(version=1,token='f'*32,
                    stamp=time.time(),vx=.05,vy=0.,yaw_rate=0.))))
                if over_limit and bridge._stream_token:
                    bridge._stream_pub.publish(String(data=json.dumps(dict(version=1,
                        token=bridge._stream_token,stamp=time.time(),vx=.08,vy=0.,yaw_rate=0.))))
                next_pub=now+.04
            executor.spin_once(timeout_sec=.005)
            if any(c.poll() is not None for c in children):raise RuntimeError('isolated process exited')
    result={'scope':'isolated ROS, synthetic state, real bridge/proxy/gateway/controller, dry-run SDK',
            'domain_id':args.domain_id,'physical_motion':False}
    try:
        spin(3)
        assert bridge._client.server_is_ready()
        spin(2.5,refresh=.05)
        assert not results,results  # stream survived many 0.25 s windows
        last_valid=time.time()
        spin(2,foreign=True)
        assert len(results)==1 and results[0]['error_code']==MotionCommand.Result.ERROR_TIMEOUT,results
        requests=[]
        for f in (out/'motion_logs').glob('*/goal_*/requests.jsonl'):
            requests.extend(json.loads(line) for line in f.read_text().splitlines())
        moves=[r for r in requests if r['api_id']==1008]
        stops=[r for r in requests if r['api_id']==1003 and float(r['timestamp'])/1e6>=last_valid]
        assert len(moves)>=12,len(moves)
        stop_delay=min(float(r['timestamp'])/1e6-last_valid for r in stops)
        assert 0<=stop_delay<=.40,stop_delay
        # Private STOP need not place the public goal into CANCELING. It must
        # preserve ERROR_CANCELED as an aborted public result, never attempt
        # the illegal EXECUTING -> CANCELED transition or lose the payload.
        bridge._motion_fault=''
        spin(1.5,refresh=.05)
        from motion_network_proxy import gateway_request
        cancel_reply={}
        def private_cancel():cancel_reply.update(gateway_request(sock,{'op':'cancel'},5))
        thread=threading.Thread(target=private_cancel,daemon=True);thread.start()
        spin(2);thread.join(1)
        assert len(results)==2 and results[-1]['error_code']==MotionCommand.Result.ERROR_CANCELED,results
        assert results[-1]['action_status']==6,results  # ABORTED; public goal was never CANCELING
        spin(1.5,refresh=.05)
        spin(2,over_limit=True)
        assert len(results)==3 and results[-1]['error_code']==MotionCommand.Result.ERROR_TIMEOUT,results
        # Updates below the protocol's .10 ceiling must still obey the
        # controller's stricter configured .06 limit and cannot renew TTL.
        result['configured_speed_limit_rejects_updates']=True
        result.update(passed=True,results=results,move_rpc_count=len(moves),expiry_stop_delay_s=stop_delay)
    finally:
        executor.shutdown();bridge._client.destroy();bridge.destroy_node();node.destroy_node();rclpy.shutdown()
        for child in reversed(children):child.terminate()
        for child in children:
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        for log in logs:log.close()
        (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))


if __name__=='__main__':main()
