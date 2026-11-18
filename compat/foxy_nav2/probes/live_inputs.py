#!/usr/bin/python3
"""Copy a fixed set of live sensor topics from robot domain 0 into loopback 99.

The source creates subscriptions only. The sink creates only whitelisted
sensor/TF publishers. Command, arm, lease, action and service APIs are absent.
Headers and health timestamps are preserved, not repaired or restamped.
"""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import platform
import queue
import signal
import socket
import struct
import threading
import time

TOPICS = (
    ('/map','nav_msgs.msg','OccupancyGrid','map'),
    ('/tf','tf2_msgs.msg','TFMessage','tf'),
    ('/tf_static','tf2_msgs.msg','TFMessage','static'),
    ('/go2w/odom/fused','nav_msgs.msg','Odometry','odom'),
    ('/go2w/lidar/navigation_scan','sensor_msgs.msg','LaserScan','sensor'),
    ('/go2w/lidar/scan','sensor_msgs.msg','LaserScan','sensor'),
    ('/go2w/nav/localization_status','std_msgs.msg','String','status'),
    ('/go2w/nav/localization_valid','std_msgs.msg','Bool','status'),
    ('/go2w/safety/lidar_fresh','std_msgs.msg','Bool','status'),
    ('/go2w/lidar/geometry_status','std_msgs.msg','String','status'),
)
HEADER=struct.Struct('!HIQ')
MAX_PAYLOAD=16*1024*1024


def loopback_configuration():
    # Six lifecycle processes plus guard, fixture, relay and cleanup need more
    # than CycloneDDS's installed default of indices 0..9. Local transport only.
    return ('<CycloneDDS><Domain><General><NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress>'
            '<AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex>'
            '<MaxAutoParticipantIndex>32</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/>'
            '</Peers></Discovery></Domain></CycloneDDS>')


def packet(index,data,receipt_ns):
    if type(index) is not int or not 0<=index<len(TOPICS) or not 0<len(data)<=MAX_PAYLOAD:
        raise ValueError('invalid whitelisted sensor packet')
    return HEADER.pack(index,len(data),receipt_ns)+data


def exact(stream,size,stop=None):
    output=bytearray()
    while len(output)<size:
        try:part=stream.recv(size-len(output))
        except socket.timeout:
            if stop is not None and stop.is_set():raise InterruptedError('sensor relay stopped')
            continue
        if not part:raise EOFError('sensor stream closed')
        output.extend(part)
    return bytes(output)


def read_packet(stream,stop=None):
    index,size,receipt_ns=HEADER.unpack(exact(stream,HEADER.size,stop))
    if not 0<=index<len(TOPICS) or not 0<size<=MAX_PAYLOAD:
        raise ValueError('sensor packet outside fixed whitelist/size bound')
    return index,exact(stream,size,stop),receipt_ns


def sensor_types():
    import importlib
    return [getattr(importlib.import_module(module),name) for _,module,name,_ in TOPICS]


def header_age(message,topic):
    if hasattr(message,'header'):
        stamp=message.header.stamp
        return time.time()-(stamp.sec+stamp.nanosec/1e9)
    if topic=='/go2w/nav/localization_status':
        try:return time.time()-float(json.loads(message.data)['wall_time'])
        except (ValueError,KeyError,TypeError):return None
    return None


def qos(kind):
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
    if kind=='sensor':return qos_profile_sensor_data
    return QoSProfile(depth=100 if kind in ('tf','static') else 1 if kind=='map' else 10,
        durability=DurabilityPolicy.TRANSIENT_LOCAL if kind in ('map','static') else DurabilityPolicy.VOLATILE,
        reliability=ReliabilityPolicy.RELIABLE)


def source(node,path,stop,stats):
    assert os.environ.get('ROS_DOMAIN_ID','0')=='0'
    assert platform.machine()=='aarch64' and Path('/etc/nv_tegra_release').is_file()
    from rclpy.serialization import serialize_message
    from geometry_msgs.msg import Twist
    physical=['/go2w/nav2_cmd_vel','/go2w/software_arm','/go2w/sport_lease/alive']
    assert all(node.count_publishers(topic)==0 for topic in physical)
    stats['physical_velocity_messages']=0
    def witness(_):stats['physical_velocity_messages']+=1
    velocity_witness=node.create_subscription(Twist,physical[0],witness,10)
    assert not path.exists(), 'never replace a live sensor socket'
    server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);server.bind(str(path));os.chmod(path,0o600);server.listen(1);server.settimeout(30.)
    pending=queue.Queue(maxsize=64);subscriptions=[];errors=[]
    for index,((topic,_,_,kind),cls) in enumerate(zip(TOPICS,sensor_types())):
        def receive(message,index=index):
            receipt=time.monotonic_ns();raw=serialize_message(message)
            topic=TOPICS[index][0]
            if topic in ('/go2w/lidar/scan','/go2w/lidar/navigation_scan','/go2w/nav/localization_status') and len(stats['timings'])<20000:
                stats['timings'].append({'topic':topic,'receipt_ns':receipt,'header_age_s':header_age(message,topic),'queue_depth':pending.qsize()})
            try:pending.put_nowait(packet(index,raw,receipt))
            except queue.Full:stats['dropped'][TOPICS[index][0]]+=1
            stats['received'][TOPICS[index][0]]+=1
        # A permissive reader can receive the reliable wheel publisher, but
        # the sink must offer reliable odometry to Foxy's controller.
        subscriptions.append(node.create_subscription(cls,topic,receive,qos('sensor' if kind=='odom' else kind)))
    conn=None;thread=None
    try:
        conn,_=server.accept();conn.settimeout(2.)
        def transmit():
            try:
                while not stop.is_set():
                    try:data=pending.get(timeout=.1)
                    except queue.Empty:continue
                    conn.sendall(data)
            except BaseException as exc:
                if not stop.is_set():errors.append(repr(exc))
                stop.set()
        thread=threading.Thread(target=transmit,daemon=True);thread.start()
        import rclpy
        while not stop.is_set():rclpy.spin_once(node,timeout_sec=.02)
        if errors:stats['stream_error']=errors[-1]
        assert stats['physical_velocity_messages']==0
        assert all(node.count_publishers(topic)==0 for topic in physical)
    finally:
        stop.set()
        if conn is not None:conn.close()
        server.close()
        if thread is not None:thread.join(timeout=3.)
        if path.exists():path.unlink()


def sink(node,path,stop,stats):
    try:from .velocity_chain import isolated_environment
    except ImportError:from velocity_chain import isolated_environment
    isolated_environment()
    from rclpy.serialization import deserialize_message
    classes=sensor_types();publishers=[node.create_publisher(cls,topic,qos(kind))
        for (topic,_,_,kind),cls in zip(TOPICS,classes)]
    conn=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);deadline=time.monotonic()+30
    while True:
        try:conn.connect(str(path));break
        except (FileNotFoundError,ConnectionRefusedError):
            if time.monotonic()>deadline:raise
            time.sleep(.05)
    conn.settimeout(.5)
    try:
        while not stop.is_set():
            try:index,raw,receipt=read_packet(conn,stop)
            except InterruptedError:break
            except EOFError:
                stats['source_stream_closed']=True
                break
            lag=(time.monotonic_ns()-receipt)/1e9
            stats['lag_max_s']=max(stats.get('lag_max_s',0.),lag)
            stats['published'][TOPICS[index][0]]+=1
            message=deserialize_message(raw,classes[index]);topic=TOPICS[index][0]
            if topic in ('/go2w/lidar/scan','/go2w/lidar/navigation_scan','/go2w/nav/localization_status') and len(stats['timings'])<20000:
                stats['timings'].append({'topic':topic,'received_ns':time.monotonic_ns(),'source_receipt_ns':receipt,'lag_s':lag,'header_age_s':header_age(message,topic)})
            publishers[index].publish(message)
    finally:conn.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('role',choices=['source','sink']);parser.add_argument('--socket',type=Path,required=True);parser.add_argument('--report',type=Path,required=True);args=parser.parse_args()
    stop=threading.Event();signal.signal(signal.SIGINT,lambda *_:stop.set());signal.signal(signal.SIGTERM,lambda *_:stop.set())
    import rclpy
    stats={'role':args.role,'started_wall':time.time(),'domain':os.environ.get('ROS_DOMAIN_ID'),'received':Counter(),'published':Counter(),'dropped':Counter(),'timings':[],'sensor_timestamps_changed':False,'motion_sent':False}
    rclpy.init();node=rclpy.create_node('acceptance_live_sensor_'+args.role)
    try:
        (source if args.role=='source' else sink)(node,args.socket.resolve(),stop,stats)
        stats['passed']=True
    except BaseException as exc:stats.update(passed=False,error=repr(exc));raise
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        stats['ended_wall']=time.time();args.report.write_text(json.dumps(stats,indent=2)+'\n')


if __name__=='__main__':main()
