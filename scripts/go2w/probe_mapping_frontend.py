#!/usr/bin/python3
"""Test mapping frontend against live sensors with EVERY output remapped.

Creates no motion, map or TF publishers. A subscriber named slam_toolbox in a
separate namespace tests mapper attachment; it is not an actual SLAM process.
"""
import argparse
import ast
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output-dir', required=True)
    args = p.parse_args()
    root = Path(__file__).resolve().parents[2]
    out = Path(args.output_dir).resolve()
    out.mkdir(parents=True, exist_ok=True)
    source = root/'ros2_ws/src/go2w_lidar_preprocessor/go2w_lidar_preprocessor/preprocessor_node.py'
    tree = ast.parse(source.read_text())
    topics = []
    for call in ast.walk(tree):
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute) and call.func.attr == 'create_publisher':
            if len(call.args) < 2 or not isinstance(call.args[1], ast.Constant):
                raise ValueError('all shadow publishers must be statically enumerated')
            topics.append(call.args[1].value)
    def shadow(topic):
        if not topic.startswith('/go2w/'):
            raise ValueError('unexpected publisher '+topic)
        return topic.replace('/go2w/', '/go2w/mapping_probe/', 1)
    command = ['ros2', 'run', 'go2w_lidar_preprocessor', 'lidar_preprocessor', '--ros-args',
               '-p', 'config_file:='+str(root/'configs/go2w/lidar_preprocess.yaml'),
               '-p', 'geometry_file:='+str(root/'configs/go2w/official_reference.yaml'),
               '-p', 'mapping_accumulation:=true', '-r', '__node:=go2w_mapping_probe']
    for topic in topics:
        command += ['-r', topic+':='+shadow(topic)]
    (out/'argv.json').write_text(json.dumps(command, indent=2)+'\n')
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    node = rclpy.create_node('mapping_frontend_observer')
    observer_nodes = []
    result = {'motion_sent': False, 'scans': {}, 'poses': [], 'statuses': [], 'phases': []}
    def on_scan(m, topic):
        result['scans'].setdefault(topic, []).append({
            'received': time.time(), 'stamp': m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
            'frame': m.header.frame_id, 'angle_min': m.angle_min, 'angle_increment': m.angle_increment,
            'range_min': m.range_min, 'range_max': m.range_max,
            'ranges': [float(v) if math.isfinite(v) else None for v in m.ranges]})
    scan_topics = ['/go2w/lidar/scan', '/go2w/lidar/mapping_scan',
                   shadow('/go2w/lidar/scan'), shadow('/go2w/lidar/mapping_scan'),
                   shadow('/go2w/lidar/mapping_keyframe_scan')]
    for topic in scan_topics:
        node.create_subscription(LaserScan, topic, lambda m, t=topic: on_scan(m, t), qos_profile_sensor_data)
    def pose(m):
        a,q = m.pose.pose.position,m.pose.pose.orientation
        result['poses'].append({'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
            'x':a.x, 'y':a.y, 'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))})
    node.create_subscription(Odometry, '/go2w/odom/fused', pose, qos_profile_sensor_data)
    node.create_subscription(String, shadow('/go2w/lidar/mapping_scan_status'),
                             lambda m: result['statuses'].append(json.loads(m.data)), 10)
    key = shadow('/go2w/lidar/mapping_keyframe_scan')
    def phase(name, seconds):
        before = len(result['scans'].get(key, []))
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.03)
            if child.poll() is not None:
                raise RuntimeError('shadow node exited early')
        result['phases'].append({'name':name, 'new_keyframes':len(result['scans'].get(key,[]))-before})
    def mapper_reader():
        n = rclpy.create_node('slam_toolbox', namespace='/mapping_frontend_probe')
        n.create_subscription(LaserScan, key, lambda _:None, qos_profile_sensor_data)
        observer_nodes.append(n)
        return n
    log = (out/'shadow.log').open('w')
    child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        phase('initial_warmup', 5)
        reader = mapper_reader()
        phase('mapper_attach', 4)
        phase('stationary_no_repeat', 4)
        reader.destroy_node();observer_nodes.remove(reader)
        phase('mapper_detach', 1)
        mapper_reader()
        phase('mapper_restart', 4)
    finally:
        os.killpg(child.pid, signal.SIGINT)
        try:
            child.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGKILL);child.wait()
        log.close()
        for n in observer_nodes:n.destroy_node()
        node.destroy_node();rclpy.shutdown()
        (out/'observations.json').write_text(json.dumps(result, allow_nan=False)+'\n')
    original = {s['stamp']:s['ranges'] for s in result['scans'].get('/go2w/lidar/scan',[])}
    pairs = [(original[s['stamp']], s['ranges']) for s in result['scans'].get(shadow('/go2w/lidar/scan'),[])
             if s['stamp'] in original]
    collision_equal = bool(pairs) and all(a == b for a,b in pairs)
    phases = {v['name']:v['new_keyframes'] for v in result['phases']}
    passed = (child.returncode == 0 and collision_equal and phases['mapper_attach'] == 1
              and phases['stationary_no_repeat'] == 0 and phases['mapper_restart'] == 1)
    import statistics
    summary = {'passed':passed, 'scope':'live shadow mapping frontend only; no physical actuation',
               'child_exit_code':child.returncode, 'phases':phases,
               'collision_scan_paired_frames':len(pairs), 'collision_scans_identical':collision_equal,
               'median_observed_bins': {t:statistics.median(sum(v is not None for v in s['ranges']) for s in frames)
                                        for t,frames in result['scans'].items() if frames}}
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary),flush=True)
    return 0 if passed else 2


if __name__ == '__main__':
    raise SystemExit(main())
