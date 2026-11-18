#!/usr/bin/python3
"""Isolated SLAM experiment on captured stationary scans; never uses robot domain.

Compares the production travel gate with a zero-travel diagnostic control.
This is replay evidence only, not permission to change the production SLAM gate.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time


def fused_stationary_ranges(frames):
    """Diagnostic control for stationary captures; retain unknown angular bins.

    Not a moving-robot scan accumulator: those require per-return transforms,
    bounds on age and motion, and an explicit ray-origin/occlusion policy.
    """
    first = frames[0]
    keys = ('angle_min', 'angle_increment', 'range_min', 'range_max', 'frame')
    if any(any(f[k] != first[k] for k in keys) or len(f['ranges']) != len(first['ranges']) for f in frames):
        raise ValueError('incompatible scan geometry')
    return [min((f['ranges'][i] for f in frames if f['ranges'][i] is not None
                 and math.isfinite(f['ranges'][i]) and first['range_min'] <= f['ranges'][i] <= first['range_max']),
                default=None) for i in range(len(first['ranges']))]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--snapshot', required=True)
    p.add_argument('--output-dir', required=True)
    p.add_argument('--domain-id', type=int, default=77)
    p.add_argument('--scan-topic', default='/go2w/lidar/mapping_scan')
    experiment = p.add_mutually_exclusive_group()
    experiment.add_argument('--production-gate-only', action='store_true')
    experiment.add_argument('--keyframe-input', action='store_true',
                            help='input already selected by the frontend; replay once with zero internal gate')
    args = p.parse_args()
    if args.domain_id == 0 or not 1 <= args.domain_id <= 100:
        raise ValueError('replay requires a separate nonzero ROS domain (1..100)')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    import yaml
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener

    source_path = Path(args.snapshot)
    source = json.loads(source_path.read_text())
    frames = source['scans'][args.scan_topic]
    poses = source.get('poses', [])
    if not frames or not poses:
        raise ValueError('captured scans and poses required')
    first = poses[0]
    max_translation = max(math.hypot(v['x']-first['x'], v['y']-first['y']) for v in poses)
    max_turn = max(abs(math.remainder(v['yaw']-first['yaw'], 2*math.pi)) for v in poses)
    # This experiment deliberately freezes the odometry to isolate scan gating.
    # Reject a moving capture rather than smearing it into a stationary replay.
    if max_translation > .02 or max_turn > .02:
        raise ValueError('capture moved; stationary replay would misrepresent geometry')
    if max_translation + frames[0]['range_max']*max_turn > .025:
        raise ValueError('capture motion exceeds half a map cell; fixed-origin fusion is unsuitable')
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    config = Path(__file__).resolve().parents[2] / 'ros2_ws/src/robot_scene_nav_bringup/config/slam_2d_mapping.yaml'
    base_params = yaml.safe_load(config.read_text())['slam_toolbox']['ros__parameters']
    exe = Path(get_package_prefix('slam_toolbox'))/'lib/slam_toolbox/async_slam_toolbox_node'
    summary = {'source_sha256': hashlib.sha256(source_path.read_bytes()).hexdigest(),
               'config_sha256': hashlib.sha256(config.read_bytes()).hexdigest(),
               'scope': 'isolated stationary replay; odometry frozen; not a live map or motion validation',
               'domain_id': args.domain_id, 'capture_max_translation_m': max_translation,
               'source_scan_topic': args.scan_topic,
               'capture_max_turn_rad': max_turn, 'motion_sent': False, 'runs': []}
    experiments = ((float(base_params['minimum_travel_distance']),0),) if args.production_gate_only else (
        ((0.,0),) if args.keyframe_input else ((.1,0),(0.,0),(.1,10)))
    for gate, prefill in experiments:
        label = 'keyframes_travel_0' if args.keyframe_input else ('prefill_10_travel_0_1' if prefill else 'travel_0_1' if gate else 'travel_0')
        params = dict(base_params, minimum_travel_distance=gate, scan_topic='/replay/scan',
                      odom_frame='replay_odom', base_frame='replay_base', map_frame='replay_map')
        with tempfile.TemporaryDirectory(prefix='go2w_replay_') as temporary:
            cfg = Path(temporary)/'params.yaml'
            cfg.write_text(yaml.safe_dump({'slam_toolbox': {'ros__parameters': params}}))
            log = (out/(label+'.log')).open('w')
            child = subprocess.Popen([str(exe), '--ros-args', '--params-file', str(cfg)], stdout=log, stderr=subprocess.STDOUT)
            rclpy.init()
            node = rclpy.create_node('stationary_replay_probe')
            broadcaster = StaticTransformBroadcaster(node)
            transform = TransformStamped()
            transform.header.stamp = node.get_clock().now().to_msg()
            transform.header.frame_id = 'replay_odom'
            transform.child_frame_id = 'replay_base'
            transform.transform.rotation.w = 1.
            broadcaster.sendTransform(transform)
            buffer = Buffer()
            listener = TransformListener(buffer, node)
            maps, corrections = [], []
            def on_map(m):
                maps.append({'width': m.info.width, 'height': m.info.height, 'resolution': m.info.resolution,
                             'origin': [m.info.origin.position.x,m.info.origin.position.y],
                             'data': list(m.data)})
            node.create_subscription(OccupancyGrid, '/map', on_map, QoSProfile(depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
            publisher = node.create_publisher(LaserScan, '/replay/scan', QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
            try:
                deadline = time.monotonic()+12
                while time.monotonic()<deadline and publisher.get_subscription_count() == 0:
                    rclpy.spin_once(node, timeout_sec=.1)
                    if child.poll() is not None:
                        raise RuntimeError('isolated SLAM exited; inspect '+str(out/(label+'.log')))
                if publisher.get_subscription_count() != 1:
                    raise RuntimeError('expected exactly one isolated SLAM scan subscriber')
                # Allow DDS scan matching and static TF discovery before first scan.
                until = time.monotonic()+1
                while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
                sent = 0
                start = time.monotonic()
                for index, f in enumerate(frames):
                    if prefill and index+1 < prefill:
                        continue
                    ranges = fused_stationary_ranges(frames[index-prefill+1:index+1]) if prefill else f['ranges']
                    msg = LaserScan()
                    msg.header.stamp = node.get_clock().now().to_msg()
                    msg.header.frame_id = 'replay_base'
                    for name in ('angle_min','angle_increment','range_min','range_max'):
                        setattr(msg,name,float(f[name]))
                    msg.angle_max = msg.angle_min+(len(f['ranges'])-1)*msg.angle_increment
                    msg.scan_time = .065
                    msg.ranges = [math.inf if r is None else r for r in ranges]
                    publisher.publish(msg);sent += 1
                    until = start+sent*.065
                    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
                    try:
                        t = buffer.lookup_transform('replay_map','replay_base',rclpy.time.Time())
                        v,q=t.transform.translation,t.transform.rotation
                        corrections.append({'x':v.x,'y':v.y,'yaw':math.atan2(2*q.w*q.z,1-2*q.z*q.z)})
                    except Exception:pass
                until = time.monotonic()+2
                while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
                if not maps:raise RuntimeError('replay produced no map')
                (out/(label+'_map.json')).write_text(json.dumps(maps[-1]))
                counts = Counter(maps[-1]['data'])
                summary['runs'].append({'minimum_travel_distance':gate, 'prefill_frames':prefill,
                    'scans_sent':sent,
                    'maps_received':len(maps), 'unique_map_hashes':len({hashlib.sha256(json.dumps(m,sort_keys=True).encode()).hexdigest() for m in maps}),
                    'cells':{'free':counts[0], 'occupied':counts[100], 'unknown':counts[-1]},
                    'max_map_pose_translation_m':max((math.hypot(v['x'],v['y']) for v in corrections),default=None),
                    'max_map_pose_yaw_rad':max((abs(v['yaw']) for v in corrections),default=None)})
                print(json.dumps(summary['runs'][-1]),flush=True)
            finally:
                node.destroy_node();rclpy.shutdown()
                child.terminate()
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:child.kill();child.wait()
                log.close()
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')


if __name__ == '__main__':
    main()
