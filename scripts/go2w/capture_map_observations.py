#!/usr/bin/python3
"""Read-only map, robot TF and per-frame scan capture; creates no publishers."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=10)
    parser.add_argument('--json', required=True)
    args = parser.parse_args()
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = rclpy.create_node('capture_map_observations')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    result = {'recorded_at': time.time(), 'motion_authorized': False,
              'maps': [], 'raw_maps': [], 'scans': {}, 'poses': []}

    def on_map(m, collection):
        grid = {'width': m.info.width, 'height': m.info.height,
                'resolution': m.info.resolution,
                'origin': [m.info.origin.position.x, m.info.origin.position.y],
                'origin_orientation': [getattr(m.info.origin.orientation, k) for k in ('x', 'y', 'z', 'w')],
                'frame': m.header.frame_id, 'data': list(m.data)}
        digest = hashlib.sha256(json.dumps(grid, sort_keys=True).encode()).hexdigest()
        rows = result[collection]
        rows.append({'received': time.time(), 'hash': digest,
                     'stamp': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                     **(grid if not rows or digest != rows[-1]['hash'] else {})})

    def on_scan(m, topic):
        result['scans'].setdefault(topic, []).append({
            'received': time.time(), 'stamp': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
            'frame': m.header.frame_id, 'angle_min': m.angle_min, 'angle_increment': m.angle_increment,
            'range_min': m.range_min, 'range_max': m.range_max,
            'ranges': [float(v) if math.isfinite(v) else None for v in m.ranges]})

    node.create_subscription(OccupancyGrid, '/map', lambda m: on_map(m, 'maps'),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                        reliability=ReliabilityPolicy.RELIABLE))
    node.create_subscription(OccupancyGrid, '/go2w/slam/map_raw',
                             lambda m: on_map(m, 'raw_maps'),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                        reliability=ReliabilityPolicy.RELIABLE))
    for topic in ('/go2w/lidar/scan', '/go2w/lidar/mapping_scan'):
        node.create_subscription(LaserScan, topic, lambda m, t=topic: on_scan(m, t),
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            try:
                t = buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            except Exception:
                continue
            p, q = t.transform.translation, t.transform.rotation
            pose = {'stamp': t.header.stamp.sec + t.header.stamp.nanosec * 1e-9,
                    'x': p.x, 'y': p.y,
                    'yaw': math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))}
            if not result['poses'] or pose['stamp'] != result['poses'][-1]['stamp']:
                result['poses'].append(pose)
    finally:
        del listener
        node.destroy_node()
        rclpy.shutdown()
    path = Path(args.json)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, allow_nan=False) + '\n')
    print(json.dumps({'output': str(path), 'maps': len(result['maps']),
                      'map_hashes': len({m['hash'] for m in result['maps']}),
                      'raw_maps': len(result['raw_maps']),
                      'raw_map_hashes': len({m['hash'] for m in result['raw_maps']}),
                      'scans': {k: len(v) for k, v in result['scans'].items()},
                      'pose': result['poses'][-1] if result['poses'] else None}))
    return 0 if result['maps'] and result['poses'] and len(result['scans']) == 2 else 2


if __name__ == '__main__':
    raise SystemExit(main())
