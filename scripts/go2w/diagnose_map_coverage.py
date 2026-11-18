#!/usr/bin/python3
"""Read-only coverage diagnosis. Sampled lateral clearance is not a proof over all headings.

Use --snapshot with capture_map_observations.py output for offline reproduction.
No obstacle is labelled a self-return from distance alone. Robot-relative results
require an observed map->base_link pose; the map origin is not the robot pose.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import time

# Independent copies of the production clearance contract, for comparison.
HALF_BODY_WIDTH_M = .215
HALF_BODY_LENGTH_M = .35
FOOTPRINT_PADDING_M = .04
HARD_SIDE_GAP_M = .40
OCCUPIED_THRESHOLD = 65
MAX_LATERAL_SEARCH_M = 2.0


def _strip_noetic():
    import sys
    sys.path[:] = [p for p in sys.path if p.rstrip('/') != '/opt/ros/noetic/lib/python3/dist-packages']


def _grab_map(timeout_s):
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from tf2_ros import Buffer, TransformListener
    rclpy.init()
    node = rclpy.create_node('diagnose_map_coverage')
    maps = []
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    node.create_subscription(OccupancyGrid, '/map', maps.append,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                        reliability=ReliabilityPolicy.RELIABLE))
    pose = None
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            try:
                t = buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                p, q = t.transform.translation, t.transform.rotation
                age = node.get_clock().now().nanoseconds * 1e-9 - (t.header.stamp.sec + t.header.stamp.nanosec * 1e-9)
                if -.1 <= age <= .3:
                    pose = {'x': p.x, 'y': p.y, 'yaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))}
            except Exception:
                pass
            if maps and pose is not None:
                break
        if not maps:
            raise ValueError('no /map received')
        m = maps[-1]
        grid = {'width': m.info.width, 'height': m.info.height, 'resolution': m.info.resolution,
                'origin': [m.info.origin.position.x, m.info.origin.position.y],
                'origin_orientation': [getattr(m.info.origin.orientation, k) for k in ('x','y','z','w')],
                'frame': m.header.frame_id, 'data': list(m.data)}
        return grid, pose
    finally:
        del listener
        node.destroy_node()
        rclpy.shutdown()


def analyze_grid(grid, pose=None, *, heading_step_deg=5):
    """Max over sampled positions/headings of min(left,right), matching lateral rays.

    The former min(all eight rays) was an isotropic inradius, not an upper bound
    on directional corridor clearance. A short, wide room is a counterexample.
    Even this corrected finite sampling cannot exclude every continuous pose,
    and a passing point alone does not validate a whole path or its footprint.
    """
    width, height = int(grid['width']), int(grid['height'])
    resolution = float(grid['resolution'])
    ox, oy = grid['origin']
    data = grid['data']
    if (width <= 0 or height <= 0 or len(data) != width*height
            or not math.isfinite(resolution) or resolution <= 0
            or not all(math.isfinite(v) for v in (ox, oy))
            or any(not isinstance(v, int) or not -1 <= v <= 100 for v in data)):
        raise ValueError('invalid occupancy grid')
    q = grid.get('origin_orientation', [0, 0, 0, 1])
    if len(q) != 4 or any(not math.isfinite(v) for v in q) or any(abs(v) > 1e-8 for v in q[:3]) or abs(abs(q[3])-1) > 1e-8:
        raise ValueError('rotated map origin is unsupported; do not silently treat it as axis aligned')
    if not math.isfinite(heading_step_deg) or not 1 <= heading_step_deg <= 90:
        raise ValueError('heading step must be between 1 and 90 degrees')
    if pose is not None and any(not math.isfinite(pose[k]) for k in ('x','y','yaw')):
        raise ValueError('invalid robot pose')

    def kind(c, r):
        if not 0 <= c < width or not 0 <= r < height:
            return 'off_map'
        v = data[r*width+c]
        return 'unknown' if v < 0 else 'occupied' if v >= OCCUPIED_THRESHOLD else 'free'

    def at(x, y):
        return kind(math.floor((x-ox)/resolution), math.floor((y-oy)/resolution))

    def world(c, r):
        return ox+(c+.5)*resolution, oy+(r+.5)*resolution

    def ray(x, y, dx, dy):
        d = resolution
        while d <= MAX_LATERAL_SEARCH_M:
            if at(x+dx*d, y+dy*d) != 'free':
                return d
            d += resolution
        return MAX_LATERAL_SEARCH_M

    free = [(c,r) for r in range(height) for c in range(width) if kind(c,r) == 'free']
    occupied = [(c,r) for r in range(height) for c in range(width) if kind(c,r) == 'occupied']
    headings = [math.radians(k*heading_step_deg) for k in range(math.ceil(180/heading_step_deg))]
    directions = [(-math.sin(h), math.cos(h)) for h in headings]
    best = None
    meeting = 0
    for c, r in free:
        x, y = world(c, r)
        point_best = None
        for h, (dx, dy) in zip(headings, directions):
            distance = min(ray(x,y,dx,dy), ray(x,y,-dx,-dy))
            gap = distance-HALF_BODY_WIDTH_M-FOOTPRINT_PADDING_M
            if point_best is None or gap > point_best['gap_m']:
                point_best = {'gap_m': gap, 'x': x, 'y': y, 'heading_deg': math.degrees(h)}
        if point_best['gap_m'] >= HARD_SIDE_GAP_M:
            meeting += 1
        if best is None or point_best['gap_m'] > best['gap_m']:
            best = point_best

    footprint = None
    radial = None
    if pose is not None:
        x, y, yaw = (pose[k] for k in ('x','y','yaw'))
        ct, st = math.cos(yaw), math.sin(yaw)
        hl, hw = HALF_BODY_LENGTH_M+FOOTPRINT_PADDING_M, HALF_BODY_WIDTH_M+FOOTPRINT_PADDING_M
        # Conservative cell intersection: an occupied cell is an area, not a point.
        ex, ey = abs(hl*ct)+abs(hw*st), abs(hl*st)+abs(hw*ct)
        b = resolution/2
        local_cell_extent = b*(abs(ct)+abs(st))
        covered, blocked = 0, Counter()
        for c in range(math.floor((x-ex-ox)/resolution), math.floor((x+ex-ox)/resolution)+1):
            for r in range(math.floor((y-ey-oy)/resolution), math.floor((y+ey-oy)/resolution)+1):
                cx, cy = world(c, r)
                dx, dy = cx-x, cy-y
                if abs(dx) > ex+b or abs(dy) > ey+b:
                    continue
                if abs(dx*ct+dy*st) > hl+local_cell_extent or abs(-dx*st+dy*ct) > hw+local_cell_extent:
                    continue
                covered += 1
                if kind(c,r) != 'free':
                    blocked[kind(c,r)] += 1
        footprint = {'pose': pose, 'cells_covered': covered, 'blocked_cells': dict(blocked),
                     'already_blocked': bool(blocked)}
        distances = [math.hypot(world(c,r)[0]-x, world(c,r)[1]-y) for c,r in occupied]
        radial = {'nearest_occupied_cell_center_m': min(distances) if distances else None,
                  'within_0_35m': sum(d < .35 for d in distances),
                  'note': 'Distance alone cannot distinguish robot self-return from a real obstacle.'}
    return {'grid': {k:v for k,v in grid.items() if k != 'data'},
            'content_sha256': hashlib.sha256(json.dumps(grid,sort_keys=True).encode()).hexdigest(),
            'cells': {'free': len(free), 'occupied': len(occupied), 'unknown': data.count(-1)},
            'free_area_m2': len(free)*resolution**2,
            'lateral_clearance_sampling': {'best': best, 'hard_gate_m': HARD_SIDE_GAP_M,
                'heading_step_deg': heading_step_deg, 'headings_tested': len(headings),
                'free_cells_meeting_hard_gate': meeting, 'free_cells_total': len(free),
                'verdict': 'some_samples_pass' if meeting else 'no_sample_passes',
                'scope': 'Cell centers and sampled headings only; not proof over all poses or permission to execute.'},
            'robot_footprint_on_map': footprint, 'occupied_relative_to_robot': radial,
            'pose_warning': None if pose is not None else 'No fresh map->base_link; robot-relative diagnosis unavailable.',
            'motion_authorized': False}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--json')
    p.add_argument('--snapshot', help='Offline capture_map_observations.py JSON')
    p.add_argument('--timeout', type=float, default=15)
    p.add_argument('--heading-step-deg', type=float, default=5)
    p.add_argument('--strip-noetic', action='store_true')
    args = p.parse_args()
    try:
        if args.snapshot:
            source = json.loads(Path(args.snapshot).read_text())
            grid = next(m for m in reversed(source['maps']) if 'data' in m)
            grid = {k:grid[k] for k in ('width','height','resolution','origin','origin_orientation','frame','data')}
            pose = source['poses'][-1] if source.get('poses') else None
        else:
            if args.strip_noetic:
                _strip_noetic()
            grid, pose = _grab_map(args.timeout)
        report = analyze_grid(grid, pose, heading_step_deg=args.heading_step_deg)
    except (ValueError, KeyError, StopIteration) as e:
        print(str(e))
        return 2
    text = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n'
    print(text, end='')
    if args.json:
        path = Path(args.json)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
