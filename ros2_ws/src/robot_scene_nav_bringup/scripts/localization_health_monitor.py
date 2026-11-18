#!/usr/bin/python3
"""Monitor actual 2D localization evidence; optionally record a read-only audit."""
from __future__ import annotations

import argparse
from collections import Counter, deque
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, os.environ.get('GO2W_PROJECT_ROOT', str(Path(__file__).resolve().parents[4])))
from app.navigation.localization_health import (
    StructuralScanFreshnessLatch,
    evaluate_localization,
    map_statistics,
    observe_motion_since,
    sanitize_non_finite,
    fresh,
    tf_fresh,
    TF_FRESHNESS_SECONDS,
    TF_FUTURE_TOLERANCE_SECONDS,
    TF_HEALTH_EDGES,
    stationary_since,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from std_srvs.srv import Empty


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_of(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def finite_pose(pose):
    p, q = pose.position, pose.orientation
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    norm = sum(x*x for x in values[3:])
    return all(math.isfinite(x) for x in values) and .9 < norm < 1.1


class TimestampGuardedBuffer(Buffer):
    """Keep rejected dynamic timestamps out of tf2's latest-pose cache.

    Health evidence still observes the original packets through on_tf. A bad
    future packet must not outlive its error state and contaminate a pose
    after fresh, normal packets resume. Static TF remains timeless.
    """

    def __init__(self, clock):
        super().__init__()
        self._health_clock = clock
        self.rejected_stamps = 0
        self._future_recovery = {}
        self._recovery_lock = threading.RLock()

    @property
    def future_recovery_pending(self):
        with self._recovery_lock:
            return sorted(self._future_recovery)

    @property
    def future_recovery_after_ns(self):
        with self._recovery_lock:
            return {edge: value[0] + int(TF_FRESHNESS_SECONDS * 1e9)
                    for edge, value in self._future_recovery.items()}

    def set_transform(self, transform, authority):
        now_ns = self._health_clock.now().nanoseconds
        now = now_ns / 1e9
        stamp = transform.header.stamp
        stamp_s = seconds(stamp)
        stamp_ns = stamp.sec * 10**9 + stamp.nanosec
        edge = transform.header.frame_id + '->' + transform.child_frame_id
        sample = {'age_s': now - stamp_s, 'receive_age_s': 0.}
        if not tf_fresh(sample):
            with self._recovery_lock:
                self.rejected_stamps += 1
                if sample['age_s'] < -TF_FUTURE_TOLERANCE_SECONDS and edge in TF_HEALTH_EDGES:
                    previous = self._future_recovery.get(edge)
                    if previous is None or stamp_ns > previous[0]:
                        self._future_recovery[edge] = (stamp_ns, stamp_s)
            return
        super().set_transform(transform, authority)
        with self._recovery_lock:
            previous = self._future_recovery.get(edge)
        # Other TF consumers still have ordinary, unfiltered caches. Do not
        # reopen health until the poisoned timestamp is stale by the existing
        # policy AND this edge has an actually buffered newer normal sample.
        if (previous is None or stamp_ns <= previous[0] or now_ns <= previous[0]
                or tf_fresh({'age_s': now-previous[1], 'receive_age_s': 0.})):
            return
        try:
            accepted = self.lookup_transform(transform.header.frame_id,
                                             transform.child_frame_id, rclpy.time.Time())
            accepted_stamp = accepted.header.stamp
            accepted_ns = accepted_stamp.sec * 10**9 + accepted_stamp.nanosec
            if accepted_ns == stamp_ns:
                with self._recovery_lock:
                    # A concurrent rejected packet may have extended it while
                    # tf2 was queried. Do not erase that newer quarantine.
                    if self._future_recovery.get(edge) == previous:
                        del self._future_recovery[edge]
        except Exception:
            # A rejected/absent transform cannot release the recovery gate.
            pass


class LocalizationHealthMonitor(Node):
    def __init__(self, *, publish=True, mode=None):
        super().__init__('go2w_localization_health' if publish else 'go2w_navigation_audit')
        self.declare_parameter('mode', 'mapping')
        self.declare_parameter('map_authorized', False)
        # mode 的**默认路径**是 ROS 参数（launch 传进来）；``mode=None`` 只给
        # 不跑 launch 的调用方（audit_navigation_runtime.py 直接用这个类当采样器）。
        # 必须在构造时定，不能构造完再 set_parameters：self.mode 在 __init__ 里
        # 就读走了，之后再改参数不会回头改它，症状是"审计自称 localization、
        # 判据却全按 mapping 跑"—— 正是这个类要防的那类假绿。
        self.mode = str(mode if mode is not None else self.get_parameter('mode').value)
        self.samples, self.arrivals, self.ages = {}, {}, {}
        self.owners = {}
        self.owners_at = 0.
        self.tf_samples = {}
        self.previous_stamps, self.regressions = {}, Counter()
        self.buffer = TimestampGuardedBuffer(self.get_clock())
        self.listener = TransformListener(
            self.buffer, self,
            qos=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.pose_previous = None
        self.jump_until = 0.
        self.graph = {'publishers': {}, 'nodes': []}
        self.latest = {}
        # 上一次因非有限值被换掉的位置集合；只在它变化时告警（见 _encode_status）。
        self._non_finite_signature = None
        self.history = deque(maxlen=200000)
        self.started = time.monotonic()
        self.valid_ticks = self.total_ticks = 0
        # Only the post-processed structural mapping scan gets a short grace
        # window.  Raw safety-scan loss, TF/odom faults and every other gate
        # still invalidate this tick immediately.
        self._structural_scan_latch = StructuralScanFreshnessLatch(.25)
        self._publish = publish
        # The independent LiDAR watchdog already detects scan loss before the
        # localization monitor's 100 ms polling timer can observe its own
        # 300 ms receive-age threshold.  Mirror only the watchdog's negative
        # edge immediately; a positive edge never grants localization by
        # itself and must still pass the full map/TF/odom/scan evaluation in
        # tick().  This removes one timer period from fail-closed propagation
        # without relaxing any freshness threshold.
        self._lidar_fresh_gate = None
        self._amcl_refresh_client = (self.create_client(Empty, '/request_nomotion_update')
                                     if self.mode == 'localization' else None)
        self._amcl_refresh_future = None
        self._amcl_refresh_at = -math.inf
        self._amcl_refresh_requests = 0
        self.valid_pub = self.create_publisher(Bool, '/go2w/nav/localization_valid', 10) if publish else None
        self.status_pub = self.create_publisher(String, '/go2w/nav/localization_status', 10) if publish else None
        sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.topics = {'map': '/map', 'scan': '/go2w/lidar/scan', 'odom': '/go2w/odom/fused', 'mapping_scan': '/go2w/lidar/mapping_scan'}
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(LaserScan, self.topics['scan'], self.on_scan, sensor)
        self.create_subscription(Odometry, self.topics['odom'], self.on_odom, sensor)
        self.create_subscription(LaserScan, self.topics['mapping_scan'], lambda m: self.on_scan(m, 'mapping_scan'), sensor)
        # AMCL publishes reliable/transient-local and can publish only once
        # while stationary. A volatile late subscriber misses that first pose.
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, latched)
        self.create_subscription(TFMessage, '/tf', self.on_tf, sensor)
        self.create_subscription(String, '/go2w/nav/tf_authorities', self.on_authorities, 10)
        self.create_subscription(Bool, '/go2w/safety/lidar_fresh', self.on_lidar_fresh, 10)
        self.create_timer(.5, self.update_graph)
        self.create_timer(.1, self.tick)

    def on_lidar_fresh(self, message):
        """Fail localization closed immediately on the watchdog's false edge.

        Recovery remains deliberately asymmetric: ``True`` is cached but not
        published here, because only ``tick`` can prove the complete
        localization invariant.
        """
        self._lidar_fresh_gate = bool(message.data)
        if self._publish and not self._lidar_fresh_gate:
            self.valid_pub.publish(Bool(data=False))

    def note(self, key, message, details):
        mono, stamp = time.monotonic(), seconds(message.header.stamp)
        now = self.get_clock().now().nanoseconds / 1e9
        if key in self.previous_stamps and stamp < self.previous_stamps[key]:
            self.regressions[key] += 1
        self.previous_stamps[key] = stamp
        self.samples[key] = {'stamp': stamp, 'received': mono, 'frame': message.header.frame_id, **details}
        self.arrivals.setdefault(key, deque(maxlen=200000)).append(mono)
        self.ages.setdefault(key, deque(maxlen=200000)).append(now - stamp)

    def on_map(self, m):
        self.note('map', m, map_statistics(m.info.width, m.info.height, m.info.resolution, m.data))

    def on_scan(self, m, key="scan"):
        self.note(key, m, {'valid_ranges': sum(math.isfinite(v) and m.range_min <= v <= m.range_max for v in m.ranges),
                            'bins': len(m.ranges)})

    def on_odom(self, m):
        previous = self.samples.get('odom')
        pose = m.pose.pose
        self.note('odom', m, {'child': m.child_frame_id, 'finite': finite_pose(pose),
                             'x': pose.position.x, 'y': pose.position.y,
                             'yaw': yaw_of(pose.orientation)})
        if 'amcl' in self.samples:
            current = self.samples['odom']
            continuous = bool(previous and current['received']-previous['received'] <= .2
                              and 0 <= current['stamp']-previous['stamp'] <= .2)
            self.samples['amcl'] = observe_motion_since(self.samples['amcl'], current,
                                                        continuous=continuous)

    def on_amcl(self, m):
        # 记下**这一帧到达时**的 odom 位姿。amcl 静止时不再重发 /amcl_pose
        # （见 localization_health.stationary_since 里的源码依据），"这个位姿还算不算
        # 数"只能拿"此后车有没有动过"来判，而"此后"要有起点，就是这里。
        odom = self.samples.get('odom') or {}
        arrived = [odom.get(k) for k in ('x', 'y', 'yaw')]
        now = self.get_clock().now().nanoseconds/1e9
        complete = bool(odom and time.monotonic()-odom['received'] <= .2
                        and -.1 <= now-odom['stamp'] <= .2
                        # A delayed AMCL packet cannot start history at NOW:
                        # the robot may have moved since its measurement time.
                        and -.1 <= now-seconds(m.header.stamp) <= 1.)
        self.note('amcl', m, {'covariance': [m.pose.covariance[i] for i in (0, 7, 35)],
                              'odom_at_arrival': arrived,
                              'motion_observation_complete': complete,
                              'max_position_excursion_m': 0., 'max_yaw_excursion_rad': 0.})

    def on_authorities(self, msg):
        self.owners = json.loads(msg.data)
        self.owners_at = time.monotonic()

    def on_tf(self, msg):
        mono = time.monotonic()
        for t in msg.transforms:
            edge = t.header.frame_id + '->' + t.child_frame_id
            if edge not in ('map->odom_fused', 'odom_fused->base_link'):
                continue
            p, q = t.transform.translation, t.transform.rotation
            values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            self.tf_samples[edge] = {'stamp': seconds(t.header.stamp), 'received': mono,
                'finite': all(math.isfinite(x) for x in values) and .9 < sum(x*x for x in values[3:]) < 1.1}

    def update_graph(self):
        self.graph = {'publishers': {key: self.count_publishers(topic) for key, topic in self.topics.items()},
                      'nodes': [('/' + ns.strip('/') + '/' + name).replace('//', '/')
                                for name, ns in self.get_node_names_and_namespaces()]}

    def snapshot(self):
        mono, now = time.monotonic(), self.get_clock().now().nanoseconds / 1e9
        def current(sample):
            return {**{k: v for k, v in sample.items() if k not in ('stamp', 'received')},
                    'age_s': now - sample['stamp'],
                    'receive_age_s': mono - sample['received']}
        result = {k: {**current(v), 'publishers': self.graph['publishers'].get(k, 0),
                      'stamp_regressions': self.regressions[k]} for k, v in self.samples.items()}
        result['tf'] = {edge: {**current(v), 'authorities': len(self.owners.get(edge, [])) if mono-self.owners_at < .2 else 0,
                               'publisher_gids': self.owners.get(edge, [])}
                        for edge, v in self.tf_samples.items()}
        nodes = self.graph['nodes']
        result['localizer_active'] = nodes.count('/slam_toolbox' if self.mode == 'mapping' else '/amcl') == 1
        required = {'/slam_toolbox', '/amcl', '/planner_server', '/controller_server', '/go2w_wheel_odom',
                    '/bt_navigator', '/go2w_localization_health'}
        result['duplicate_nodes'] = [n for n, count in Counter(nodes).items() if count > 1 and n in required]
        result['map_authorized'] = bool(self.get_parameter('map_authorized').value)
        try:
            t = self.buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            p, q = t.transform.translation, t.transform.rotation
            yaw = yaw_of(q)
            pose = (p.x, p.y, yaw, mono)
            if self.pose_previous:
                old = self.pose_previous
                dyaw = abs(math.atan2(math.sin(yaw-old[2]), math.cos(yaw-old[2])))
                if mono-old[3] < .5 and (math.hypot(p.x-old[0], p.y-old[1]) > .4 or dyaw > .6):
                    self.jump_until = mono + 2
            self.pose_previous = pose
            result['pose'] = {'x': p.x, 'y': p.y, 'yaw': yaw, 'frame': 'map'}
        except Exception:
            pass
        result['pose_jump'] = mono < self.jump_until
        result['tf_future_recovery_pending'] = self.buffer.future_recovery_pending
        result['tf_future_recovery_after_ns'] = self.buffer.future_recovery_after_ns
        result['nodes'] = nodes
        return result

    def tick(self):
        snapshot = self.snapshot()
        self._maybe_refresh_amcl(snapshot)
        health = evaluate_localization(snapshot, mode=self.mode)
        health.setdefault('checks', {})['lidar_fresh_gate'] = (
            self._lidar_fresh_gate is True
        )
        if self._lidar_fresh_gate is not True:
            if 'lidar_fresh_gate' not in health.setdefault('blocking', []):
                health['blocking'].append('lidar_fresh_gate')
            health['valid'] = False
        raw_structural_fresh = bool(
            health.get('checks', {}).get('mapping_scan_fresh', False)
        )
        effective_structural_fresh, structural_grace = (
            self._structural_scan_latch.update(
                structural_fresh=raw_structural_fresh,
                safety_scan_fresh=bool(
                    health.get('checks', {}).get('scan_fresh', False)
                ),
                now=time.monotonic(),
            )
        )
        if effective_structural_fresh and not raw_structural_fresh:
            health['checks']['mapping_scan_fresh'] = True
            health['blocking'] = [
                item for item in health.get('blocking', [])
                if item != 'mapping_scan_fresh'
            ]
            health['valid'] = not health['blocking']
        health['structural_scan_grace_active'] = structural_grace
        health['raw_mapping_scan_fresh'] = raw_structural_fresh
        latest = {'wall_time': time.time(), 'uptime_s': time.monotonic()-self.started,
                  **health, 'evidence': snapshot,
                  'tf_buffer_rejected_stamps': self.buffer.rejected_stamps,
                  'amcl_nomotion_refresh_requests': self._amcl_refresh_requests}
        latest, wire = self._encode_status(latest)
        self.latest = latest
        self.total_ticks += 1
        self.valid_ticks += bool(latest['valid'])
        self.history.append({'t': latest['uptime_s'], 'valid': latest['valid'],
                             'blocking': latest['blocking']})
        if self._publish:
            # 两个出口用**同一个**结论。后端把这两路是 AND 起来用的
            # （localization_valid_provider 同时要 Bool 和 status 的 valid），
            # 让它们不一致只会让现场更难判。
            self.valid_pub.publish(Bool(data=bool(latest['valid'])))
            self.status_pub.publish(String(data=wire))

    def _maybe_refresh_amcl(self, snapshot):
        """Establish actual fresh evidence for startup/late audit subscribers.

        A cached pose without preceding wheel history cannot prove stationary
        operation. Ask AMCL to process another real scan; never invent history,
        set an initial pose, authorize a map, arm, or command physical motion.
        The non-publishing audit sampler can make the same estimator request.
        """
        client = self._amcl_refresh_client
        if client is None:
            return
        amcl,odom = snapshot.get('amcl',{}),snapshot.get('odom',{})
        if (amcl.get('motion_observation_complete') is True
                and (fresh(amcl,.75) or stationary_since(amcl,odom))):
            return
        if (not fresh(odom,.2) or odom.get('finite') is not True
                or not fresh(snapshot.get('mapping_scan',{}),.2,stamp_limit=.35)
                or snapshot.get('map',{}).get('valid') is not True
                or snapshot.get('localizer_active') is not True):
            return
        now = time.monotonic()
        if now-self._amcl_refresh_at < 1.:
            return
        future = self._amcl_refresh_future
        if future is not None and not future.done():
            # Do not stack pending requests behind a failed estimator.
            return
        if not client.service_is_ready():
            return
        try:
            self._amcl_refresh_future = client.call_async(Empty.Request())
            self._amcl_refresh_at = now
            self._amcl_refresh_requests += 1
        except Exception as error:
            self._amcl_refresh_at = now
            self.get_logger().warn('AMCL no-motion evidence refresh failed: '+str(error))

    def _encode_status(self, latest: dict) -> tuple[dict, str]:
        """把这一拍的状态编成**严格 JSON**；编不出来也不能把监视器打死（§P3-6）。

        快照里带的是原始诊断数值（地图分辨率、odom 的 x/y、amcl 协方差、
        map->base_link 的 pose…），任何一个 NaN/inf 都会让
        ``json.dumps(..., allow_nan=False)`` 抛 ValueError。而这里是**定时器回调**：
        抛出去等于把监视器打死，它却是 ``/go2w/nav/localization_valid`` 的发布者。
        之后后端看到的是"没人发布"，归因指向定位失效，真相却是"监视器自己崩了"。

        所以：换掉非有限值（``allow_nan=True`` 会写出非法 JSON，不能糊），把换掉的
        位置留在 payload 里当证据，并且**把这一拍判为无效** —— 数据都没读干净，
        不能声称定位有效（fail-closed）。
        """
        try:
            return latest, json.dumps(latest, allow_nan=False)
        except ValueError:
            pass
        cleaned, offenders = sanitize_non_finite(latest)
        cleaned['non_finite_diagnostics'] = offenders
        # 只在这里改 valid/blocking：不改 evaluate_localization 的判据，那条
        # 判据本身已经对分辨率/odom/TF/协方差各自判过一次；这里补的是它没覆盖的
        # 那些原始数值（典型是 result['pose'] 的 x/y/yaw，进入判据前没有有限性检查）。
        cleaned['valid'] = False
        blocking = list(cleaned.get('blocking') or [])
        if 'non_finite_diagnostics' not in blocking:
            blocking.append('non_finite_diagnostics')
        cleaned['blocking'] = blocking
        signature = tuple(offenders)
        if signature != getattr(self, '_non_finite_signature', None):
            # 只在**换掉的位置变了**的时候出声。这些数值每拍都在，逐拍一条会
            # 把日志淹掉，等于没有信号。
            self._non_finite_signature = signature
            self.get_logger().warn(
                '诊断数值里有非有限值，已按 None 编码并把这一拍判为无效：'
                + ', '.join(offenders[:8]) + ('…' if len(offenders) > 8 else ''))
        return cleaned, json.dumps(cleaned, allow_nan=False)

    def report(self):
        metrics = {}
        for key, arrivals in self.arrivals.items():
            a = sorted(self.ages[key])
            metrics[key] = {'samples': len(arrivals), 'hz': (len(arrivals)-1)/(arrivals[-1]-arrivals[0]) if len(arrivals)>1 else 0,
                            'age_p95_s': a[min(len(a)-1, int(len(a)*.95))], 'age_max_s': max(a),
                            'stamp_regressions': self.regressions[key]}
        return {**self.latest, 'metrics': metrics, 'valid_ticks': self.valid_ticks,
                'total_ticks': self.total_ticks, 'history': list(self.history)}


def main():
    args = argparse.ArgumentParser()
    args.add_argument('--duration', type=float, default=0)
    args.add_argument('--output')
    args.add_argument('--no-publish', action='store_true')
    options = args.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = LocalizationHealthMonitor(publish=not options.no_publish)
    try:
        while rclpy.ok() and (not options.duration or time.monotonic()-node.started < options.duration):
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        report = node.report()
        if options.output:
            target = Path(options.output)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if options.duration:
        print(json.dumps({k: report.get(k) for k in ('valid', 'blocking', 'metrics', 'valid_ticks', 'total_ticks')}))
        return 0 if report.get('valid') else 1
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
