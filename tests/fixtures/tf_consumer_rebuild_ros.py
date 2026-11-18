"""PC-only DDS fixture: restart real monitor and search pose reader processes."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def write(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, allow_nan=False))
    temporary.replace(path)


def read(path):
    return json.loads(path.read_text()) if path.exists() else {}


def identity():
    fields = Path('/proc/self/stat').read_text().rsplit(')', 1)[1].split()
    return dict(pid=os.getpid(), start_ticks=int(fields[19]),
        boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())


def stop_event():
    # Keep asynchronous KeyboardInterrupt out of rclpy's C message conversion;
    # stop the fixture executor loop first, then clean up live ROS resources.
    stopped = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    return stopped


def source(out, tag):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from app.ros_compat import init_preserving_python_signals
    init_preserving_python_signals()
    stopped = stop_event()
    deadline = time.monotonic() + 45
    node = rclpy.create_node('pc_rebuild_fixture_source')
    localizer = rclpy.create_node('slam_toolbox')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(localizer)
    tf_pubs = [localizer.create_publisher(TFMessage, '/tf', 100),
               node.create_publisher(TFMessage, '/tf', 100)]
    owners_pub = node.create_publisher(String, '/go2w/nav/tf_authorities', 10)
    map_pub = node.create_publisher(OccupancyGrid, '/map', QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    odom_pub = node.create_publisher(Odometry, '/go2w/odom/fused', 10)
    scan_pubs = [node.create_publisher(LaserScan, topic, 10) for topic in
                 ('/go2w/lidar/scan', '/go2w/lidar/mapping_scan')]
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 20
    grid.info.resolution = .2
    grid.info.origin.orientation.w = 1.
    grid.data = [100] + [0] * 399
    odom = Odometry()
    odom.header.frame_id, odom.child_frame_id = 'odom_fused', 'base_link'
    odom.pose.pose.position.x = .2
    odom.pose.pose.orientation.w = 1.
    scan = LaserScan()
    scan.header.frame_id = 'base_link'
    scan.range_min, scan.range_max = .05, 10.
    scan.ranges = [2.] * 32
    scan.angle_increment = .1
    scan.angle_max = 3.1

    def publish():
        control = read(out / 'control.json')
        bad = control.get('fault') == 'future'
        now = node.get_clock().now().to_msg()
        for index, (parent, child, x) in enumerate([
                ('map', 'odom_fused', 1.), ('odom_fused', 'base_link', .2)]):
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = parent, child
            tf.header.stamp.sec = now.sec + (60 if bad else 0)
            tf.header.stamp.nanosec = now.nanosec
            tf.transform.translation.x = x + (.1 if bad and index == 0 else 0.)
            tf.transform.rotation.w = 1.
            tf_pubs[index].publish(TFMessage(transforms=[tf]))
        grid.header.stamp = odom.header.stamp = scan.header.stamp = now
        map_pub.publish(grid)
        odom_pub.publish(odom)
        for pub in scan_pubs:
            pub.publish(scan)
        info = node.get_publishers_info_by_topic('/tf')
        names = ('slam_toolbox', 'pc_rebuild_fixture_source')
        edges = ('map->odom_fused', 'odom_fused->base_link')
        owners_pub.publish(String(data=json.dumps({edge: [list(p.endpoint_gid)
            for p in info if p.node_name == name] for edge, name in zip(edges, names)})))
        write(out / 'source_ack.json', dict(control, published_at=time.monotonic(),
            future_stamp=now.sec + now.nanosec / 1e9 + (60 if bad else 0.)))

    node.create_timer(.05, publish)
    write(out / (tag + '_identity.json'), identity())
    try:
        while rclpy.ok() and not stopped.is_set() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        localizer.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def observe(out, tag, is_monitor):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from app.ros_compat import init_preserving_python_signals
    init_preserving_python_signals()
    stopped = stop_event()
    deadline = time.monotonic() + 45
    executor = SingleThreadedExecutor()
    listener = backend = None
    actions, twists, statuses = [], [], {}
    cleanup_errors = []
    if is_monitor:
        path = ROOT / 'ros2_ws/src/robot_scene_nav_bringup/scripts/localization_health_monitor.py'
        spec = importlib.util.spec_from_file_location('rebuild_monitor', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        node = module.LocalizationHealthMonitor(publish=True, mode='mapping')
        listener = node.listener
    else:
        from geometry_msgs.msg import Twist
        from std_msgs.msg import Bool, String
        from tf2_ros import Buffer, TransformListener
        from rclpy.action import ActionClient
        from rclpy.time import Time
        from app.navigation.nav2_search_backend import create_nav2_search_backend
        node = rclpy.create_node('pc_rebuild_pose_consumer')
        buffer = Buffer()
        listener = TransformListener(buffer, node)

        def refuse_action(*args, **kwargs):
            actions.append('unexpected_send')
            raise AssertionError('fixture only reads pose; actions forbidden')
        ActionClient.send_goal_async = refuse_action
        # Real ActionClient entities are created and destroyed; no server or
        # mock action factory is used, and send_goal_async is guarded above.
        backend = create_nav2_search_backend(node, dry_run=True, tf_buffer=buffer,
            tf_lookup_timeout_s=0., stop_topic='/pc_rebuild/no_motion')
        def status(msg):
            statuses.update(status=json.loads(msg.data), status_received=time.monotonic())
        def boolean(msg):
            statuses.update(boolean=msg.data, boolean_received=time.monotonic())
        node.create_subscription(String, '/go2w/nav/localization_status', status, 10)
        node.create_subscription(Bool, '/go2w/nav/localization_valid', boolean, 10)
        node.create_subscription(Twist, '/pc_rebuild/no_motion', lambda msg: twists.append(1), 10)
        trace = (out / (tag + '_trace.jsonl')).open('x')

        def sample():
            pose = backend.get_pose()
            raw = None
            try:
                transform = buffer.lookup_transform('map', 'base_link', Time())
                stamp = transform.header.stamp
                raw = dict(x=transform.transform.translation.x,
                    age_s=node.get_clock().now().nanoseconds / 1e9 - stamp.sec - stamp.nanosec / 1e9)
            except Exception:
                pass
            row = dict(statuses, received=time.monotonic(), raw_pose=raw,
                pose=None if pose is None else dict(x=pose.x, y=pose.y, yaw=pose.yaw),
                action_sends=len(actions), twist_messages=len(twists))
            write(out / (tag + '_state.json'), row)
            trace.write(json.dumps(row) + '\n')
            trace.flush()
        node.create_timer(.05, sample)
    executor.add_node(node)
    write(out / (tag + '_identity.json'), identity())
    try:
        while rclpy.ok() and not stopped.is_set() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
    except KeyboardInterrupt:
        pass
    finally:
        if listener is not None:
            listener.unregister()
            listener.unregister = lambda: None
        if backend is not None:
            from scripts.go2w.run_semantic_exploration import _teardown_nav2_io
            cleanup_errors = _teardown_nav2_io(SimpleNamespace(
                _nav2_io_executor=executor, _nav2_io_node=node,
                _search_nav2_backend=SimpleNamespace(_inner=backend)))
            trace.close()
        else:
            executor.shutdown()
            node.destroy_node()
        write(out / (tag + '_cleanup.json'), dict(errors=cleanup_errors,
            action_sends=len(actions), twist_messages=len(twists), real_action_clients=backend is not None))
        rclpy.shutdown()


def run(out):
    assert os.environ.get('ROS_DOMAIN_ID') == '98'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('RMW_IMPLEMENTATION') == 'rmw_cyclonedds_cpp'
    out.mkdir(exist_ok=False)
    children, phases = {}, []
    report = dict(pc_only=True, robot_contacted=False, motion_sent=False,
        real_action_clients=True, native_nav2_servers_tested=False, phases=phases)

    def start(role, tag):
        with (out / (tag + '.log')).open('x') as log:
            child = subprocess.Popen([sys.executable, __file__, role, str(out), '--tag', tag],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children[tag] = child
        return child

    def stop(tag):
        child = children[tag]
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            child.wait(timeout=5)
        assert child.returncode == 0, (tag, child.returncode)

    def phase(tag, predicate, count=3, timeout=7):
        after, seen, rows = time.monotonic(), set(), []
        deadline = after + timeout
        while time.monotonic() < deadline:
            assert all(p.poll() is None or p.returncode == 0 for p in children.values())
            assert children[tag].poll() is None, tag + ' exited'
            row = read(out / (tag + '_state.json'))
            stamp = row.get('status_received', 0.)
            if (stamp > after and row.get('boolean_received', 0.) > after
                    and row.get('received', 0.) - stamp < .3
                    and row.get('received', 0.) - row['boolean_received'] < .3 and stamp not in seen
                    and predicate(row)):
                seen.add(stamp)
                rows.append(row)
                if len(rows) >= count:
                    phases.append(dict(consumer=tag, samples=rows))
                    return rows
            time.sleep(.01)
        raise TimeoutError(json.dumps(read(out / (tag + '_state.json'))))

    def healthy(row):
        return row.get('boolean') is True and row.get('status', {}).get('valid') is True

    def correct(row):
        return healthy(row) and row.get('pose') is not None and abs(row['pose']['x'] - 1.2) < 1e-6

    def closed(row):
        # Do not accept ordinary startup (missing map/graph/sensor evidence)
        # as proof that the injected TF fault keeps this chain closed.
        blockers = set(row.get('status', {}).get('blocking', []))
        tf_blockers = {'tf_future_cache_recovered', 'map->odom_fused:fresh',
                       'odom_fused->base_link:fresh'}
        return (row.get('boolean') is False and row.get('status', {}).get('valid') is False
            and 'tf_future_cache_recovered' in blockers and blockers <= tf_blockers)

    def poisoned(row):
        return row.get('raw_pose') is not None and abs(row['raw_pose']['x'] - 1.3) < 1e-6

    try:
        write(out / 'control.json', dict(seq=0, fault='normal'))
        start('source', 'source')
        start('monitor', 'monitor_01')
        start('consumer', 'consumer_01')
        phase('consumer_01', correct)
        original_consumer = read(out / 'consumer_01_identity.json')
        write(out / 'control.json', dict(seq=1, fault='future'))
        phase('consumer_01', lambda row: closed(row) and poisoned(row))
        poison_stamp = read(out / 'source_ack.json')['future_stamp']
        write(out / 'control.json', dict(seq=2, fault='normal'))
        phase('consumer_01', lambda row: closed(row) and poisoned(row)
            and row['status']['blocking'] == ['tf_future_cache_recovered'])
        stop('monitor_01')
        start('monitor', 'monitor_02')
        phase('consumer_01', lambda row: healthy(row) and poisoned(row) and row['pose'] is None)
        assert original_consumer == read(out / 'consumer_01_identity.json')
        report['isolated_monitor_restart_incomplete_recovery_reproduced'] = True
        stop('consumer_01')
        stop('monitor_02')
        start('monitor', 'monitor_03')
        start('consumer', 'consumer_02')
        phase('consumer_02', correct)
        assert time.time() < poison_stamp - 10., 'recovery must be rebuild, not waiting for poison expiry'
        report['two_consumer_rebuild_correct_before_poison_expiry'] = True
        # A still-broken producer must not be made healthy by restarting both.
        write(out / 'control.json', dict(seq=3, fault='future'))
        phase('consumer_02', lambda row: closed(row) and poisoned(row))
        stop('consumer_02')
        stop('monitor_03')
        start('monitor', 'monitor_04')
        start('consumer', 'consumer_03')
        phase('consumer_03', lambda row: closed(row) and poisoned(row) and row['pose'] is None)
        report['rebuild_with_bad_source_stays_closed'] = True
        report['passed'] = True
    except Exception as error:
        report.update(passed=False, error=repr(error))
        raise
    finally:
        write(out / 'control.json', dict(seq=4, fault='normal'))
        for tag in reversed(list(children)):
            child = children[tag]
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    child.terminate()
                    try:
                        child.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait(timeout=2)
        report['child_exit_codes'] = {tag: p.returncode for tag, p in children.items()}
        report['children_terminal'] = all(p.poll() is not None for p in children.values())
        report['cleanup'] = {tag: read(out / (tag + '_cleanup.json')) for tag in children if tag != 'source'}
        report['cleanup_passed'] = (all(code == 0 for code in report['child_exit_codes'].values())
            and all(row and not row.get('errors') and row.get('action_sends') == 0
                and row.get('twist_messages') == 0 for row in report['cleanup'].values()))
        report['passed'] = report.get('passed', False) and report['cleanup_passed']
        write(out / 'report.json', report)
    assert all(code == 0 for code in report['child_exit_codes'].values())
    assert all(not row.get('errors') and row.get('action_sends') == 0
        and row.get('twist_messages') == 0 for row in report['cleanup'].values())


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('role', choices=('source', 'monitor', 'consumer', 'run'))
    parser.add_argument('output', type=Path)
    parser.add_argument('--tag', default='run')
    args = parser.parse_args()
    if args.role == 'run':
        run(args.output)
    elif args.role == 'source':
        source(args.output, args.tag)
    else:
        observe(args.output, args.tag, args.role == 'monitor')
