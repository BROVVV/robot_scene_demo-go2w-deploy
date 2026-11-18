"""Synthetic source and real health/pose reader roles for loopback TF acceptance."""
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

ROOT = Path(__file__).resolve().parents[3]
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
    deadline = time.monotonic() + 180
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
                 ('/go2w/lidar/scan', '/go2w/lidar/mapping_scan', '/go2w/lidar/navigation_scan')]
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 20
    grid.info.resolution = .2
    grid.info.origin.orientation.w = 1.
    grid.data = [100] + [0] * 399
    odom = Odometry()
    odom.header.frame_id, odom.child_frame_id = 'odom_fused', 'base_link'
    odom.pose.pose.position.x = .2
    odom.pose.pose.position.y = 1.5
    odom.pose.pose.orientation.w = 1.
    scan = LaserScan()
    scan.header.frame_id = 'base_link'
    scan.range_min, scan.range_max = .05, 6.2
    scan.ranges = [6.] * 720
    scan.angle_min = -__import__('math').pi
    scan.angle_increment = __import__('math').pi/360
    scan.angle_max = scan.angle_min+719*scan.angle_increment

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
            tf.transform.translation.x = x + (2. if bad and index == 0 else 0.)
            tf.transform.translation.y = 1.5 if index == 1 else 0.
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
        ack = dict(control, published_at=time.monotonic(),
            future_stamp=now.sec + now.nanosec / 1e9 + (60 if bad else 0.))
        write(out / 'source_ack.json', ack)
        first = out / ('first_control_ack_' + str(control.get('seq', 'unknown')) + '.json')
        if not first.exists():write(first, ack)

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
    deadline = time.monotonic() + 180
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


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('role', choices=('source', 'monitor', 'consumer'))
    parser.add_argument('output', type=Path)
    parser.add_argument('--tag', default='run')
    args = parser.parse_args()
    from velocity_chain import isolated_environment
    isolated_environment()
    if args.role == 'source':
        source(args.output, args.tag)
    else:
        observe(args.output, args.tag, args.role == 'monitor')
