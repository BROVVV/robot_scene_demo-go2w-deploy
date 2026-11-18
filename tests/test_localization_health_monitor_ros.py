"""AMCL arrival must establish history only from a fresh measurement/odometry."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import time

import pytest

pytest.importorskip('rclpy', reason='requires sourced ROS Python')

path = Path(__file__).resolve().parents[1] / 'ros2_ws/src/robot_scene_nav_bringup/scripts/localization_health_monitor.py'
spec = importlib.util.spec_from_file_location('localization_monitor_arrival_test', path)
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)


def test_lidar_watchdog_false_edge_immediately_fails_closed_but_true_never_grants():
    published = []
    fake = SimpleNamespace(
        _publish=True,
        _lidar_fresh_gate=None,
        valid_pub=SimpleNamespace(publish=lambda message: published.append(message.data)),
    )
    monitor.LocalizationHealthMonitor.on_lidar_fresh(
        fake, SimpleNamespace(data=False)
    )
    assert fake._lidar_fresh_gate is False
    assert published == [False]

    monitor.LocalizationHealthMonitor.on_lidar_fresh(
        fake, SimpleNamespace(data=True)
    )
    assert fake._lidar_fresh_gate is True
    assert published == [False]


@pytest.mark.parametrize('amcl_age,odom_age,receive_age,expected', [
    (.05, .05, .01, True), (9., .05, .01, False), (-1., .05, .01, False),
    (.05, 1., .01, False), (.05, .05, 1., False),
])
def test_amcl_history_is_not_fabricated_at_arrival(amcl_age, odom_age, receive_age, expected):
    captured = {}
    fake = SimpleNamespace(
        samples={'odom': {'x': 1., 'y': 2., 'yaw': .1, 'stamp': 100.-odom_age,
                          'received': time.monotonic()-receive_age}},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)),
        note=lambda key, msg, details: captured.update(details))
    stamp_ns = round((100.-amcl_age)*1e9)
    message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp_ns//10**9, nanosec=stamp_ns%10**9)),
                              pose=SimpleNamespace(covariance=[.01]*36))
    monitor.LocalizationHealthMonitor.on_amcl(fake, message)
    assert captured['motion_observation_complete'] is expected


def test_late_stationary_observer_asks_for_measurement_without_fabricating_history():
    from copy import deepcopy
    requests=[]
    future=SimpleNamespace(done=lambda:True)
    node=SimpleNamespace(_amcl_refresh_client=SimpleNamespace(
        service_is_ready=lambda:True,call_async=lambda req:requests.append(req) or future),
        _amcl_refresh_future=None,_amcl_refresh_at=-float('inf'),_amcl_refresh_requests=0)
    snapshot=dict(map={'valid':True},mapping_scan={'age_s':.05,'receive_age_s':.01},
        odom={'age_s':.05,'receive_age_s':.01,'finite':True},localizer_active=True,
        amcl={'age_s':99.,'receive_age_s':.01,'motion_observation_complete':False})
    original=deepcopy(snapshot)
    monitor.LocalizationHealthMonitor._maybe_refresh_amcl(node,snapshot)
    assert len(requests)==1 and node._amcl_refresh_requests==1
    assert snapshot==original  # No made-up pose, covariance, or history.
    monitor.LocalizationHealthMonitor._maybe_refresh_amcl(node,snapshot)
    assert len(requests)==1  # Rate limited, even if the first reply completed.


@pytest.mark.parametrize('problem',['odom_stale','scan_stale','map_invalid','inactive','pending','no_service'])
def test_nomotion_refresh_requires_live_inputs_and_never_stacks_requests(problem):
    calls=[]
    snapshot=dict(map={'valid':True},mapping_scan={'age_s':.01,'receive_age_s':.01},
        odom={'age_s':.01,'receive_age_s':.01,'finite':True},localizer_active=True)
    node=SimpleNamespace(_amcl_refresh_client=SimpleNamespace(
        service_is_ready=lambda:problem!='no_service',call_async=lambda req:calls.append(req)),
        _amcl_refresh_future=SimpleNamespace(done=lambda:False) if problem=='pending' else None,
        _amcl_refresh_at=-float('inf'),_amcl_refresh_requests=0)
    if problem=='odom_stale':snapshot['odom']['age_s']=1.
    elif problem=='scan_stale':snapshot['mapping_scan']['age_s']=1.
    elif problem=='map_invalid':snapshot['map']['valid']=False
    elif problem=='inactive':snapshot['localizer_active']=False
    monitor.LocalizationHealthMonitor._maybe_refresh_amcl(node,snapshot)
    assert calls==[]


def test_late_subscriber_receives_cached_pose_without_inventing_stationary_history():
    import os
    import subprocess
    import sys
    code=r'''
import importlib.util,sys,time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rclpy.executors import SingleThreadedExecutor
spec=importlib.util.spec_from_file_location('late_monitor',sys.argv[1])
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
rclpy.init(args=[])
publisher_node=rclpy.create_node('cached_amcl_test_source')
publisher=publisher_node.create_publisher(PoseWithCovarianceStamped,'/amcl_pose',QoSProfile(
    depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
pose=PoseWithCovarianceStamped();pose.header.frame_id='map';pose.header.stamp=publisher_node.get_clock().now().to_msg()
pose.pose.pose.orientation.w=1.;pose.pose.covariance[0]=pose.pose.covariance[7]=pose.pose.covariance[35]=.01
publisher.publish(pose)
rclpy.spin_once(publisher_node,timeout_sec=.05)
observer=module.LocalizationHealthMonitor(publish=False,mode='localization')
executor=SingleThreadedExecutor();executor.add_node(publisher_node);executor.add_node(observer)
try:
    deadline=time.monotonic()+1.5
    while 'amcl' not in observer.samples and time.monotonic()<deadline:executor.spin_once(timeout_sec=.01)
    assert 'amcl' in observer.samples,'late subscription must receive the actual cached measurement'
    assert observer.samples['amcl']['motion_observation_complete'] is False
    assert observer._amcl_refresh_requests==0  # No fresh wheel/scan inputs exist.
finally:
    executor.shutdown();observer.destroy_node();publisher_node.destroy_node();rclpy.shutdown()
'''
    environment=dict(os.environ,ROS_DOMAIN_ID='98',ROS_LOCALHOST_ONLY='1')
    result=subprocess.run([sys.executable,'-c',code,str(path)],env=environment,
        text=True,capture_output=True,timeout=6)
    assert result.returncode==0,result.stdout+result.stderr
