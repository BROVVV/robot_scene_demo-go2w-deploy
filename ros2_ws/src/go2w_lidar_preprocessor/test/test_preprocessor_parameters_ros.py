"""An actual parameter RPC must not crash the sensor node (isolated domain)."""
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor
from go2w_lidar_preprocessor.preprocessor_node import LidarPreprocessor


def test_parameter_rpc_preserves_rclpy_parameter_store(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '79')
    root = Path(__file__).resolve().parents[4]
    rclpy.init(args=['--ros-args', '-p', 'config_file:='+str(root/'configs/go2w/lidar_preprocess.yaml'),
                    '-p', 'geometry_file:='+str(root/'configs/go2w/official_reference.yaml')])
    node = LidarPreprocessor()
    reader = rclpy.create_node('preprocessor_parameter_regression')
    executor = SingleThreadedExecutor();executor.add_node(node);executor.add_node(reader)
    try:
        assert node._ready
        client = reader.create_client(GetParameters, '/go2w_lidar_preprocessor/get_parameters')
        assert client.wait_for_service(timeout_sec=5)
        future = client.call_async(GetParameters.Request(names=['mapping_accumulation','config_file']))
        executor.spin_until_future_complete(future, timeout_sec=5)
        assert future.done() and future.exception() is None
        assert future.result().values[0].bool_value is True
        assert future.result().values[1].string_value.endswith('lidar_preprocess.yaml')
    finally:
        executor.shutdown();reader.destroy_node();node.destroy_node();rclpy.shutdown()
