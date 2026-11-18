"""The streaming heartbeat must cross the network/local ROS graph boundary."""

import json
from pathlib import Path
import queue
import sys
import threading

import pytest

pytest.importorskip("rclpy", reason="requires ROS Python")

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts/go2w"))
import motion_network_proxy as network_module
from motion_local_gateway import MotionLocalGateway
from motion_network_proxy import MotionNetworkProxy
from std_msgs.msg import String


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Request:
    def __init__(self, payload):
        self.payload = payload
        self.response = None

    def finish(self, response):
        self.response = response


def update(token="a" * 32, vx=.30):
    return json.dumps({
        "version": 1, "token": token, "stamp": 100.0,
        "vx": vx, "vy": 0.0, "yaw_rate": 0.0,
    })


def test_local_gateway_republishes_heartbeat_verbatim():
    gateway = MotionLocalGateway.__new__(MotionLocalGateway)
    gateway._velocity_update_pub = Publisher()
    request = Request({"data": update()})
    gateway._publish_velocity_update(request)
    assert request.response == {"ok": True}
    assert [message.data for message in gateway._velocity_update_pub.messages] == [update()]


def test_network_proxy_relays_valid_heartbeat_and_drops_malformed(monkeypatch):
    proxy = MotionNetworkProxy.__new__(MotionNetworkProxy)
    proxy._socket_path = Path("/tmp/test-motion-gateway.sock")
    proxy._velocity_relay_queue = queue.Queue(maxsize=1)
    proxy._velocity_relay_stop = threading.Event()
    proxy._relay_velocity_update(String(data=update()))
    assert proxy._velocity_relay_queue.get_nowait() == update()
    proxy._relay_velocity_update(String(data=update(token="short")))
    proxy._relay_velocity_update(String(data="{broken"))
    assert proxy._velocity_relay_queue.empty()


def test_network_proxy_worker_forwards_without_using_ros_callback_thread(monkeypatch):
    calls = []
    monkeypatch.setattr(network_module, "gateway_request",
        lambda socket, payload, timeout: calls.append((socket, payload, timeout)) or {"ok": True})
    proxy = MotionNetworkProxy.__new__(MotionNetworkProxy)
    proxy._socket_path = Path("/tmp/test-motion-gateway.sock")
    proxy._velocity_relay_queue = queue.Queue(maxsize=1)
    proxy._velocity_relay_stop = threading.Event()
    proxy._velocity_relay_queue.put(update())
    thread = threading.Thread(target=proxy._velocity_relay_loop)
    proxy._velocity_relay_thread = thread
    thread.start()
    for _ in range(100):
        if calls:
            break
        threading.Event().wait(.001)
    proxy.close()
    assert calls == [(proxy._socket_path,
        {"op": "velocity_update", "data": update()}, .20)]
