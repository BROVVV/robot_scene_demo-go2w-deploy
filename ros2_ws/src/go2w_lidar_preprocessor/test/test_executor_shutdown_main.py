"""The actual node main must drain queued work before invalidating resources."""
from concurrent.futures import ThreadPoolExecutor
from threading import Event,Timer
from types import SimpleNamespace

import pytest
pytest.importorskip('rclpy')
from go2w_lidar_preprocessor import preprocessor_node as module


def test_main_closes_executor_after_queued_work_and_before_node(monkeypatch):
    release=Event();occupied=Event();stopped=Event();stopped.set();calls=[]
    pool=ThreadPoolExecutor(max_workers=1)
    def occupy():occupied.set();assert release.wait(timeout=2)
    blocker=pool.submit(occupy);assert occupied.wait(timeout=1)
    future=pool.submit(lambda:calls.append('queued_callback'))
    class Executor:
        _executor=pool
        def add_node(self,node):calls.append('add_node')
        def spin_once(self,**kwargs):pytest.fail('pre-stopped loop must not spin')
        def shutdown(self):
            assert future.done(),'executor destroyed its guard before pending callback'
            calls.append('executor_shutdown')
    def destroy():
        assert future.done();calls.append('node_destroy')
    monkeypatch.setattr(module,'LidarPreprocessor',lambda:SimpleNamespace(destroy_node=destroy))
    monkeypatch.setattr(module,'MultiThreadedExecutor',lambda **kwargs:Executor())
    monkeypatch.setattr(module,'threading',SimpleNamespace(Event=lambda:stopped))
    monkeypatch.setattr(module,'signal',SimpleNamespace(SIGINT=2,SIGTERM=15,
        getsignal=lambda s:None,signal=lambda *args:None))
    monkeypatch.setattr(module.rclpy,'init',lambda **kwargs:None)
    monkeypatch.setattr(module.rclpy,'ok',lambda:True)
    monkeypatch.setattr(module.rclpy,'shutdown',lambda:calls.append('context_shutdown'))
    timer=Timer(.05,release.set);timer.start()
    try:
        module.main();blocker.result(timeout=1);future.result(timeout=1)
        assert calls==['add_node','queued_callback','executor_shutdown','node_destroy','context_shutdown']
    finally:release.set();timer.cancel();pool.shutdown(wait=True)
