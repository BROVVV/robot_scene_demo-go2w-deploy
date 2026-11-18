"""Reproduce queued-but-untracked work without initializing ROS."""
from concurrent.futures import ThreadPoolExecutor
from threading import Event,Timer

import pytest

from go2w_lidar_preprocessor.executor_shutdown import shutdown_executor


class GuardExecutor:
    def __init__(self):
        self._executor=ThreadPoolExecutor(max_workers=1)
        self.guard_live=True
        self.shutdown_calls=0
    def shutdown(self):
        self.shutdown_calls+=1
        self.guard_live=False
        return True
    def callback(self):
        if not self.guard_live:raise RuntimeError('guard was already destroyed')
        return 'callback finished while guard live'


@pytest.mark.parametrize('safe',[False,True])
def test_queued_callback_must_finish_before_guard_is_destroyed(safe):
    executor=GuardExecutor();entered=Event();release=Event()
    def occupy():
        entered.set();assert release.wait(timeout=2)
    blocker=executor._executor.submit(occupy);assert entered.wait(timeout=1)
    future=executor._executor.submit(executor.callback)
    assert not future.running() and not future.done()
    timer=Timer(.05,release.set);timer.start()
    try:
        assert (shutdown_executor(executor) if safe else executor.shutdown()) is True
        release.set();blocker.result(timeout=1)
        if safe:assert future.result(timeout=1)=='callback finished while guard live'
        else:
            with pytest.raises(RuntimeError,match='already destroyed'):future.result(timeout=1)
        assert executor.shutdown_calls==1
    finally:release.set();timer.cancel();executor._executor.shutdown(wait=True)


def test_single_thread_executor_uses_its_public_shutdown():
    class Single:
        def shutdown(self):return 'closed'
    assert shutdown_executor(Single())=='closed'


def test_callback_failure_is_not_cleared_or_replaced_by_cleanup():
    executor=GuardExecutor()
    def fail():raise ValueError('real callback failure')
    future=executor._executor.submit(fail)
    assert shutdown_executor(executor)
    with pytest.raises(ValueError,match='real callback failure'):future.result()
