"""Finish queued callbacks before Foxy's executor destroys their guard.

Call from the owning spin thread after scheduling has stopped, before node or
context destruction. Foxy's inherited Executor.shutdown waits for callbacks
already inside the work tracker, but queued pool handlers have not entered it.
Those handlers still hold the original guard and may trigger it on completion.
"""
from concurrent.futures import ThreadPoolExecutor


def shutdown_executor(executor):
    pool = getattr(executor, '_executor', None)
    if isinstance(pool, ThreadPoolExecutor):
        pool.shutdown(wait=True)
    # Newer ROS versions also drain here; ThreadPoolExecutor shutdown is
    # idempotent. Single-thread executors have no pool to drain first.
    return executor.shutdown()
