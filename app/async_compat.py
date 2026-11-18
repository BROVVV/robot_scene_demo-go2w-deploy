"""Thread offloading with context propagation on Python 3.8 and newer."""
import asyncio
import contextvars
import functools


_native_to_thread = getattr(asyncio, "to_thread", None)


async def to_thread(func, /, *args, **kwargs):
    if _native_to_thread is not None:
        return await _native_to_thread(func, *args, **kwargs)
    loop = asyncio.get_running_loop()
    context = contextvars.copy_context()
    call = functools.partial(context.run, func, *args, **kwargs)
    return await loop.run_in_executor(None, call)
