import asyncio
import contextvars
import threading

import pytest

from app import async_compat


@pytest.mark.parametrize("legacy", [False, True])
def test_offload_keeps_context_and_keyword_arguments(monkeypatch, legacy):
    if legacy:
        monkeypatch.setattr(async_compat, "_native_to_thread", None)
    token = contextvars.ContextVar("request_token", default="none")
    main_thread = threading.get_ident()

    def work(value, *, multiplier):
        assert threading.get_ident() != main_thread
        assert token.get() == "caller"
        token.set("worker")
        return value * multiplier

    async def check():
        token.set("caller")
        assert await async_compat.to_thread(work, 7, multiplier=3) == 21
        assert token.get() == "caller"

    asyncio.run(check())


def test_legacy_offload_propagates_worker_exception(monkeypatch):
    monkeypatch.setattr(async_compat, "_native_to_thread", None)

    def fail():
        raise ValueError("worker failed")

    async def check():
        with pytest.raises(ValueError, match="worker failed"):
            await async_compat.to_thread(fail)

    asyncio.run(check())
