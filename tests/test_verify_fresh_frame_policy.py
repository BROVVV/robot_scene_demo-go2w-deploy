"""Tests for same-frame verify de-duplication and fresh-frame re-verify."""

from __future__ import annotations

from app.live_robot.verify_cache import VerificationCache, VerificationCacheEntry


def test_same_frame_verify_is_cached():
    cache = VerificationCache()
    key = cache.make_key("bundle_1", "垃圾桶", [0.1, 0.2, 0.3, 0.4])
    assert cache.get(key) is None
    cache.put(key, VerificationCacheEntry(confirmed=False, confidence=0.2))
    assert cache.get(key) is not None
    # Same frame and bbox -> cached; no extra API call.
    second = cache.get(cache.make_key("bundle_1", "垃圾桶", [0.1, 0.2, 0.3, 0.4]))
    assert second is not None
    assert second.confirmed is False


def test_fresh_frame_has_different_key():
    cache = VerificationCache()
    key_a = cache.make_key("bundle_1", "垃圾桶", [0.1, 0.2, 0.3, 0.4])
    key_b = cache.make_key("bundle_2", "垃圾桶", [0.1, 0.2, 0.3, 0.4])
    assert key_a != key_b
    cache.put(key_a, VerificationCacheEntry(confirmed=False, confidence=0.2))
    assert cache.get(key_b) is None


def test_quantized_bbox_handles_tiny_float_drift():
    cache = VerificationCache()
    key_a = cache.make_key("b", "t", [0.123456, 0.234567, 0.345678, 0.456789])
    key_b = cache.make_key("b", "t", [0.1235, 0.2346, 0.3457, 0.4568])
    assert key_a == key_b
