"""Bounded input wait shared by semantic observations, without authorizing motion."""
import time


def wait_for_spatial_inputs(provider):
    """Wait for incoming provider data within the existing 40 × 50 ms budget.

    Nav2 reports NO_GLOBAL_SPATIAL_POSE while its independent health/TF inputs
    recover. It is as unavailable as CAMERA_LOCAL for this wait. Treating it
    as ready exits after the first poll and can feed an avoidable invalid pose
    into the pose validator. Every consumer still applies its own quality,
    TF, timestamp and safety checks after this wait.
    """
    spin = getattr(provider, 'spin_once', None)
    if spin is None:
        return False
    for _ in range(40):
        spin()
        time.sleep(.05)
        quality = str(getattr(provider, 'quality', lambda: 'CAMERA_LOCAL')())
        if quality not in {'CAMERA_LOCAL', 'NO_GLOBAL_SPATIAL_POSE'}:
            return True
    return False
