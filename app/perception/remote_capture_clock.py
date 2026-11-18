"""Bound camera capture time to the workstation clock without changing clocks."""
import math

from app.perception.rgbd_source import RGBDFrameUnavailable


def align_capture_time(host_timestamp, age_seconds, *, started, received, elapsed):
    """Server age is measured when JSON is generated, within the HTTP interval.

    Midpoint alignment has at most half an RTT plus the service's millisecond
    rounding uncertainty. The raw server timestamp is retained by the caller.
    Every response establishes its own bound, so reboot/clock steps cannot
    reuse an old offset. Slow requests and local clock jumps fail closed.
    """
    host, age = float(host_timestamp), float(age_seconds)
    values = (host, age, started, received, elapsed)
    if (not all(math.isfinite(v) for v in values) or age < 0
            or not 1.4e9 <= host <= 2.2e9 or elapsed < 0
            or abs(received-started-elapsed) > .005):
        raise RGBDFrameUnavailable('invalid camera clock observation')
    uncertainty = elapsed/2 + .001
    if uncertainty > .03:
        raise RGBDFrameUnavailable('camera clock uncertainty exceeds 30 ms')
    midpoint = (started+received)/2
    return midpoint-age, dict(method='http_midpoint_and_server_frame_age',
        server_minus_local_seconds=host+age-midpoint,
        uncertainty_seconds=uncertainty, request_round_trip_seconds=elapsed)


def shared_host_capture_time(host_timestamp, age_seconds, *, started, received,
                             elapsed, max_age_seconds):
    """Use an explicitly co-hosted service's capture clock without RTT shifts.

    The caller must restrict this mode to literal loopback addresses. Server
    generation time must still lie within this request, wall time must agree
    with monotonic elapsed time, and actual capture age must remain bounded.
    """
    host, age = float(host_timestamp), float(age_seconds)
    values = (host, age, started, received, elapsed, max_age_seconds)
    if (not all(math.isfinite(v) for v in values) or age < 0 or elapsed < 0
            or not 1.4e9 <= host <= 2.2e9 or max_age_seconds <= 0
            or abs(received - started - elapsed) > .005):
        raise RGBDFrameUnavailable('invalid shared-host camera clock observation')
    server_generated = host + age
    if not started - .001 <= server_generated <= received + .001:
        raise RGBDFrameUnavailable('camera clock is not shared with this host')
    if received - host > max_age_seconds or host > received + .001:
        raise RGBDFrameUnavailable('shared-host RGB-D capture is stale or in the future')
    return host, dict(method='shared_host_capture_clock',
        server_minus_local_seconds=0.0, uncertainty_seconds=0.0,
        request_round_trip_seconds=elapsed, capture_age_seconds=received-host)
