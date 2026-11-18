#!/usr/bin/env python3
"""Archive atomic RGB-D and locally received posture without commanding a robot.

Run with the ROS system Python on the camera host. Receipt-time association is
not hardware synchronization, a calibration acceptance or motion permission.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from capture_motion_geometry import finite_or_none, pair_clouds_to_states


def observe_clock():
    """Bracket the wall read so a descheduled clock sample is not treated as exact."""
    before = time.monotonic_ns()
    wall = time.time_ns()
    after = time.monotonic_ns()
    return wall, (before+after)//2, (after-before+1)//2


def capture_monotonic_ns(frame):
    """Convert a certified same-host capture time using this fetch's anchor."""
    if frame['rgbd'].get('timestamp_quality') != 'shared_host_capture':
        raise ValueError('RGB-D capture does not have a shared-host timestamp')
    clock = frame['fetch_clock']
    w0, m0 = clock['began_wall_ns'], clock['began_monotonic_ns']
    w1, m1 = clock['ended_wall_ns'], clock['ended_monotonic_ns']
    u0, u1 = clock['began_uncertainty_ns'], clock['ended_uncertainty_ns']
    captured = frame['capture_wall_ns']
    if any(type(value) is not int for value in (w0, m0, w1, m1, u0, u1, captured)):
        raise ValueError('capture clock observations must be integer nanoseconds')
    if min(u0, u1) < 0 or max(u0, u1) > 1_000_000:
        raise ValueError('clock sampling uncertainty exceeds 1 ms')
    if m1 < m0 or abs((w1-w0)-(m1-m0)) > 5_000_000:
        raise ValueError('wall clock changed during RGB-D fetch')
    mapped = m0 + captured-w0
    if mapped < m0-500_000_000 or mapped > m1+1_000_000:
        raise ValueError('capture is stale before fetch or in the future')
    return mapped


def associate_posture(frames, states, *, tolerance_ns=30_000_000):
    """Keep every frame, including invalid/unpaired ones; never invent posture."""
    received = [state['received_monotonic_ns'] for state in states]
    # Reuse the recorder's ordered-timestamp validation, even for empty frames.
    pair_clouds_to_states([], received, tolerance_ns=tolerance_ns)
    result = []
    for frame in frames:
        row = dict(nearest_state_index=-1, state_receive_delta_ms=None,
                   receipt_alignment_bound_ms=None, capture_monotonic_ns=None, posture_usable=False)
        try:
            capture = capture_monotonic_ns(frame)
        except (ValueError, KeyError, TypeError) as error:
            row['reason'] = str(error); result.append(row); continue
        row['capture_monotonic_ns'] = capture
        anchor = frame['fetch_clock']
        candidates = []
        for index, state in enumerate(states):
            wall = state.get('received_wall_ns')
            mono = state['received_monotonic_ns']
            uncertainty = state.get('clock_uncertainty_ns')
            if (type(wall) is int and type(mono) is int and
                    type(uncertainty) is int and 0 <= uncertainty <= 1_000_000 and
                    abs((wall-anchor['began_wall_ns'])-(mono-anchor['began_monotonic_ns']))
                    + uncertainty + anchor['began_uncertainty_ns'] <= 5_000_000):
                candidates.append(index)
        match = int(pair_clouds_to_states([capture],
            [received[index] for index in candidates], tolerance_ns=tolerance_ns)[0])
        if match < 0:
            row['reason'] = 'no state within receipt-time tolerance and same clock segment'
        else:
            index = candidates[match]; state = states[index]
            row['nearest_state_index'] = index
            row['state_receive_delta_ms'] = (received[index]-capture)/1e6
            # Include clock-read bounds and 1us for floating Unix timestamp
            # quantization. This is still a receipt bound, not sensor-time sync.
            bound = (abs(received[index]-capture) + state['clock_uncertainty_ns']
                     + anchor['began_uncertainty_ns'] + 1_000)
            row['receipt_alignment_bound_ms'] = bound/1e6
            q, rpy = state.get('q', []), state.get('rpy', [])
            values = q+rpy
            usable = (state.get('state_finite') is True and len(q) == 12 and len(rpy) == 3
                      and all(type(value) in (int, float) and math.isfinite(value) for value in values))
            row['posture_usable'] = usable and bound <= tolerance_ns
            row['reason'] = ('nearest state has invalid posture' if not usable else
                ('clock uncertainty exceeds receipt-time tolerance' if bound > tolerance_ns else
                 'nearest local receipt; not hardware synchronized'))
        result.append(row)
    return result


def portable_frame(frame, directory):
    result = dict(frame)
    result['source_original_refs'] = {name: frame[name] for name in ('color_ref', 'depth_ref')}
    for name in ('color_ref', 'depth_ref'):
        result[name] = Path(frame[name]).resolve().relative_to(directory.resolve()).as_posix()
    return result


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--base-url', default='http://127.0.0.1:8080')
    parser.add_argument('--frames', type=int, default=3)
    parser.add_argument('--interval', type=float, default=.5)
    parser.add_argument('--duration', type=float, default=12.)
    parser.add_argument('--lowstate-topic', default='/lf/lowstate')
    args = parser.parse_args(argv)
    if not 1 <= args.frames <= 10:
        parser.error('--frames must be 1..10')
    if not math.isfinite(args.interval) or not .1 <= args.interval <= 2.:
        parser.error('--interval must be .1..2 seconds')
    if not math.isfinite(args.duration) or not 2 <= args.duration <= 30:
        parser.error('--duration must be 2..30 seconds')
    if args.duration < .7 + (args.frames-1)*args.interval:
        parser.error('duration is shorter than requested sampling window')
    return args


def main(argv=None):
    args = parse_args(argv)
    from app.perception.realsense_http_rgbd_source import RealSenseHTTPRGBDSource
    # This validates literal loopback and shared-host clock mode. It does not
    # start, restart, configure or take ownership of the camera service.
    output = Path(args.output_dir).resolve()
    if output.exists():
        raise ValueError('capture output already exists')
    output.mkdir(parents=True)
    source = RealSenseHTTPRGBDSource(args.base_url, cache_dir=output/'rgbd',
        max_cached_frames=args.frames+2, request_timeout=2., max_age_seconds=.5,
        clock_mode='shared_host')
    provenance = output/'provenance'; provenance.mkdir()
    for filename, relative in [('sensor_profile.yaml', 'configs/go2w/navigation_sensor_profile.yaml'),
        ('official_go2w.urdf', 'configs/go2w/calibration_20261019/official_go2w.urdf'),
        ('official_reference.yaml', 'configs/go2w/official_reference.yaml')]:
        shutil.copy2(ROOT/relative, provenance/filename)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from unitree_go.msg import LowState
    frames, states, errors = [], [], []
    interrupted = False
    started = time.time()
    rclpy.init(); node = rclpy.create_node('go2w_rgbd_posture_passive_capture')

    def state_received(message):
        if len(states) >= 20_000:
            raise RuntimeError('posture sample limit reached')
        wall, mono, uncertainty = observe_clock()
        q = [finite_or_none(message.motor_state[index].q) for index in range(12)]
        rpy = [finite_or_none(value) for value in message.imu_state.rpy]
        states.append(dict(received_wall_ns=wall, received_monotonic_ns=mono,
            clock_uncertainty_ns=uncertainty,
            q=q, rpy=rpy, battery_soc=int(message.bms_state.soc),
            device_tick_raw=int(message.tick) if hasattr(message, 'tick') else None,
            wheel_dq=([finite_or_none(message.motor_state[index].dq) for index in range(12,16)]
                      if len(message.motor_state) >= 16 else None),
            state_finite=len(rpy) == 3 and all(value is not None for value in q+rpy)))

    def fetch():
        before_wall, before_mono, before_uncertainty = observe_clock()
        frame = source.get_latest().to_dict()
        after_wall, after_mono, after_uncertainty = observe_clock()
        return dict(rgbd=portable_frame(frame, output), capture_wall_ns=round(frame['timestamp']*1e9),
            fetch_clock=dict(began_wall_ns=before_wall, began_monotonic_ns=before_mono,
                began_uncertainty_ns=before_uncertainty, ended_uncertainty_ns=after_uncertainty,
                ended_wall_ns=after_wall, ended_monotonic_ns=after_mono))

    node.create_subscription(LowState, args.lowstate_topic, state_received, qos_profile_sensor_data)
    pool = ThreadPoolExecutor(max_workers=1)
    future = None
    try:
        beginning = time.monotonic(); deadline = beginning+args.duration
        next_frame = beginning+.5  # Warm state samples before requesting a frame.
        while len(frames) < args.frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if future is None and time.monotonic() >= next_frame:
                future = pool.submit(fetch)
            if future is not None and future.done():
                row = future.result(); future = None
                if all(old['rgbd']['frame_id'] != row['rgbd']['frame_id'] for old in frames):
                    frames.append(row)
                next_frame = time.monotonic()+args.interval
        # Receive the closest sample on either side of the last frame too.
        end = time.monotonic()+.2
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        if len(frames) != args.frames:
            errors.append('requested frame count not captured before deadline')
    except KeyboardInterrupt:
        interrupted = True; errors.append('operator interrupted capture')
    except Exception as error:
        errors.append(type(error).__name__+': '+str(error))
    finally:
        if future is not None:
            future.cancel()
        pool.shutdown(wait=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    associations = associate_posture(frames, states)
    for frame, association in zip(frames, associations):
        frame['association'] = association
    hashes = {path.relative_to(output).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
              for path in sorted(output.rglob('*')) if path.is_file()}
    complete = (not errors and len(frames) == args.frames and
                all(row['posture_usable'] for row in associations))
    metadata = dict(schema='go2w.rgbd_posture_receipt_capture.v1', started_wall=started,
        finished_wall=time.time(), duration_requested_s=args.duration,
        lowstate_topic=args.lowstate_topic, boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        refs_relative_to='metadata.json parent', frames=frames, states=states, errors=errors,
        association='nearest local receipt time; not hardware synchronization',
        association_tolerance_ms=30, sha256=hashes, capture_complete=complete,
        calibration_written=False, calibration_accepted=False,
        reference_config_only=True, current_tf_not_captured=True, stationary_state_not_asserted=True,
        physical_command_sent=False, command_publishers_created=0)
    (output/'metadata.json').write_text(json.dumps(metadata,allow_nan=False,indent=2)+'\n')
    print(json.dumps(dict(output_dir=str(output), frames=len(frames), state_samples=len(states),
                         usable_pairs=sum(row['posture_usable'] for row in associations),
                         capture_complete=complete, calibration_accepted=False, physical_command_sent=False)))
    return 130 if interrupted else (0 if complete else 2)


if __name__ == '__main__':
    raise SystemExit(main())
