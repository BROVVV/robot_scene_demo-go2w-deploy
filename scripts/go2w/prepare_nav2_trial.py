#!/usr/bin/env python3
"""Prepare a fixed trial profile; never starts ROS or grants motion."""
import argparse
import json
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from app.navigation.trial_budget import NavigationTrialBudget


def parameter_source(ros_distro=None):
    distro = os.environ.get('ROS_DISTRO', '') if ros_distro is None else ros_distro
    if ros_distro is None and not distro:
        import platform
        if platform.machine() == 'aarch64' and Path('/etc/nv_tegra_release').is_file():
            distro = 'foxy'
    name = 'nav2_params_foxy_execution.yaml' if distro == 'foxy' else 'nav2_params_humble.yaml'
    return Path(__file__).resolve().parents[2]/'ros2_ws/src/robot_scene_nav_bringup/config'/name


def validate_boot_review(audit, *, profile_sha256, ledger_path, now):
    import math
    geometry=audit.get('sensor_geometry',{})
    trial=audit.get('trial_budget_profile',{})
    bridge=trial.get('parameters',{}).get('/go2w_cmd_vel_bridge',{})
    age=now-float(audit.get('wall_time',0))
    if (not math.isfinite(age) or not -.1<=age<=30 or audit.get('healthy_for_plan') is not True
            or geometry.get('valid') is not True or geometry.get('profile_sha256')!=profile_sha256
            or audit.get('physical_execution',{}).get('verdict')!='disabled'
            or trial.get('configuration_valid') is not True
            or bridge.get('trial_ledger')!=str(Path(ledger_path).resolve())
            or bridge.get('execution_enabled') is not False):
        raise ValueError('new boot review requires a fresh healthy read-only audit of this geometry and ledger')


def live_robot_boot_id():
    import subprocess
    import uuid
    import platform
    import re
    # The sidecar release runs on the robot itself. Self-SSH is neither
    # configured nor needed there; use the local kernel boot ID only when the
    # Jetson owns the robot's wired address.
    if platform.machine() == 'aarch64' and Path('/etc/nv_tegra_release').is_file():
        addresses = subprocess.run(
            ['ip', '-4', '-o', 'addr', 'show'],
            check=True, capture_output=True, text=True, timeout=3,
        ).stdout
        owners = {name for name in re.findall(
            r'^\d+:\s+(\S+)\s+inet 192[.]168[.]123[.]18/', addresses, re.MULTILINE)
            if name != 'lo'}
        if len(owners) > 1:
            raise ValueError('robot internal address belongs to multiple interfaces')
        if len(owners) == 1:
            return str(uuid.UUID(Path('/proc/sys/kernel/random/boot_id').read_text().strip()))
    key_path = Path.home() / '.ssh' / 'id_ed25519_unitree_robotscene'
    known_hosts = Path.home() / '.ssh' / 'known_hosts'
    command = [
        'ssh', '-F', '/dev/null', '-S', 'none',
        '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=4',
        '-o', 'IdentitiesOnly=yes', '-o', 'ClearAllForwardings=yes',
        '-o', 'StrictHostKeyChecking=yes',
        '-o', f'UserKnownHostsFile={known_hosts}',
    ]
    if key_path.is_file():
        command.extend(['-i', str(key_path)])
    command.extend([
        'unitree@192.168.123.18',
        'cat /proc/sys/kernel/random/boot_id',
    ])
    value=subprocess.run(command,
        check=True,capture_output=True,text=True,timeout=6).stdout.strip()
    return str(uuid.UUID(value))


def trial_parameters(base, maximum_speed_mps=.05, *, velocity_smoother=None, forward_only=False,
                     bounded_search=False):
    import copy
    if maximum_speed_mps not in (.05,.12,.30):raise ValueError('unsupported trial speed')
    data=copy.deepcopy(base)
    params=data['controller_server']['ros__parameters']
    # A bounded trial verifies centimetre-scale arrival.  NavFn's production
    # 0.10 m fallback tolerance can otherwise return a path that ends near the
    # start for a 0.12 m request, after which the controller and backend use
    # incompatible definitions of success.  Keep the fallback strictly below
    # the 0.03 m trial goal tolerance.
    data['planner_server']['ros__parameters']['GridBased']['tolerance']=.01
    params['general_goal_checker'].update(xy_goal_tolerance=.03,yaw_goal_tolerance=.05,stateful=False)
    params['progress_checker']['required_movement_radius']=.02
    # On the 15 W Jetson profile the first local motion Action handshake can
    # take a little over eight seconds while the 2-D perception/navigation
    # graph is starting.  The production 8 s progress timeout then aborts the
    # Nav2 controller before the first Move RPC reaches the SDK.  Bounded
    # trials already have an independent one-step ledger, 0.20 m travel cap,
    # speed cap and live STOP path, so allow only this generated trial profile
    # enough startup time without weakening the regular navigation profile.
    params['progress_checker']['movement_time_allowance']=15.0
    params['FollowPath'].update(desired_linear_vel=maximum_speed_mps,
        regulated_linear_scaling_min_speed=.05,min_approach_linear_velocity=.05,
        rotate_to_heading_angular_vel=.10)
    if maximum_speed_mps in (.12, .30):
        # At a 10cm goal the Humble RPP default approach rule reduces even
        # desired=.10/.12 to .05, reproducing the known actuator dead zone.
        # The separately authorized calibration tests actuator response at
        # .12 only on short, straight, preferred-clearance paths (backend
        # predicate). Both RPP reduction floors must participate: changing
        # approach alone still leaves the cost/curvature floor at .05.
        # The final desired-speed clamp preserves lower corridor caps, and
        # collision detection, watchdogs and independent .2m travel remain.
        params['FollowPath']['min_approach_linear_velocity']=maximum_speed_mps
        params['FollowPath']['regulated_linear_scaling_min_speed']=maximum_speed_mps
    if velocity_smoother is not None:
        data['velocity_smoother']=copy.deepcopy(velocity_smoother['velocity_smoother'])
        data['velocity_smoother']['ros__parameters']['max_velocity'][0]=maximum_speed_mps
        if forward_only or bounded_search:
            # Match the independent bridge guard: a transient angular command
            # above .05 would otherwise latch the whole forward test closed.
            smooth=data['velocity_smoother']['ros__parameters']
            smooth['max_velocity'][2]=.05 if forward_only else .1
            smooth['min_velocity'][0]=0.
            smooth['min_velocity'][2]=-.05 if forward_only else -.1
    if forward_only:
        params['FollowPath']['rotate_to_heading_angular_vel']=.05
    return data


def main():
    import yaml
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ledger',required=True)
    p.add_argument('--params-output',required=True)
    p.add_argument('--initialize-window')
    p.add_argument('--forward-test',action='store_true',
        help='initialize one separately authorized forward-only <=3m test')
    p.add_argument('--collision-diagnostic',action='store_true',
        help='initialize one supervised forward diagnostic capped at 0.30m observed travel')
    p.add_argument('--speed-calibration-030',action='store_true',
        help='initialize one <=0.30m straight calibration goal at 0.30m/s')
    p.add_argument('--bounded-search',action='store_true',
        help='initialize three <=0.6m actions charged within a cumulative 3m test scope')
    p.add_argument('--field-search',action='store_true',
        help='initialize five <=3m goals in a separately authorized <=15m field test')
    p.add_argument('--prior-translation-charge-m',type=float,
        help='mandatory conservative charge for preceding tests in the same 3m scope')
    p.add_argument('--maximum-steps',type=int,choices=(1,3),
        help='initialization only: one observed recheck or the existing three-step window')
    p.add_argument('--maximum-speed-mps',type=float,choices=(.05,.12),
        help='only with initialization of a separately authorized new calibration window')
    p.add_argument('--reviewed-robot-boot-id')
    p.add_argument('--verify-live-robot',action='store_true')
    p.add_argument('--review-boot-audit',help='fresh read-only audit after a reboot; preserves consumed slots')
    args=p.parse_args()
    if args.field_search and (not args.initialize_window or args.forward_test or args.collision_diagnostic or args.speed_calibration_030 or args.bounded_search
            or args.maximum_steps is not None or args.maximum_speed_mps is not None
            or args.prior_translation_charge_m is not None):
        p.error('--field-search requires a fresh window and its fixed policy')
    if args.bounded_search and (not args.initialize_window or args.forward_test or args.collision_diagnostic or args.speed_calibration_030
            or args.maximum_steps is not None or args.maximum_speed_mps is not None
            or args.prior_translation_charge_m is None):
        p.error('--bounded-search requires a new window, prior charge, and its fixed policy')
    if args.prior_translation_charge_m is not None and not args.bounded_search:
        p.error('--prior-translation-charge-m requires --bounded-search')
    if args.forward_test and (not args.initialize_window or args.maximum_steps is not None
                             or args.maximum_speed_mps is not None or args.collision_diagnostic or args.speed_calibration_030):
        p.error('--forward-test requires a new window and its fixed forward-only policy')
    if args.collision_diagnostic and (not args.initialize_window
            or args.maximum_steps is not None or args.maximum_speed_mps is not None
            or args.forward_test or args.speed_calibration_030 or args.bounded_search or args.field_search):
        p.error('--collision-diagnostic requires a new window and its fixed 0.30m policy')
    if args.speed_calibration_030 and (not args.initialize_window
            or args.maximum_steps is not None or args.maximum_speed_mps is not None
            or args.forward_test or args.collision_diagnostic or args.bounded_search
            or args.field_search or args.prior_translation_charge_m is not None):
        p.error('--speed-calibration-030 requires a new window and its fixed policy')
    if (args.maximum_speed_mps is not None or args.maximum_steps is not None) and not args.initialize_window:
        p.error('speed is immutable ledger policy; only select it when initializing a new window')
    if args.initialize_window and args.review_boot_audit:
        p.error('new-window initialization and existing-window boot review are separate operations')
    if args.initialize_window:
        if not args.reviewed_robot_boot_id:
            p.error('initialization requires the read-only reviewed robot boot id')
        if args.field_search:
            budget=NavigationTrialBudget.initialize_field_search(args.ledger,
                window_id=args.initialize_window,
                reviewed_robot_boot_id=args.reviewed_robot_boot_id)
        elif args.bounded_search:
            budget=NavigationTrialBudget.initialize_search(args.ledger,window_id=args.initialize_window,
                reviewed_robot_boot_id=args.reviewed_robot_boot_id,
                prior_translation_charge_m=args.prior_translation_charge_m)
        elif args.forward_test:
            budget=NavigationTrialBudget.initialize_forward(args.ledger,window_id=args.initialize_window,
                reviewed_robot_boot_id=args.reviewed_robot_boot_id)
        elif args.collision_diagnostic:
            budget=NavigationTrialBudget.initialize_collision_diagnostic(
                args.ledger, window_id=args.initialize_window,
                reviewed_robot_boot_id=args.reviewed_robot_boot_id)
        elif args.speed_calibration_030:
            budget=NavigationTrialBudget.initialize_speed_calibration_030(
                args.ledger, window_id=args.initialize_window,
                reviewed_robot_boot_id=args.reviewed_robot_boot_id)
        else:
            budget=NavigationTrialBudget.initialize(args.ledger,window_id=args.initialize_window,
                reviewed_robot_boot_id=args.reviewed_robot_boot_id,maximum_speed_mps=args.maximum_speed_mps or .05,
                maximum_steps=args.maximum_steps if args.maximum_steps is not None else 3)
    else:
        budget=NavigationTrialBudget(args.ledger)
    if args.review_boot_audit:
        import hashlib,time
        evidence=Path(args.review_boot_audit).read_bytes()
        profile=Path(__file__).resolve().parents[2]/'configs/go2w/navigation_sensor_profile.yaml'
        profile_hash=hashlib.sha256(profile.read_bytes()).hexdigest()
        validate_boot_review(json.loads(evidence),profile_sha256=profile_hash,ledger_path=budget.path,now=time.time())
        budget.record_boot_review(boot_id=live_robot_boot_id(),audit_sha256=hashlib.sha256(evidence).hexdigest(),
            sensor_profile_sha256=profile_hash)
    if args.verify_live_robot:
        boot=live_robot_boot_id()
        remaining=budget.require_reviewed_boot(boot)
        print(json.dumps(dict(robot_boot_review_matches=True,remaining_reserved_steps=remaining,
            authorizes_motion=False)))
    source=parameter_source()
    target=Path(args.params_output).resolve();target.parent.mkdir(parents=True,exist_ok=True)
    smoother=source.with_name('velocity_smoother.yaml')
    target.write_text(yaml.safe_dump(trial_parameters(yaml.safe_load(source.read_text()),budget.MAX_SPEED_MPS,
        velocity_smoother=yaml.safe_load(smoother.read_text()),
        forward_only=budget.forward_only,bounded_search=budget.bounded_search),sort_keys=False))
    print(json.dumps(dict(ledger=str(budget.path),params=str(target),maximum_reserved_steps=budget.MAX_STEPS,
        requested_step_m=3. if budget.field_search else .45 if budget.bounded_search else
            .30 if budget.MAX_TRANSLATION_M in (.35,.60) else
            .12 if budget.MAX_TRANSLATION_M==.2 else None,
        observed_application_stop_m=budget.MAX_TRANSLATION_M-budget.STOP_RESERVE_M,
        maximum_translation_m=budget.MAX_TRANSLATION_M,maximum_linear_speed_mps=budget.MAX_SPEED_MPS,
        authorizes_motion=False)))


if __name__=='__main__':
    main()
