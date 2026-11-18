"""Persistent, bounded Nav2 trial accounting and observed-motion stop gate.

This is an additional application gate, not an actuator watchdog or a motion
authorization. The existing bridge, collision and freshness gates still apply.
"""
from contextlib import contextmanager
import fcntl
import json
import math
import os
from pathlib import Path
import tempfile
import time
import uuid


class TrialBudgetError(RuntimeError):
    pass


class NavigationTrialBudget:
    MAX_STEPS = 3
    MAX_TRANSLATION_M = .20
    STOP_RESERVE_M = .05
    MAX_SPEED_MPS = .05
    MAX_TURN_RAD = math.radians(20)
    MAX_ROTATION_TRANSLATION_M = .10

    def __init__(self, path):
        self.path = Path(path).resolve()
        self.token = None
        self._policy_version = None
        # Missing/corrupt ledgers never silently become a new allowance.
        with self._locked() as data:
            self._validate(data)

    @classmethod
    def initialize(cls, path, *, window_id, reviewed_robot_boot_id, maximum_speed_mps=.05,
                   maximum_steps=3):
        """Create once after field review; calling again cannot reset a budget."""
        path = Path(path).resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        if (type(maximum_steps) is not int or maximum_steps not in (1,3)
                or maximum_steps == 1 and maximum_speed_mps != .12):
            raise TrialBudgetError('single recheck requires the working .12 m/s policy')
        data = dict(version=1, window_id=window_id, created_wall=time.time(),
            reviewed_robot_boot_id=reviewed_robot_boot_id,
            maximum_steps=maximum_steps, maximum_translation_m=cls.MAX_TRANSLATION_M,
            stop_reserve_m=cls.STOP_RESERVE_M, steps=[], active_token=None,
            scope='additional Nav2 application stop gate; does not authorize motion')
        if maximum_speed_mps not in (.05,.12):
            raise TrialBudgetError('unsupported trial speed policy')
        if maximum_speed_mps==.12:
            data.update(version=2,maximum_speed_mps=.12,purpose='supervised_velocity_calibration')
            if maximum_steps == 1:
                data.update(version=3,purpose='single_observed_recheck')
        with path.open('x') as stream:
            json.dump(data, stream, allow_nan=False, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        return cls(path)

    @classmethod
    def initialize_forward(cls, path, *, window_id, reviewed_robot_boot_id):
        """One explicitly authorized forward test, at most 3m, never renewable."""
        path = Path(path).resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        if not window_id or not reviewed_robot_boot_id:
            raise TrialBudgetError('forward test requires a window and reviewed boot')
        data = dict(version=4, window_id=window_id,
            reviewed_robot_boot_id=reviewed_robot_boot_id,
            purpose='forward_observed_recheck', forward_only=True,
            maximum_steps=1, maximum_speed_mps=.12,
            maximum_translation_m=3., stop_reserve_m=.15,
            maximum_turn_rad=.15, maximum_lateral_m=.15, maximum_duration_s=60.,
            steps=[], active_token=None,
            scope='one observed forward test only; no unrestricted search authorization')
        with path.open('x') as stream:
            json.dump(data, stream, allow_nan=False, indent=2)
            stream.flush(); os.fsync(stream.fileno())
        return cls(path)

    @classmethod
    def initialize_collision_diagnostic(cls, path, *, window_id, reviewed_robot_boot_id):
        """One forward diagnostic capped at the operator-authorized 0.20 m."""
        path = Path(path).resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        if not window_id or not reviewed_robot_boot_id:
            raise TrialBudgetError('collision diagnostic requires a window and reviewed boot')
        data = dict(version=7, window_id=window_id,
            created_wall=time.time(),
            reviewed_robot_boot_id=reviewed_robot_boot_id,
            purpose='supervised_collision_diagnostic', forward_only=True,
            maximum_steps=1, maximum_speed_mps=.12,
            maximum_translation_m=.25, stop_reserve_m=.05,
            maximum_turn_rad=.15, maximum_lateral_m=.10, maximum_duration_s=15.,
            steps=[], active_token=None,
            scope='one supervised forward diagnostic; observed application stop <=0.20m')
        with path.open('x') as stream:
            json.dump(data, stream, allow_nan=False, indent=2)
            stream.flush(); os.fsync(stream.fileno())
        return cls(path)

    @classmethod
    def initialize_speed_calibration_030(cls, path, *, window_id, reviewed_robot_boot_id):
        """One non-renewable 0.30 m/s straight-line calibration goal.

        The 0.60 m wheel-travel envelope is deliberately larger than the
        requested 0.30 m goal so the independent bridge can include its full
        outstanding command/watchdog horizon without rejecting the first
        slice.  The application still reserves only a <=0.30 m straight goal.
        """
        path = Path(path).resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        if not window_id or not reviewed_robot_boot_id:
            raise TrialBudgetError('0.30 m/s calibration requires a window and reviewed boot')
        data = dict(version=8, window_id=window_id,
            reviewed_robot_boot_id=reviewed_robot_boot_id,
            purpose='supervised_velocity_calibration_0_30', forward_only=True,
            maximum_steps=1, maximum_speed_mps=.30,
            maximum_translation_m=.60, stop_reserve_m=.15,
            maximum_turn_rad=.15, maximum_lateral_m=.10, maximum_duration_s=15.,
            steps=[], active_token=None,
            scope='one supervised <=0.30m straight goal at <=0.30m/s; no search authorization')
        with path.open('x') as stream:
            json.dump(data, stream, allow_nan=False, indent=2)
            stream.flush(); os.fsync(stream.fileno())
        return cls(path)

    @classmethod
    def initialize_search(cls, path, *, window_id, reviewed_robot_boot_id,
                          prior_translation_charge_m):
        """Three <=0.6m actions within a shared, conservatively charged 3m scope.

        Charge the entire physical envelope of every action, including failed
        actions. The independent bridge enforces each envelope. No refund or
        counter reset is needed to admit the next autonomous-search goal.
        """
        if (not window_id or not reviewed_robot_boot_id
                or not isinstance(prior_translation_charge_m,(int,float))
                or not math.isfinite(prior_translation_charge_m)
                or not 0<=prior_translation_charge_m<=1.2):
            raise TrialBudgetError('prior charges plus 3 x 0.6m must fit the 3m scope')
        path=Path(path).resolve();path.parent.mkdir(parents=True,exist_ok=True)
        data=dict(version=5,window_id=window_id,reviewed_robot_boot_id=reviewed_robot_boot_id,
            purpose='bounded_search_session',maximum_steps=3,maximum_speed_mps=.12,
            maximum_translation_m=.6,stop_reserve_m=.1,maximum_scope_translation_m=3.,
            prior_translation_charge_m=float(prior_translation_charge_m),
            steps=[],active_token=None,
            scope='three independently bounded actions; total scope including prior charges <=3m')
        with path.open('x') as stream:
            json.dump(data,stream,allow_nan=False,indent=2);stream.flush();os.fsync(stream.fileno())
        return cls(path)

    @classmethod
    def initialize_field_search(cls, path, *, window_id, reviewed_robot_boot_id):
        """Create the separately authorized five-goal, <=3m-per-goal window."""
        if not window_id or not reviewed_robot_boot_id:
            raise TrialBudgetError('field search requires a window and reviewed boot')
        path=Path(path).resolve();path.parent.mkdir(parents=True,exist_ok=True)
        data=dict(version=6,window_id=window_id,reviewed_robot_boot_id=reviewed_robot_boot_id,
            purpose='supervised_five_step_field_search',maximum_steps=5,
            maximum_speed_mps=.12,maximum_translation_m=3.,stop_reserve_m=.15,
            maximum_scope_translation_m=15.,steps=[],active_token=None,
            scope='five independently bounded goals, each <=3m; conservative total <=15m')
        with path.open('x') as stream:
            json.dump(data,stream,allow_nan=False,indent=2);stream.flush();os.fsync(stream.fileno())
        return cls(path)

    @contextmanager
    def _locked(self):
        with self.path.with_suffix(self.path.suffix + '.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            try:
                data = json.loads(self.path.read_text())
                yield data
            except (OSError, ValueError, KeyError, TypeError, IndexError, AttributeError) as exc:
                raise TrialBudgetError('trial ledger unreadable or invalid') from exc

    def _validate(self, data):
        version=data.get('version')
        maximum_steps=5 if version==6 else 1 if version in (3,4,7,8) else 3
        translation=(.25 if version==7 else 3. if version in (4,6)
                     else .6 if version in (5,8) else .2)
        reserve=.15 if version in (4,6,8) else .1 if version==5 else .05
        if (type(version) is not int or version not in (1,2,3,4,5,6,7,8)
                or self._policy_version is not None and version!=self._policy_version
                or version in (2,3) and (data.get('maximum_speed_mps')!=.12
                    or data.get('purpose')!=('single_observed_recheck' if version==3
                                            else 'supervised_velocity_calibration'))
                or version==4 and (data.get('maximum_speed_mps')!=.12
                    or data.get('purpose')!='forward_observed_recheck'
                    or data.get('forward_only') is not True
                    or data.get('maximum_turn_rad')!=.15
                    or data.get('maximum_lateral_m')!=.15
                    or data.get('maximum_duration_s')!=60.)
                or version==5 and (data.get('maximum_speed_mps')!=.12
                    or data.get('purpose')!='bounded_search_session'
                    or data.get('maximum_scope_translation_m')!=3.
                    or not isinstance(data.get('prior_translation_charge_m'),(int,float))
                    or not math.isfinite(data['prior_translation_charge_m'])
                    or not 0<=data['prior_translation_charge_m']<=1.2)
                or version==6 and (data.get('maximum_speed_mps')!=.12
                    or data.get('purpose')!='supervised_five_step_field_search'
                    or data.get('maximum_scope_translation_m')!=15.)
                or version==7 and (data.get('maximum_speed_mps')!=.12
                    or data.get('purpose')!='supervised_collision_diagnostic'
                    or data.get('forward_only') is not True
                    or data.get('maximum_turn_rad')!=.15
                    or data.get('maximum_lateral_m')!=.10
                    or data.get('maximum_duration_s')!=15.)
                or version==8 and (data.get('maximum_speed_mps')!=.30
                    or data.get('purpose')!='supervised_velocity_calibration_0_30'
                    or data.get('forward_only') is not True
                    or data.get('maximum_turn_rad')!=.15
                    or data.get('maximum_lateral_m')!=.10
                    or data.get('maximum_duration_s')!=15.)
                or type(data.get('maximum_steps')) is not int
                or data.get('maximum_steps') != maximum_steps
                or data.get('maximum_translation_m') != translation
                or data.get('stop_reserve_m') != reserve
                or not isinstance(data.get('steps'), list)
                or len(data['steps']) > maximum_steps):
            raise TrialBudgetError('trial ledger policy mismatch')
        self._policy_version=version
        self.MAX_STEPS=maximum_steps
        self.MAX_SPEED_MPS=.05 if version==1 else .30 if version==8 else .12
        self.MAX_TRANSLATION_M=translation
        self.STOP_RESERVE_M=reserve
        self.MAX_TURN_RAD=.15 if version in (4,7,8) else math.radians(20)
        self.MAX_LATERAL_M=.10 if version in (7,8) else .15
        self.MAX_DURATION_S=15. if version in (7,8) else 60.
        self.forward_only=version in (4,7,8)
        self.collision_diagnostic=version==7
        self.speed_calibration_0_30=version==8
        self.bounded_search=version in (5,6)
        self.field_search=version==6

    def _save(self, data):
        descriptor, temporary = tempfile.mkstemp(dir=self.path.parent, prefix='.trial-')
        try:
            with os.fdopen(descriptor, 'w') as stream:
                json.dump(data, stream, allow_nan=False, indent=2)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
            directory = os.open(self.path.parent, os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def require_reviewed_boot(self, boot_id):
        """Read-only admission check; never renews or refunds an allowance."""
        with self._locked() as data:
            self._validate(data)
            if not boot_id or data.get('reviewed_robot_boot_id') != boot_id:
                raise TrialBudgetError('robot rebooted since field review; geometry/time review required')
            if data.get('active_token') is not None:
                raise TrialBudgetError('unfinished trial step; field review required')
            remaining=self.MAX_STEPS-len(data['steps'])
            if remaining<=0:
                raise TrialBudgetError('trial step budget exhausted')
            return remaining

    def record_boot_review(self, *, boot_id, audit_sha256, sensor_profile_sha256):
        """Bind a fresh read-only review while preserving every consumed slot."""
        hashes=(audit_sha256,sensor_profile_sha256)
        if (not boot_id or any(not isinstance(s,str) or len(s)!=64 or
                any(c not in '0123456789abcdef' for c in s) for s in hashes)):
            raise TrialBudgetError('boot review needs bound audit and sensor profile hashes')
        with self._locked() as data:
            self._validate(data)
            if data.get('active_token') is not None:
                raise TrialBudgetError('unfinished trial step; review cannot clear its reservation')
            data.setdefault('boot_reviews',[]).append(dict(previous_boot_id=data.get('reviewed_robot_boot_id'),
                robot_boot_id=boot_id,reviewed_at=time.time(),audit_sha256=audit_sha256,
                sensor_profile_sha256=sensor_profile_sha256))
            data['reviewed_robot_boot_id']=boot_id
            self._save(data)

    @staticmethod
    def _pose(pose):
        if pose is None or not all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw)):
            raise TrialBudgetError('trial requires a finite current pose')
        return dict(x=pose.x, y=pose.y, yaw=pose.yaw, frame=pose.frame_id)

    def reserve(self, *, goal_id, path_length_m, goal_x, goal_y, goal_yaw, pose,
                tracking_pose=None):
        planning_pose = self._pose(pose)
        # The goal envelope is expressed in map coordinates, but physical
        # travel must be accumulated in continuous odometry.  Using map poses
        # here double-counts SLAM corrections and can stop a stationary or
        # slowly moving robot at the travel limit.
        initial = self._pose(tracking_pose if tracking_pose is not None else pose)
        limit = self.MAX_TRANSLATION_M - self.STOP_RESERVE_M
        if (not all(math.isfinite(v) for v in (path_length_m, goal_x, goal_y, goal_yaw))
                or not 0 <= path_length_m <= limit
                or math.hypot(goal_x-pose.x, goal_y-pose.y) > limit
                or abs(math.remainder(goal_yaw-pose.yaw, 2*math.pi)) > self.MAX_TURN_RAD):
            raise TrialBudgetError('trial path/goal exceeds bounded step envelope')
        if self.forward_only:
            dx,dy=goal_x-pose.x,goal_y-pose.y
            forward=dx*math.cos(pose.yaw)+dy*math.sin(pose.yaw)
            lateral=-dx*math.sin(pose.yaw)+dy*math.cos(pose.yaw)
            if forward < 0 or abs(lateral) > .05:
                raise TrialBudgetError('forward test requires a goal straight ahead')
        with self._locked() as data:
            self._validate(data)
            if data['active_token'] is not None:
                raise TrialBudgetError('unfinished trial step; field review required')
            if len(data['steps']) >= self.MAX_STEPS:
                raise TrialBudgetError('trial step budget exhausted')
            token = uuid.uuid4().hex
            requested_translation_m = math.hypot(goal_x-pose.x, goal_y-pose.y)
            requested_turn_rad = abs(math.remainder(goal_yaw-pose.yaw, 2*math.pi))
            # A short goal must not inherit the whole policy envelope as its
            # retry allowance.  Nav2 may clear/replan the same goal, so without
            # a goal-specific bound a 0.15 m request can accumulate almost the
            # full 0.50 m search allowance while making no useful progress.
            # Keep 0.10 m for controller settling/odometry noise, but never
            # exceed the policy's existing observed-travel stop threshold.
            observed_travel_limit_m = min(
                self.MAX_TRANSLATION_M-self.STOP_RESERVE_M,
                max(float(path_length_m), requested_translation_m)+0.10,
            )
            data['steps'].append(dict(token=token, goal_id=goal_id,
                reserved_at=time.time(), state='reserved', planning_pose=planning_pose,
                initial_pose=initial,
                last_pose=initial, traveled_m=0., turned_rad=0.,
                planned_path_length_m=float(path_length_m),
                observed_travel_limit_m=observed_travel_limit_m,
                requested_translation_m=requested_translation_m,
                requested_turn_rad=requested_turn_rad, stop_reason=''))
            data['active_token'] = token
            self._save(data)  # Reserve durably BEFORE any arm/send, including retries.
            self.token = token

    def observe(self, pose):
        with self._locked() as data:
            self._validate(data)
            if self.token is None or data['active_token'] != self.token:
                raise TrialBudgetError('trial reservation identity lost')
            step = data['steps'][-1]
            if step['token'] != self.token:
                raise TrialBudgetError('trial step identity mismatch')
            try:
                current = self._pose(pose)
                previous = step['last_pose']
                if current['frame'] != previous['frame']:
                    raise TrialBudgetError('trial pose frame changed')
                step['traveled_m'] += math.hypot(current['x']-previous['x'], current['y']-previous['y'])
                origin=step['initial_pose']
                delta_yaw=abs(math.remainder(current['yaw']-origin['yaw'],2*math.pi))
                if self._policy_version in (5,6):
                    # Bounded-search policies bound maximum heading excursion.  Nav2's controller
                    # legitimately alternates small corrections on a straight
                    # path; summing their absolute values exhausted 20° while
                    # the robot's net heading changed less than 3°.
                    step['turned_rad']=max(step['turned_rad'],delta_yaw)
                else:
                    step['turned_rad'] += abs(math.remainder(
                        current['yaw']-previous['yaw'],2*math.pi))
                step['last_pose'] = current
                dx,dy=current['x']-origin['x'],current['y']-origin['y']
                forward=dx*math.cos(origin['yaw'])+dy*math.sin(origin['yaw'])
                lateral=-dx*math.sin(origin['yaw'])+dy*math.cos(origin['yaw'])
                if self.forward_only and (
                    forward < -.02 or abs(lateral) > self.MAX_LATERAL_M
                    or time.time()-step['reserved_at'] > self.MAX_DURATION_S):
                    step['stop_reason']='forward test direction/time limit'
                elif (step.get('requested_translation_m', math.inf) <= 1e-3
                        and step.get('requested_turn_rad', 0.) > 1e-3
                        and step['traveled_m'] >= self.MAX_ROTATION_TRANSLATION_M):
                    step['stop_reason'] = 'trial rotation translation limit'
                elif (self.bounded_search
                        and step['traveled_m'] >= step.get(
                            'observed_travel_limit_m',
                            self.MAX_TRANSLATION_M-self.STOP_RESERVE_M,
                        )):
                    step['stop_reason'] = 'trial goal-specific observed travel limit'
                elif step['traveled_m'] >= self.MAX_TRANSLATION_M-self.STOP_RESERVE_M:
                    step['stop_reason'] = 'trial observed travel limit'
                elif step['turned_rad'] >= self.MAX_TURN_RAD:
                    step['stop_reason'] = 'trial observed turn limit'
            except TrialBudgetError as exc:
                step['stop_reason'] = str(exc)
            self._save(data)
            return step['stop_reason']

    def finish(self):
        if self.token is None:
            return
        with self._locked() as data:
            self._validate(data)
            if data['active_token'] != self.token or data['steps'][-1]['token'] != self.token:
                raise TrialBudgetError('cannot finish another trial reservation')
            data['steps'][-1]['state'] = 'finished'
            data['steps'][-1]['finished_at'] = time.time()
            data['active_token'] = None
            self._save(data)
            self.token = None
