"""Independent odometry guard for fixed bounded trial ledgers.

Reads reservations but never creates allowance, arms, or modifies the ledger.
After a bridge restart an existing reservation is refused. A new reservation
must be created while this guard is already watching fresh wheel odometry.
"""
import json
import math
from pathlib import Path
import time


class TrialVelocityGuard:
    def __init__(self, ledger, *, started_at=None):
        self.ledger = Path(ledger)
        self.started_at = time.time() if started_at is None else started_at
        self.token = None
        self.closed = set()
        self.pose = None
        self.stamp = self.received = None
        self.traveled = self.turned = 0.
        self.reason = ''
        self.maximum_speed_mps=.05
        self._policy_version=None
        self.initial_pose=None
        self.reserved_at=None
        try:
            data=json.loads(self.ledger.read_text())
            self.maximum_speed_mps=self._policy_speed(data)
            self._policy_version=data['version']
        except (OSError,ValueError,KeyError,TypeError):
            pass  # check() reports invalid policy before permitting dispatch

    @staticmethod
    def _policy_speed(data):
        if not isinstance(data,dict) or type(data.get('version')) is not int:raise ValueError('version')
        if data['version']==1:return .05
        if (data['version']==2 and data.get('maximum_speed_mps')==.12
                and data.get('purpose')=='supervised_velocity_calibration'):return .12
        if (data['version']==3 and data.get('maximum_speed_mps')==.12
                and data.get('purpose')=='single_observed_recheck'):return .12
        if (data['version']==4 and data.get('maximum_speed_mps')==.12
                and data.get('purpose')=='forward_observed_recheck'
                and data.get('forward_only') is True
                and data.get('maximum_turn_rad')==.15
                and data.get('maximum_lateral_m')==.15
                and data.get('maximum_duration_s')==60.):return .12
        if (data['version']==5 and data.get('maximum_speed_mps')==.12
                and data.get('purpose')=='bounded_search_session'
                and data.get('maximum_scope_translation_m')==3.
                and isinstance(data.get('prior_translation_charge_m'),(int,float))
                and math.isfinite(data['prior_translation_charge_m'])
                and 0<=data['prior_translation_charge_m']<=1.2):return .12
        if (data['version']==6 and data.get('maximum_speed_mps')==.12
                and data.get('purpose')=='supervised_five_step_field_search'
                and data.get('maximum_scope_translation_m')==15.):return .12
        if (data['version']==7 and data.get('maximum_speed_mps')==.12
                and data.get('purpose')=='supervised_collision_diagnostic'
                and data.get('forward_only') is True
                and data.get('maximum_turn_rad')==.15
                and data.get('maximum_lateral_m')==.10
                and data.get('maximum_duration_s')==15.):return .12
        if (data['version']==8 and data.get('maximum_speed_mps')==.30
                and data.get('purpose')=='supervised_velocity_calibration_0_30'
                and data.get('forward_only') is True
                and data.get('maximum_turn_rad')==.15
                and data.get('maximum_lateral_m')==.10
                and data.get('maximum_duration_s')==15.):return .30
        raise ValueError('unsupported calibration policy')

    def odometry(self, *, x, y, yaw, frame, stamp, received):
        if (not all(math.isfinite(v) for v in (x,y,yaw,stamp,received))
                or frame != 'odom_fused' or self.stamp is not None and stamp <= self.stamp):
            if self.token:
                self.reason = 'trial_odometry_invalid'
            self.pose = None
            return
        if self.pose is not None and self.token:
            self.traveled += math.hypot(x-self.pose[0], y-self.pose[1])
            self.turned += abs(math.remainder(yaw-self.pose[2], 2*math.pi))
        self.pose = (x,y,yaw)
        self.stamp,self.received = stamp,received

    def check(self, *, now, received_now, linear, angular, duration):
        try:
            data = json.loads(self.ledger.read_text())
            speed=self._policy_speed(data)
            steps = data['steps']
            forward_only=data['version'] in (4,7,8)
            maximum_steps=5 if data['version']==6 else 1 if data['version'] in (3,4,7,8) else 3
            translation=(.25 if data['version']==7 else 3. if data['version'] in (4,6)
                         else .6 if data['version'] in (5,8) else .2)
            reserve=.15 if data['version'] in (4,6,8) else .1 if data['version']==5 else .05
            if (self._policy_version is not None and data['version']!=self._policy_version
                    or type(data['maximum_steps']) is not int
                    or data['maximum_steps'] != maximum_steps
                    or data['maximum_translation_m'] != translation or data['stop_reserve_m'] != reserve
                    or not isinstance(steps,list) or not len(steps) <= maximum_steps):
                raise ValueError('policy')
            self._policy_version=data['version']
            self.maximum_speed_mps=speed
            token = data['active_token']
            if token is None:
                if self.token:
                    self.closed.add(self.token)
                self.token = None
                return 'trial_step_not_reserved'
            step = steps[-1]
            if (not isinstance(token,str) or not token or step['token'] != token
                    or step['state'] != 'reserved' or step['reserved_at'] <= self.started_at
                    or not math.isfinite(step['reserved_at']) or step['reserved_at'] > now+.05
                    or token in self.closed):
                raise ValueError('reservation')
            if token != self.token:
                if self.token:
                    # Switching is valid only after the previous backend step
                    # has finished durably; never reset travel by swapping tokens.
                    previous = next(s for s in steps if s['token'] == self.token)
                    if previous['state'] != 'finished':
                        raise ValueError('overlap')
                    self.closed.add(self.token)
                self.token = token
                self.traveled = self.turned = 0.
                self.reason = ''
                self.initial_pose=self.pose
                self.reserved_at=step['reserved_at']
        except (OSError, ValueError, KeyError, TypeError, IndexError, StopIteration):
            if self.token:
                self.reason = 'trial_ledger_invalid'
            return 'trial_ledger_invalid'
        if (self.pose is None or self.stamp is None or self.received is None
                or not -.05 <= now-self.stamp <= .2
                or not 0 <= received_now-self.received <= .2):
            self.reason = self.reason or 'trial_odometry_stale'
        if not all(math.isfinite(v) for v in (linear,angular,duration)) or not 0 < duration <= .25:
            self.reason = self.reason or 'trial_command_invalid'
        if abs(linear) > self.maximum_speed_mps+1e-6 or abs(angular) > .100001:
            self.reason = self.reason or 'trial_speed_exceeded'
        if forward_only:
            if linear < 0 or abs(angular) > .050001:
                self.reason=self.reason or 'trial_forward_only'
            if now-self.reserved_at > (15. if data['version'] in (7,8) else 60.):
                self.reason=self.reason or 'trial_duration_limit'
            if self.pose is not None and self.initial_pose is not None:
                x,y,yaw=self.initial_pose
                dx,dy=self.pose[0]-x,self.pose[1]-y
                lateral_limit=.10 if data['version'] in (7,8) else .15
                if dx*math.cos(yaw)+dy*math.sin(yaw)<-.02 or abs(-dx*math.sin(yaw)+dy*math.cos(yaw))>lateral_limit:
                    self.reason=self.reason or 'trial_forward_deviation'
        # Conservatively include a full outstanding action slice, 0.30s
        # watchdog, 0.20s odometry age and 0.10s scheduling allowance.
        horizon = duration + .6
        if (self.traveled + abs(linear)*horizon + .01 >= translation
                or forward_only and self.traveled >= translation-reserve):
            self.reason = self.reason or 'trial_travel_limit'
        # Field-search goals may make small alternating steering corrections.
        # Summing |delta yaw| treated harmless left/right controller hunting as
        # a 20 degree turn even when wheel odometry stayed within 3 degrees of
        # the initial heading.  The v6 envelope is a maximum *heading
        # excursion* from the reserved pose; v4 remains the stricter
        # cumulative forward-only recheck.
        observed_turn=self.turned
        if (data['version'] in (5,6) and self.pose is not None
                and self.initial_pose is not None):
            observed_turn=abs(math.remainder(
                self.pose[2]-self.initial_pose[2],2*math.pi))
        if observed_turn + abs(angular)*horizon + .02 >= (.15 if forward_only else math.radians(20)):
            self.reason = self.reason or 'trial_turn_limit'
        return self.reason
