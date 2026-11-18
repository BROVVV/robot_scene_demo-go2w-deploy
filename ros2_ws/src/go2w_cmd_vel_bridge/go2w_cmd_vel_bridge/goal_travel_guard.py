"""Per-goal wheel travel guard, independent of Nav2 paths and map corrections.

The bridge owns the epoch and counters. Preparing a goal cannot move; running
requires an acknowledged prepare. Heartbeat expiry and odometry faults latch
until explicit close. New goals require a stationary, fresh wheel stream.
"""
import math
import uuid


class GoalTravelGuard:
    MAXIMUM_M = 3.0
    LEASE_S = .5
    PREPARE_S = 3.0
    STOP_RESERVE_M = .10

    def __init__(self):
        self.epoch = uuid.uuid4().hex
        self.token = None
        self.phase = 'idle'
        self.reason = ''
        self.traveled = 0.
        self.limit = self.MAXIMUM_M
        self.pose = None
        self.stamp = self.received = self.quiet_since = None
        self.heartbeat = self.request_stamp = None
        self.last_created = -math.inf

    def _fault(self, reason):
        if self.token:
            self.reason = self.reason or reason
        return reason

    def odometry(self, *, x, y, yaw, frame, child, stamp, received):
        if (not all(math.isfinite(v) for v in (x,y,yaw,stamp,received))
                or frame != 'odom_fused' or child != 'base_link'
                or self.stamp is not None and stamp <= self.stamp):
            self._fault('goal_travel_odometry_invalid')
            self.pose = self.quiet_since = None
            return
        if self.pose is not None:
            dt = stamp-self.stamp
            distance = math.hypot(x-self.pose[0], y-self.pose[1])
            delta_yaw = yaw-self.pose[2]
            if not all(math.isfinite(v) for v in (dt,distance,delta_yaw)):
                self._fault('goal_travel_odometry_invalid')
                self.pose = self.quiet_since = None
                return
            turn = abs(math.remainder(delta_yaw, 2*math.pi))
            if self.token:
                if dt > .2 or distance > .6*dt+.01 or turn > 1.*dt+.02:
                    self._fault('goal_travel_odometry_discontinuity')
                    self.pose = self.quiet_since = None
                    return
                self.traveled += distance
                if self.traveled >= self.limit-self.STOP_RESERVE_M:
                    self._fault('goal_travel_limit')
            quiet = dt <= .2 and distance <= .02*dt and turn <= .05*dt
            if not quiet:
                self.quiet_since = None
            elif self.quiet_since is None:
                self.quiet_since = received
        self.pose = (x,y,yaw)
        self.stamp,self.received = stamp,received

    def _fresh(self, now, received_now):
        return (self.pose is not None and self.stamp is not None and self.received is not None
                and -.05 <= now-self.stamp <= .2
                and 0 <= received_now-self.received <= .2)

    def _expire(self, now, received_now):
        if self.token:
            if not self._fresh(now, received_now):
                self._fault('goal_travel_odometry_stale')
            ttl = self.PREPARE_S if self.phase == 'prepared' else self.LEASE_S
            if (not 0 <= received_now-self.heartbeat <= ttl
                    or not -.05 <= now-self.request_stamp <= ttl):
                self._fault('goal_travel_heartbeat_stale')

    def request(self, data, *, now, received_now, can_begin=True):
        self._expire(now,received_now)
        try:
            token = data['token']
            phase = data['phase']
            stamp = float(data['stamp'])
            limit = float(data['limit_m'])
            created = float(data['created'])
            if (data['version'] != 1 or data['epoch'] != self.epoch
                    or not isinstance(token,str) or not token or len(token)>128
                    or phase not in ('prepare','run','close')
                    or not all(math.isfinite(v) for v in (stamp,limit,created))
                    or not -.05 <= now-stamp <= self.LEASE_S
                    or not 0 < limit <= self.MAXIMUM_M):
                raise ValueError('invalid request')
        except (KeyError,TypeError,ValueError,OverflowError):
            return self._fault('goal_travel_request_invalid')
        if phase == 'close':
            if token == self.token:
                self.token = None
                self.phase = 'idle'
                self.reason = ''
            return ''
        if self.token is None:
            if (phase != 'prepare' or created <= self.last_created
                    or not -.05 <= now-created <= self.LEASE_S
                    or not can_begin or not self._fresh(now,received_now)
                    or self.quiet_since is None or received_now-self.quiet_since < .25):
                return 'goal_travel_not_ready'
            self.token,self.phase = token,'prepared'
            self.limit,self.traveled,self.reason = limit,0.,''
            self.last_created = created
        elif token != self.token or created != self.last_created or limit != self.limit:
            return self._fault('goal_travel_overlapping_goal')
        if self.reason:
            return self.reason
        # Reliable DDS preserves publisher order; a regressing heartbeat must
        # never extend a lease or rewind running into prepared.
        if self.request_stamp is not None and stamp < self.request_stamp:
            return self._fault('goal_travel_request_regressed')
        if phase == 'prepare' and self.phase == 'running':
            return self._fault('goal_travel_phase_regressed')
        self.heartbeat,self.request_stamp = received_now,stamp
        if phase == 'run':
            self.phase = 'running'
        return ''

    def check(self, *, now, received_now, linear, angular, duration):
        self._expire(now,received_now)
        if not self.token:
            return 'goal_travel_not_reserved'
        if self.reason:
            return self.reason
        if self.phase != 'running':
            return 'goal_travel_not_running'
        if (not all(math.isfinite(v) for v in (linear,angular,duration))
                or not 0 < duration <= .25 or abs(linear) > .300001 or abs(angular) > .200001):
            return self._fault('goal_travel_command_invalid')
        # Include a whole outstanding slice, wheel age, bridge/watchdog
        # scheduling, and a fixed reserve. Actual braking remains a field test.
        predicted = self.traveled + abs(linear)*(duration+.6)
        if predicted >= self.limit-self.STOP_RESERVE_M:
            return self._fault('goal_travel_limit')
        return ''

    def status(self, *, now, received_now):
        self._expire(now,received_now)
        return dict(version=1,epoch=self.epoch,token=self.token,phase=self.phase,
            reason=self.reason, traveled_m=self.traveled,limit_m=self.limit,
            ready=(self.token is None and self._fresh(now,received_now)
                   and self.quiet_since is not None and received_now-self.quiet_since >= .25),
            stamp=now)
