"""Client for the independent velocity bridge's per-goal wheel counter.

No session duration/step allowance: each explicit goal gets a new handshake.
Only the ROS factory wires this client; dry runs and durable trials keep their
own contracts. No path length or map pose can reset a running wheel counter.
"""
import math
import threading
import time
import uuid


class GoalTravelError(RuntimeError):
    pass


class GoalTravelBudget:
    def __init__(self, send, *, now=time.time, monotonic=time.monotonic):
        self._send,self._now,self._monotonic = send,now,monotonic
        self._lock = threading.Lock()
        self._status = None
        self._received = -math.inf
        self.request = None
        self._started = self._phase_started = 0.
        self._acknowledged = False
        self.reason = ''

    def update(self, status):
        with self._lock:
            self._status = dict(status) if isinstance(status,dict) else None
            self._received = self._monotonic()

    def _read(self):
        with self._lock:
            status = dict(self._status) if self._status else {}
            received = self._received
        try:
            valid = (status['version'] == 1 and isinstance(status['epoch'],str)
                and status['epoch'] and 0 <= self._monotonic()-received <= .5
                and -.05 <= self._now()-float(status['stamp']) <= .5)
        except (KeyError,ValueError,TypeError,OverflowError):
            valid = False
        return status if valid else None

    def _fail(self, reason):
        self.reason = self.reason or reason
        raise GoalTravelError(self.reason)

    def _publish(self, phase):
        self.request.update(phase=phase,stamp=self._now())
        self._send(dict(self.request))

    def prepare(self, goal_id, limit_m):
        if not math.isfinite(limit_m) or not 0 < limit_m <= 3.:
            self._fail('goal_travel_invalid_limit')
        status = self._read()
        if self.request is None:
            if status is None or status.get('ready') is not True or status.get('token') is not None:
                return None
            self.reason = ''
            self._started = self._phase_started = self._monotonic()
            self._acknowledged = False
            self.request = dict(version=1,epoch=status['epoch'],token=uuid.uuid4().hex,
                goal_id=goal_id,created=self._now(),limit_m=limit_m)
            self._publish('prepare')
            return None
        if self.request['goal_id'] != goal_id or self.request['limit_m'] != limit_m:
            self._fail('goal_travel_overlapping_goal')
        self._publish('prepare')
        return self._ack(status,'prepared')

    def activate(self):
        if self.request['phase'] != 'run':
            self._phase_started = self._monotonic()
            self._acknowledged = False
        return self.check()

    def check(self):
        if self.request is None:
            self._fail('goal_travel_not_reserved')
        self._publish('run')
        return self._ack(self._read(),'running')

    def _ack(self, status, phase):
        if self.reason:
            raise GoalTravelError(self.reason)
        if status is not None:
            if status['epoch'] != self.request['epoch']:
                self._fail('goal_travel_bridge_restarted')
            if status.get('token') == self.request['token']:
                if status.get('reason'):
                    self._fail(str(status['reason']))
                if status.get('limit_m') != self.request['limit_m']:
                    self._fail('goal_travel_limit_mismatch')
                if status.get('phase') == phase:
                    self._acknowledged = True
                    return True
            elif self._acknowledged:
                self._fail('goal_travel_reservation_lost')
        if self._acknowledged or self._monotonic()-self._phase_started > .5:
            self._fail('goal_travel_ack_stale')
        return None

    def snapshot(self):
        status = self._read()
        if status and self.request and status.get('token') == self.request['token']:
            return {k:status.get(k) for k in ('traveled_m','limit_m','reason','epoch')}
        return {}

    def finish(self):
        if self.request is not None:
            try:
                self._publish('close')
            finally:
                self.request = None
                self._acknowledged = False
