"""Frozen pre-cache reference; original module SHA256 f547582eef16c8fea2ebf71989101a07bbc815e661edf6124d81c34979ac3903."""
from collections import deque
import math
import numpy as np

class MappingScanAccumulator:
    def __init__(self, *, window_s=.75, minimum_span_s=.60, max_frames=32,
                 max_alignment_error_m=.0125):
        if not (0 < minimum_span_s <= window_s <= 2 and 2 <= max_frames <= 128
                and math.isfinite(max_alignment_error_m) and max_alignment_error_m > 0):
            raise ValueError('invalid mapping accumulation limits')
        self.window_s, self.minimum_span_s = window_s, minimum_span_s
        self.max_frames, self.max_alignment_error_m = max_frames, max_alignment_error_m
        self.frames = deque(maxlen=max_frames)
        self.geometry = None
        self.started = False
        self.status = {'mode': 'warming_up', 'frames': 0}

    def reset(self):
        self.frames.clear()
        self.started = False

    def add(self, ranges, *, stamp, pose, angle_min, angle_increment, range_min, range_max):
        """Return ranges in the CURRENT base pose, or None while initially warming.

        Pose must be odom->base at the acquisition stamp. Maximum endpoint
        displacement across the window is bounded to a quarter of a 5cm cell.
        This does not certify near-field coverage or physical self filtering.
        """
        a = np.asarray(ranges, dtype=float)
        values = (stamp, *pose, angle_min, angle_increment, range_min, range_max)
        if (a.ndim != 1 or not len(a) or not all(math.isfinite(v) for v in values)
                or stamp <= 0 or angle_increment <= 0 or range_min <= 0 or range_max <= range_min):
            self.reset()
            raise ValueError('invalid scan or stamped pose')
        geometry = (len(a), angle_min, angle_increment, range_min, range_max)
        if self.geometry != geometry:
            self.reset()
            self.geometry = geometry
        if self.frames and (stamp <= self.frames[-1][0] or stamp-self.frames[-1][0] > self.window_s):
            self.reset()
        while self.frames and stamp-self.frames[0][0] > self.window_s:
            self.frames.popleft()
        x, y, yaw = pose
        moving = any(math.hypot(x-p[0], y-p[1]) + range_max*abs(math.remainder(yaw-p[2], 2*math.pi))
                     > self.max_alignment_error_m for _,p,_ in self.frames)
        if moving:
            self.frames.clear()
        a = np.where(np.isfinite(a) & (a >= range_min) & (a <= range_max), a, math.inf)
        self.frames.append((stamp, tuple(pose), a.copy()))
        span = stamp-self.frames[0][0]
        full = len(self.frames) >= 2 and span >= self.minimum_span_s
        if not self.started and not full:
            self.status = {'mode':'warming_up', 'frames':len(self.frames), 'span_s':span}
            return None
        if not full:
            self.status = {'mode':'raw_moving_or_refilling', 'frames':len(self.frames), 'span_s':span}
            return a.copy()
        self.started = True
        result = np.full(len(a), math.inf)
        angles = angle_min + np.arange(len(a))*angle_increment
        cy, sy = math.cos(yaw), math.sin(yaw)
        for _, (px,py,pyaw), r in self.frames:
            valid = np.isfinite(r)
            lx,ly = r[valid]*np.cos(angles[valid]), r[valid]*np.sin(angles[valid])
            c,s = math.cos(pyaw),math.sin(pyaw)
            wx,wy = px+c*lx-s*ly-x, py+s*lx+c*ly-y
            nx,ny = cy*wx+sy*wy, -sy*wx+cy*wy
            distance = np.hypot(nx,ny)
            bearing = np.arctan2(ny,nx)
            # Map +/-pi to the represented full-circle interval. Do not wrap
            # a partial-FOV scan into directions that were never measured.
            if len(a)*angle_increment >= 2*math.pi-1e-5:
                bearing = (bearing-angle_min)%(2*math.pi)+angle_min
            bins = np.rint((bearing-angle_min)/angle_increment).astype(int)
            keep = (bins>=0)&(bins<len(a))&(distance>=range_min)&(distance<=range_max)
            np.minimum.at(result,bins[keep],distance[keep])
        # Preserve every current return; compensation quantization must never
        # discard a new obstacle or replace its range by an older farther one.
        result = np.minimum(result,a)
        self.status = {'mode':'accumulated_stationary', 'frames':len(self.frames), 'span_s':span,
                       'current_bins':int(np.isfinite(a).sum()), 'assembled_bins':int(np.isfinite(result).sum())}
        return result
