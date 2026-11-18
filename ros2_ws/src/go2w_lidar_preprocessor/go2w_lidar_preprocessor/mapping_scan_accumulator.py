"""Bounded near-stationary scan assembly, exclusively for structural mapping.

Missing returns stay unknown. Moving scans are passed through, never assembled
across a large origin change. The collision scan never enters this component.
"""
from collections import deque
import math

import numpy as np


class MappingKeyframeSelector:
    """One stable scan per translation OR rotation keyframe, plus mapper attach."""
    def __init__(self, *, minimum_translation_m=.05, minimum_rotation_rad=.05):
        if not all(math.isfinite(v) and v > 0 for v in (minimum_translation_m, minimum_rotation_rad)):
            raise ValueError('keyframe thresholds must be finite and positive')
        self.minimum_translation_m = minimum_translation_m
        self.minimum_rotation_rad = minimum_rotation_rad
        self.last_pose = None
        self.force_next = True

    def request_initial_scan(self):
        self.force_next = True

    def select(self, pose, *, stable):
        if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
            return False
        # Initial/restarted mappers wait for a dense scan. Afterwards moving
        # keyframes may use the fresh raw scan, as the original mapper did;
        # history is NEVER fused across motion.
        if not stable and (self.last_pose is None or self.force_next):
            return False
        if self.last_pose is None:
            moved, turned = math.inf, math.inf
        else:
            moved = math.hypot(pose[0]-self.last_pose[0],pose[1]-self.last_pose[1])
            turned = abs(math.remainder(pose[2]-self.last_pose[2],2*math.pi))
        if self.force_next or moved >= self.minimum_translation_m or turned >= self.minimum_rotation_rad:
            self.last_pose = tuple(pose)
            self.force_next = False
            return True
        return False


class MappingScanAccumulator:
    def __init__(self, *, window_s=.75, minimum_span_s=.60, max_frames=32,
                 max_alignment_error_m=.0125):
        if not (0 < minimum_span_s <= window_s <= 2 and 2 <= max_frames <= 128
                and math.isfinite(max_alignment_error_m) and max_alignment_error_m > 0):
            raise ValueError('invalid mapping accumulation limits')
        self.window_s, self.minimum_span_s = window_s, minimum_span_s
        self.max_frames, self.max_alignment_error_m = max_frames, max_alignment_error_m
        self.frames = deque(maxlen=max_frames)
        # Historical scans and their stamped poses are immutable. Keep their
        # world endpoints, but always reproject into the CURRENT input pose.
        self._world_endpoints = {}
        self._angles = None
        self.geometry = None
        self.started = False
        self.status = {'mode': 'warming_up', 'frames': 0}

    def reset(self):
        self.frames.clear()
        self._world_endpoints.clear()
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
            self._angles = None
        if self.frames and (stamp <= self.frames[-1][0] or stamp-self.frames[-1][0] > self.window_s):
            self.reset()
        while self.frames and stamp-self.frames[0][0] > self.window_s:
            expired, _, _ = self.frames.popleft()
            self._world_endpoints.pop(expired, None)
        x, y, yaw = pose
        moving = any(math.hypot(x-p[0], y-p[1]) + range_max*abs(math.remainder(yaw-p[2], 2*math.pi))
                     > self.max_alignment_error_m for _,p,_ in self.frames)
        if moving:
            self.frames.clear()
            self._world_endpoints.clear()
        a = np.where(np.isfinite(a) & (a >= range_min) & (a <= range_max), a, math.inf)
        if len(self.frames) == self.max_frames:
            expired, _, _ = self.frames.popleft()
            self._world_endpoints.pop(expired, None)
        saved = a.copy()
        saved.flags.writeable = False
        self.frames.append((stamp, tuple(pose), saved))
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
        if self._angles is None:
            self._angles = angle_min + np.arange(len(a))*angle_increment
            self._angles.flags.writeable = False
        angles = self._angles
        cy, sy = math.cos(yaw), math.sin(yaw)
        for frame_stamp, (px,py,pyaw), r in self.frames:
            endpoints = self._world_endpoints.get(frame_stamp)
            if endpoints is None:
                valid = np.isfinite(r)
                lx,ly = r[valid]*np.cos(angles[valid]), r[valid]*np.sin(angles[valid])
                c,s = math.cos(pyaw),math.sin(pyaw)
                endpoints = (px+c*lx-s*ly, py+s*lx+c*ly)
                for value in endpoints:
                    value.flags.writeable = False
                self._world_endpoints[frame_stamp] = endpoints
            wx,wy = endpoints[0]-x, endpoints[1]-y
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
