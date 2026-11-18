"""Limited-distance free evidence from observed ground returns.

Known occupied cells always win. No ray extends beyond its actual measured
floor hit, and no wall-time restamping is involved in this pure operation.
"""
from collections import OrderedDict
import math
import numpy as np


class FloorRayMemory:
    """Bounded, persistent evidence made only from measured floor returns.

    A short wall-time deque makes an observed cell alternate between free and
    unknown while the robot is stationary.  Store one farthest *measured* ray
    per angular bin and 10 cm odometry-origin bucket for the life of the
    mapping process instead.  The downstream fusion still stops at occupied
    cells, so remembered floor evidence can never erase a current obstacle.
    """

    def __init__(self, *, origin_resolution_m=.10, angular_bins=180, max_origins=5000):
        if (not math.isfinite(origin_resolution_m) or origin_resolution_m <= 0
                or not isinstance(angular_bins, int) or angular_bins < 1
                or not isinstance(max_origins, int) or max_origins < 1):
            raise ValueError('invalid floor ray memory configuration')
        self.origin_resolution_m = float(origin_resolution_m)
        self.angular_bins = angular_bins
        self.max_origins = max_origins
        self._origins = OrderedDict()

    def add(self, start, endpoints):
        start = np.asarray(start, dtype=float)
        endpoints = np.asarray(endpoints, dtype=float)
        if (start.shape != (3,) or endpoints.ndim != 2 or endpoints.shape[1] != 3
                or not np.isfinite(start).all() or not np.isfinite(endpoints).all()):
            raise ValueError('invalid measured floor rays')
        key = tuple(np.floor(start[:2] / self.origin_resolution_m).astype(int))
        bins = self._origins.pop(key, {})
        delta = endpoints[:, :2] - start[:2]
        distance = np.linalg.norm(delta, axis=1)
        angle = np.arctan2(delta[:, 1], delta[:, 0])
        bucket = np.floor((angle + math.pi) / (2 * math.pi) * self.angular_bins).astype(int)
        for endpoint, ray_distance, ray_bin in zip(endpoints, distance, bucket):
            if not .10 < ray_distance <= 1.25:
                continue
            ray_bin = int(ray_bin) % self.angular_bins
            previous = bins.get(ray_bin)
            if previous is None or ray_distance > previous[0]:
                bins[ray_bin] = (float(ray_distance), start.copy(), endpoint.copy())
        self._origins[key] = bins
        while len(self._origins) > self.max_origins:
            self._origins.popitem(last=False)

    def rays(self):
        return [(start, endpoint) for bins in self._origins.values()
                for _, start, endpoint in bins.values()]

    @property
    def origin_count(self):
        return len(self._origins)


def measured_floor_endpoints(points, origin, *, angular_bins=180):
    """Select actual endpoints in the horizontal odometry plane.

    Two-degree bins span less than one 5cm cell at the 1.25m range limit.
    The sensor's tilted/rotated raw XY plane is not the ground plane.
    """
    values=np.asarray(points,dtype=float);start=np.asarray(origin,dtype=float)
    if (values.ndim!=2 or values.shape[1]!=3 or start.shape!=(3,)
            or not np.isfinite(start).all() or not isinstance(angular_bins,int)
            or angular_bins<1):
        raise ValueError('invalid measured floor endpoints')
    delta=values[:,:2]-start[:2];distance=np.linalg.norm(delta,axis=1)
    keep=np.isfinite(values).all(axis=1)&(distance>.10)&(distance<=1.25)
    values=values[keep];delta=delta[keep];distance=distance[keep]
    if not len(values):return values
    angle=np.arctan2(delta[:,1],delta[:,0])
    bins=np.floor((angle+math.pi)/(2*math.pi)*angular_bins).astype(int)%angular_bins
    # Sort once instead of scanning the whole return array for each bin.
    # Bin order, farthest measured endpoint and first-return tie breaking
    # remain identical to the per-bin argmax implementation.
    order=np.lexsort((np.arange(len(distance)),-distance,bins))
    ordered_bins=bins[order]
    first=np.r_[True,ordered_bins[1:]!=ordered_bins[:-1]]
    return values[order[first]]


def _ray_cells(start, end, origin, resolution):
    """Visit every intersected cell, including brief corner crossings.

    Fixed-distance samples can jump across an occupied cell at a shallow
    corner and incorrectly clear unknown cells behind it.
    """
    a=(start-origin)/resolution;b=(end-origin)/resolution
    # Keep traversal arithmetic in native double scalars. NumPy scalar
    # comparisons/updates inside every visited cell dominate this small loop.
    ax,ay=map(float,a);bx,by=map(float,b)
    ix,iy=math.floor(ax),math.floor(ay);ex,ey=math.floor(bx),math.floor(by)
    dx,dy=bx-ax,by-ay;sx=(dx>0)-(dx<0);sy=(dy>0)-(dy<0)
    txstep=1/abs(dx) if sx else math.inf
    tystep=1/abs(dy) if sy else math.inf
    tx=((ix+(1 if sx>0 else 0))-ax)/dx if sx else math.inf
    ty=((iy+(1 if sy>0 else 0))-ay)/dy if sy else math.inf
    yield ix,iy
    while (ix!=ex or iy!=ey) and min(tx,ty)<=1.:
        if abs(tx-ty)<=1e-12 and max(tx,ty)<=1.:
            # All three cells touch the ray at the same grid corner. An
            # occupied side cell stops clearing before the diagonal cell.
            yield ix+sx,iy
            yield ix,iy+sy
            ix+=sx;iy+=sy;tx+=txstep;ty+=tystep
        elif tx<ty:
            ix+=sx;tx+=txstep
        else:
            iy+=sy;ty+=tystep
        yield ix,iy


def add_floor_visibility(data, *, width, height, resolution, origin, rays):
    # Cell access is scalar and irregular. Built-in integer lists avoid a
    # NumPy scalar allocation/comparison at each traversal step. Retain the
    # original input/output coercions and independent immutable obstacle grid.
    grid=np.array(data,dtype=np.int16).reshape(height,width).reshape(-1).tolist()
    result=grid.copy()
    if not resolution>0 or not all(math.isfinite(v) for v in (*origin,resolution)):
        raise ValueError('invalid floor-map grid')
    for start,end in rays:
        start=np.asarray(start,dtype=float);end=np.asarray(end,dtype=float)
        if not (np.isfinite(start).all() and np.isfinite(end).all()):continue
        delta=end-start;distance=float(np.linalg.norm(delta))
        margin=max(.10,resolution*1.5)
        # Only repair the observed near field. Distant ground returns do not
        # establish a complete vertical free column along their projection.
        if not margin<distance<=1.25:continue
        # Stop short of the real return to allow for range/pose uncertainty.
        end_short=start+((distance-margin)/distance)*delta
        for column,row in _ray_cells(start,end_short,np.asarray(origin),resolution):
            if not (0<=column<width and 0<=row<height):break
            index=row*width+column
            if grid[index]>=65:break
            if result[index]<0:result[index]=0
    return np.asarray(result,dtype=np.int8).tolist()
