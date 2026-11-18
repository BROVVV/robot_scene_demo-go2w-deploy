"""Tight raw-frame surface model; never a wide body-frame exclusion box."""
import numpy as np
from scipy.spatial import cKDTree


def measured_low_self_band(points_base, region):
    """Return the measured low wheel/leg band in ``base_link``.

    This geometry is shared by the moving raw-surface extension and the
    structural-map-only exclusion.  Keeping the validation in one place
    prevents either path from growing into an unreviewed near-field mask.
    """
    points_base=np.asarray(points_base,dtype=float)
    if points_base.ndim!=2 or points_base.shape[1]!=3:
        raise ValueError('invalid base-frame points')
    required=('x_abs_max','y_abs_min','y_abs_max','z_min','z_max')
    if not isinstance(region,dict) or any(key not in region for key in required):
        raise ValueError('moving self-return region is incomplete')
    x_abs_max,y_abs_min,y_abs_max,z_min,z_max=(float(region[key]) for key in required)
    if (not 0 < x_abs_max <= .45 or not 0 <= y_abs_min < y_abs_max <= .45
            or not -.6 <= z_min < z_max <= .1):
        raise ValueError('moving self-return region is unsafe')
    finite=np.isfinite(points_base).all(axis=1)
    return (finite&(np.abs(points_base[:,0])<=x_abs_max)
            &(np.abs(points_base[:,1])>=y_abs_min)
            &(np.abs(points_base[:,1])<=y_abs_max)
            &(points_base[:,2]>=z_min)&(points_base[:,2]<=z_max))


class RawSelfReturnFilter:
    def __init__(self, points, radius):
        points=np.asarray(points,dtype=float)
        if (points.ndim!=2 or points.shape[1]!=3 or not len(points)
                or not np.isfinite(points).all() or not 0 < radius <= .03):
            raise ValueError('invalid raw self-return surface model')
        self.tree=cKDTree(points)
        self.radius=radius

    def keep(self, points):
        points=np.asarray(points,dtype=float)
        keep=np.ones(len(points),dtype=bool)
        finite=np.isfinite(points).all(axis=1)
        # Only membership of the reviewed surface radius matters. Bound the
        # exact search so distant environment points do not traverse the whole
        # self model. Include equality at the radius in the search bound.
        bound=np.nextafter(self.radius,np.inf)
        keep[finite]=self.tree.query(points[finite],distance_upper_bound=bound)[0]>self.radius
        return keep

    def keep_moving(self, points, points_base, *, radius, region):
        """Filter gait-shifted returns only inside a measured low self band."""
        points=np.asarray(points,dtype=float)
        points_base=np.asarray(points_base,dtype=float)
        if (points.ndim!=2 or points.shape[1]!=3 or points_base.shape!=points.shape
                or not self.radius <= radius <= .05):
            raise ValueError('invalid moving self-return inputs')
        # The reviewed 3 cm model remains active over the whole cloud.  Only
        # points in the measured low leg band can use the 5 cm gait extension.
        keep=self.keep(points)
        band=measured_low_self_band(points_base,region)&np.isfinite(points).all(axis=1)
        if band.any():
            bound=np.nextafter(radius,np.inf)
            keep[band]=self.tree.query(points[band],distance_upper_bound=bound)[0]>radius
        return keep
