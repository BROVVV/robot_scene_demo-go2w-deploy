"""Zero-copy view of a standard ROS signed-byte occupancy array only."""


def int8_grid_view(data):
    """Keep all other inputs on their existing validation/coercion path.

    In particular, an unsigned byte 255 must not become unknown (-1), and a
    float or a wide integer must not be truncated into a valid map cell.
    """
    try:
        view = memoryview(data)
    except TypeError:
        return None
    if (view.ndim != 1 or view.format != 'b' or view.itemsize != 1
            or not view.c_contiguous):
        return None
    import numpy as np
    return np.frombuffer(view, dtype=np.int8)
