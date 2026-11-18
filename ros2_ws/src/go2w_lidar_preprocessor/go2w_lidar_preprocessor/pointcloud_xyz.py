"""Read named XYZ from PointCloud2 bytes without a per-point generator.

No ROS dependency; preserve point order, NaNs, byte order and row padding.
The returned float64 array is independent of the unchanged input message.
"""
import operator

import numpy as np


# sensor_msgs/PointField scalar datatype IDs, shared by Foxy and Humble.
_TYPES = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2',
          5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def _index(value, name):
    try:
        result = operator.index(value)
    except TypeError as error:
        raise ValueError('invalid PointCloud2 '+name) from error
    if result < 0:
        raise ValueError('negative PointCloud2 '+name)
    return result


def read_xyz(message):
    """Return all named scalar XYZ values as a row-major (height*width, 3) array.

    Raise ValueError for malformed layouts so the caller can revoke freshness.
    No filtering, range adjustment or coordinate transform happens here.
    """
    width = _index(message.width, 'width')
    height = _index(message.height, 'height')
    point_step = _index(message.point_step, 'point_step')
    row_step = _index(message.row_step, 'row_step')
    if not width or not height:
        return np.empty((0, 3), dtype=np.float64)
    if point_step == 0 or row_step < width*point_step:
        raise ValueError('PointCloud2 strides do not contain complete rows')
    try:
        buffer = memoryview(message.data)
    except TypeError as error:
        raise ValueError('PointCloud2 data has no byte buffer') from error
    if not buffer.contiguous or buffer.nbytes < height*row_step:
        raise ValueError('PointCloud2 data is truncated or noncontiguous')
    fields = {}
    endian = '>' if message.is_bigendian else '<'
    for field in message.fields:
        if field.name not in ('x', 'y', 'z'):
            continue
        if field.name in fields:
            raise ValueError('duplicate PointCloud2 '+field.name+' field')
        datatype = _index(field.datatype, 'field datatype')
        count = _index(field.count, 'field count')
        offset = _index(field.offset, 'field offset')
        if datatype not in _TYPES or count != 1:
            raise ValueError('unsupported PointCloud2 '+field.name+' scalar layout')
        dtype = np.dtype(endian+_TYPES[datatype])
        if offset+dtype.itemsize > point_step:
            raise ValueError('PointCloud2 '+field.name+' extends beyond point_step')
        fields[field.name] = (dtype, offset)
    if set(fields) != {'x', 'y', 'z'}:
        raise ValueError('PointCloud2 must contain named x, y and z fields')
    dtype = np.dtype({'names': ['x', 'y', 'z'],
                      'formats': [fields[name][0] for name in ('x', 'y', 'z')],
                      'offsets': [fields[name][1] for name in ('x', 'y', 'z')],
                      'itemsize': point_step})
    view = np.ndarray((height, width), dtype=dtype, buffer=buffer,
                      strides=(row_step, point_step))
    return np.column_stack([view[name].reshape(-1) for name in ('x', 'y', 'z')]).astype(
        np.float64, copy=False)
