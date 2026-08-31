import math

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

from go2w_sensor_time_bridge.measure_time_sync import point_time_range
from go2w_sensor_time_bridge.time_sync_core import (
    fit_clock,
    fit_cloud_imu_relative_clock,
    transform_seconds,
)


def test_fit_recovers_offset_and_drift():
    scale = 1.0 + 25.0e-6
    offset = 534.25
    samples = []
    for index in range(1300):
        sensor = 1000.0 + index * 0.1
        jitter = math.sin(index) * 0.0002
        samples.append((sensor, scale * sensor + offset + jitter))
    fit = fit_clock(samples)
    assert fit.stable
    assert abs(fit.drift_ppm - 25.0) < 0.1
    assert abs(fit.offset_seconds - offset) < 0.001
    assert fit.fit_rmse_ms < 1.0


def test_short_measurement_is_explicitly_unstable():
    samples = [(float(index), float(index) + 10.0) for index in range(20)]
    fit = fit_clock(samples)
    assert not fit.stable


def test_transform_rejects_invalid_time():
    assert transform_seconds(10.0, 1.0, 5.0) == 15.0


def test_point_time_range_honours_point_stride():
    message = PointCloud2()
    message.height = 1
    message.width = 3
    message.point_step = 32
    message.row_step = 96
    message.fields = [
        PointField(name="time", offset=24, datatype=PointField.FLOAT32, count=1)
    ]
    data = bytearray(message.row_step)
    view = np.ndarray(
        shape=(message.width,),
        dtype="<f4",
        buffer=data,
        offset=24,
        strides=(message.point_step,),
    )
    view[:] = [0.002, 0.001, 0.003]
    message.data = data

    summary = point_time_range(message)

    assert summary["available"]
    assert abs(summary["minimum"] - 0.001) < 1e-8
    assert abs(summary["maximum"] - 0.003) < 1e-8
    assert summary["finite_points"] == 3


def test_relative_clock_fit_does_not_compare_unix_epoch_intercepts():
    epoch = 1_785_980_000.0
    cloud = []
    imu = []
    for index in range(1300):
        receive = epoch + index * 0.1
        cloud.append((receive - 540.060 + index * 0.1e-6, receive))
    for index in range(32500):
        receive = epoch + index * 0.004
        imu.append((receive - 540.000, receive))

    fit = fit_cloud_imu_relative_clock(cloud, imu)

    assert fit.stable
    assert abs(fit.reference_offset_seconds + 0.059935) < 0.001
    assert abs(fit.relative_drift_ppm - 1.0) < 0.1
    assert fit.fit_rmse_ms < 0.01
