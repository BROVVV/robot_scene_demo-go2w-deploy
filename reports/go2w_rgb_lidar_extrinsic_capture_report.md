# Go2-W RGB–LiDAR extrinsic capture report

Status date: 2026-08-06 (Asia/Shanghai)  
Result: **CAPTURE PASS / CALIBRATION TARGET BLOCKED**  
Safety: the robot remained stationary and no motion command was sent.

## Captured evidence

The read-only camera and aligned LiDAR cloud were recorded in six physical
target poses. All accepted bags closed cleanly and all extracted image/cloud
pairs were inside the 50 ms synchronization gate.

| Pose | Camera board pose | Bag duration | Image / cloud count | Maximum extracted delta |
|---|---:|---:|---:|---:|
| near | 0.648 m, approximately frontal | 20.087 s | 362 / 309 | 24.643 ms |
| medium | 1.000 m, approximately frontal | 20.082 s | 271 / 309 | 39.188 ms |
| far | 1.303 m, approximately frontal | 20.092 s | 358 / 309 | 33.316 ms |
| tilt left | 0.883 m, normal yaw -30.8 deg | 20.081 s | 344 / 309 | 31.787 ms |
| tilt right 01 | 0.843 m, normal yaw +49.1 deg | 20.098 s | 213 / 309 | 45.320 ms |
| tilt right 02 | 0.900 m, normal yaw +30.8 deg | 20.079 s | 340 / 307 | 47.747 ms |

The +49.1 degree pose was rejected for calibration because the target region
had no usable LiDAR returns under the official-pose projection. It is retained
as rejection evidence. The replacement +30.8 degree pose restored 138 returns
inside the broad image target ROI in the inspected frame, but multi-pose LiDAR
change analysis showed that those returns were not a stable target plane.

The enhanced extractor preserves `intensity`, `ring`, and per-point `time`
beside the index-stable XYZ array. Its v2 datasets have unique scene labels and
explicit near/medium/far bands, so future overlays cannot overwrite one
another.

## Why no extrinsic was installed

Camera PnP was healthy in every pose: all 54 corners were detected, with the
physical 15 mm square size. The failure is target observability in the L2
cloud. The small paper board was attached to a black/patterned box and backed
by other nearly coplanar boxes. It therefore did not create a repeatable,
separable LiDAR plane or perimeter.

Across the five retained pose datasets, 1,686--2,368 persistent LiDAR voxels
were observed per pose. After removing voxels also occupied in other poses,
the medium and replacement-right poses retained only 57 and 17 distinct
voxels. RANSAC candidates were dominated by floor, backing-box, and background
planes, and no selection produced one physically plausible rigid transform
across the frontal, left-tilted, and right-tilted target normals.

A deliberately non-authoritative structural-edge prototype was also rejected:
its held-out mean-of-scene edge distance improved only from 33.237 px to
32.939 px, while the project gate is 5 px. No value from that prototype was
written to production configuration.

Consequently:

- `configs/go2w/sensor_extrinsics.yaml` remains `uncalibrated` and
  `confirmed: false`;
- RGB–LiDAR fusion remains fail-closed;
- the result authorizes neither fusion nor motion;
- the moved-position recheck is not attempted because robot movement is
  prohibited.

## Required replacement target

Prepare one rigid, matte-white planar board at least **0.60 x 0.60 m** (larger
is preferable for the sparse L2 scan). Mount the existing measured checkerboard
flat at its center. Add 30--50 mm retroreflective tape patches at the four
outer board corners if available. Place the board with at least 0.30 m of empty
depth separation from walls, boxes, and chairs; do not mount it on the current
Lenovo box stack.

The next stationary capture should use at least five poses: frontal near,
frontal medium, frontal far, yaw about -30 degrees, and yaw about +30 degrees.
The robot must remain stationary. A moved-robot validation remains explicitly
blocked by the user's no-motion constraint, even if a candidate passes the
stationary overlays.

## 2026-08-06 new-box re-check and frame-direction finding

A replacement cardboard box with the same 9x6, 15 mm checkerboard was checked
with a fresh 15 s stationary bag
(`outputs/calibration/extrinsic_newbox_front_01/rosbag`). Camera PnP still
passes cleanly (mean 0.298 px, max 0.794 px; board center 0.736 m ahead of the
camera, ~0.096 m below the optical axis).

The L2 published cloud covers elevations only from about -1.15 deg to
+89.6 deg, and the paper (7-8 deg below the camera axis at 0.75 m) sits below
the lidar horizon. The lidar therefore returns only a thin stable band from the
box upper edge (~0.9-1.2 m in front, z 0.50-0.66 m in the raw z-up frame), not
the paper plane. The current placement still fails the plane-based
observability gate; the box must be raised so the paper center is at least
0.15-0.25 m above the lidar horizon.

Additionally, the raw `/utlidar/cloud` and `/utlidar/imu` indicate a z-up
published frame (IMU static acceleration ~(2.36, 0.04, 9.82) m/s^2), while the
pinned official base->lidar pitch of 2.8782 rad flips it upside down. The
earlier stationary "ground plane" audit and the old six-pose failure analysis
were performed in that flipped frame and need a short read-only direction
re-validation before any future calibration is trusted. No configuration value
was changed by this re-check.
