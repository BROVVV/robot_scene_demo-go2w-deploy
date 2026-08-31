# plain_slam_ros2 License Note (Go2-W deployment)

`plain_slam_ros2` (https://github.com/NaokiAkai/plain_slam_ros2) is released by
its author for **academic / personal (non-commercial) use free of charge**.

> Commercial use of plain_slam_ros2 requires the author's written permission.

This project (`robot_scene_demo` / Go2-W deployment) vendors `plain_slam_ros2`
as a dependency:

- pinned commit: see `configs/go2w/plain_slam_lock.yaml`;
- source checkout: `ros2_ws/src/plain_slam_ros2`;
- all Go2-W adaptions live OUTSIDE the upstream package in
  `ros2_ws/src/go2w_plain_slam_bridge` and are Apache-2.0.

Before any commercial deployment, obtain written permission from the upstream
author (NaokiAkai) and keep the permission record with this repository.

No modifications are made to the upstream `plain_slam_ros2` sources in the
first integration version. If upstream bug fixes become unavoidable, they must
be recorded under `patches/plain_slam_ros2/*.patch` with a description.