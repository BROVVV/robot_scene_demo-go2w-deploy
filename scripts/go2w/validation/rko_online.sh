#!/usr/bin/env bash
# 在线跑 rko 链路：rko_lio + 薄桥接 + 独立 slam_3d_node 实例（不碰 plain_slam 的权威话题）。
set -uo pipefail
cd /home/mxt/robotscene
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE CONDA_SHLVL
source scripts/go2w/setup_environment.sh > /tmp/rko_online_env.log 2>&1
export PYTHONPATH="${HOME}/.local/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
OUT=/tmp/rko_online
rm -rf "$OUT"
mkdir -p "$OUT"

ros2 launch rko_lio odometry.launch.py mode:=online \
  lidar_topic:=/hesai/pandarxt16/points_raw \
  imu_topic:=/go2w/slam/imu \
  base_frame:=base_link odom_frame:=rko_odom \
  config_file:=/tmp/rko_go2w_live.yaml autodetect:=false \
  > "$OUT/rko.log" 2>&1 &
echo $! > "$OUT/rko.pid"
sleep 10

/usr/bin/python3 scripts/go2w/rko_slam_bridge.py --ros-args \
  -p odom_topic:=/rko_lio/odom \
  -p lidar_topic:=/hesai/pandarxt16/points_raw \
  -p pose_topic:=/go2w/rko_slam/imu_pose \
  -p scan_topic:=/go2w/rko_slam/deskewed_scan \
  -p scan_source:=raw \
  -p odom_time_offset_s:=0.0 \
  > "$OUT/bridge.log" 2>&1 &
echo $! > "$OUT/bridge.pid"
sleep 3

ros2 run plain_slam_ros2 slam_3d_node --ros-args \
  --params-file /tmp/rko_slam_params.yaml -r __node:=rko_slam_3d \
  > "$OUT/slam3d.log" 2>&1 &
echo $! > "$OUT/slam3d.pid"

wait
