#!/usr/bin/env bash
# plain_slam 点云缓存目录的磁盘保护。
#
# 为什么需要：slam_3d_node 每加一个关键帧就往 map_cloud_dir 写
# raw_cloudN.bin + filtered_cloudN.bin，而且**从不删除** —— 回环检测
# （slam_3d_interface.cpp GetTargetCandidateIndices）和 BuildMapCloud() 都要
# 读回这些文件，删旧文件等于破坏建图。唯一的清理由是建图收尾的
# WriteMapCloudPCD()。start 脚本只在启动时 rm -rf 一次，所以会话跑得够久，
# 这个目录就能把根文件系统写到 100%（2026-09-11 实测 47G，/tmp 与 / 同盘）。
#
# 本脚本不碰建图代码、不删任何缓存文件，只在越界时有序停掉 mapping 栈
# （复用 stop_plain_slam_mapping.sh，它不动 /go2w/odom/fused 和运动栈）。
# 恢复建图：重跑 start_plain_slam_mapping.sh，它会清空缓存目录。
#
# 阈值（环境变量，可从 start 脚本透传）：
#   GO2W_SLAM_MAP_CACHE_DIR     缓存目录，默认 /tmp/go2w_plain_slam
#   GO2W_SLAM_DISK_MAX_CACHE_GB 缓存目录上限，默认 20
#   GO2W_SLAM_DISK_MIN_FREE_GB  磁盘剩余下限，默认 20
#   GO2W_SLAM_DISK_CHECK_SECONDS 检查周期，默认 30
#   GO2W_SLAM_DISK_GUARD_PIDFILE 把本进程 pid 写到这里（给 stop 脚本用）
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cache_dir="${GO2W_SLAM_MAP_CACHE_DIR:-/tmp/go2w_plain_slam}"
max_cache_gb="${GO2W_SLAM_DISK_MAX_CACHE_GB:-20}"
min_free_gb="${GO2W_SLAM_DISK_MIN_FREE_GB:-20}"
interval_s="${GO2W_SLAM_DISK_CHECK_SECONDS:-30}"

# pid 自己写，不用调用方的 $!：`setsid ... &` 在开了 job control 的 shell 里
# 会 fork（setsid 当上进程组组长就必须 fork），$! 拿到的是 fork 完就退出的那个
# 父进程，存进 pid 文件就是个死 pid —— stop 脚本会因此永远停不掉本守护。
if [[ -n "${GO2W_SLAM_DISK_GUARD_PIDFILE:-}" ]]; then
  printf '%s\n' "$$" >"${GO2W_SLAM_DISK_GUARD_PIDFILE}"
fi

max_cache_b=$(( max_cache_gb * 1024 * 1024 * 1024 ))
min_free_b=$(( min_free_gb * 1024 * 1024 * 1024 ))

# 剩余空间按缓存目录所在的分区算；目录还没建起来时退回它最近的父目录。
free_probe="${cache_dir}"
while [[ ! -d "${free_probe}" && "${free_probe}" != "/" ]]; do
  free_probe="$(dirname -- "${free_probe}")"
done

to_gb() { awk -v b="$1" 'BEGIN { printf "%.1f", b / 1073741824 }'; }

printf 'disk guard: cache=%s 上限=%sGB 剩余下限=%sGB 周期=%ss 分区=%s\n' \
  "${cache_dir}" "${max_cache_gb}" "${min_free_gb}" "${interval_s}" "${free_probe}"

while true; do
  cache_b="$(du -sb -- "${cache_dir}" 2>/dev/null | cut -f1 || true)"
  free_b="$(df -B1 --output=avail -- "${free_probe}" 2>/dev/null | tail -n 1 || true)"
  cache_b="${cache_b:-0}"
  free_b="${free_b:-0}"

  if (( cache_b > max_cache_b || free_b < min_free_b )); then
    printf '[TRIPPED] %s\n' "$(date -Is)"
    printf '  %s = %s GB（上限 %s GB）\n' \
      "${cache_dir}" "$(to_gb "${cache_b}")" "${max_cache_gb}"
    printf '  %s 可用 = %s GB（下限 %s GB）\n' \
      "${free_probe}" "$(to_gb "${free_b}")" "${min_free_gb}"
    printf '%s\n' '  停止 mapping 栈以保住磁盘（缓存文件保留，下次 start 会清空）。'
    # 告诉 stop 脚本别把我们自己杀掉，好让上面的结论留在日志里。
    GO2W_SLAM_DISK_GUARD_TRIPPED=1 \
      bash "${script_dir}/stop_plain_slam_mapping.sh" || true
    printf '%s\n' '[TRIPPED] mapping 栈已停止；重跑 start_plain_slam_mapping.sh 恢复。'
    exit 0
  fi

  sleep "${interval_s}"
done
