#!/usr/bin/env bash
# 实体遥控器主动建图：一键启动（只读软件，机器狗由 Unitree 实体遥控器操作）。
#
# 计划书：docs/GO2W_遥控主动建图与周期物体落图_MVP实施计划_20260918.md §6.2
#
# 启动顺序（计划书 §6.2 A–F）：
#   A. 环境与传感器预检（只读，不查询也不修改运动控制权）
#   B. 复用或启动 plain_slam（已有健康发布者就复用，绝不启第二套）
#   C. 创建本次语义会话（归档上一份 events.jsonl / status.json）
#   D. 启动语义侧车 remote_control_semantic_mapping_worker.py
#   E. 启动地图快照写入器 webui_map_snapshot_writer.py
#   F. 以只读模式启动现有 WebUI（MANUAL_DEMO_DISPLAY_ONLY=1）
#
# 这个脚本不发送任何运动命令，也不调用任何 Unitree 运动 API：
# 不发布 /go2w/motion、cmd_vel 或 /wirelesscontroller，不调用 arm/stop/estop，
# 不启动自主搜索 / Nav2 / frontier。侧车内的 PlaceGraph 只记录真实经过的
# 稀疏观察点供综合拓扑显示，不参与运动。进程只读话题、只写自己的
# outputs/remote_control_mapping/。停止机器狗请用实体遥控器。
#
# 选项：
#   --map-mode 2d|3d  地图模式（默认 2d）
#   --port N           WebUI 端口（默认 8766；演示台默认 8765 已被 WASD 占用）
#   --no-web           不起 WebUI（只跑侧车 + 快照写入器）
#   --no-browser       不自动打开浏览器
#   --force-new-slam   即使已有 plain_slam 也重开一个 SLAM session（默认不）
#   --no-slam-start    没有 plain_slam 时直接报错，不代为启动
#   --smoke            只做计划书 §10.2 的单轮无运动烟雾检查，然后退出
#   --smoke-target TXT 配合 --smoke：候选出现后最多做一次真实 VLM 复核
#                      （不启 SLAM / 快照 / WebUI；机器狗必须处于静止）
#   --detector NAME    grounded_sam（默认）或 yoloe_seg
#   --debug            详细输出
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
# 所有子进程都在仓库根下跑：.env、相对于 cwd 的配置路径和 `sys.path.insert(0,".")`
# 都按这个前提写。
cd "${project_root}"

pid_root="${project_root}/runtime/go2w/pids"
slam_session_log="${project_root}/runtime/go2w/sessions/plain_slam_go2w.launch.log"

# 本模式的产物全部落在这里，与 WASD 演示台（outputs/manual_web_demo/）和自主搜索
# （outputs/live_runs/）分开。
runtime_root="${project_root}/outputs/remote_control_mapping"
runtime_dir="${runtime_root}/runtime"
log_root="${runtime_root}/logs"
mkdir -p "${runtime_root}" "${runtime_dir}" "${log_root}" "${pid_root}"

events_path="${runtime_dir}/events.jsonl"
status_path="${runtime_dir}/status.json"
mapper_pidfile="${runtime_dir}/semantic_mapper.pid"
writer_pidfile="${runtime_dir}/snapshot_writer.pid"
web_pidfile="${runtime_dir}/web.pid"
# ownership marker：只有**本脚本**启动的 plain_slam，停止脚本才会去停。
slam_owner_marker="${runtime_dir}/slam_started_by_remote_mapping"

# 地图快照为什么不写 WebUI 默认的 outputs/autonomous_search/runtime/：
# 那个文件由自主搜索地图桥写；手动演示也有独立目录。遥控建图同样使用
# 自己的目录，避免任何两个模式交替覆盖 map_epoch 和 mapping_health。
# 本模式给 WebUI 换一个独立的快照路径，前端协议和 slam_map_3d.js 完全不动，
# 只是不再和演示台抢同一个文件。用 REMOTE_MAPPING_SNAPSHOT_PATH 可覆盖。
snapshot_path="${REMOTE_MAPPING_SNAPSHOT_PATH:-${runtime_dir}/slam_map_3d.json}"
# 计划书 §2.3：地图元数据与对象快照同样留在本模式的独立目录里，和快照同一个
# 理由 —— 不与演示台/自主搜索抢同一个文件。写端（快照写入器）、对象生产者
# （语义侧车）和读取端（本模式自己的 WebUI）用的是这三个显式路径。
meta_path="${REMOTE_MAPPING_META_PATH:-${runtime_dir}/slam_map_meta.json}"
objects_path="${REMOTE_MAPPING_OBJECTS_PATH:-${runtime_dir}/semantic_objects.json}"
topology_path="${REMOTE_MAPPING_TOPOLOGY_PATH:-${runtime_dir}/semantic_topology.json}"
target_watch_request="${GO2W_TARGET_WATCH_REQUEST:-${runtime_dir}/target_watch_request.json}"
target_watch_state="${GO2W_TARGET_WATCH_STATE:-${runtime_dir}/target_watch_state.json}"
target_watch_evidence_dir="${GO2W_TARGET_WATCH_EVIDENCE_DIR:-${runtime_dir}/target_watch_evidence}"
vlm_socket="${project_root}/runtime/go2w/siliconflow_vlm.sock"
vlm_owner_marker="${runtime_dir}/vlm_started_by_remote_mapping"
mapping_config="${REMOTE_MAPPING_CONFIG:-${project_root}/configs/go2w/semantic_mapping.yaml}"
web_port_default="${REMOTE_MAPPING_WEB_PORT:-8766}"

web_port="${web_port_default}"
map_mode="${REMOTE_MAPPING_MAP_MODE:-2d}"
start_web=1
open_browser=1
force_new_slam=0
allow_slam_start=1
smoke=0
smoke_target=""
debug=0
detector_cli=""
while (( $# )); do
  case "$1" in
    --map-mode) map_mode="${2:?--map-mode 需要 2d 或 3d}"; shift 2 ;;
    --port) web_port="${2:?--port 需要参数}"; shift 2 ;;
    --no-web) start_web=0; shift ;;
    --no-browser) open_browser=0; shift ;;
    --force-new-slam) force_new_slam=1; shift ;;
    --no-slam-start) allow_slam_start=0; shift ;;
    --smoke) smoke=1; shift ;;
    --smoke-target) smoke_target="${2:?--smoke-target 需要自然语言目标}"; shift 2 ;;
    --detector) detector_cli="${2:?--detector 需要 grounded_sam 或 yoloe_seg}"; shift 2 ;;
    --debug) debug=1; shift ;;
    *) printf 'ERROR: 未知选项: %s\n' "$1" >&2; exit 2 ;;
  esac
done

if [[ -n "${smoke_target}" && "${smoke}" != "1" ]]; then
  printf 'ERROR: --smoke-target 只能与 --smoke 一起使用\n' >&2
  exit 2
fi

if [[ "${debug}" == "1" ]]; then
  export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
fi

step() { printf '\n== %s\n' "$1"; }
ok()   { printf '[OK]   %s\n' "$1"; }
warn() { printf '[WARN] %s\n' "$1" >&2; }
die()  { printf '[FAIL] %s\n' "$1" >&2; exit 2; }
[[ "${map_mode}" == "2d" || "${map_mode}" == "3d" ]] || die '--map-mode 只能是 2d 或 3d'
if [[ "${GO2W_PERCEPTION_MODE:-full}" == "2d" && "${map_mode}" != "2d" ]]; then
  die '当前运行档只允许二维建图；恢复三维前请先明确修改运行档。'
fi
printf '%s\n' "${map_mode}" >"${runtime_dir}/map_mode"

# ---------------------------------------------------------------------------
# A. 环境与传感器预检
# ---------------------------------------------------------------------------
step 'A. 环境与传感器预检'

set +u
# shellcheck disable=SC1091
source "${script_dir}/setup_environment.sh"
set -u

if [[ -z "${CYCLONEDDS_URI:-}" || -z "${GO2W_HOST_IP:-}" ]]; then
  die 'DDS 环境没有钉到机器狗侧地址（setup_environment.sh 未生效）。'
fi
ok "DDS: interface=${GO2W_INTERFACE} address=${GO2W_HOST_IP}"

if ! command -v ros2 >/dev/null 2>&1; then
  die 'source 之后仍然找不到 ros2 CLI。'
fi

# 演示台用的 conda Python（有 dotenv、fastapi，也是唯一装了 torch 的那个）。
conda_python=""
for candidate in \
  "${GO2W_CONDA_PYTHON:-}" \
  "$HOME/anaconda3/envs/go2_robot_scene_demo/bin/python" \
  "$HOME/miniconda3/envs/go2_robot_scene_demo/bin/python" \
  /home/mxt/anaconda3/envs/go2_robot_scene_demo/bin/python \
  "${CONDA_PREFIX:-}/bin/python"; do
  if [[ -n "${candidate}" && -x "${candidate}" ]]; then
    conda_python="${candidate}"
    break
  fi
done
[[ -n "${conda_python}" ]] || die '找不到 go2_robot_scene_demo 的 Python。'

# .env 必须先灌进环境变量再起子进程 —— 这一步不是可选的。
#
# 侧车要 rclpy，只能跑在系统 Python 下；而系统 Python **没有 python-dotenv**，
# app/config.py 顶部的 try/except 会让 load_dotenv 静默退化成空操作。于是
# get_settings() 返回的是默认值而不是 .env：GROUNDED_SAM_PYTHON 停在 "python"、
# GROUNDED_SAM_PYTHONPATH 停在 /root/gpufree-data/...（本机根本不存在）。侧车的
# 解释器回退链接着会挑到 .venv/bin/python —— 那个环境**没有 torch**，检测器
# 子进程起不来，每一帧都失败，页面只显示 degraded，没有任何一处指向真正的原因。
#
# 本仓既有的做法是让长驻父进程（WebUI，跑在 conda Python 下）load_dotenv 把
# .env 灌进 os.environ，子进程继承。这里照做，且沿用 load_dotenv 的默认语义：
# **已存在的环境变量优先，.env 只补空缺**，所以不会覆盖上面 setup_environment.sh
# 钉好的 DDS 变量。
env_exports="$("${conda_python}" - <<'PY' 2>/dev/null || true
import os
import shlex
try:
    from dotenv import dotenv_values
except ModuleNotFoundError:
    raise SystemExit(0)
for key, value in (dotenv_values(os.path.join(os.path.abspath("."), ".env")) or {}).items():
    if value is None or key in os.environ:
        continue
    print(f"export {key}={shlex.quote(value)}")
# The detached WebUI reset supervisor inherits the already-loaded environment.
# In that valid case there are no new exports, which must not be confused with
# a missing/broken python-dotenv installation.
print("export ROBOTSCENE_DOTENV_READY=1")
PY
)"
if [[ -z "${env_exports}" ]]; then
  die "没能从 ${conda_python} 读出 .env（缺 python-dotenv？）。没有 .env 时检测器解释器会错。"
fi
eval "${env_exports}"
ok ".env 已灌入环境（$(grep -c '^export ' <<<"${env_exports}") 项，已存在的变量不覆盖）"

# CLI > environment > YAML > built-in default.  Resolve exactly once and pass
# the same value to preflight, smoke and the long-running worker.
yaml_detector="$(${conda_python} - "${mapping_config}" <<'PY' 2>/dev/null || true
import sys, yaml
try:
    data = yaml.safe_load(open(sys.argv[1], encoding="utf-8")) or {}
    print((data.get("perception") or {}).get("detector") or "")
except Exception:
    pass
PY
)"
detector="${detector_cli:-${REMOTE_MAPPING_DETECTOR:-${yaml_detector:-grounded_sam}}}"
[[ "${detector}" == "grounded_sam" || "${detector}" == "yoloe_seg" ]] \
  || die "--detector 只能是 grounded_sam 或 yoloe_seg（实际: ${detector}）"
ok "检测路线: ${detector}（会话期间锁定，不热切换）"

target_watch_enabled="$(${conda_python} - "${mapping_config}" <<'PY' 2>/dev/null || true
import sys, yaml
try:
    data = yaml.safe_load(open(sys.argv[1], encoding="utf-8")) or {}
    print("1" if (data.get("target_watch") or {}).get("enabled", True) else "0")
except Exception:
    print("0")
PY
)"
[[ "${target_watch_enabled}" == "1" ]] \
  && ok '被动目标监看: enabled' \
  || warn '被动目标监看: disabled（普通遥控建图不受影响）'

# 侧车要 rclpy + 仓库依赖，只能用系统 Python（conda 环境没有 rclpy）。
# setup_environment.sh 会清掉 PYTHONPATH，所以这里补上与系统 Python 匹配的用户
# site —— 本仓的 app 依赖就装在 ~/.local/lib/python3.N/site-packages。
#
# **故意不 export**：这份 PYTHONPATH 是给系统 Python（3.10）用的，一旦全局导出，
# 演示台的 conda Python（3.11）也会继承，它的 pydantic_core 会去 3.10 的目录里
# 找 `_pydantic_core` 而 ImportError —— fastapi 跟着起不来，WebUI 直接启动失败。
# 本仓已经栽过两次同类（noetic 污染 PYTHONPATH、PYTHONNOUSERSITE 挡住 pydantic），
# 所以这里按调用点显式加前缀，不做全局导出。
system_python="/usr/bin/python3"
[[ -x "${system_python}" ]] || die "系统 Python 不存在: ${system_python}（rclpy 只能用它）"
system_py_minor="$("${system_python}" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
system_user_site="${HOME}/.local/lib/python${system_py_minor}/site-packages"
if [[ "${GO2W_JETSON_PROFILE:-0}" == "1" ]]; then
  # ROS uses /usr/bin/python3, while app dependencies are pinned in the
  # separate Python 3.8 Web/Core venv. Expose only that venv to this worker.
  system_user_site="${GO2W_ROS_APP_SITE_PACKAGES:-}"
  [[ -d "${system_user_site}" ]] || die "Jetson ROS app site-packages 不存在: ${system_user_site:-<未设置>}"
  export PYTHONNOUSERSITE=1
else
  [[ -d "${system_user_site}" ]] || warn "用户 site 不存在: ${system_user_site}（app 依赖可能装不齐）"
  unset PYTHONNOUSERSITE
fi
# 系统 Python 的调用统一走这个前缀。注意它展开的是**调用时**的 PYTHONPATH。
sys_py() { PYTHONPATH="${system_user_site}${PYTHONPATH:+:${PYTHONPATH}}" "${system_python}" "$@"; }

topic_publisher_count() {
  # 返回发布者数量；查询失败返回 -1（与"0 个发布者"区分开）。
  local info
  info="$(timeout 5 ros2 topic info "$1" -v 2>/dev/null || true)"
  local count
  count="$(grep -m1 -o 'Publisher count: [0-9]*' <<<"${info}" | grep -o '[0-9]*' || true)"
  if [[ -n "${count}" ]]; then
    printf '%s' "${count}"
    return 0
  fi
  # ``ros2 topic info`` reports an absent topic as an error with no count.
  # Absence is the normal cold-start state and must be 0, not a generic query
  # failure (-1), otherwise the owner-start branch is skipped entirely.
  local topics
  if topics="$(timeout 5 ros2 topic list 2>/dev/null)"; then
    if grep -Fxq -- "$1" <<<"${topics}"; then
      printf '%s' '-1'
    else
      printf '%s' '0'
    fi
  else
    printf '%s' '-1'
  fi
}

topic_alive() { [[ "$(topic_publisher_count "$1")" -ge 1 ]] 2>/dev/null; }

topic_receiving() {
  # $1 话题，$2 超时秒数：窗口内有数据才算"真的在发布"。
  # Foxy 的 ros2 topic hz 可能看见发布者却收不到 sensor QoS 消息。
  sys_py "${script_dir}/check_ros_topic_flow.py" "$1" "$2" >/dev/null 2>&1
}

wait_topic_alive() {
  local topic="$1" deadline="$2" waited=0
  while (( waited < deadline )); do
    topic_alive "${topic}" && return 0
    sleep 1; waited=$((waited + 1))
  done
  return 1
}

# A2. 地图模式对应的激光输入必须有唯一发布者且真的有数据。
if [[ "${map_mode}" == "2d" && "${smoke}" == "1" ]]; then
  # Camera-local smoke does not consume mapping_scan.  Validate the real lidar
  # source read-only, but do not start a preprocessor that the one-shot branch
  # would otherwise leave behind.
  lidar_topic=/go2w/sensors/cloud
elif [[ "${map_mode}" == "2d" ]]; then
  lidar_topic=/go2w/lidar/mapping_scan
  if ! topic_alive "${lidar_topic}"; then
    cloud_count="$(topic_publisher_count /go2w/sensors/cloud)"
    [[ "${cloud_count}" == "1" ]] || die \
      '/go2w/lidar/mapping_scan 缺席且 /go2w/sensors/cloud 不是唯一有效上游；无法安全代起激光预处理。'
    setsid ros2 run go2w_lidar_preprocessor lidar_preprocessor --ros-args \
      -p "config_file:=${project_root}/configs/go2w/lidar_preprocess.yaml" \
      -p "geometry_file:=${project_root}/configs/go2w/official_reference.yaml" \
      -p "sensor_profile:=${project_root}/configs/go2w/navigation_sensor_profile.yaml" \
      -p "mapping_keyframe_minimum_translation_m:=${REMOTE_MAPPING_KEYFRAME_TRANSLATION_M:-0.05}" \
      -p "mapping_keyframe_minimum_rotation_rad:=${REMOTE_MAPPING_KEYFRAME_ROTATION_RAD:-0.05}" \
      -p "publish_pointclouds:=$([[ "${GO2W_PERCEPTION_MODE:-full}" == 2d ]] && echo false || echo true)" \
      -p "publish_floor_visibility_2d:=$([[ "${GO2W_PERCEPTION_MODE:-full}" == 2d ]] && echo true || echo false)" \
      >"${log_root}/lidar_preprocessor.log" 2>&1 < /dev/null &
    printf '%s\n' "$!" >"${runtime_dir}/lidar_preprocessor.pid"
    : >"${runtime_dir}/lidar_preprocessor_started_by_remote_mapping"
    wait_topic_alive "${lidar_topic}" 30 || die '代起激光预处理后 mapping_scan 仍未就绪。'
  fi
else
  lidar_topic=/hesai/pandarxt16/points_raw
fi
lidar_count="$(topic_publisher_count "${lidar_topic}")"
if [[ "${lidar_count}" -le 0 ]]; then
  die "${lidar_topic} 没有发布者；先启动现有激光驱动与预处理。"
fi
if [[ "${lidar_count}" -gt 1 ]]; then
  die "${lidar_topic} 有 ${lidar_count} 个发布者，拒绝在重复传感器源上建图。"
fi
if [[ "${map_mode}" == "2d" ]]; then
  # mapping_scan 要等轮式里程计后才出帧；轮式里程计在 B 段按唯一发布者规则
  # 启动。这里若先等 mapping_scan，会形成“等里程计 → 等扫描”的启动环依赖。
  cloud_count="$(topic_publisher_count /go2w/sensors/cloud)"
  [[ "${cloud_count}" == "1" ]] || die \
    "/go2w/sensors/cloud 有 ${cloud_count} 个发布者，二维建图要求唯一原始点云源。"
  if ! topic_receiving /go2w/sensors/cloud 15; then
    die '/go2w/sensors/cloud 有发布者但 15 秒内没有数据。'
  fi
  if [[ "${smoke}" == "1" ]]; then
    ok "camera-local smoke：原始点云持续有数据（未启动激光预处理）"
  else
    ok "二维激光预处理已发布，原始点云持续有数据（扫描数据门在里程计启动后检查）"
  fi
else
  if ! topic_receiving "${lidar_topic}" 15; then
    die "${lidar_topic} 有发布者但 15 秒内没有数据。"
  fi
  ok "地图激光输入: ${lidar_topic} 发布中且有数据"
fi

# A3. IMU：没有只降级告警，不挡启动（计划书 §6.2 A.3）。
if topic_alive /utlidar/imu; then
  ok 'IMU: /utlidar/imu 发布中'
else
  warn '/utlidar/imu 没有发布者；LIO 可能降级为 IMU_DEGRADED。继续启动，运动链不受影响。'
fi

# A4. D435 HTTP RGB-D：这一条是硬门。侧车的 RGB 和 depth 全部来自它，
#     起不来就没有任何物体能落图，属于"看起来在跑其实一个物体都不会出"的假绿。
rgbd_base_url="${REMOTE_MAPPING_RGBD_BASE_URL:-${D435_BASE_URL:-http://192.168.123.18:8080}}"

# 采集/关键帧参数。放在 A 段而不是 D 段：§10.2 的冒烟分支在 B 之前就要用它们，
# 留在后面会让 --smoke 撞上未定义变量（set -u 直接退出）。
capture_poll="${REMOTE_MAPPING_CAPTURE_POLL_SECONDS:-0.2}"
keyframe_max_s="${REMOTE_MAPPING_KEYFRAME_MAX_SECONDS:-1.0}"
keyframe_dist_m="${REMOTE_MAPPING_KEYFRAME_DISTANCE_M:-0.75}"
keyframe_yaw_deg="${REMOTE_MAPPING_KEYFRAME_YAW_DEG:-15}"
queue_size="${REMOTE_MAPPING_QUEUE_SIZE:-20}"

# 机器狗的 HTTP 服务必须**绕过宿主机的 HTTP 代理**。宿主上常年带着
# http_proxy/ALL_PROXY（no_proxy 只写了 localhost/127.0.0.1 和几个 API 域名），
# 而 192.168.123.18 不在里面：curl 会把请求发给代理然后超时，表现为
# "D435 服务无响应"，而 `curl --noproxy '*'` 和原生 TCP 其实都是通的 ——
# 一个纯粹的假红，而且指向完全错误的方向（让人去机器狗上重启服务）。
#
# 在这里补 no_proxy 而不是只给 curl 加 --noproxy：worker 是用 python requests
# 取帧的，它同样认 http_proxy；export 出去子进程一起受益，一处修好两条路。
# 只**追加**主机名，不动操作员原有的代理设置。
rgbd_host="${rgbd_base_url#*://}"
rgbd_host="${rgbd_host%%/*}"
rgbd_host="${rgbd_host%%:*}"
no_proxy="${no_proxy:+${no_proxy},}${rgbd_host},localhost,127.0.0.1"
NO_PROXY="${NO_PROXY:+${NO_PROXY},}${rgbd_host},localhost,127.0.0.1"
export no_proxy NO_PROXY

rgbd_health="$(timeout 8 curl -fsS "${rgbd_base_url}/health" 2>/dev/null || true)"
if [[ -z "${rgbd_health}" ]]; then
  die "D435 RGB-D HTTP 服务无响应: ${rgbd_base_url}/health
       机器狗侧先跑 scripts/go2w/start_realsense_stream.sh start，再重试。
       （已尝试绕过 HTTP 代理；若手工 curl 通而这里不通，看 no_proxy=${no_proxy}）"
fi
rgbd_latest="$(timeout 8 curl -fsS "${rgbd_base_url}/rgbd/latest.json" 2>/dev/null || true)"
if [[ -z "${rgbd_latest}" ]] || ! grep -q '"frame_id"' <<<"${rgbd_latest}"; then
  die "D435 RGB-D 服务在但 /rgbd/latest.json 没有有效帧: ${rgbd_latest:0:200}"
fi
ok "D435 RGB-D: ${rgbd_base_url} 有有效帧"

# A5. 模型与 Python 环境。检查到具体文件，失败时直接指出该看哪。
if ! sys_py - <<'PY' >/dev/null 2>&1
import rclpy, numpy  # noqa: F401
from app.config import get_settings
from app.spatial.plain_slam_spatial_provider import PlainSlamSpatialProvider  # noqa: F401
from app.spatial.nav2_spatial_provider import Nav2SpatialProvider  # noqa: F401
from app.spatial.semantic_object_map import SemanticObjectMap  # noqa: F401
from app.detectors.factory import create_local_detector  # noqa: F401
PY
then
  die "系统 Python 跑不通侧车依赖（rclpy / app.*）。检查 PYTHONPATH=${PYTHONPATH:-<空>}。"
fi

if [[ "${detector}" == "grounded_sam" ]]; then
  model_python="$(sys_py -c 'import sys; sys.path.insert(0, ".")
from app.config import get_settings
print(get_settings().grounded_sam_python or "")' 2>/dev/null || true)"
  if [[ -z "${model_python}" || ! -x "${model_python}" ]]; then
    die "GROUNDED_SAM_PYTHON 不可执行: '${model_python:-<未设置>}'。检查 .env。"
  fi
  model_paths="$(sys_py -c 'import sys; sys.path.insert(0, ".")
from app.config import get_settings
s = get_settings(); print(s.grounded_sam_pythonpath or "")' 2>/dev/null || true)"
  [[ -n "${model_paths}" ]] || die 'GROUNDED_SAM_PYTHONPATH 为空；检查 .env。'
  first_model_path="${model_paths%%:*}"
  [[ -d "${first_model_path}" ]] || die "GROUNDED_SAM_PYTHONPATH 第一段不存在: ${first_model_path}"
  "${model_python}" -c 'import torch' >/dev/null 2>&1 \
    || die "${model_python} 里 import torch 失败；Grounded-SAM 起不来。"
else
  readarray -t yoloe_values < <(sys_py -c 'import sys; sys.path.insert(0, ".")
from app.config import get_settings
s=get_settings(); print(s.yoloe_python); print(s.yoloe_model); print(s.yoloe_device); print(s.yoloe_precision); print(s.yoloe_text_encoder)' 2>/dev/null)
  model_python="${yoloe_values[0]:-}"
  yoloe_model="${yoloe_values[1]:-}"
  yoloe_device="${yoloe_values[2]:-}"
  yoloe_precision="${yoloe_values[3]:-}"
  yoloe_text_encoder="${yoloe_values[4]:-}"
  [[ -x "${model_python}" ]] || die "YOLOE_PYTHON 不可执行: ${model_python:-<未设置>}"
  [[ -f "${yoloe_model}" ]] || die "YOLOE_MODEL 不存在: ${yoloe_model:-<未设置>}"
  if [[ "${yoloe_model}" == *-seg-pf.pt ]]; then
    yoloe_prompt_mode='prompt_free'
  elif [[ "${yoloe_model}" == *-seg.pt ]]; then
    yoloe_prompt_mode='text_prompt'
    [[ -f "${yoloe_text_encoder}" ]] || die "YOLOE_TEXT_ENCODER 不存在: ${yoloe_text_encoder:-<未设置>}"
  else
    die 'YOLOE_MODEL 必须是 *-seg.pt 或 *-seg-pf.pt。'
  fi
  "${model_python}" -c 'import torch, ultralytics; assert torch.cuda.is_available()' >/dev/null 2>&1 \
    || die "${model_python} 无法 import torch/ultralytics 或 CUDA 不可用。"
fi
ok "检测器解释器: ${model_python}（${detector} 依赖可用）"

# A6. 机器狗时钟（只读）。D435 服务用**机器狗的钟**给帧打时间戳，而落图要按
#     时间戳找拍摄时刻的 SLAM 位姿。侧车有一致性门（--max-clock-skew-seconds，
#     默认 5s）：超了就用接收时刻代替拍摄时刻，状态里 timestamp_source 变成
#     receive_time_clock_skew、每个物体带一个 clock_skew_seconds。
#     2026-09-18 实测机器狗快 13.6s ⇒ 每一帧都在降级。
#
#     只告警不挡启动：这是**降级**不是故障，识别和遥控都不受影响；而且按计划书
#     §6.2 的口径，非致命项一律不挡运动链。修法交给操作员，因为改钟要先停 SLAM。
#
#     判据和容差都不在这里重写，一律走 sync_robot_clock.sh（它直接读 worker 的
#     常量），避免两处各写一个 5.0、改了一处忘了另一处。
if [[ "${GO2W_JETSON_PROFILE:-0}" == "1" && ( "${rgbd_host}" == "127.0.0.1" || "${rgbd_host}" == "localhost" ) ]]; then
  ok '机器狗时钟: RGB-D 服务与项目使用本机时钟，无需跨机对时'
elif command -v sshpass >/dev/null 2>&1; then
  clock_out="$(ROBOT_IP="${rgbd_host}" bash "${script_dir}/sync_robot_clock.sh" status 2>&1)" \
    && clock_rc=0 || clock_rc=$?
  clock_skew="$(grep -m1 '偏差' <<<"${clock_out}" || true)"
  if [[ "${clock_rc}" -eq 0 ]]; then
    ok "机器狗时钟: ${clock_skew:-已同步}"
  elif [[ "${clock_rc}" -eq 3 ]]; then
    warn "${clock_skew:-机器狗时钟与宿主不同步}
         落图会降级：时间戳改用接收时刻（timestamp_source=receive_time_clock_skew）。
         不挡启动。要修的话**必须先停 SLAM**（改钟会让跨机对齐的时间轴出现台阶，
         已知后果是地图重影），然后：
           bash scripts/go2w/stop_plain_slam_mapping.sh
           bash scripts/go2w/sync_robot_clock.sh apply
           bash scripts/go2w/start_remote_control_mapping.sh"
  else
    warn "机器狗时钟测不到（ssh 不通，rc=${clock_rc}）；跳过这项检查，不挡启动。"
  fi
else
  warn '没有 sshpass，跳过机器狗时钟检查（不影响落图，只是时间戳可能降级）。'
fi

# A7. 机器狗电量（只读）。**2026-09-18 就是这样停的**：17:43 起建图，18:06 狗断电，
#     宿主 enp3s0 随即掉载波（雷达与狗共用这条链路，一起失联），建图数据源断供、
#     地图冻结成 FROZEN_LAST_GOOD，直到两天后重新上电才自己恢复。
#     而电量一直在 /lf/lowstate 的 bms_state.soc 上流着 —— 全仓库没有任何地方读它。
#
#     开机前拦一道：电量太低就别开始，否则建到一半必然重演同一次停机。判据只看
#     数字，不猜狗还能跑多久（放电曲线和负载有关，猜不准）。
#
#     读不到只告警不挡启动，与 A3（IMU）/A6（时钟）同样的降级口径：DDS 或狗还没
#     起来时本来就可能读不到，那不是"电量有问题"的证据。
#     阈值可用 REMOTE_MAPPING_MIN_BATTERY_SOC 覆盖（默认 20）；告警带取它 +15。
battery_floor="${REMOTE_MAPPING_MIN_BATTERY_SOC:-20}"
battery_soc="$(PYTHONPATH="${system_user_site}${PYTHONPATH:+:${PYTHONPATH}}" \
  timeout 8 "${system_python}" - <<'PY' 2>/dev/null || true
import time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unitree_go.msg import LowState

rclpy.init()
node = rclpy.create_node("go2w_mapping_battery_preflight")
readings = []
node.create_subscription(
    LowState, "/lf/lowstate",
    lambda msg: readings.append(int(msg.bms_state.soc)),
    QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
)
deadline = time.monotonic() + 5.0
while not readings and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
if readings and 0 <= readings[-1] <= 100:
    print(readings[-1])
node.destroy_node()
rclpy.shutdown()
PY
)"
if [[ ! "${battery_soc}" =~ ^[0-9]+$ ]]; then
  warn "机器狗电量读不到（/lf/lowstate 无数据）；跳过这项检查，不挡启动。
       建图前最好确认狗不是靠余电在跑 —— 断电会让 enp3s0 掉载波，
       雷达和狗一起失联、地图冻结，而且不会自愈。"
elif (( battery_soc < battery_floor )); then
  die "机器狗电量 ${battery_soc}%（低于 ${battery_floor}%）。先充电再建图。
       电量耗尽时狗会直接断电：enp3s0 掉载波 ⇒ 点云与 IMU 同时断供 ⇒ 地图冻结。
       确实要用余电跑，就显式降低门槛：REMOTE_MAPPING_MIN_BATTERY_SOC=<更低的值>"
elif (( battery_soc < battery_floor + 15 )); then
  warn "机器狗电量 ${battery_soc}%（低于 $(( battery_floor + 15 ))%）。建图中途可能没电，建议先充一会儿。"
else
  ok "机器狗电量: ${battery_soc}%"
fi

# A8. 检测用的 GPU（只读）。**2026-09-21 就是这样静默烂掉的**：worker 的 --device
#     是 auto，`grounded_sam_worker.py::_select_device` 在 CUDA 起不来时**静默返回
#     cpu** —— 识别从 3.0s/帧 掉到 72s/帧，而 status.json 照报 state=running、实体
#     照落、日志里没有一处红。实测那个 worker 一个人吃 735% CPU，把宿主 load 拉到
#     14（16 核），而宿主同时还在跑 SLAM —— CPU 被抢正是 LIO 发散的已知诱因。
#
#     根因不在这一层：内核 9月18 18:11:57 就落过
#       NVRM: Xid 154, GPU recovery action changed from 0x0 (None) to 0x2
#              (Node Reboot Required)
#     之后每次 CUDA 初始化要么再 Xid 31、要么 "CUDA unknown error"，内核原话是
#     `requiring os reboot to recover` —— 只有重启宿主能解。所以这里不猜也不修，
#     只把那个一直没人读的信号读出来，拦在开机前。
#
#     判据用 libcuda 的 cuInit：那正是 torch 自己第一步做的事，且不必 import torch
#     （省十几秒，也不受 conda / noetic 那堆环境变量影响）。返回 0 才算可用。
#
#     为什么是硬门而不是告警：CPU 回退不是"慢一点"，是**拿 7 个核去和 SLAM 抢**，
#     对外却表现得像正常。确实要在这种状态下建图（放弃物体落图），显式放行：
#       REMOTE_MAPPING_ALLOW_CPU_DETECTION=1
cuda_rc="$(python3 - <<'PY' 2>/dev/null
import ctypes
try:
    lib = ctypes.CDLL("libcuda.so.1")
except OSError:
    print("NOLIB")
    raise SystemExit
print(lib.cuInit(0))
PY
)" || cuda_rc="PROBE_FAILED"
# 按**结果的形状**分派，不按"跑没跑起来"：0 = 可用；NOLIB 或任何非零数字 =
# 探针确实跑到了 libcuda、并说它不能用（硬门）；其余（空、异常退出）=
# 探针自身不可用，按 A3/A6 的降级口径跳过，不挡启动。
if [[ "${cuda_rc}" == "0" ]]; then
  ok "检测 GPU: CUDA 可用"
elif [[ "${cuda_rc}" != "NOLIB" && ! "${cuda_rc}" =~ ^[0-9]+$ ]]; then
  warn "检测 GPU 探不出来（探针结果: ${cuda_rc:-空}）；跳过这项检查，不挡启动。
       但要留意 worker 的 --device=auto 在 CUDA 不可用时会**静默退回 CPU**
       （实测 72s/帧、一个人吃 7 个核），日志里不会红。"
elif [[ "${detector}" == "grounded_sam" && "${REMOTE_MAPPING_ALLOW_CPU_DETECTION:-0}" == "1" ]]; then
  warn "检测 GPU 不可用（探针结果: ${cuda_rc}），已按 REMOTE_MAPPING_ALLOW_CPU_DETECTION=1 放行。
       侧车会在 CPU 上跑识别：约 72s/帧，且持续占用多个核，会与 SLAM 抢 CPU。"
else
  die "检测 GPU 不可用（cuInit 探针结果: ${cuda_rc}）—— CUDA 初始化不了，
       worker 会静默退回 CPU。直接跑的话：识别从 3s/帧 退化到 72s/帧、一个进程吃
       7 个核，而 status.json 照样报 state=running、实体照落 —— 那是假绿不是故障灯。
       先确认是不是内核在要求重启（Xid 154 那条）：
         grep -iE 'NVRM.*(Xid|requires os reboot)' /var/log/kern.log | tail -5
       若是，重启宿主即可恢复（会一并停掉 SLAM / WebUI / 侧车）。
       确实要先建图、暂不要物体落图，就显式放行：
         REMOTE_MAPPING_ALLOW_CPU_DETECTION=1 bash scripts/go2w/start_remote_control_mapping.sh"
fi

mapping_warmup="${REMOTE_MAPPING_STARTUP_WARMUP:-auto}"
if [[ "$mapping_warmup" == auto ]]; then
  # The pinned Jetson environment keeps the CUDA gate above; the persistent
  # worker performs model initialization instead of loading it twice.
  [[ "${GO2W_JETSON_PROFILE:-0}" == 1 ]] && mapping_warmup=0 || mapping_warmup=1
fi
[[ "$mapping_warmup" == 0 || "$mapping_warmup" == 1 ]] || die 'REMOTE_MAPPING_STARTUP_WARMUP must be auto, 0 or 1'
if [[ "${detector}" == "yoloe_seg" && "$mapping_warmup" == 1 ]]; then
  step "A9. YOLOE-Seg ${yoloe_prompt_mode} CUDA warm-up"
  warmup_stderr="${log_root}/yoloe_warmup.stderr.log"
  warmup_command=("${model_python}" app/detectors/yoloe_seg_worker.py
    --model "${yoloe_model}" --device "${yoloe_device}"
    --precision "${yoloe_precision}" --warmup)
  if [[ "${yoloe_prompt_mode}" == 'text_prompt' ]]; then
    warmup_command+=(--text-encoder "${yoloe_text_encoder}")
  fi
  warmup_json="$(timeout 180 "${warmup_command[@]}" 2>"${warmup_stderr}")" || {
    warmup_error="$(tail -c 1200 "${warmup_stderr}" 2>/dev/null || true)"
    die "YOLOE-Seg warm-up 失败（不回退）。stdout=${warmup_json:0:600} stderr=${warmup_error}"
  }
  warmup_check="$("${system_python}" -c '
import json, sys
lines = sys.stdin.read().splitlines()
if len(lines) != 1:
    raise SystemExit(f"expected exactly one JSON line, got {len(lines)}")
payload = json.loads(lines[0])
if not isinstance(payload, dict) or payload.get("ok") is not True:
    raise SystemExit(f"invalid health payload: {payload!r}")
if payload.get("backend") != "yoloe_seg" or payload.get("fallback_used") is not False:
    raise SystemExit(f"unexpected backend/fallback: {payload!r}")
' <<<"${warmup_json}" 2>&1)" || {
    warmup_error="$(tail -c 1200 "${warmup_stderr}" 2>/dev/null || true)"
    die "YOLOE-Seg warm-up 协议无效（stdout 必须恰好是一行 JSON）: ${warmup_check}; stdout=${warmup_json:0:600}; stderr=${warmup_error}"
  }
  ok "YOLOE-Seg warm-up 成功，模式=${yoloe_prompt_mode}"
elif [[ "${detector}" == "yoloe_seg" ]]; then
  ok 'YOLOE-Seg 由常驻识别进程初始化，省去一次性预热进程的重复模型加载'
fi

# 目标监看复用单槽 VLM daemon。不可用只让目标复核降级，绝不阻止普通建图。
if [[ "${target_watch_enabled}" == "1" && ( "${smoke}" != "1" || -n "${smoke_target}" ) ]]; then
  vlm_socket_ready=0
  if [[ -S "${vlm_socket}" ]] && "${system_python}" - "${vlm_socket}" <<'PY' >/dev/null 2>&1
import socket, sys
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
sock.settimeout(0.5)
try:
    sock.connect(sys.argv[1])
finally:
    sock.close()
PY
  then
    vlm_socket_ready=1
  fi
  if [[ "${vlm_socket_ready}" == "1" ]]; then
    ok 'VLM daemon: 复用已有健康 socket（不创建 ownership marker）'
  else
    rm -f "${vlm_owner_marker}"
    if bash "${script_dir}/start_vlm_daemon.sh"; then
      : >"${vlm_owner_marker}"
      ok 'VLM daemon: 由遥控建图模式启动'
    else
      warn 'VLM daemon 启动失败；普通建图继续，目标监看将显示 DEGRADED。'
    fi
  fi
fi

# ---------------------------------------------------------------------------
# §10.2 单轮无运动烟雾检查：跑一轮识别就退出，不碰 SLAM / 快照 / WebUI。
#
# 为什么值得单独给一个开关：worker 自己需要 rclpy（只有 /usr/bin/python3 有），
# 而检测器子进程需要 torch（只有 conda 环境有），两边不是同一个解释器。直接把
# 计划书 §10.2 那段 `<正确 Python> ... --once` 抄到命令行跑，会命中
# "GROUNDED_SAM_PYTHON 无效，且没有可用的回退解释器"（因为没有 .env 注入），
# 看起来像检测器坏了，其实是环境没进来。这里复用 A 段刚做完的 .env 注入与
# 解释器校验，保证冒烟和正式启动走的是同一套环境。
# ---------------------------------------------------------------------------
if [[ "${smoke}" == "1" ]]; then
  step '§10.2 单轮无运动烟雾检查（只读，不发运动命令）'
  smoke_events="/tmp/go2w_remote_mapping_smoke_events.jsonl"
  smoke_status="/tmp/go2w_remote_mapping_smoke_status.json"
  smoke_watch_request="/tmp/go2w_remote_mapping_smoke_target_watch_request.json"
  smoke_watch_state="/tmp/go2w_remote_mapping_smoke_target_watch_state.json"
  smoke_watch_evidence="/tmp/go2w_remote_mapping_smoke_target_watch_evidence"
  : > "${smoke_events}"
  rm -f "${smoke_status}" "${smoke_watch_request}" "${smoke_watch_state}"
  mkdir -p "${smoke_watch_evidence}"
  smoke_target_args=()
  if [[ -n "${smoke_target}" ]]; then
    smoke_target_args=(--smoke-target "${smoke_target}")
  fi

  # 冒烟失败时不要因为 set -e 直接跳出，先把产物打出来——否则失败原因只能靠猜。
  smoke_rc=0
  PYTHONPATH="${system_user_site}${PYTHONPATH:+:${PYTHONPATH}}" \
    "${system_python}" "${script_dir}/remote_control_semantic_mapping_worker.py" \
    --events "${smoke_events}" \
    --status "${smoke_status}" \
    --capture-poll-seconds "${capture_poll}" \
    --keyframe-max-seconds "${keyframe_max_s}" \
    --keyframe-distance-m "${keyframe_dist_m}" \
    --keyframe-yaw-deg "${keyframe_yaw_deg}" \
    --queue-size "${queue_size}" \
    --rgbd-base-url "${rgbd_base_url}" \
    --map-meta-path /tmp/go2w_remote_mapping_smoke_meta.json \
    --object-snapshot-path /tmp/go2w_remote_mapping_smoke_objects.json \
    --topology-snapshot-path /tmp/go2w_remote_mapping_smoke_topology.json \
    --target-watch-request "${smoke_watch_request}" \
    --target-watch-state "${smoke_watch_state}" \
    --target-watch-evidence-dir "${smoke_watch_evidence}" \
    --mapping-config "${mapping_config}" \
    --detector "${detector}" \
    --map-mode "${map_mode}" \
    --once --camera-local-smoke --once-timeout-seconds 180 \
    "${smoke_target_args[@]}" || smoke_rc=$?

  printf '\n---- 冒烟结果 ----\n'
  smoke_verdict="$("${system_python}" - "${smoke_status}" "${smoke_events}" <<'PY' || true
import json, sys
try:
    status = json.load(open(sys.argv[1]))
except Exception as exc:                      # 文件没生成时要说清楚
    print(f'状态文件读不到（{exc}）—— 侧车大概率在预检就退出了')
    status = {}
for key in ('state', 'slam_ready', 'captured_keyframes', 'processed_keyframes',
            'failed_keyframes', 'last_detection_count', 'last_localized_count',
            'persistent_object_count', 'last_error'):
    if key in status:
        print(f'{key:28} {status[key]}')
rows = 0
bad = 0
with open(sys.argv[2], encoding='utf-8') as handle:
    for line in handle:
        if not line.strip():
            continue
        rows += 1
        try:
            json.loads(line)
        except Exception:
            bad += 1
print(f'{"events 行数":28} {rows}（不可解析 {bad}）')

# 无 SLAM smoke 只验证真实 mask 与 camera_xyz，绝不伪造 map_xyz。正式短路线
# 在 SLAM 已启动后继续验证 map_xyz。
# 检测到了但一个都没落图时，上面每一行看起来都正常。检出>0 而落图=0 时明确
# 说"这一项没被检验"，不要把"跑通了"读成"验过了"。
det = status.get('last_detection_count') or 0
loc = status.get('last_localized_count') or 0
print('VERDICT=CAMERA_XYZ_OK' if loc > 0
      else ('VERDICT=CAMERA_XYZ_UNVERIFIED' if det > 0 else 'VERDICT=EMPTY_SCENE'))
PY
)"
  printf '%s\n' "${smoke_verdict}"
  printf '%s\n' '------------------'
  if [[ "${smoke_rc}" -ne 0 ]]; then
    # 冒烟不重定向到 ${log_root}/semantic_mapper.log，侧车日志就在上面这段输出里。
    if [[ -f "${vlm_owner_marker}" ]]; then
      bash "${script_dir}/stop_vlm_daemon.sh" || true
      rm -f "${vlm_owner_marker}"
    fi
    die "烟雾检查失败（退出码 ${smoke_rc}）。侧车日志、状态与事件都在上面。"
  fi
  ok "烟雾检查通过：拿到 RGB-D、跑通一轮识别、events 每行可解析。"
  case "${smoke_verdict}" in
    *VERDICT=CAMERA_XYZ_OK*)
      ok "本轮有物体拿到 camera_xyz —— 无 SLAM smoke 的深度定位判据已实际检验。" ;;
    *VERDICT=CAMERA_XYZ_UNVERIFIED*)
      warn "本轮检出了物体但没有任何一个拿到 camera_xyz —— 深度定位判据**未被检验**。"
      warn "换到有明显静态物体的朝向再跑一次；若持续如此，看 last_error 与侧车日志。" ;;
    *)
      warn "本轮没有检出物体（可能是合法空结果）—— §10.2 第三项判据**未被检验**。" ;;
  esac
  ok '本次没有启动 SLAM / 快照写入器 / WebUI，也没有发送任何运动命令。'
  if [[ -f "${vlm_owner_marker}" ]]; then
    bash "${script_dir}/stop_vlm_daemon.sh" || true
    rm -f "${vlm_owner_marker}"
    ok '本轮拥有的 VLM daemon 已停止。'
  fi
  exit 0
fi

# ---------------------------------------------------------------------------
# B. 复用或启动选定地图栈
# ---------------------------------------------------------------------------
step "B. ${map_mode} 地图栈"

if [[ "${map_mode}" == "2d" ]]; then
  # 二维 SLAM 只接收专用 mapping_scan。原始 Pandar 点云或三维 LIO 话题
  # 不能冒充这个输入；A2 已复用或按所有权启动现有激光预处理。
  odom_count="$(topic_publisher_count /go2w/odom/fused)"
  [[ "${odom_count}" -le 1 ]] || die "/go2w/odom/fused 有 ${odom_count} 个发布者，拒绝重复轮式里程计。"
  if [[ "${odom_count}" == "1" ]] && ! topic_receiving /go2w/odom/fused 3; then
    # A reset can stop the owned wheel-odom process milliseconds before this
    # launcher starts.  CycloneDDS may briefly retain the dead endpoint in the
    # graph; publisher-count alone would then falsely reuse it and leave the
    # mapping scan waiting for odometry forever.  Give discovery a bounded
    # cleanup window, but never start a second owner while a live endpoint
    # remains ambiguous.
    for _ in $(seq 1 20); do
      odom_count="$(topic_publisher_count /go2w/odom/fused)"
      [[ "${odom_count}" == "0" ]] && break
      topic_receiving /go2w/odom/fused 1 && break
      sleep 0.5
    done
    if [[ "${odom_count}" == "1" ]] && ! topic_receiving /go2w/odom/fused 2; then
      die '/go2w/odom/fused 有残留发布者但没有数据；拒绝重复启动轮式里程计。'
    fi
  fi
  if [[ "${odom_count}" == "0" ]]; then
    setsid /usr/bin/python3 "${script_dir}/exec_ros2.py" launch go2w_lio_bringup wheel_odom.launch.py \
      >"${log_root}/wheel_odom.log" 2>&1 < /dev/null &
    printf '%s\n' "$!" >"${runtime_dir}/wheel_odom.pid"
    : >"${runtime_dir}/wheel_odom_started_by_remote_mapping"
    wait_topic_alive /go2w/odom/fused 60 || die '轮式里程计 60 秒内未就绪。'
  fi
  if ! topic_receiving /go2w/lidar/mapping_scan 15; then
    die '/go2w/lidar/mapping_scan 在轮式里程计启动后仍没有数据。'
  fi
  localization_pidfile="${project_root}/outputs/autonomous_search/runtime/localization_2d.pid"
  if [[ -f "${localization_pidfile}" ]]; then
    localization_pid="$(tr -cd '0-9' <"${localization_pidfile}")"
    localization_cmd=""
    [[ -r "/proc/${localization_pid}/cmdline" ]] && localization_cmd="$(tr '\0' ' ' <"/proc/${localization_pid}/cmdline")"
    if [[ "${localization_cmd}" == *'localization_2d.launch.py mode:=localization'* ]]; then
      die '当前是 AMCL 旧地图定位模式，不能冒充在线二维扩图；请先按其所有权停止。'
    fi
  fi
  map_count_before="$(topic_publisher_count /map)"
  if [[ "${map_count_before}" == "0" ]]; then
    [[ "${allow_slam_start}" == "1" ]] || die '--no-slam-start 已给出且没有健康的二维 mapping。'
    bash "${script_dir}/start_2d_localization.sh" --mode mapping
    : >"${runtime_dir}/localization_2d_started_by_remote_mapping"
  else
    bash "${script_dir}/start_2d_localization.sh" --mode mapping
    ok '复用项目所有的二维 mapping 会话。'
  fi
  wait_topic_alive /map 60 || die '/map 在 60 秒内没有发布者。'
  wait_topic_alive /go2w/slam/keyframe_graph 60 || die 'SLAM 位姿图在 60 秒内没有发布者。'
  ok '二维地图、轮式里程计与 SLAM 位姿图已就绪'
else
slam_topics=(/go2w/slam/map_3d /go2w/slam/odom_base /go2w/slam/ready)
slam_healthy=1
for topic in "${slam_topics[@]}"; do
  count="$(topic_publisher_count "${topic}")"
  if [[ "${count}" != "1" ]]; then
    slam_healthy=0
    break
  fi
done

if [[ "${force_new_slam}" == "1" ]]; then
  slam_healthy=0
  printf '%s\n' '--force-new-slam：即使已有 plain_slam 也重开会话。'
fi

if [[ "${slam_healthy}" == "1" ]]; then
  ok 'plain_slam 已健康（三个话题各一个发布者），复用，不启第二套。'
else
  printf '%s\n' 'plain_slam 不完整或缺席，准备启动。'
  if [[ "${allow_slam_start}" != "1" ]]; then
    die '--no-slam-start 已给出且没有健康的 plain_slam。'
  fi
  # start_plain_slam_mapping.sh 会新建 SLAM session 并清掉自己的旧缓存
  # （计划书 §6.2 B 明确要求这一步不要在语义 worker 里重复做）。
  # 它自己会先 pkill 掉任何残留的 plain_slam 节点 —— 上面已经确认不健康，安全。
  if ! bash "${script_dir}/start_plain_slam_mapping.sh"; then
    die "start_plain_slam_mapping.sh 失败；看 ${slam_session_log}"
  fi
  # ownership marker 只在本脚本启动了 SLAM 时写；停止脚本据此决定是否去停 SLAM。
  {
    printf 'started_by=%s\n' 'start_remote_control_mapping.sh'
    printf 'started_at=%s\n' "$(date -Is)"
    printf 'plain_slam_pid_file=%s\n' "${pid_root}/plain_slam_go2w.pid"
  } >"${slam_owner_marker}"
  ok "plain_slam 由本模式启动（marker: ${slam_owner_marker}）"
fi

# 等 ready / odom / map。ready 是 SLAM 自报的门，odom_base 要有实际数据
# （有发布者但没数据是假绿：LIO 可能已经卡死）。
for topic in "${slam_topics[@]}"; do
  if ! wait_topic_alive "${topic}" 60; then
    die "${topic} 在 60 秒内没有出现唯一发布者；SLAM 未就绪。看 ${slam_session_log}"
  fi
done
if ! topic_receiving /go2w/slam/odom_base 20; then
  die '/go2w/slam/odom_base 有发布者但 20 秒内没有数据；不继续（地图位姿会全错）。'
fi
ok 'plain_slam: ready / odom_base / map_3d 就绪，odom 有数据'
# map_3d 不查 hz：机器狗静止时地图不更新是**预期**行为，拿它当门会误报。
fi

# ---------------------------------------------------------------------------
# C. 本次语义会话
# ---------------------------------------------------------------------------
step 'C. 本次语义会话'

session_stamp="$(date +%Y%m%d_%H%M%S)"
# 归档而不是删除：pslam_map 在新 SLAM session 里重新定义，旧坐标系的物体绝不能
# 混进新地图（计划书 §6.2 C）。但旧文件对排障有用，留一份带时间戳的。
if [[ -s "${events_path}" ]]; then
  mv -f "${events_path}" "${runtime_dir}/events_${session_stamp}.jsonl"
  ok "上一份 events.jsonl 已归档为 events_${session_stamp}.jsonl"
fi
: >"${events_path}"
# status.json 必须清掉：上一轮如果留下 state=running，下面的就绪轮询会立刻
# 通过 —— 那正是"进程早就死了但页面显示 running"的假绿。
if [[ -f "${status_path}" ]]; then
  mv -f "${status_path}" "${runtime_dir}/status_${session_stamp}.json"
fi
mkdir -p "${target_watch_evidence_dir}"
sys_py - "${target_watch_request}" "${target_watch_state}" "${detector}" <<'PY'
import sys, time
from app.perception.passive_target_watch import atomic_write_json, empty_state

request_path, state_path, detector = sys.argv[1:]
atomic_write_json(request_path, {
    "schema_version": 1,
    "mapping_session_id": "teleop_0",
    "generation": 0,
    "enabled": False,
    "command": "clear",
    "raw_query": "",
    "profile": None,
    "submitted_at": time.time(),
})
atomic_write_json(state_path, empty_state("teleop_0", detector_backend=detector))
PY
ok "会话目录: ${runtime_dir}"

# ---------------------------------------------------------------------------
# D. 语义侧车
# ---------------------------------------------------------------------------
step 'D. 语义侧车（只读感知）'

# setsid 让侧车脱离本 shell 的进程组：启动器退出不能把建图侧车带走。
setsid env PYTHONPATH="${system_user_site}${PYTHONPATH:+:${PYTHONPATH}}" \
  "${system_python}" "${script_dir}/remote_control_semantic_mapping_worker.py" \
  --events "${events_path}" \
  --status "${status_path}" \
  --capture-poll-seconds "${capture_poll}" \
  --keyframe-max-seconds "${keyframe_max_s}" \
  --keyframe-distance-m "${keyframe_dist_m}" \
  --keyframe-yaw-deg "${keyframe_yaw_deg}" \
  --queue-size "${queue_size}" \
  --rgbd-base-url "${rgbd_base_url}" \
  --map-meta-path "${meta_path}" \
  --object-snapshot-path "${objects_path}" \
  --topology-snapshot-path "${topology_path}" \
  --target-watch-request "${target_watch_request}" \
  --target-watch-state "${target_watch_state}" \
  --target-watch-evidence-dir "${target_watch_evidence_dir}" \
  --mapping-config "${mapping_config}" \
  --detector "${detector}" \
  --map-mode "${map_mode}" \
  >"${log_root}/semantic_mapper.log" 2>&1 < /dev/null &
mapper_pid=$!
printf '%s\n' "${mapper_pid}" >"${mapper_pidfile}"
ok "语义侧车 pid=${mapper_pid} 日志=${log_root}/semantic_mapper.log"

mapper_state=""
for _ in $(seq 1 90); do
  if ! kill -0 "${mapper_pid}" 2>/dev/null; then
    mapper_state="exited"
    break
  fi
  mapper_state="$(sys_py -c '
import json, sys
try:
    with open(sys.argv[1], encoding="utf-8") as fh:
        print(json.load(fh).get("state") or "")
except Exception:
    print("")
' "${status_path}" 2>/dev/null || true)"
  # A brand-new 2-D map has no global pose until the operator drives past the
  # first measurable structure.  The physical-remote WebUI must come up while
  # the read-only semantic sidecar waits, otherwise neither side can make the
  # condition true.  ``waiting`` is emitted only after detector setup succeeds.
  if [[ "${mapper_state}" == "running" || "${mapper_state}" == "degraded" \
        || ( "${map_mode}" == "2d" && "${mapper_state}" == "waiting" ) ]]; then
    break
  fi
  sleep 1
done

case "${mapper_state}" in
  running)  ok '语义侧车: running' ;;
  degraded) warn '语义侧车: degraded（地图仍会画，物体可能缺；看日志里的 last_error）' ;;
  waiting)
    if [[ "${map_mode}" == "2d" ]]; then
      warn '语义侧车: waiting（新二维地图尚无全局位姿；WebUI 先启动，遥控产生有效地图后会自动进入 running）'
    else
      die '语义侧车在非二维模式意外停留于 waiting。'
    fi
    ;;
  *)
    printf '\n---- semantic_mapper.log 末尾 ----\n' >&2
    tail -n 25 "${log_root}/semantic_mapper.log" >&2 || true
    printf '%s\n' '---------------------------------' >&2
    die "语义侧车 90 秒内没有进入 running/degraded（state=${mapper_state:-unknown}）。"
    ;;
esac

# ---------------------------------------------------------------------------
# E. 地图快照写入器
# ---------------------------------------------------------------------------
step 'E. 地图快照写入器'

# 写入器只写地图。物体由语义侧车按 map_epoch 写进 semantic_objects.json，读取端
# 两个标识对得上才叠画 —— 写入器不再去 glob 任何 events.jsonl 猜"哪份物体属于
# 当前地图"（那条路 2026-09-17 已被证伪）。
if [[ "${map_mode}" == "2d" ]]; then
  setsid env PYTHONPATH="${system_user_site}${PYTHONPATH:+:${PYTHONPATH}}" \
    "${system_python}" "${script_dir}/nav2_web_map_bridge.py" \
    --output "${snapshot_path}" --meta-output "${meta_path}" \
    >"${log_root}/snapshot_writer.log" 2>&1 < /dev/null &
else
  setsid env PYTHONPATH="${system_user_site}${PYTHONPATH:+:${PYTHONPATH}}" \
    "${system_python}" "${script_dir}/webui_map_snapshot_writer.py" --ros-args \
  -p map_topic:="${REMOTE_MAPPING_MAP_TOPIC:-/go2w/slam/map_3d}" \
  -p odom_topic:="${REMOTE_MAPPING_ODOM_TOPIC:-/go2w/slam/odom_base}" \
  -p snapshot_path:="${snapshot_path}" \
  -p meta_path:="${meta_path}" \
  -p trajectory_path:="${runtime_dir}/slam_map_trajectory.json" \
  -p max_points:="${GO2W_WEBUI_SNAPSHOT_MAX_POINTS:-60000}" \
  -p max_range_m:="${GO2W_WEBUI_SNAPSHOT_MAX_RANGE_M:-25.0}" \
  -p min_range_m:=1.0 \
  -p voxel_size_m:="${GO2W_WEBUI_SNAPSHOT_VOXEL_M:-0.05}" \
  -p min_z_m:="${GO2W_WEBUI_SNAPSHOT_MIN_Z_M:-0.15}" \
  -p max_z_m:="${GO2W_WEBUI_SNAPSHOT_MAX_Z_M:-2.80}" \
  -p trajectory_radius_m:="${GO2W_WEBUI_SNAPSHOT_TRAJ_RADIUS_M:-12.0}" \
  -p focus_margin_m:="${GO2W_WEBUI_SNAPSHOT_FOCUS_MARGIN_M:-3.0}" \
  -p period_s:="${REMOTE_MAPPING_SNAPSHOT_PERIOD_S:-1.0}" \
    >"${log_root}/snapshot_writer.log" 2>&1 < /dev/null &
fi
writer_pid=$!
printf '%s\n' "${writer_pid}" >"${writer_pidfile}"
sleep 2
if ! kill -0 "${writer_pid}" 2>/dev/null; then
  tail -n 20 "${log_root}/snapshot_writer.log" >&2 || true
  die '快照写入器启动即退出；看上面日志。'
fi
ok "快照写入器 pid=${writer_pid} 快照=${snapshot_path}"

# ---------------------------------------------------------------------------
# F. 只读 WebUI
# ---------------------------------------------------------------------------
if [[ "${start_web}" == "1" ]]; then
  step 'F. 只读 WebUI'

  if ! "${conda_python}" -c 'import fastapi, uvicorn' >/dev/null 2>&1; then
    die "${conda_python} 缺 fastapi/uvicorn。"
  fi

  web_host="${MANUAL_DEMO_HOST:-127.0.0.1}"
  # Probe a concrete local address; wildcard bind addresses can hit a proxy.
  case "${web_host}" in
    0.0.0.0) web_probe_host=127.0.0.1 ;;
    ::|'[::]') web_probe_host='[::1]' ;;
    *) web_probe_host="${web_host}" ;;
  esac
  if command -v ss >/dev/null 2>&1 && ss -ltn 2>/dev/null | grep -q ":${web_port} "; then
    die "端口 ${web_port} 已被占用；换 --port 或先停掉占用者。"
  fi

  web_runtime_dir="${runtime_dir}/web"
  web_logs_dir="${log_root}/web"
  mkdir -p "${web_runtime_dir}" "${web_logs_dir}"

  # 只读硬门在 app/manual_web_demo/{config,web_server,search_routes}.py 里：
  # /api/control/*、/api/estop*、/api/llm/enable、/api/search/{start,resume,estop}
  # 一律 403；控制 WebSocket 输入一律拒收。这里只是把开关打开。
  MANUAL_DEMO_DISPLAY_ONLY=1 \
  MANUAL_DEMO_CONTROL_ENABLED=0 \
  MANUAL_DEMO_LLM_ENABLED=0 \
  MANUAL_DEMO_RUNTIME_DIR="${web_runtime_dir}" \
  MANUAL_DEMO_LOGS_DIR="${web_logs_dir}" \
  GO2W_WEBUI_MODE="${GO2W_WEBUI_MODE:-teleop_mapping}" \
  GO2W_ACTIVE_DETECTOR="${detector}" \
  GO2W_RUNTIME_ADMIN_ENABLED="${GO2W_RUNTIME_ADMIN_ENABLED:-1}" \
  GO2W_SLAM_MAP_SNAPSHOT="${snapshot_path}" \
  GO2W_SLAM_MAP_META="${meta_path}" \
  GO2W_SEMANTIC_OBJECTS_SNAPSHOT="${objects_path}" \
  GO2W_MAPPING_TOPOLOGY_SNAPSHOT="${topology_path}" \
  GO2W_TARGET_WATCH_REQUEST="${target_watch_request}" \
  GO2W_TARGET_WATCH_STATE="${target_watch_state}" \
  GO2W_TARGET_WATCH_EVIDENCE_DIR="${target_watch_evidence_dir}" \
  setsid "${conda_python}" -m uvicorn app.manual_web_demo.web_server:app \
    --host "${web_host}" --port "${web_port}" --timeout-graceful-shutdown 5 \
    >"${log_root}/web_server.log" 2>&1 < /dev/null &
  web_pid=$!
  printf '%s\n' "${web_pid}" >"${web_pidfile}"
  ok "WebUI pid=${web_pid}（只读）"

  web_ready=0
  for _ in $(seq 1 60); do
    if curl --noproxy '*' --max-time 3 -fsS "http://${web_probe_host}:${web_port}/api/status" >/dev/null 2>&1; then
      web_ready=1; break
    fi
    kill -0 "${web_pid}" 2>/dev/null || break
    sleep 0.5
  done
  if [[ "${web_ready}" != "1" ]]; then
    tail -n 25 "${log_root}/web_server.log" >&2 || true
    die 'WebUI 没有就绪。'
  fi

  # 只读门自检：控制入口必须 403，只读端点必须可用。真机上也能一眼看出
  # "开关打开了但门没生效"（比如误配了别的 web_server）。
  enable_code="$(curl --noproxy '*' --max-time 3 -s -o /dev/null -w '%{http_code}' -X POST \
    "http://${web_probe_host}:${web_port}/api/control/enable" || true)"
  status_code="$(curl --noproxy '*' --max-time 3 -s -o /dev/null -w '%{http_code}' \
    "http://${web_probe_host}:${web_port}/api/status" || true)"
  if [[ "${enable_code}" != "403" || "${status_code}" != "200" ]]; then
    die "只读门没生效：/api/control/enable=${enable_code}（应为 403）、/api/status=${status_code}（应为 200）。"
  fi
  ok '只读门自检通过：/api/control/enable=403，/api/status=200'

  web_url="http://${web_host}:${web_port}"
  printf '\nWebUI（只读）: %s\n' "${web_url}"
  if [[ "${open_browser}" == "1" ]] && command -v xdg-open >/dev/null 2>&1; then
    xdg-open "${web_url}" >/dev/null 2>&1 || true
  fi
fi

# ---------------------------------------------------------------------------
# 收尾
# ---------------------------------------------------------------------------
cat <<EOF

================ 实体遥控器主动建图已启动 ================
 机器狗只由 Unitree 实体遥控器操作。本软件不发送任何运动指令。

 观测间隔 : 位移 ${keyframe_dist_m} m / 转角 ${keyframe_yaw_deg}° / 最长 ${keyframe_max_s} s（先到先算）
 队列上限 : ${queue_size}
 检测路线 : ${detector}
 事件     : ${events_path}
 状态     : ${status_path}
 快照     : ${snapshot_path}
 日志     : ${log_root}/

 停机顺序 : ① 先用实体遥控器把机器狗停稳
            ② scripts/go2w/stop_remote_control_mapping.sh
==========================================================
EOF
