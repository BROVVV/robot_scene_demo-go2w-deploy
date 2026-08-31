#!/usr/bin/env bash
set -Eeuo pipefail

profile="core"
for arg in "$@"; do
  case "$arg" in
    --profile=core|--profile=go2w|--profile=full) profile="${arg#*=}" ;;
    *) printf 'ERROR: unknown argument: %s\n' "$arg" >&2; exit 2 ;;
  esac
done

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
python="${project_root}/.venv/bin/python"
cd "$project_root"

fail() { printf 'FAIL: %s\n' "$*" >&2; exit 2; }
pass() { printf 'PASS: %s\n' "$*"; }

[[ -x "$python" ]] || fail 'project .venv is missing; run bootstrap first'
[[ -f "${project_root}/.env" ]] || fail '.env is missing'

"$python" - <<'PY'
import fastapi, networkx, numpy, openai, pydantic, streamlit, yaml
from app.manual_web_demo.web_server import app
assert app is not None
print('PASS: Python imports and FastAPI application')
PY

(
  cd "$project_root"
  "$python" -m pytest -q \
    tests/test_go2w_experimental_backend.py \
    tests/test_motion_bounds.py \
    tests/test_search_session_archive.py \
    tests/test_search_state_store.py
)
pass 'core regression tests'

bash -n \
  "${project_root}/scripts/bootstrap_fresh_machine.sh" \
  "${project_root}/scripts/go2w/start_autonomous_search_web.sh" \
  "${project_root}/scripts/go2w/start_semantic_exploration.sh"
pass 'launcher syntax'

if [[ "$profile" == go2w || "$profile" == full ]]; then
  [[ -f /opt/ros/humble/setup.bash ]] || fail 'ROS 2 Humble is missing'
  [[ -f "${project_root}/ros2_ws/install/setup.bash" ]] \
    || fail 'project ROS workspace is not built'
  [[ -x "${project_root}/unitree_go2w_control/ros2_ws/install/go2w_motion_control/lib/go2w_motion_control/go2w_motion_action_server" ]] \
    || fail 'Go2-W motion action server is not built'
  [[ -d "${project_root}/ros2_ws/src/hesai_ros_driver/.git" ]] \
    || fail 'Hesai driver source is missing'
  "$python" - <<'PY'
import rclpy
from app.manual_web_demo.search_executor import _resolve_worker_python
assert _resolve_worker_python()
print('PASS: project environment can import ROS 2 rclpy')
PY
  pass 'Go2-W/ROS build artifacts'
fi

if [[ "$profile" == full ]]; then
  grounded="${project_root}/external/Grounded-SAM-2"
  [[ -s "$grounded/checkpoints/sam2.1_hiera_tiny.pt" ]] \
    || fail 'SAM2 checkpoint is missing'
  [[ -s "$grounded/gdino_checkpoints/groundingdino_swint_ogc.pth" ]] \
    || fail 'GroundingDINO checkpoint is missing'
  "$python" - <<'PY'
import groundingdino
import sam2
print('PASS: GroundingDINO and SAM2 imports')
PY
fi

pass "deployment profile ${profile}"
