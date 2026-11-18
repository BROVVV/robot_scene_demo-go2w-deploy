#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
python_bin="/home/brov/miniconda3/envs/go2_robot_scene_demo/bin/python"
target=""
mode="observe_only"
detector="llm"
semantic_reasoning="false"
search_reasoner="legacy"
search_reasoner_mode="shadow"

while (( $# )); do
  case "$1" in
    --target) target="${2:-}"; shift 2 ;;
    --mode) mode="${2:-}"; shift 2 ;;
    --detector) detector="${2:-}"; shift 2 ;;
    --semantic-reasoning) semantic_reasoning="true"; shift ;;
    --search-reasoner) search_reasoner="${2:-}"; shift 2 ;;
    --search-reasoner-mode) search_reasoner_mode="${2:-}"; shift 2 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
  esac
done
if [[ -z "${target}" ]]; then
  printf '%s\n' '--target is required' >&2
  exit 2
fi
case "${mode}" in
  observe_only|step_search|nav2_plan_only|nav2_execute) ;;
  *) printf 'Unsupported mode: %s\n' "${mode}" >&2; exit 2 ;;
esac
case "${search_reasoner}" in
  legacy|semantic_navigation|hybrid) ;;
  *) printf 'Unsupported search reasoner: %s\n' "${search_reasoner}" >&2; exit 2 ;;
esac
case "${search_reasoner_mode}" in
  shadow|active) ;;
  *) printf 'Unsupported search reasoner mode: %s\n' "${search_reasoner_mode}" >&2; exit 2 ;;
esac

cd "${project_root}"
if [[ -f .env.go2w ]]; then
  set -a
  source .env.go2w
  set +a
fi

if [[ "${mode}" == "step_search" ]]; then
  printf '%s\n' 'BLOCKED: short-step motion is disabled for this deployment and operator session.' >&2
  exit 3
fi

if [[ "${mode}" == nav2_* ]]; then
  gate_file="outputs/go2w_acceptance/navigation_gate/${mode#nav2_}.json"
  set +e
  "${python_bin}" scripts/go2w/evaluate_navigation_gate.py \
    --mode "${mode}" --output "${gate_file}"
  gate_status=$?
  set -e
  if (( gate_status != 0 )); then
    printf 'BLOCKED: %s capability gate is closed. Evidence: %s\n' "${mode}" "${gate_file}" >&2
    exit 3
  fi
fi

if [[ "${mode}" == "nav2_execute" ]]; then
  printf '%s\n' 'Type I_CONFIRM_GO2W_NAVIGATION_EXECUTION to continue:'
  read -r confirmation
  if [[ "${confirmation}" != "I_CONFIRM_GO2W_NAVIGATION_EXECUTION" ]]; then
    printf '%s\n' 'BLOCKED: confirmation phrase mismatch.' >&2
    exit 3
  fi
fi

pid_root="${project_root}/runtime/go2w/pids"
mkdir -p "${pid_root}"
semantic_args=(
  --search-reasoner "${search_reasoner}"
  --search-reasoner-mode "${search_reasoner_mode}"
)
if [[ "${semantic_reasoning}" == "true" ]]; then
  semantic_args+=(--semantic-reasoning)
fi
setsid "${python_bin}" run_live_robot_demo.py \
  --target "${target}" \
  --detector "${detector}" \
  --search-mode "${mode}" \
  --spool-root "${GO2W_FRAME_SPOOL_DIR:-runtime/go2w/spool}" \
  --output-root "${GO2W_SESSION_OUTPUT_DIR:-outputs/live_sessions}" \
  "${semantic_args[@]}" &
search_pid=$!
printf '%s\n' "${search_pid}" >"${pid_root}/search.pid"
cleanup_search() {
  kill -TERM -- "-${search_pid}" 2>/dev/null || true
  wait "${search_pid}" 2>/dev/null || true
  rm -f "${pid_root}/search.pid"
}
trap cleanup_search EXIT INT TERM
set +e
wait "${search_pid}"
search_status=$?
set -e
rm -f "${pid_root}/search.pid"
trap - EXIT INT TERM
exit "${search_status}"
