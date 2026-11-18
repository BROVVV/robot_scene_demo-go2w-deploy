#!/usr/bin/env bash
# 本地 VLM（机器狗上的 llama-server）接入后的静态端到端回归。
#
# 与 run_semantic_static.sh 的唯一区别：daemon 带 `GO2W_LOCAL_VLM_VERIFY=1`
# 启动。于是链路变成
#
#   相机图像 → 候选（云端 quick，不变） → **本地 3B 复核** → 高层建议动作
#
# 全程 `--dry-run-motion`：完整跑感知/推理，但一条运动指令都不发
# （`run_semantic_exploration.py:3194` 在 execute_step 里提前返回）。
#
# 两道硬防线，缺一条这个回归就是自欺：
#
#  1. **本地服务不可达就直接退出**。若放任它跑，`_verify_detect` 会抛异常、
#     上层记 VERIFY_ERROR，而日志里看不出"打的是云端还是本地" —— 结果是
#     一场看起来跑通了、实际根本没验证本地接入的假绿。
#  2. **必须先 stop 再 start daemon**。`start_vlm_daemon.sh` 见到已在跑的
#     pid 会直接 `exit 0`，旧进程不会带上新导出的环境变量；而 `get_settings()`
#     是 lru_cache，环境只在进程启动时读一次。不重启 = 开关设了个寂寞。
#
# 前置：工作站上要有一条到机器狗 18081 的转发，例如
#   ssh -F /dev/null -N -L 18081:127.0.0.1:18081 unitree@192.168.123.18
# （`-F /dev/null` 是绕开 ~/.ssh/config 里会撞端口的 RemoteForward）
set -uo pipefail
cd /home/mxt/robotscene

LOCAL_BASE_URL="${GO2W_LOCAL_VLM_BASE_URL:-http://127.0.0.1:18081/v1}"
LOCAL_IMAGES="${GO2W_LOCAL_VLM_VERIFY_IMAGES:-full+crop}"
TARGET="${TARGET:-绿色垃圾桶}"
MAX_SECONDS="${MAX_SECONDS:-150}"

RUN_ID="localVlmStatic_$(date +%Y%m%d_%H%M%S)"
OUT="artifacts/slam_validation/${RUN_ID}"
mkdir -p "$OUT"
echo "$OUT" > /tmp/local_vlm_static_dir.txt
echo "run_id=$RUN_ID out=$OUT target=$TARGET images=$LOCAL_IMAGES max_seconds=$MAX_SECONDS"

# ---- 0. 本地服务必须真的可达（--noproxy 绕开 ~/.profile 里的常驻代理）------
if ! curl -sS --noproxy '*' -m 5 -o /dev/null "${LOCAL_BASE_URL%/v1}/health"; then
  echo "ERROR: 本地 llama-server 不可达：${LOCAL_BASE_URL%/v1}/health" >&2
  echo "       先架转发：ssh -F /dev/null -N -L 18081:127.0.0.1:18081 unitree@192.168.123.18" >&2
  exit 2
fi

# ---- 1. 带开关重启 daemon -------------------------------------------------
export GO2W_LOCAL_VLM_VERIFY=1
export GO2W_LOCAL_VLM_BASE_URL="$LOCAL_BASE_URL"
export GO2W_LOCAL_VLM_MODEL="${GO2W_LOCAL_VLM_MODEL:-lfm-local}"
export GO2W_LOCAL_VLM_API_KEY="${GO2W_LOCAL_VLM_API_KEY:-sk-local}"
export GO2W_LOCAL_VLM_VERIFY_IMAGES="$LOCAL_IMAGES"
export GO2W_LOCAL_VLM_TIMEOUT_SECONDS="${GO2W_LOCAL_VLM_TIMEOUT_SECONDS:-180}"

bash scripts/go2w/stop_vlm_daemon.sh > "$OUT/daemon_stop.log" 2>&1 || true
bash scripts/go2w/start_vlm_daemon.sh > "$OUT/daemon_start.log" 2>&1 || true
cat "$OUT/daemon_stop.log" "$OUT/daemon_start.log"

# 进程起来不代表开关生效：从 daemon 自己的 /proc 环境里回读，眼见为实。
daemon_pid="$(tr -cd '0-9' < runtime/go2w/pids/siliconflow_vlm.pid 2>/dev/null || true)"
if [[ -n "$daemon_pid" ]]; then
  echo "--- daemon pid=$daemon_pid 实际环境 ---"
  tr '\0' '\n' < "/proc/${daemon_pid}/environ" | grep '^GO2W_LOCAL_VLM' | tee "$OUT/daemon_env.txt"
  echo "daemon_started_at=$(stat -c %y "/proc/${daemon_pid}" 2>/dev/null)" | tee -a "$OUT/daemon_env.txt"
fi

# ---- 2. 定点搜索（干跑，不发运动指令）--------------------------------------
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE CONDA_SHLVL
source scripts/go2w/setup_environment.sh > "$OUT/env.log" 2>&1
export PYTHONPATH="${HOME}/.local/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
PY=/usr/bin/python3

timeout $((MAX_SECONDS + 120)) "$PY" scripts/go2w/run_semantic_exploration.py \
  --target "$TARGET" --backend go2w_experimental \
  --rgbd-source --dry-run-motion --spatial-v2 --spatial-provider plain_slam \
  --max-seconds "$MAX_SECONDS" --session-dir "$OUT" --output "$OUT/events.jsonl" \
  --operator-supervised-experiment \
  --profile-config configs/go2w/high_level_experiment.yaml \
  > "$OUT/worker.log" 2>&1
echo "worker_rc=$?" >> "$OUT/worker.log"

# ---- 3. 现场速判 -----------------------------------------------------------
echo "--- 运动指令是否泄漏（应全空/全 dry_run）---"
grep -oE '"dry_run_motion"|"cmd_vel"|"velocity_command"|"motion_command"' "$OUT/events.jsonl" 2>/dev/null | sort | uniq -c
echo "--- 复核事件 ---"
"$PY" - "$OUT/events.jsonl" <<'PY' 2>&1 | tail -40
import json, sys
from pathlib import Path
p = Path(sys.argv[1])
if not p.is_file():
    print("没有 events.jsonl")
    raise SystemExit(1)
n_verify = n_err = 0
for line in p.read_text(encoding="utf-8", errors="replace").splitlines():
    try:
        ev = json.loads(line)
    except Exception:
        continue
    blob = json.dumps(ev, ensure_ascii=False)
    if "verif" in blob.lower():
        n_verify += 1
        if "VERIFY_TIMEOUT" in blob or "VERIFY_ERROR" in blob:
            n_err += 1
        print(blob[:400])
print(f"\n[汇总] 含 verify 的事件 {n_verify} 条，其中超时/出错 {n_err} 条")
PY

echo "DONE $OUT" | tee /tmp/local_vlm_static_status.txt
