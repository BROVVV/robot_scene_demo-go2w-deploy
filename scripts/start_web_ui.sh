#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="${1:-8501}"

cd "$ROOT_DIR"
exec conda run -n go2_robot_scene_demo streamlit run streamlit_app.py \
  --server.address 0.0.0.0 \
  --server.port "$PORT" \
  --server.headless true
