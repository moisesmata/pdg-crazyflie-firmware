#!/usr/bin/env bash
# Repository-local dependencies; ROS Jazzy must already be installed.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
command -v uv >/dev/null || { echo 'Install uv first.' >&2; exit 2; }
[[ -f /opt/ros/jazzy/setup.bash ]] || { echo 'Install ROS 2 Jazzy first.' >&2; exit 2; }
export UV_CACHE_DIR="$ROOT/.cache/uv"
mkdir -p "$UV_CACHE_DIR"
if [[ ! -x .venv/bin/python ]]; then
  uv venv --python /usr/bin/python3 --system-site-packages .venv
fi
uv sync --locked --extra dev --inexact
uv pip install --python .venv/bin/python cflib==0.1.34 motioncapture==1.1 vcstool==0.3.0
printf 'Next: source scripts/local_env.sh\n'
