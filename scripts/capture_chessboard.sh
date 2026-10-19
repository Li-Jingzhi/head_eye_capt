#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
source scripts/luna_network.sh
source scripts/luna_dds_env.sh
bash scripts/start_orin_camera.sh
python3 scripts/luna_stereo_dds_viewer.py --board-dir "${1:-data/chessboard_$(date +%Y%m%d_%H%M%S)}"
