#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
LATEST="runtime/new_sh5_yellow/latest_pose.json"
python3 orin_rectified_stream/viewer.py --metadata-only --contract configs/yellow_measurement_v1.json --latest "$LATEST" \
    >/tmp/luna_gq_vpi_viewer.log 2>&1 &
VIEWER_PID=$!
trap 'kill "$VIEWER_PID" 2>/dev/null || true; wait "$VIEWER_PID" 2>/dev/null || true' EXIT
READY=false
for _ in $(seq 1 90); do
    [[ -f "$LATEST" ]] && python3 - "$LATEST" <<'PY' && { READY=true; break; } || true
import json, sys, time
from capt_head.measurement_contract import load_contract, valid_measurement
d=json.load(open(sys.argv[1]))
valid_measurement(d, load_contract()[0])
assert time.monotonic_ns()-d["timestamp_monotonic_ns"] < 1_000_000_000
PY
    sleep 1
done
$READY || { echo "Timed out waiting for fresh VPI/CUDA stereo metadata" >&2; exit 1; }
python3 -m gq_calibration.capture --start --start-pose "${2:-1}" \
    --output "${1:-data/gq_$(date +%Y%m%d_%H%M%S).jsonl}"
