#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
if pgrep -x FXPlatform >/dev/null; then
    cat >&2 <<'EOF'
Detected the compiled 00040701/FXPlatform upper controller.
The controller accepts only one client. Exit drag mode and close FXPlatform,
then rerun this command; it will connect and enter right-arm DragJoint itself.
EOF
    exit 2
fi
LATEST=runtime/new_sh5_yellow/latest_pose.json
OVERLAY=runtime/live_base_comparison.json
python3 orin_rectified_stream/viewer.py --contract configs/yellow_measurement_v1.json --latest "$LATEST" \
    >/tmp/luna_live_base_comparison_viewer.log 2>&1 &
VIEWER_PID=$!
cleanup() {
    kill "$VIEWER_PID" 2>/dev/null || true
    wait "$VIEWER_PID" 2>/dev/null || true
}
trap cleanup EXIT
READY=false
for _ in $(seq 1 90); do
    kill -0 "$VIEWER_PID" 2>/dev/null || { cat /tmp/luna_live_base_comparison_viewer.log >&2; exit 1; }
    [[ -f "$LATEST" ]] && python3 - "$LATEST" <<'PY' && { READY=true; break; } || true
import json, sys, time
from capt_head.measurement_contract import load_contract, valid_measurement
d=json.load(open(sys.argv[1]))
valid_measurement(d, load_contract()[0])
assert time.monotonic_ns()-d["timestamp_monotonic_ns"] < 1_000_000_000
PY
    sleep 1
done
$READY || { echo "Timed out waiting for fresh calibrated 1280x992 yellow pixels" >&2; exit 1; }
python3 -m live_base_comparison.main --feedback sdk --start-drag "$@"
