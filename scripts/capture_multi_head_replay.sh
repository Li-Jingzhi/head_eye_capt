#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
PATH_JSON="${1:-data/ball_taught_20261016_104726.json}"
START_POSE="${2:-1}"
STEREO="configs/orin_6_6_7_100_20261014/luna_rectified_stereo.json"
LATEST="runtime/new_sh5_yellow/latest_pose.json"
TAG="$(date +%Y%m%d_%H%M%S)"
TARGETS=(0 -25 -20 -25 20 -25 -40 -25 40 -25 -60 -25 60 -25
         -20 -10 20 -10 -40 -10 40 -10 -60 -10 60 -10
         -20 0 20 0 -40 0 40 0 -60 0 60 0)

test -f "$PATH_JSON" || { echo "Missing taught path: $PATH_JSON" >&2; exit 1; }
test -f "$STEREO" || { echo "Missing stereo calibration: $STEREO" >&2; exit 1; }
if pgrep -x FXPlatform >/dev/null || pgrep -f '/00040701/FX_PLATFORM/UI.py' >/dev/null || \
        pgrep -f 'live_base_comparison.main' >/dev/null; then
    echo "Stop FXPlatform/UI.py/live_base_comparison first; the controller accepts only one client." >&2
    exit 2
fi

LOG="/tmp/luna_multi_head_viewer_$$.log"
python3 orin_rectified_stream/viewer.py --metadata-only --contract configs/yellow_measurement_v1.json --latest "$LATEST" </dev/null >"$LOG" 2>&1 &
VIEWER_PID=$!
trap 'kill "$VIEWER_PID" 2>/dev/null || true; wait "$VIEWER_PID" 2>/dev/null || true' EXIT
READY=false
for _ in $(seq 1 90); do
    kill -0 "$VIEWER_PID" 2>/dev/null || { cat "$LOG" >&2; exit 1; }
    [[ -f "$LATEST" ]] && python3 - "$LATEST" <<'PY' && { READY=true; break; } || true
import json, sys, time
from capt_head.measurement_contract import load_contract, valid_measurement
d=json.load(open(sys.argv[1]))
valid_measurement(d, load_contract()[0])
assert time.monotonic_ns()-d["timestamp_monotonic_ns"] < 1_000_000_000
PY
    sleep 1
done
$READY || { echo "Timed out waiting for fresh matching RTP metadata" >&2; exit 1; }

echo "Each pose moves the head, positions the arm along the retained path, then captures the same reverse waypoints."
for ((i=0; i<${#TARGETS[@]}; i+=2)); do
    (( i/2+1 < START_POSE )) && continue
    yaw="${TARGETS[i]}"
    pitch="${TARGETS[i+1]}"
    output="data/multi_head_${TAG}_yaw${yaw}_pitch${pitch}.jsonl"
    echo "Head pose $((i/2+1))/$(( ${#TARGETS[@]} / 2 )): [$yaw, $pitch] -> $output"
    python3 -m interactive_ball_calibration.capture replay --run \
        --stereo "$STEREO" --latest "$LATEST" --path "$PATH_JSON" --output "$output" \
        --head-target "$yaw" "$pitch"
    python3 - "$output" <<'PY'
import json, sys
from collections import Counter
rows = [json.loads(line) for line in open(sys.argv[1], encoding="utf-8") if line.strip()]
bad = [row for row in rows if not row.get("capture_valid", True)]
print(f"Pose result: valid={len(rows)-len(bad)}/{len(rows)}, "
      f"rejections={dict(Counter(row.get('rejection_reason') for row in bad))}")
PY
done
