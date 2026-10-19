#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
export PYTHONDONTWRITEBYTECODE=1

START=false
for arg in "$@"; do
    [[ "$arg" == "--start" ]] && START=true
done

if ! $START; then
    exec python3 -m capt_head.drag_capture "$@"
fi

python3 orin_rectified_stream/viewer.py --metadata-only \
    --contract configs/yellow_measurement_v1.json \
    --latest runtime/new_sh5_yellow/latest_pose.json \
    >/tmp/luna_yellow_viewer.log 2>&1 &
VIEWER_PID=$!
cleanup() {
    kill "$VIEWER_PID" 2>/dev/null || true
    wait "$VIEWER_PID" 2>/dev/null || true
}
trap cleanup EXIT
sleep 2
if ! kill -0 "$VIEWER_PID" 2>/dev/null; then
    cat /tmp/luna_yellow_viewer.log >&2
    exit 1
fi
python3 -m capt_head.drag_capture "$@"
