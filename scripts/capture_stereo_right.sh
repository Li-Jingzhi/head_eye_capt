#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
OUTPUT="data/right_stereo_$(date +%Y%m%d_%H%M%S).jsonl"
echo "Writing synchronized right-arm q7 and stereo yellow pixels to $OUTPUT"
exec bash scripts/capture_drag_right.sh --start \
    --camera-wait-seconds 90 \
    --latest runtime/new_sh5_yellow/latest_pose.json \
    --output "$OUTPUT"
