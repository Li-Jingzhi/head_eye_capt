#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
case "${1:-}" in
    teach)
        shift
        exec /usr/bin/python3 -u -m interactive_ball_calibration.pro_capture "$@"
        ;;
    replay)
        [[ $# -ge 2 ]] || { echo "Usage: $0 replay TAUGHT_PATH" >&2; exit 2; }
        TAUGHT="$2"
        shift 2
        exec /usr/bin/python3 -u -m interactive_ball_calibration.pro_capture --replay "$TAUGHT" "$@"
        ;;
    *) echo "Usage: $0 {teach|replay TAUGHT_PATH} [--check]" >&2; exit 2 ;;
esac
