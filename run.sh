#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"
export PYTHONDONTWRITEBYTECODE=1
exec python3 -m capt_head.cli "$@"
