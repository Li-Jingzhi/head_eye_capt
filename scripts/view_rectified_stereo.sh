#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
source scripts/luna_dds_env.sh
python3 scripts/luna_stereo_dds_viewer.py --raw \
  --rectify configs/orin_6_6_7_100_20261014/luna_raw_fisheye_stereo.json
