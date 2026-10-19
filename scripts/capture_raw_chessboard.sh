#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
TAG="$(date +%Y%m%d_%H%M%S)"
DEST="data/raw_chessboard_${TAG}"
REMOTE="/home/nvidia/raw_chessboard_${TAG}"
SSH=(-Y -o BindInterface=eno0 -o ControlMaster=auto -o ControlPath=/tmp/luna_raw_x11_%C -o ControlPersist=600)
STATUS="runtime/raw_chessboard_status.txt"
echo 'RAW: 12x9 squares = 11x8 INNER corners, square 35 mm. Auto save after holding still; q finishes.'
source scripts/luna_dds_env.sh
echo 'SAVED: 0   Move board to varied poses; hold still to capture' >"$STATUS"
(while true; do
    count=$(ssh "${SSH[@]}" nvidia@6.6.7.100 \
        "find '$REMOTE' -maxdepth 1 -name '*_left.png' 2>/dev/null | wc -l") || count='?'
    echo "SAVED: $count   11x8 inner corners, 35 mm   Ctrl+C: finish" >"$STATUS"
    sleep 1
done) &
POLL_PID=$!
python3 scripts/luna_stereo_dds_viewer.py --raw --status-file "$STATUS" >/tmp/luna_raw_color_viewer.log 2>&1 &
VIEWER_PID=$!
trap 'kill "$VIEWER_PID" "$POLL_PID" 2>/dev/null || true; wait "$VIEWER_PID" "$POLL_PID" 2>/dev/null || true' EXIT
ssh "${SSH[@]}" nvidia@6.6.7.100 \
    "source /opt/ros/humble/setup.bash; source /opt/kernelmind/apex/install/setup.bash; source /home/nvidia/apex_gento_devel/install/setup.bash; export ROS_DOMAIN_ID=20 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; python3 /home/nvidia/luna_board_capture_raw.py --output '$REMOTE' --seconds 3600"
scp -r -o BindInterface=eno0 -o ControlPath=/tmp/luna_raw_x11_%C "nvidia@6.6.7.100:$REMOTE" "$DEST"
python3 -m camera_calibration.fit_raw --images "$DEST" --output "data/raw_stereo_${TAG}.json"
