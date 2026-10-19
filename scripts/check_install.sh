#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
echo '== Local passive CAPT 2DOF deployment =='
python3 -B - <<'PY'
import sys,cv2,numpy
print('Python:',sys.version.split()[0], 'OpenCV:',cv2.__version__, 'NumPy:',numpy.__version__)
PY
echo '== Synthetic mathematical validation =='
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
echo '== CLI and data source checks =='
./run.sh --help >/dev/null
./scripts/capture_drag_right.sh --help >/dev/null
bash -n scripts/capture_stereo_right.sh
bash -n scripts/start_orin_camera.sh scripts/luna_dds_env.sh scripts/capture_gq.sh scripts/capture_chessboard.sh
bash -n scripts/luna_network.sh configs/orin_6_6_7_100_20261014/start_luna_stereo.sh
bash -n interactive_ball_calibration/run.sh
python3 -m py_compile camera_calibration/fit.py gq_calibration/capture.py gq_calibration/fit.py \
  interactive_ball_calibration/geometry.py interactive_ball_calibration/capture.py \
  interactive_ball_calibration/fit.py interactive_ball_calibration/fit_projection.py \
  interactive_ball_calibration/fit_sphere_registration.py \
  interactive_ball_calibration/fit_kinematic_correction.py \
  interactive_ball_calibration/fit_pose9d_consistency.py \
  interactive_ball_calibration/fit_yellow_xyz_consistency.py \
  interactive_ball_calibration/fit_yellow_xyz_v2.py \
  interactive_ball_calibration/yellow_xyz_release.py \
  interactive_ball_calibration/yellow_xyz_infer.py \
  interactive_ball_calibration/upgrade_sphere_centers.py
python3 -m py_compile scripts/luna_stereo_dds_viewer.py
python3 -m py_compile capt_head/measurement_contract.py capt_head/subpixel_ball.py \
  capt_head/four_ball_pose.py capt_head/fixed_head_points.py capt_head/fit_fixed_head_points.py \
  orin_rectified_stream/sender.py orin_rectified_stream/viewer.py orin_rectified_stream/start.py
python3 orin_rectified_stream/start.py --check
bash -n scripts/capture_drag_right.sh scripts/capture_stereo_right.sh \
  scripts/capture_gq.sh scripts/capture_multi_head_replay.sh live_base_comparison/run.sh
python3 scripts/luna_board_capture.py --output /tmp/unused --check
test -f configs/orin_6_6_7_100_20261014/luna_stereo_undistorted.yaml
test -f configs/orin_6_6_7_100_20261014/sh5.yaml
test -f configs/luna_new.ini
echo 'PASS: local software checks passed; this passive check does not remeasure live Orin FPS or FK accuracy.'
echo 'No camera/SDK or motor commands run.'
