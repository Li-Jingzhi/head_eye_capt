"""Export repo calibration to a self-contained deployment bundle (no device access)."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import numpy as np

from stereo_robot.geometry import CalibrationError, StereoRobot, array, rotation, transform
from stereo_robot.robot_geometry import read_robot_model


ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def arm_profile(path, side, raw_hash, stereo_hash, mdh_hash):
    model = json.loads(Path(path).read_text())
    if model.get('units') != 'm' or model.get('stereo_model_sha256') != stereo_hash:
        raise CalibrationError(f'{side} registration units/stereo hash mismatch')
    if model.get('schema') == 'head_eye_pro_fixed_camera_registration_v1':
        if model.get('frame_definition') != f'left_camera_from_{side}_arm_MDH_parent':
            raise CalibrationError(f'Registration is not for the {side} arm; do not relabel right-arm data as left')
        if model.get('source_calibration_sha256') != raw_hash:
            raise CalibrationError('Registration raw calibration hash mismatch')
        T = transform(model['left_camera_from_arm_mdh_T'], 'camera_from_mdh_T')
        if not (abs(T[:3, :3] - rotation(model['left_camera_from_arm_mdh_R'], 'registration R')).max() < 1e-10
                and abs(T[:3, 3] - array(model['left_camera_from_arm_mdh_t_m'], (3,), 'registration t')).max() < 1e-10):
            raise CalibrationError('Registration T differs from R/t')
        bounds = model.get('camera_xyz_bounds_m')
        metrics = model.get('blocked_cv_metrics')
    elif model.get('schema') == 'head_eye_pro_candidate_observation_model_v1':
        if model.get('method') not in ('rigid_l2', 'rigid_huber', 'rigid_covariance', 'rigid_pixel'):
            raise CalibrationError('Only rigid models can be exported as geometric extrinsics')
        expected_frame = f'FK flange pose in {side}-arm MDH parent'
        if model.get('input') != expected_frame:
            raise CalibrationError(f'Registration is not for the {side} arm')
        T = transform(model['arm_to_camera_T_before_depth_mapping'], 'camera_from_mdh_T')
        bounds = dict(min=model['bounds']['camera_min_m'], max=model['bounds']['camera_max_m'])
        metrics = model.get('cv_metrics')
    else:
        raise CalibrationError('Unsupported registration schema')
    if model.get('mdh_sha256') != mdh_hash:
        raise CalibrationError('Registration robot geometry hash mismatch')
    return dict(side=side, units='m', camera_from_mdh_T=T.tolist(),
                base_from_mdh_T=None, base_frame=None, base_frame_verified=False,
                independently_verified=model.get('independently_verified') is True,
                stereo_model_sha256=stereo_hash, mdh_sha256=mdh_hash,
                registration_sha256=sha(path), registration_source=Path(path).name,
                source_capture_sha256=model.get('source_capture_sha256', model.get('capture_uncompressed_sha256')),
                camera_calibration_bounds_m=bounds, validation_metrics=metrics)


def export_bundle(destination, raw, stereo, mdh, right, left=None, settings=None):
    """Build and validate in a staging directory, then publish to a new path."""
    destination = Path(destination).resolve()
    if destination.exists():
        raise FileExistsError('Use a new bundle directory; existing calibrations are not overwritten')
    raw_hash, stereo_hash, mdh_hash = sha(raw), sha(stereo), sha(mdh)
    arms = {'right': arm_profile(right, 'right', raw_hash, stereo_hash, mdh_hash),
            'left': arm_profile(left, 'left', raw_hash, stereo_hash, mdh_hash) if left else None}
    if settings:
        for side, values in json.loads(Path(settings).read_text()).items():
            if side not in arms or arms[side] is None:
                raise CalibrationError(f'Cannot configure uncalibrated arm: {side}')
            allowed = {'base_from_mdh_T', 'base_frame', 'base_frame_verified'}
            if not set(values).issubset(allowed):
                raise CalibrationError('Unknown arm settings; use only base transform/frame and verification')
            arms[side].update(values)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent, prefix='.calibration-') as temporary:
        staging = Path(temporary) / 'bundle'; staging.mkdir()
        files = {'raw_fisheye.json': raw, 'rectified_stereo.json': stereo, 'robot_geometry.ini': mdh}
        for filename, source in files.items():
            shutil.copyfile(source, staging / filename)
        (staging/'robot_model.json').write_text(json.dumps(read_robot_model(staging/'robot_geometry.ini'),
                                                          indent=2,allow_nan=False)+'\n')
        (staging / 'arms.json').write_text(json.dumps(arms, indent=2, allow_nan=False) + '\n')
        if left:
            right_model, left_model = [json.loads(Path(path).read_text()) for path in (right, left)]
            alignment = dict(schema='head_eye_pro_dual_arm_alignment_v1', units='m',
                reference_frame='right_arm_mdh_parent', source_frame='left_arm_mdh_parent',
                right_mdh_from_left_mdh_T=(np.linalg.inv(np.asarray(arms['right']['camera_from_mdh_T'])) @
                                           np.asarray(arms['left']['camera_from_mdh_T'])).tolist(),
                flange_to_ball_center_m={side: array(model['flange_to_ball_center_m'],(3,),'pF').tolist()
                                        for side, model in (('right',right_model),('left',left_model))},
                pf_usage='Calibration evidence in each native flange frame; never used for generic target XYZ.',
                right_registration_sha256=sha(right), left_registration_sha256=sha(left),
                right_anchor_fixed=True, independently_verified=False)
            (staging / 'arm_alignment.json').write_text(json.dumps(alignment,indent=2,allow_nan=False)+'\n')
        manifest = dict(schema='stereo_robot_point_bundle_v2',
                        sha256={p.name: sha(p) for p in sorted(staging.iterdir())},
                        camera_transfer_condition='Same physical stereo rig and unchanged camera-to-arm mounting; new hardware/mount requires recalibration.')
        (staging / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        StereoRobot(staging)
        staging.rename(destination)
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--raw', type=Path, default=ROOT / 'configs/orin_pro_raw_fisheye_stereo_20261019.json')
    parser.add_argument('--stereo', type=Path, default=ROOT / 'configs/orin_pro_rectified_stereo_20261019.json')
    parser.add_argument('--mdh', type=Path, default=ROOT / 'configs/1004.ini')
    parser.add_argument('--right', type=Path, default=ROOT / 'reports/pro_recalibration_20261020_360/rigid_candidate.json')
    parser.add_argument('--left', type=Path, default=ROOT/'configs/orin_pro_left_ball_registration_20261020_424.json',
                        help='Left-arm native registration, explicitly identified as left')
    parser.add_argument('--settings', type=Path, help='Optional measured arm-base transforms and frame names')
    args = parser.parse_args()
    print(export_bundle(args.output, args.raw, args.stereo, args.mdh, args.right, args.left, args.settings))


if __name__ == '__main__':
    main()
