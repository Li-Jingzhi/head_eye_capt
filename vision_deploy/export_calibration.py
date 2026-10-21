"""Export repo calibration to a self-contained deployment bundle (no device access)."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import numpy as np

from stereo_robot.geometry import CalibrationError, StereoRobot, array, rotation, transform
from stereo_robot.robot_geometry import read_robot_model


ROOT = Path(__file__).resolve().parents[1]
JOINT = ROOT/'reports/shared_pf_joint_kernel_20261021/affine_l60_a3.json'


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
                parameter_constraints=model.get('parameter_constraints', {}),
                stereo_model_sha256=stereo_hash, mdh_sha256=mdh_hash,
                registration_sha256=sha(path), registration_source=Path(path).name,
                source_capture_sha256=model.get('source_capture_sha256', model.get('capture_uncompressed_sha256')),
                camera_calibration_bounds_m=bounds, validation_metrics=metrics)


def joint_profiles(path, raw_hash, stereo_hash, mdh_hash):
    path = Path(path)
    model = json.loads(path.read_text())
    comparison = json.loads((path.parent/'comparison.json').read_text())
    variant = model['variant']
    if (model.get('schema') != 'shared_pf_joint_gaussian_experiment_v1' or
            model.get('units') != 'm' or model.get('map_kind') != 'affine' or
            model.get('correction_kind') != 'gaussian' or comparison['selected'] != variant or
            comparison['candidates'][variant]['model_file'] != path.name):
        raise CalibrationError('Expected the selected joint affine/Gaussian model and comparison evidence')
    arms = {}
    for side in ('right', 'left'):
        source = comparison['data'][side]
        archive = ROOT/source['path']/source['manifest']['files'][0]['file']
        if sha(archive) != source['manifest']['files'][0]['compressed_sha256']:
            raise CalibrationError('Joint capture provenance mismatch')
        with gzip.open(archive, 'rt') as stream:
            record = json.loads(next(stream))
        if (record['source_calibration_sha256'], record['stereo_model_sha256'], record['mdh_sha256']) != (raw_hash, stereo_hash, mdh_hash):
            raise CalibrationError('Joint model camera/MDH sources disagree with export')
        arms[side] = dict(side=side, units='m', mapping_model='shared_gaussian_affine',
            observation_from_mdh_T=model['observation_from_mdh_T'][side],
            base_from_mdh_T=None, base_frame=None, base_frame_verified=False,
            independently_verified=False, stereo_model_sha256=stereo_hash, mdh_sha256=mdh_hash,
            registration_sha256=sha(path), registration_source=path.name,
            source_capture_sha256=source['manifest']['files'][0]['uncompressed_sha256'],
            parameter_constraints=dict(shared_pf=True, kernel_length_m=model['kernel_length_m'],
                                       kernel_regularization=model['kernel_regularization']),
            camera_calibration_bounds_m=model['training_camera_bounds_m'][side],
            validation_metrics=comparison['candidates'][variant]['cv'][side])
    correction = dict(schema='shared_gaussian_observation_correction_v1', units='m',
                      source_model_sha256=sha(path), **{key:model[key] for key in (
                          'variant', 'shared_pf_m', 'kernel_length_m', 'kernel_regularization',
                          'kernel_centers_m', 'kernel_coefficients_m')})
    alignment = dict(schema='stereo_robot_affine_dual_arm_alignment_v1', units='m',
        reference_frame='right_arm_mdh_parent', source_frame='left_arm_mdh_parent',
        right_mdh_from_left_mdh_T=(np.linalg.inv(np.asarray(model['observation_from_mdh_T']['right'])) @
                                   np.asarray(model['observation_from_mdh_T']['left'])).tolist(),
        flange_to_ball_center_m={side:model['shared_pf_m'] for side in ('right','left')},
        right_anchor_fixed=False, independently_verified=False,
        mapping_kind='affine',
        pf_usage='Same local flange offset for both arms; calibration evidence only, never applied to generic target XYZ.',
        note='Common observation field cancels in the interarm map; affine alignment is not an independently measured rigid base transform.')
    quality = dict(variant=variant, source_model_sha256=sha(path),
        comparison_sha256=sha(path.parent/'comparison.json'),
        candidate_cv=comparison['candidates'][variant]['cv'], nested_selection_cv=comparison['nested'],
        independent_rigid_baseline_cv=comparison['baseline'],
        warnings=[w for w in comparison['warnings'] if not w.startswith('Deployment unchanged')]+
                 ['This joint model is deployed; both arm maps were re-estimated, without a fixed prior right anchor.'])
    return arms, correction, alignment, quality


def export_bundle(destination, raw, stereo, mdh, right=None, left=None, settings=None, joint=None):
    """Build and validate in a staging directory, then publish to a new path."""
    destination = Path(destination).resolve()
    if destination.exists():
        raise FileExistsError('Use a new bundle directory; existing calibrations are not overwritten')
    raw_hash, stereo_hash, mdh_hash = sha(raw), sha(stereo), sha(mdh)
    if joint:
        if right or left:
            raise CalibrationError('Joint and legacy per-arm registrations are mutually exclusive')
        arms, correction, alignment, quality = joint_profiles(joint, raw_hash, stereo_hash, mdh_hash)
    else:
        if right is None:
            raise CalibrationError('Supply a joint model or an explicit legacy right registration')
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
        if joint:
            for name,value in [('observation_correction.json', correction), ('arm_alignment.json', alignment),
                               ('calibration_quality.json', quality)]:
                (staging/name).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
        elif left:
            right_model, left_model = [json.loads(Path(path).read_text()) for path in (right, left)]
            alignment = dict(schema='head_eye_pro_dual_arm_alignment_v1', units='m',
                reference_frame='right_arm_mdh_parent', source_frame='left_arm_mdh_parent',
                right_mdh_from_left_mdh_T=(np.linalg.inv(np.asarray(arms['right']['camera_from_mdh_T'])) @
                                           np.asarray(arms['left']['camera_from_mdh_T'])).tolist(),
                flange_to_ball_center_m={side: array(model['flange_to_ball_center_m'],(3,),'pF').tolist()
                                        for side, model in (('right',right_model),('left',left_model))},
                parameter_constraints={side: model.get('parameter_constraints', {}) for side, model in (('right',right_model),('left',left_model))},
                pf_usage='Calibration evidence in each native flange frame; never used for generic target XYZ.',
                right_registration_sha256=sha(right), left_registration_sha256=sha(left),
                right_anchor_fixed=True, independently_verified=False)
            (staging / 'arm_alignment.json').write_text(json.dumps(alignment,indent=2,allow_nan=False)+'\n')
        manifest = dict(schema='stereo_robot_point_bundle_v3' if joint else 'stereo_robot_point_bundle_v2',
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
    parser.add_argument('--joint', type=Path, help='Joint affine/Gaussian model; default is the latest selected model')
    parser.add_argument('--right', type=Path, help='Explicit legacy rigid right registration, disables default joint export')
    parser.add_argument('--left', type=Path, help='Optional legacy left registration, requires --right')
    parser.add_argument('--settings', type=Path, help='Optional measured arm-base transforms and frame names')
    args = parser.parse_args()
    if args.left and not args.right:
        parser.error('--left requires an explicit --right')
    joint = args.joint or (JOINT if args.right is None else None)
    print(export_bundle(args.output, args.raw, args.stereo, args.mdh, args.right, args.left, args.settings, joint))


if __name__ == '__main__':
    main()
