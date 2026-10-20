"""Align left-arm calibration to a fixed right-arm MDH reference via the camera."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from .pro_capture import ROOT, digest, flange_pose, mdh_rows
from .pro_fit import load_capture, metrics, read_capture


def registration_matrix(model, side):
    if model.get('units') != 'm':
        raise ValueError('Registration units must be m')
    if model.get('schema') == 'head_eye_pro_fixed_camera_registration_v1':
        if model.get('frame_definition') != f'left_camera_from_{side}_arm_MDH_parent':
            raise ValueError(f'Expected {side}-arm native registration, not relabelled coordinates')
        T = np.asarray(model['left_camera_from_arm_mdh_T'], float)
    elif model.get('schema') == 'head_eye_pro_candidate_observation_model_v1':
        if model.get('input') != f'FK flange pose in {side}-arm MDH parent' or model.get('method') not in (
                'rigid_l2', 'rigid_huber', 'rigid_covariance', 'rigid_pixel'):
            raise ValueError(f'Expected rigid {side}-arm registration')
        T = np.asarray(model['arm_to_camera_T_before_depth_mapping'], float)
    else:
        raise ValueError('Unsupported registration schema')
    if (T.shape != (4, 4) or not np.isfinite(T).all() or
            not np.allclose(T[3], [0, 0, 0, 1], atol=1e-12, rtol=0) or
            not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-8, rtol=0) or
            abs(np.linalg.det(T[:3, :3]) - 1) > 1e-8):
        raise ValueError('Expected a finite rigid homogeneous transform')
    pf = np.asarray(model['flange_to_ball_center_m'], float)
    if pf.shape != (3,) or not np.isfinite(pf).all() or np.linalg.norm(pf) > .2:
        raise ValueError('Invalid native flange offset')
    return T


def align_models(right, left):
    for field in ('mdh_sha256', 'stereo_model_sha256', 'source_calibration_sha256'):
        if not right.get(field) or right[field] != left.get(field):
            raise ValueError(f'Left/right registration provenance mismatch: {field}')
    camera_from_right = registration_matrix(right, 'right')
    camera_from_left = registration_matrix(left, 'left')
    right_from_left = np.linalg.inv(camera_from_right) @ camera_from_left
    return dict(schema='head_eye_pro_dual_arm_alignment_v1', units='m',
                reference_frame='right_arm_mdh_parent', source_frame='left_arm_mdh_parent',
                camera_from_right_mdh_T=camera_from_right.tolist(),
                camera_from_left_mdh_T=camera_from_left.tolist(),
                right_mdh_from_left_mdh_T=right_from_left.tolist(),
                left_mdh_from_right_mdh_T=np.linalg.inv(right_from_left).tolist(),
                flange_to_ball_center_m={side: model['flange_to_ball_center_m']
                                         for side, model in (('right', right), ('left', left))},
                pf_frame_definition='Each offset is in its own arm flange frame; not in the right MDH parent.',
                mdh_sha256=right['mdh_sha256'], stereo_model_sha256=right['stereo_model_sha256'],
                source_calibration_sha256=right['source_calibration_sha256'],
                right_anchor_fixed=True, independently_verified=False,
                warnings=['Right camera extrinsics are fixed, not re-estimated by the left capture.',
                          'Inter-arm transform is derived from two camera registrations, not independently measured.',
                          'Left consistency scores do not include uncertainty in the right anchor.',
                          'MDH parents are not independently verified as physical arm bases.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--right', type=Path, default=ROOT/'reports/pro_recalibration_20261020_360/rigid_candidate.json')
    parser.add_argument('--left', type=Path, default=ROOT/'configs/orin_pro_left_ball_registration_20261020_424.json')
    parser.add_argument('--capture', type=Path, default=ROOT/'datasets/left_drag_replay_20261020_110645_673779/replay_capture.jsonl.gz')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() or args.output.with_suffix('.csv').exists():
        raise FileExistsError('Use a new output path to preserve previous results')
    right, left = [json.loads(path.read_text()) for path in (args.right, args.left)]
    result = align_models(right, left)
    rows, rejected = load_capture(args.capture, ROOT/'configs/orin_pro_rectified_stereo_20261019.json',
        ROOT/'configs/1004.ini', ROOT/'configs/orin_pro_raw_fisheye_stereo_20261019.json', arm='left')
    capture_hash = hashlib.sha256(read_capture(args.capture)).hexdigest()
    if left['capture_uncompressed_sha256'] != capture_hash:
        raise ValueError('Left calibration was fitted on another capture')
    right_from_camera = np.linalg.inv(np.array(result['camera_from_right_mdh_T']))
    right_from_left = np.array(result['right_mdh_from_left_mdh_T'])
    chain = mdh_rows(ROOT/'configs/1004.ini', 0)
    pf = np.array(left['flange_to_ball_center_m'])
    deltas, errors = [], []
    for row in rows:
        pose = right_from_left @ flange_pose(row['q7_deg'], chain)
        vision = right_from_camera[:3, :3] @ row['sphere_center_left_camera_m'] + right_from_camera[:3, 3]
        flange = vision - pose[:3, :3] @ pf
        delta = flange - pose[:3, 3]
        deltas.append(delta)
        errors.append(dict(waypoint_index=row['waypoint_index'], error_mm=float(np.linalg.norm(delta)*1000),
            vision_flange_x_m=flange[0], vision_flange_y_m=flange[1], vision_flange_z_m=flange[2],
            fk_flange_x_m=pose[0,3], fk_flange_y_m=pose[1,3], fk_flange_z_m=pose[2,3]))
    result.update(right_registration_sha256=digest(args.right), left_registration_sha256=digest(args.left),
                  left_capture_sha256=capture_hash, captured_positions=len(rows)+rejected,
                  valid_positions=len(rows), rejected_positions=rejected,
                  left_fit_metrics_in_right_mdh=metrics(np.array(deltas)),
                  left_blocked_cv_distance_metrics={key:value for key,value in left['blocked_cv_metrics'].items()
                                                   if not key.startswith('axis_')},
                  cv_note='Distances are invariant under the fixed camera-to-right rigid transform; left CV does not validate the right anchor.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    with args.output.with_suffix('.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(errors[0])); writer.writeheader(); writer.writerows(errors)
    print(json.dumps(dict(right_mdh_from_left_mdh_T=result['right_mdh_from_left_mdh_T'],
                          flange_to_ball_center_m=result['flange_to_ball_center_m'],
                          left_fit_metrics_in_right_mdh=result['left_fit_metrics_in_right_mdh']), indent=2))


if __name__ == '__main__':
    main()
