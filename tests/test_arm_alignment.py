import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from interactive_ball_calibration.align_arm_frames import align_models
from interactive_ball_calibration.pro_capture import ROOT,flange_pose,mdh_rows
from interactive_ball_calibration.pro_fit import fit,load_capture,predict,read_capture


class ArmAlignmentTest(unittest.TestCase):
    def model(self, side, T):
        return dict(schema='head_eye_pro_fixed_camera_registration_v1', units='m',
                    frame_definition=f'left_camera_from_{side}_arm_MDH_parent',
                    left_camera_from_arm_mdh_T=T.tolist(),flange_to_ball_center_m=[.001,.002,.1],
                    mdh_sha256='mdh',stereo_model_sha256='stereo',source_calibration_sha256='raw')

    def test_alignment_direction_and_pf_local_frames(self):
        C_R=np.array([[0.,-1,0,.2],[1,0,0,-.1],[0,0,1,.4],[0,0,0,1]])
        R_L=np.array([[1.,0,0,.05],[0,0,-1,-.2],[0,1,0,.03],[0,0,0,1]])
        right=self.model('right',C_R);left=self.model('left',C_R@R_L)
        left['flange_to_ball_center_m']=[-.002,.003,.095]
        result=align_models(right,left)
        np.testing.assert_allclose(result['right_mdh_from_left_mdh_T'],R_L,atol=1e-12)
        np.testing.assert_allclose(np.array(result['left_mdh_from_right_mdh_T'])@R_L,np.eye(4),atol=1e-12)
        point=np.array([.1,.2,.3,1.])
        np.testing.assert_allclose(np.linalg.inv(C_R)@(C_R@R_L@point),R_L@point,atol=1e-12)
        self.assertEqual(result['flange_to_ball_center_m']['left'],left['flange_to_ball_center_m'])
        self.assertEqual(result['camera_from_right_mdh_T'],right['left_camera_from_arm_mdh_T'])
        self.assertFalse(result['independently_verified'])

    def test_alignment_rejects_relabelled_arm_or_incompatible_camera(self):
        right=self.model('right',np.eye(4));left=self.model('left',np.eye(4))
        with self.assertRaisesRegex(ValueError,'left-arm'):
            align_models(right,right)
        left['stereo_model_sha256']='different'
        with self.assertRaisesRegex(ValueError,'provenance mismatch'):
            align_models(right,left)

    def test_left_capture_cannot_use_right_chain_contract(self):
        source=ROOT/'datasets/left_drag_replay_20261020_110645_673779/replay_capture.jsonl.gz'
        rows=[json.loads(line) for line in read_capture(source).splitlines()]
        valid=[row for row in rows if row.get('capture_valid') is True][:30]
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'left.jsonl';path.write_text('\n'.join(json.dumps(row) for row in valid))
            with self.assertRaisesRegex(ValueError,'contract mismatch'):
                load_capture(path,ROOT/'configs/orin_pro_rectified_stereo_20261019.json',ROOT/'configs/1004.ini',
                             ROOT/'configs/orin_pro_raw_fisheye_stereo_20261019.json',arm='right')

    def test_left_fit_recovers_known_native_parameters_with_heldout(self):
        rng=np.random.default_rng(20261020)
        q=rng.uniform(-50,50,(70,7));chain=mdh_rows(ROOT/'configs/1004.ini',0)
        poses=np.array([flange_pose(point,chain) for point in q])
        offset=np.array([.001,-.002,.099]);R=np.array([[1.,0,0],[0,0,-1],[0,1,0]]);t=np.array([.1,.2,.3])
        points=predict(poses,offset,R,t)
        rows=[dict(waypoint_index=i,q7_deg=q[i].tolist(),sphere_center_left_camera_m=points[i].tolist()) for i in range(len(q))]
        with patch('interactive_ball_calibration.pro_fit.load_capture',return_value=(rows,0)):
            model,_=fit(ROOT/'datasets/left_drag_replay_20261020_110645_673779/replay_capture.jsonl.gz',
                ROOT/'configs/orin_pro_rectified_stereo_20261019.json',ROOT/'configs/1004.ini',
                ROOT/'configs/orin_pro_raw_fisheye_stereo_20261019.json',arm='left')
        self.assertEqual(model['frame_definition'],'left_camera_from_left_arm_MDH_parent')
        self.assertEqual(model['arm_index'],0)
        np.testing.assert_allclose(model['flange_to_ball_center_m'],offset,atol=1e-7)
        np.testing.assert_allclose(model['left_camera_from_arm_mdh_R'],R,atol=1e-7)
        self.assertLess(model['blocked_cv_metrics']['max_mm'],.001)

    def test_fixed_pf_z_recovers_other_parameters_and_rejects_invalid_constraint(self):
        from interactive_ball_calibration.pro_fit import solve_fixed_pf_z
        from scipy.spatial.transform import Rotation
        rng = np.random.default_rng(101)
        poses = np.repeat(np.eye(4)[None], 50, axis=0)
        poses[:, :3, :3] = Rotation.random(50, random_state=rng).as_matrix()
        poses[:, :3, 3] = rng.uniform(-.3, .3, (50, 3))
        offset = np.array([.001, -.002, .101])
        R = Rotation.from_rotvec([.2, -.4, .1]).as_matrix()
        t = np.array([.1, .2, .3])
        points = predict(poses, offset, R, t)
        fitted, fitted_R, fitted_t, _ = solve_fixed_pf_z(poses, points, .101)
        self.assertEqual(fitted[2], .101)
        np.testing.assert_allclose(fitted, offset, atol=1e-8)
        np.testing.assert_allclose(fitted_R, R, atol=1e-8)
        np.testing.assert_allclose(fitted_t, t, atol=1e-8)
        for value in (float('nan'), .201):
            with self.assertRaises(ValueError): solve_fixed_pf_z(poses, points, value)


if __name__=='__main__':unittest.main()
