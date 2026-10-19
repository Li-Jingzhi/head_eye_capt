"""Synthetic unseen-angle recovery and exposure-time regression checks only."""
import copy
import unittest
from pathlib import Path

import numpy as np

from interactive_ball_calibration.continuous_head import (
    SCHEMA, coverage, fit_geometry, head_mdh, head_poses, map_points,
    pose_at_exposure, query)
from interactive_ball_calibration.geometry import flange_pose

MDH=Path(__file__).resolve().parents[1]/'configs/luna_new.ini'


class ContinuousHeadTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rows=head_mdh(MDH)
        cls.truth=dict(schema=SCHEMA,head_mdh=cls.rows.tolist(),
                       mount_parameters=[.2,-.1,.15,.04,-.03,.05],
                       mdh_from_head_parent_R=np.eye(3).tolist(),
                       mdh_from_head_parent_t_m=[.1,.02,-.5])
        cls.grid=np.array([(y,p) for y in (-10,0,10) for p in (-30,-25,-20)],float)
        rng=np.random.default_rng(44)
        cls.angles=np.repeat(cls.grid,20,axis=0)
        cls.camera=rng.uniform([-.1,-.1,.25],[.1,.1,.5],(180,3))
        target=map_points(cls.truth,cls.camera,cls.angles)
        cls.fitted=fit_geometry(cls.camera,target,cls.angles,cls.rows)

    def certified_fixture(self):
        model=copy.deepcopy(self.truth)
        model.update(approved=True,do_not_deploy=False,coverage=coverage(self.grid),
                     validation=dict(unseen_head_static_passed=True,dynamic_passed=False,
                                     dynamic_speed_limit_deg_s=[20.,20.]),
                     timing_radius_m=1.,max_timing_error_mm=.2,support_radius_mm=20.)
        for key in ('mdh_sha256','head_mdh_sha256','stereo_model_sha256','source_calibration_sha256',
                    'pixel_domain','image_size','sensor_ids','sphere_center_method'):
            model[key]='synthetic_'+key
        x=np.array([.03,0,.4]);angle=np.array([0.,-25.])
        point=map_points(model,x[None,:],angle[None,:])[0]
        model['support_mdh_m']=[point.tolist()]
        obs={key:model[key] for key in ('mdh_sha256','head_mdh_sha256','stereo_model_sha256',
             'source_calibration_sha256','pixel_domain','image_size','sensor_ids','sphere_center_method')}
        obs.update(capture_valid=True,sphere_center_left_camera_m=x.tolist(),
                   head_angles_deg=angle.tolist(),measurement_mode='stationary_fused',
                   valid_frames=50,robot_window_head_range_deg=[0.,0.],head_velocity_deg_s=[0.,0.])
        return model,obs,point

    def history(self):
        return [dict(timestamp_ns=1_000_000_000,head_angles_deg=[-.1,-25.],clock_id='verified'),
                dict(timestamp_ns=1_020_000_000,head_angles_deg=[.1,-25.],clock_id='verified')]

    def moving(self,obs):
        obs.update(measurement_mode='moving_single_frame',valid_frames=1,exposure_time_valid=True,
                   left_exposure_timestamp_ns=1_010_000_000,right_exposure_timestamp_ns=1_011_000_000,
                   timestamp_uncertainty_ns=1000,clock_id='verified')

    def test_mdh_matches_existing_modified_dh_convention(self):
        pose=head_poses(self.rows,[[8.,-22.]])[0]
        expected=flange_pose([8.,-22.,0,0,0,0,0],self.rows)
        np.testing.assert_allclose(pose,expected,atol=1e-12)

    def test_recovers_unseen_angles_in_noise_free_synthetic_data(self):
        rng=np.random.default_rng(19)
        camera=rng.uniform([-.1,-.1,.25],[.1,.1,.5],(20,3))
        angles=rng.uniform([-8,-29],[8,-21],(20,2))
        error=map_points(self.fitted,camera,angles)-map_points(self.truth,camera,angles)
        self.assertLess(np.max(np.linalg.norm(error,axis=1))*1000,.05)

    def test_current_three_heads_do_not_qualify_arbitrary_angle_coverage(self):
        c=coverage([[-10,-25],[10,-25],[0,-20]])
        self.assertFalse(c['coverage_qualified'])
        self.assertEqual(c['pitch_settings'],2)
        self.assertTrue(coverage(self.grid)['coverage_qualified'])

    def test_static_unseen_angle_query_and_approval_hull_guards(self):
        model,obs,point=self.certified_fixture()
        obs['head_angles_deg']=[2.,-24.]
        point=map_points(model,np.array([obs['sphere_center_left_camera_m']]),np.array([[2.,-24.]]))[0]
        model['support_mdh_m']=[point.tolist()]
        np.testing.assert_allclose(query(model,obs),point,atol=1e-12)
        bad=copy.deepcopy(obs);bad['head_angles_deg']=[30.,-25.]
        with self.assertRaisesRegex(ValueError,'hull'):query(model,bad)
        bad=copy.deepcopy(obs);bad['head_velocity_deg_s']=[1.,0.]
        with self.assertRaisesRegex(ValueError,'Stopped'):query(model,bad)
        model['approved']=False
        with self.assertRaisesRegex(ValueError,'not independently approved'):query(model,obs)

    def test_exposure_interpolation_and_no_extrapolation_or_wrong_clock(self):
        angle,speed,span=pose_at_exposure(self.history(),1_010_000_000,'verified')
        np.testing.assert_allclose(angle,[0.,-25.]);np.testing.assert_allclose(speed,[10.,0.])
        self.assertEqual(span,20.)
        with self.assertRaises(ValueError):pose_at_exposure(self.history(),900_000_000,'verified')
        with self.assertRaisesRegex(ValueError,'clock'):pose_at_exposure(self.history(),1_010_000_000,'different')
        bad=self.history();bad.reverse()
        with self.assertRaisesRegex(ValueError,'unordered'):pose_at_exposure(bad,1_010_000_000,'verified')

    def test_moving_requires_validation_exposure_sync_speed_and_single_frame(self):
        model,obs,point=self.certified_fixture();self.moving(obs)
        with self.assertRaisesRegex(ValueError,'not validated'):query(model,obs,history=self.history())
        model['validation']['dynamic_passed']=True
        np.testing.assert_allclose(query(model,obs,history=self.history()),point,atol=1e-12)
        for key,value in [('valid_frames',50),('exposure_time_valid',False)]:
            bad=copy.deepcopy(obs);bad[key]=value
            with self.assertRaisesRegex(ValueError,'single synchronized'):query(model,bad,history=self.history())
        bad=copy.deepcopy(obs);bad['right_exposure_timestamp_ns']=1_020_000_000
        with self.assertRaisesRegex(ValueError,'timing budget'):query(model,bad,history=self.history())
        model['validation']['dynamic_speed_limit_deg_s']=[5.,5.]
        with self.assertRaisesRegex(ValueError,'speed range'):query(model,obs,history=self.history())


if __name__=='__main__':unittest.main()
