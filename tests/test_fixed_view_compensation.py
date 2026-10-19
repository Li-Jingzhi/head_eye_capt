"""Synthetic accuracy, frozen-label validation, bounds and inference equivalence."""
import json
import unittest

import cv2
import numpy as np

from interactive_ball_calibration.depth_compensation import correct, fit
from interactive_ball_calibration.fit_fixed_view_compensation import fit_local, fit_stage
from interactive_ball_calibration.fit_yellow_xyz_v2 import SCHEMA, correction_m, predict_points


class FixedViewCompensationTests(unittest.TestCase):
    def setUp(self):
        rng=np.random.default_rng(217)
        self.camera=rng.uniform([-.08,-.06,.25],[.15,.12,.5],(120,3))
        self.R=cv2.Rodrigues(np.array([.13,-.21,.08]))[0]
        self.t=np.array([.03,-.04,.11])
        self.fb=23.8

    def test_disparity_bias_recovery_on_unseen_points(self):
        true=dict(order=0,coefficients_px=[.22],focal_baseline_px_m=self.fb,max_correction_px=1.)
        corrected=correct(true,self.camera)
        target=(corrected-self.t)@self.R
        model,R,t=fit(self.camera[:90],target[:90],self.fb,order=0,ridge=1e-8)
        predicted=(correct(model,self.camera[90:])-t)@R
        self.assertLess(np.max(np.linalg.norm(predicted-target[90:],axis=1)),1e-6)
        self.assertAlmostEqual(model['coefficients_px'][0],.22,places=4)

    def test_fitting_does_not_read_heldout_labels(self):
        target=(self.camera-self.t)@self.R
        train=np.arange(90)
        a=fit_stage(self.camera,target,train,(0,.1),self.fb)
        target[90:]+=100
        b=fit_stage(self.camera,target,train,(0,.1),self.fb)
        np.testing.assert_allclose(a[3],b[3],atol=1e-12)

    def test_serialized_depth_and_local_correction_matches_online_prediction(self):
        depth=dict(order=1,coefficients_px=[.1,.05,-.03,.02],focal_baseline_px_m=self.fb,max_correction_px=1.)
        base=(correct(depth,self.camera)-self.t)@self.R
        error=np.column_stack([self.camera[:,0]**2,np.zeros(120),np.zeros(120)])*.02
        local=fit_local(self.camera,error,np.arange(120),(.04,.1))
        model=dict(schema=SCHEMA,approved=True,camera_from_mdh_R=self.R.tolist(),
                   camera_from_mdh_t_m=self.t.tolist(),support_points_camera_m=self.camera.tolist(),
                   support_radius_mm=20,residual=dict(kind='depth_then_residual',depth=depth,
                   camera_from_mdh_R=self.R.tolist(),residual=local))
        model=json.loads(json.dumps(model))
        np.testing.assert_allclose(predict_points(model,self.camera),base+correction_m(local,self.camera),atol=1e-12)
        model['approved']=False
        with self.assertRaisesRegex(ValueError,'not approved'):
            predict_points(model,self.camera[0])

    def test_depth_bounds_and_invalid_points_rejected(self):
        model=dict(order=0,coefficients_px=[1.01],focal_baseline_px_m=self.fb,max_correction_px=1.)
        with self.assertRaisesRegex(ValueError,'bounds'):
            correct(model,self.camera)
        model['coefficients_px']=[0.]
        for xyz in ([[0.,0.,0.]],[[0.,0.,float('nan')]]):
            with self.assertRaises(ValueError):
                correct(model,xyz)

    def test_poly_residual_has_consistent_units(self):
        error=np.column_stack([self.camera[:,0],self.camera[:,1],self.camera[:,2]])*.001
        local=fit_local(self.camera,error,np.arange(120),('poly',1,1e-8))
        np.testing.assert_allclose(correction_m(local,self.camera),error,atol=1e-9)


if __name__=='__main__':
    unittest.main()
