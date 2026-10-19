"""Accuracy and uncertainty checks, plus regressions for invalid precision data."""
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from interactive_ball_calibration.gaussian_process import fit, predict
from interactive_ball_calibration.capture import fuse_camera_observations, assert_fixed
from interactive_ball_calibration.fit_yellow_xyz_consistency import _load, metrics
from interactive_ball_calibration.fit_yellow_xyz_v2 import correction_m
try:
    import test_yellow_xyz_consistency as helper
except ModuleNotFoundError:
    import tests.test_yellow_xyz_consistency as helper


class GaussianProcessPrecisionTests(unittest.TestCase):
    def test_unseen_smooth_residual_and_serialized_prediction(self):
        rng=np.random.default_rng(33)
        x=rng.uniform([-.05,-.05,.3],[.10,.10,.45],(90,3))
        def signal(a):
            return np.column_stack([.5*np.sin(a[:,0]/.04), .3*a[:,1]/.1, .7*np.cos(a[:,2]/.1)])/1000
        model=fit(x[:70],signal(x[:70]))
        model=json.loads(json.dumps(model))
        mean,std=predict(model,x[70:],with_std=True)
        self.assertLess(np.percentile(np.linalg.norm(mean-signal(x[70:]),axis=1)*1000,95),.3)
        np.testing.assert_allclose(correction_m(model,x[70:]),mean)
        self.assertTrue(np.isfinite(std).all())
        self.assertTrue((std>0).all())
        _,far=predict(model,x[70:]+1.,with_std=True)
        self.assertGreater(far.mean(),std.mean())

    def test_quality_proxy_validation_and_constant_output(self):
        x=np.random.default_rng(15).uniform([0,0,.3],[.1,.1,.4],(15,3))
        for quality in (np.zeros(15),np.full(15,np.nan),np.ones(14)):
            with self.assertRaises(ValueError):
                fit(x,np.zeros_like(x),quality=quality)
        model=fit(x,np.zeros_like(x),quality=np.ones(15))
        np.testing.assert_allclose(predict(model,x),0,atol=1e-15)


class CapturePrecisionRegressionTests(unittest.TestCase):
    def observations(self, feedback=False):
        rows=[]
        for i in range(50):
            pixels=[650.,500.,585.,500.]
            frame={'frame_number':i}
            if feedback:
                frame.update(capture_q7_deg=[0.]*7,capture_head_deg=[0.,-25.],capture_feedback_monotonic_ns=i)
            sphere=dict(center_left_camera_m=[.05,0.,.4+i*1e-7],corrected_stereo_pixels=pixels,
                        projection_correction_px=[0.]*4,stereo_reprojection_error_px=.01)
            rows.append((pixels,i,.01,frame,sphere))
        return rows

    def test_nonconsecutive_duplicate_cannot_count_as_new_frame(self):
        rows=self.observations();rows[-1]=rows[0]
        with self.assertRaisesRegex(ValueError,'Repeated camera'):
            fuse_camera_observations(rows)

    def test_window_feedback_and_covariance_saved_and_motion_rejected(self):
        rows=self.observations(feedback=True)
        result=fuse_camera_observations(rows)
        self.assertEqual(len(result['robot_window_q7_deg']),50)
        self.assertEqual(len(result['sphere_center_framewise_m']),50)
        self.assertEqual(len(result['stereo_pixels_framewise']),50)
        self.assertEqual(len(result['sphere_center_corrected_pixels_framewise']),50)
        self.assertEqual(len(result['camera_window_frames']),50)
        self.assertEqual(result['camera_window_frames'][-1]['frame_number'],49)
        self.assertEqual(np.shape(result['sphere_center_frame_covariance_mm2']),(3,3))
        self.assertGreater(result['sphere_center_half_window_shift_mm'],0.)
        rows[-1][3]['capture_q7_deg'][0]=.021
        with self.assertRaisesRegex(ValueError,'0.02 deg'):
            fuse_camera_observations(rows)

    def test_nan_feedback_and_bad_numeric_metrics_rejected(self):
        rt={'head':{'fb_pos':[np.nan,-25.,0.]},'body':{'fb_pos':[0.]*6}}
        # head_angles expects the SDK layout; use the real layout from the helper.
        from unittest.mock import patch
        with patch('interactive_ball_calibration.capture.head_angles',return_value=np.array([np.nan,-25.,0.])):
            with self.assertRaisesRegex(RuntimeError,'Invalid head/body'):
                assert_fixed(rt,np.array([0.,-25.]),np.zeros(6))
        for error in (np.empty((0,3)),[[np.nan,0.,0.]],[[0.,1.]]):
            with self.assertRaises(ValueError):
                metrics(error)
        m=metrics(np.array([[0,0,.001],[0,0,.003]]))
        self.assertAlmostEqual(m['distance_std_mm'],1.)
        self.assertAlmostEqual(m['axis_bias_mm'][2],2.)

    def test_archived_loader_refuses_nan_feedback(self):
        with tempfile.TemporaryDirectory() as folder:
            capture,rows=helper.YellowXYZConsistencyTest().make_capture(Path(folder))
            for key in ('q7_deg','head_angles_deg','body_q_deg'):
                changed=json.loads(json.dumps(rows));changed[0][key][0]=float('nan')
                capture.write_text('\n'.join(json.dumps(r) for r in changed)+'\n')
                with self.assertRaises(ValueError):
                    _load(capture,helper.MDH)


if __name__=='__main__':
    unittest.main()
