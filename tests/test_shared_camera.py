import unittest
import numpy as np

from tests import test_dual_pf
from interactive_ball_calibration.compare_dual_pf import SIDES, subset
from interactive_ball_calibration.refine_shared_camera import (
    VARIANTS, choose, correct_camera, fit, flange_errors, polynomial,
)


class SharedCameraTest(unittest.TestCase):
    def test_undistorted_data_remain_unchanged_for_every_variant(self):
        data,pf=test_dual_pf.DualPfTest().synthetic()
        for name in VARIANTS:
            with self.subTest(name=name):
                model=fit({s:subset(data[s],np.arange(45)) for s in SIDES},name)
                np.testing.assert_allclose(model['shared_pf_m'],pf,atol=1e-7)
                for s in SIDES:
                    np.testing.assert_allclose(flange_errors(subset(data[s],np.arange(45,60)),model,s),0,atol=1e-7)
                    np.testing.assert_allclose(correct_camera(data[s]['points'],model),data[s]['points'],atol=1e-7)

    def test_common_disparity_bias_generalizes_to_unseen_poses(self):
        data,_=test_dual_pf.DualPfTest().synthetic()
        for s in SIDES:
            physical=data[s]['points'];q=data[s]['P'][1,0,3]
            data[s]['points']=physical/(1+.12*physical[:,2:]/q)
        train={s:subset(data[s],np.arange(45)) for s in SIDES}
        rigid=fit(train,'shared_rigid');corrected=fit(train,'disparity_plane_03px')
        errors={}
        for name,model in (('rigid',rigid),('corrected',corrected)):
            errors[name]=np.linalg.norm(np.concatenate([flange_errors(subset(data[s],np.arange(45,60)),model,s) for s in SIDES]))
        self.assertLess(errors['corrected'],errors['rigid']*.25)

    def test_common_nonlinear_camera_bias_generalizes(self):
        data,_=test_dual_pf.DualPfTest().synthetic()
        coefficients=np.zeros((6,3));coefficients[0,0]=.001;coefficients[2,2]=.0006;coefficients[3,1]=-.001
        for s in SIDES:
            physical=data[s]['points'];observed=physical.copy()
            for _ in range(25):observed=physical-polynomial(observed)@coefficients
            data[s]['points']=observed
        train={s:subset(data[s],np.arange(45)) for s in SIDES}
        rigid=fit(train,'shared_rigid');corrected=fit(train,'xyz_quad_2mm')
        rigid_error=np.linalg.norm(np.concatenate([flange_errors(subset(data[s],np.arange(45,60)),rigid,s) for s in SIDES]))
        corrected_error=np.linalg.norm(np.concatenate([flange_errors(subset(data[s],np.arange(45,60)),corrected,s) for s in SIDES]))
        self.assertLess(corrected_error,rigid_error*.4)

    def test_invalid_points_and_singular_disparity_are_rejected(self):
        model=dict(correction_kind='disparity_plane',correction_coefficients=[100,0,0],signed_projection_baseline_px_m=24)
        for point in ([[0,0,-1]],[[0,0,float('nan')]],[[0,0,1]]):
            with self.assertRaises(ValueError):correct_camera(point,model)

    def test_selection_uses_worse_arm_tail_error(self):
        def value(a,b):return {'right':dict(p95_mm=a,rms_mm=.8),'left':dict(p95_mm=b,rms_mm=.8)}
        self.assertEqual(choose({'a':value(.4,1.5),'b':value(1.1,1.2)}),'b')


if __name__=='__main__':unittest.main()
