import unittest
import numpy as np
from scipy.spatial.transform import Rotation

from interactive_ball_calibration.compare_dual_pf import (
    METHODS, SIDES, fit_pair, flange_delta, select, subset,
)
from interactive_ball_calibration.compare_pro_calibration import model_predict, project, splits


class DualPfTest(unittest.TestCase):
    def synthetic(self, method='rigid_l2', difference=None):
        rng=np.random.default_rng(20261020)
        data={}; truth={}
        pf=np.array([.001,-.002,.101])
        K=np.array([[400.,0,640],[0,400,496],[0,0,1.]])
        P=np.array([K@np.c_[np.eye(3),np.zeros(3)],K@np.c_[np.eye(3),[.06,0,0]]])
        for i,s in enumerate(SIDES):
            poses=np.repeat(np.eye(4)[None],60,axis=0)
            poses[:,:3,:3]=Rotation.random(60,random_state=rng).as_matrix()
            poses[:,:3,3]=rng.uniform(-.08,.08,(60,3))
            x=np.r_[[.1+i*.2,-.2,.05],[.02,-.03,.6+i*.1],pf+(np.zeros(3) if difference is None or s=='right' else difference)]
            if method=='affine': x=np.r_[(Rotation.from_rotvec(x[:3]).as_matrix()@np.diag([1.003,.998,1.001])).ravel(),x[3:]]
            elif method=='similarity': x=np.r_[x,np.log(1.003)]
            elif method=='depth_bias': x=np.r_[x,.012]
            truth[s]=dict(method=method,parameters=x)
            points=model_predict(poses,truth[s])
            data[s]=dict(poses=poses,points=points,pixels=project(points,P),P=P,
                         weights=np.repeat(np.eye(3)[None]*1000,60,axis=0))
        return data,pf

    def test_shared_pf_recovers_all_models_on_unseen_poses(self):
        for method in METHODS:
            with self.subTest(method=method):
                data,pf=self.synthetic(method)
                model=fit_pair({s:subset(data[s],np.arange(45)) for s in SIDES},method,'shared')
                for s in SIDES:
                    np.testing.assert_allclose(model['models'][s]['flange_to_ball_center_m'],pf,atol=2e-6)
                    np.testing.assert_allclose(flange_delta(subset(data[s],np.arange(45,60)),model['models'][s]),0,atol=2e-7)
                np.testing.assert_array_equal(model['models']['right']['flange_to_ball_center_m'],model['models']['left']['flange_to_ball_center_m'])

    def test_near_constraint_is_enforced_when_tools_disagree(self):
        data,_=self.synthetic(difference=np.array([0,0,.002]))
        independent=fit_pair(data,'rigid_l2','independent')
        near=fit_pair(data,'rigid_l2','near_0p5mm',independent['models'])
        offsets=[np.array(near['models'][s]['flange_to_ball_center_m']) for s in SIDES]
        self.assertLessEqual(np.linalg.norm(offsets[0]-offsets[1]),.0005+1e-12)
        self.assertTrue(near['near_difference_bound_active'])
        self.assertGreater(np.linalg.norm(flange_delta(data['left'],near['models']['left'])),1e-4)
        np.testing.assert_allclose(flange_delta(data['left'],independent['models']['left']),0,atol=1e-7)

    def test_paired_inner_and_outer_splits_exclude_both_test_sets(self):
        for n in (288,424):
            for train,held in splits(np.arange(n)):
                for it,ih in splits(train,folds=4):
                    self.assertFalse(set(it)&set(held))
                    self.assertFalse(set(ih)&set(held))
                    self.assertGreater(np.min(abs(it[:,None]-held)),2)
                    self.assertGreater(np.min(abs(it[:,None]-ih)),2)

    def test_selection_does_not_hide_bad_arm_and_filters_rigid(self):
        def record(a,b):return {'right':dict(p95_mm=a,rms_mm=.8),'left':dict(p95_mm=b,rms_mm=.8)}
        scores={'affine/shared':record(.9,.9),'rigid_l2/shared':record(1.1,1.2),
                'rigid_huber/shared':record(.5,1.6),'rigid_l2/independent':record(1.,1.1)}
        self.assertEqual(select(scores),'affine/shared')
        self.assertEqual(select(scores,'rigid'),'rigid_l2/independent')
        self.assertEqual(select(scores,'shared_rigid'),'rigid_l2/shared')


if __name__=='__main__':unittest.main()
