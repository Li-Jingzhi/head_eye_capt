"""Discrete-head registration, residual sharing, grouping and runtime guards."""
import json
import unittest

import cv2
import numpy as np

from interactive_ball_calibration.multi_head_compensation import (
    SCHEMA, register, baseline, fit_residual, residual, predict)
from interactive_ball_calibration.validate_pf_rt import make_splits


class MultiHeadCompensationTests(unittest.TestCase):
    def setUp(self):
        rng=np.random.default_rng(52)
        self.heads=np.tile(np.array(['-10,-25','0,-20','10,-25']),80)
        points=rng.uniform([-.1,-.1,.25],[.1,.1,.45],(80,3))
        self.target=np.repeat(points,3,axis=0)
        self.camera=np.zeros_like(self.target)
        for k,h in enumerate(sorted(set(self.heads))):
            R=cv2.Rodrigues(np.array([.05,k*.15,.02]))[0];t=np.array([.01*k,.02,.1])
            self.camera[self.heads==h]=self.target[self.heads==h]@R.T+t
        self.train=np.arange(180);self.test=np.arange(180,240)

    def test_unseen_points_and_heldout_labels_cannot_change_registration(self):
        tf=register(self.camera,self.target,self.heads,self.train)
        b=baseline(tf,self.camera,self.heads)
        np.testing.assert_allclose(b,self.target,atol=1e-12)
        changed=self.target.copy();changed[self.test]+=1
        tf2=register(self.camera,changed,self.heads,self.train)
        self.assertEqual(tf,tf2)

    def test_head_specific_kernel_never_borrows_other_heads_labels(self):
        base=self.target.copy();target=base.copy()
        target[self.heads=='-10,-25'] += [.001,0,0]
        spec=('per_head',.05,.1)
        model=fit_residual(spec,self.camera,target,self.heads,base,self.train)
        result=residual(json.loads(json.dumps(model)),self.camera,base,self.heads)
        np.testing.assert_allclose(result[self.heads=='10,-25'],0,atol=1e-12)
        self.assertGreater(result[self.heads=='-10,-25',0].mean(),.0005)

    def test_shared_and_additive_residuals_fit_smooth_field(self):
        base=self.target.copy();target=base.copy()
        target[:,0]+=.0005*np.sin(base[:,0]/.08)
        for kind in ('shared','additive'):
            model=fit_residual((kind,.1,.01),self.camera,target,self.heads,base,self.train)
            pred=base+residual(model,self.camera,base,self.heads)
            error=np.linalg.norm(pred[self.test]-target[self.test],axis=1)*1000
            self.assertLess(np.percentile(error,95),.25)

    def test_public_prediction_has_head_space_and_approval_guards(self):
        tf=register(self.camera,self.target,self.heads,self.train)
        model=dict(schema=SCHEMA,approved=True,do_not_deploy=False,transforms=tf,
                   residual=dict(kind='rigid'),support_radius_mm=20,
                   support_camera_by_head={h:self.camera[self.heads==h].tolist() for h in set(self.heads)})
        x=self.camera[0];np.testing.assert_allclose(predict(model,x,[-10,-25],head_velocity_deg_s=[0,0]),self.target[0],atol=1e-12)
        with self.assertRaisesRegex(ValueError,'stopped-head'):
            predict(model,x,[-10,-25])
        with self.assertRaisesRegex(ValueError,'stopped-head'):
            predict(model,x,[-10,-25],head_velocity_deg_s=[1,0])
        with self.assertRaisesRegex(ValueError,'Unobserved'):
            predict(model,x,[0,-25],head_velocity_deg_s=[0,0])
        with self.assertRaisesRegex(ValueError,'Outside'):
            predict(model,x+[1,0,0],[-10,-25],head_velocity_deg_s=[0,0])
        model['approved']=False
        with self.assertRaisesRegex(ValueError,'not approved'):
            predict(model,x,[-10,-25])

    def test_same_waypoint_never_leaks_between_train_and_test(self):
        rows=[dict(group_id=f'w{i//3}',q7_deg=[float(i//3)]*7) for i in range(240)]
        poses=np.broadcast_to(np.eye(4),(240,4,4)).copy();poses[:,:3,3]=self.target
        for _,_,tr,te in make_splits(rows,poses):
            self.assertTrue({rows[i]['group_id'] for i in tr}.isdisjoint({rows[i]['group_id'] for i in te}))


if __name__=='__main__':
    unittest.main()
