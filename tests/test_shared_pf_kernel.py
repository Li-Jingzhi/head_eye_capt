import json
import unittest

import numpy as np

from tests import test_dual_pf
from interactive_ball_calibration.compare_dual_pf import SIDES, fit_pair, subset
from interactive_ball_calibration.improve_shared_pf import (
    camera_to_arm, choose, fit, flange_errors, kernel, plain_model, relative_score,
)


class SharedPfKernelTest(unittest.TestCase):
    def test_exact_geometry_recovers_shared_tool_on_unseen_poses(self):
        for method, variant in (('rigid_l2','rigid_l40_a1'), ('affine','affine_l40_a3')):
            data, pf = test_dual_pf.DualPfTest().synthetic(method)
            train = {s:subset(data[s], np.arange(45)) for s in SIDES}
            model = fit(train, variant)
            np.testing.assert_allclose(model['shared_pf_m'], pf, atol=1e-7)
            for s in SIDES:
                held = subset(data[s], np.arange(45,60))
                np.testing.assert_allclose(flange_errors(held, model, s), 0, atol=1e-7)
                reloaded = json.loads(json.dumps(model))
                np.testing.assert_array_equal(camera_to_arm(held['points'], model, s),
                                              camera_to_arm(held['points'], reloaded, s))

    def test_common_smooth_bias_improves_unseen_positions(self):
        data, _ = test_dual_pf.DualPfTest().synthetic('affine')
        center = np.array([[.015,-.03,.68]])
        coefficients = np.array([[.004,-.003,.005]])
        for s in SIDES:
            physical = data[s]['points'].copy()
            observed = physical.copy()
            for _ in range(40):
                observed = physical-kernel(observed, center, .07)@coefficients
            data[s]['points'] = observed
        train = {s:subset(data[s], np.arange(45)) for s in SIDES}
        seed = fit_pair(train, 'affine', 'shared')
        baseline = plain_model(seed)
        corrected = fit(train, 'affine_l60_a0.3', seed)
        def error(model):
            return np.linalg.norm(np.concatenate([flange_errors(subset(data[s], np.arange(45,60)), model, s) for s in SIDES]))
        self.assertLess(error(corrected), error(baseline)*.8)

    def test_target_inference_does_not_use_tool_or_pose(self):
        data, _ = test_dual_pf.DualPfTest().synthetic()
        model = fit(data, 'rigid_l40_a1')
        before = camera_to_arm(data['right']['points'], model, 'right')
        model['shared_pf_m'] = [900,900,900]
        np.testing.assert_array_equal(camera_to_arm(data['right']['points'], model, 'right'), before)

    def test_invalid_inputs_and_models_rejected(self):
        data, _ = test_dual_pf.DualPfTest().synthetic()
        model = fit(data, 'rigid_l40_a1')
        for p in ([[0,0,0]], [[0,0,-1]], [[0,float('nan'),1]], [0,0,1]):
            with self.assertRaises(ValueError):
                camera_to_arm(p, model, 'right')
        with self.assertRaises(ValueError):
            camera_to_arm([[0,0,1]], model, 'other')
        model['kernel_coefficients_m'][0][0] = float('nan')
        with self.assertRaises(ValueError):
            camera_to_arm([[0,0,1]], model, 'right')

    def test_selection_requires_all_six_baseline_metrics(self):
        def scores(r,p,f):
            return {s:dict(rms_mm=r,p95_mm=p,fraction_within_1mm=f) for s in SIDES}
        baseline = scores(.9,1.5,.78)
        candidates = {'better_tail_but_worse_coverage':scores(.8,1.3,.76),
                      'all_better':scores(.85,1.45,.79)}
        self.assertEqual(choose(candidates,baseline), 'all_better')
        self.assertLess(relative_score(candidates['all_better'],baseline), 1)
        self.assertGreater(relative_score(candidates['better_tail_but_worse_coverage'],baseline), 1)


if __name__ == '__main__':
    unittest.main()
