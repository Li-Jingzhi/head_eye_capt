"""Synthetic recovery, inverse mapping and nested split leakage checks."""
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

from interactive_ball_calibration.compare_pro_calibration import (
    METHODS, ROOT, camera_to_arm, cross_session_validation, fit_model, model_predict, prepare, project, sphere_points, splits,
)


class ComparisonTest(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(20261020)
        self.poses = np.repeat(np.eye(4)[None], 70, axis=0)
        self.poses[:, :3, :3] = Rotation.random(70, random_state=rng).as_matrix()
        self.poses[:, :3, 3] = rng.uniform(-.08, .08, (70, 3))
        self.x = np.r_[.1, -.2, .05, .02, -.03, .6, .002, -.003, .1]
        K = np.array([[400., 0, 640], [0, 400, 496], [0, 0, 1]])
        self.P = np.array([K @ np.c_[np.eye(3), np.zeros(3)], K @ np.c_[np.eye(3), [.06, 0, 0]]])
        self.weights = np.repeat(np.eye(3)[None] * 1000, 70, axis=0)

    def test_all_models_recover_unseen_synthetic_positions_and_invert(self):
        for method in METHODS:
            with self.subTest(method=method):
                x = self.x.copy()
                if method == 'affine':
                    M = Rotation.from_rotvec(x[:3]).as_matrix() @ np.diag([1.005, .997, 1.002])
                    x = np.r_[M.ravel(), x[3:]]
                elif method == 'similarity':
                    x = np.r_[x, np.log(1.008)]
                elif method == 'depth_bias':
                    x = np.r_[x, .012]
                truth = dict(method=method, parameters=x)
                points = model_predict(self.poses, truth)
                model = fit_model(method, self.poses[:50], points[:50], project(points[:50], self.P), self.weights[:50], self.P)
                np.testing.assert_allclose(model_predict(self.poses[50:], model), points[50:], atol=2e-7)
                np.testing.assert_allclose(model['flange_to_ball_center_m'], self.x[6:9], atol=2e-6)
                arm = self.poses[:, :3, 3] + np.einsum('nij,j->ni', self.poses[:, :3, :3], self.x[6:9])
                np.testing.assert_allclose(camera_to_arm(points, model), arm, atol=2e-7)

    def test_nested_splits_do_not_use_outer_test_or_neighbours(self):
        seen = []
        for train, held in splits(np.arange(142)):
            seen.extend(held)
            self.assertGreater(np.min(abs(train[:, None] - held)), 2)
            inner_seen = []
            for inner_train, inner_held in splits(train, folds=4):
                self.assertTrue(set(inner_train).issubset(train))
                self.assertTrue(set(inner_held).issubset(train))
                self.assertGreater(np.min(abs(inner_train[:, None] - inner_held)), 2)
                inner_seen.extend(inner_held)
            self.assertEqual(sorted(inner_seen), train.tolist())
        self.assertEqual(sorted(seen), list(range(142)))

    def test_huber_limits_influence_of_outliers(self):
        clean = model_predict(self.poses, dict(method='rigid_l2', parameters=self.x))
        contaminated = clean.copy()
        contaminated[:5] += [.02, -.015, .01]
        models = {m: fit_model(m, self.poses[:50], contaminated[:50], project(contaminated[:50], self.P),
                               self.weights[:50], self.P) for m in ('rigid_l2', 'rigid_huber')}
        error = {m: np.linalg.norm(model_predict(self.poses[50:], model) - clean[50:]) for m, model in models.items()}
        self.assertLess(error['rigid_huber'], error['rigid_l2'] * .3)

    def test_silhouette_correction_recovers_synthetic_sphere_centers(self):
        points = model_predict(self.poses, dict(method='rigid_l2', parameters=self.x))
        centers = project(points, self.P)
        ellipses = centers.copy()
        for eye, projection in enumerate(self.P):
            principal = projection[:2, 2]
            ellipses[:, 2*eye:2*eye+2] = principal + (centers[:, 2*eye:2*eye+2] - principal) / (1 - (.02 / points[:, 2:])**2)
        np.testing.assert_allclose(sphere_points(ellipses, self.P, .04), points, atol=1e-10)
        np.testing.assert_allclose(sphere_points(centers, self.P, 0), points, atol=1e-10)

    def test_preparation_keeps_valid_positions_when_archive_contains_rejections(self):
        rows = [dict(q7_deg=[0.] * 7, sphere_center_left_camera_m=[0, 0, .4],
                     sphere_center_corrected_stereo_pixels=[640, 496, 700, 496],
                     sphere_center_framewise_m=[[0, 0, .4]] * 50) for _ in range(3)]
        manifest = dict(valid_points=3, captured_points=5)
        with patch('interactive_ball_calibration.compare_pro_calibration.verify_archive', return_value=manifest), \
             patch('interactive_ball_calibration.compare_pro_calibration.load_capture', return_value=(rows, 2)):
            prepared = prepare(ROOT / 'datasets/unused_mock')
        self.assertEqual(len(prepared[0]), 3)
        self.assertEqual(prepared[-1], manifest)
        with patch('interactive_ball_calibration.compare_pro_calibration.verify_archive', return_value=manifest), \
             patch('interactive_ball_calibration.compare_pro_calibration.load_capture', return_value=(rows, 0)):
            with self.assertRaisesRegex(ValueError, 'counts disagree'):
                prepare(ROOT / 'datasets/unused_mock')

    def test_cross_session_validation_reports_shift_in_both_directions(self):
        shifted = self.x.copy(); shifted[3:6] += [.001, -.002, .003]
        ref_model = dict(method='rigid_l2', parameters=self.x)
        cur_model = dict(method='rigid_l2', parameters=shifted)
        points = model_predict(self.poses[:50], ref_model)
        current = model_predict(self.poses[50:], cur_model)
        reference = ([dict(waypoint_index=i) for i in range(50)], self.poses[:50], points,
                     project(points, self.P), None, self.weights[:50], self.P, {'valid_points': 50})
        latest = ([dict(waypoint_index=i) for i in range(20)], self.poses[50:], current,
                  None, None, None, None, {'valid_points': 20})
        with patch('interactive_ball_calibration.compare_pro_calibration.METHODS', ('rigid_l2',)):
            result, errors = cross_session_validation(reference, latest, {'rigid_l2': {'model': cur_model}})
        model = result['methods']['rigid_l2']
        np.testing.assert_allclose(model['reference_to_current']['axis_bias_mm'], [-1, 2, -3], atol=1e-5)
        np.testing.assert_allclose(model['current_to_reference']['axis_bias_mm'], [1, -2, 3], atol=1e-5)
        self.assertEqual(model['reference_to_current']['samples'], 20)
        self.assertEqual(model['current_to_reference']['samples'], 50)
        self.assertEqual(len(errors), 70)


if __name__ == '__main__':
    unittest.main()
