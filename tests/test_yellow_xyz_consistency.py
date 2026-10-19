"""Regression tests for yellow-only fitting: no four-ball or online q7 dependency."""
import json
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from capt_head.drag_capture import mdh_provenance
from interactive_ball_calibration.fit_yellow_xyz_consistency import (
    apply, fit, metrics)
from interactive_ball_calibration.geometry import arm_mdh, flange_pose


ROOT = Path(__file__).resolve().parents[1]
MDH = ROOT / "configs/luna_new.ini"


class YellowXYZConsistencyTest(unittest.TestCase):
    def make_capture(self, folder):
        rng = np.random.default_rng(17)
        chain = arm_mdh(MDH)
        pf = np.array([-.003, .005, .101])
        R = cv2.Rodrigues(np.array([.15, -.10, .06]))[0]
        t = np.array([-.04, .10, .3])
        rows = []
        for i in range(85):
            q = np.array([-30, -40, 25, -95, 30, -16, 7], float)
            q += rng.normal(0, 20, 7)
            pose = flange_pose(q, chain)
            physical = pose[:3, 3] + pose[:3, :3]@pf
            camera_xyz = R@physical+t
            rows.append({
                "capture_kind": "interactive_ball_auto",
                "capture_valid": True,
                "waypoint_index": i,
                "valid_frames": 50,
                "q7_deg": q.tolist(),
                "head_angles_deg": [0., -25.],
                "body_q_deg": [0]*6,
                "mdh_sha256": mdh_provenance(MDH),
                "pixel_domain": "ideal_pinhole",
                "image_size": [1280, 992],
                "sphere_center_method": "sphere_center_projection_stereo_v1",
                "sphere_center_left_camera_m": camera_xyz.tolist(),
                "source_calibration_sha256": "synthetic123",
                "stereo_model_sha256": "synthetic_stereo123",
                "sensor_ids": ["head_left:csi://0", "head_right:csi://1"],
            })
        capture = folder / "single_capture.jsonl"
        capture.write_text("\n".join(json.dumps(r) for r in rows)+"\n", encoding="utf-8")
        return capture, rows

    def test_recovers_consistent_3d_mapping_and_never_autodeploys(self):
        with tempfile.TemporaryDirectory() as tmp:
            capture, _ = self.make_capture(Path(tmp))
            report = fit(capture, MDH)
            self.assertEqual(report["capture_valid_rows"], 85)
            self.assertEqual(report["train_samples"], 68)
            self.assertEqual(report["untouched_samples"], 17)
            self.assertTrue(report["untouched_xyz_p95_under_1mm"])
            self.assertLess(report["untouched_corrected_xyz"]["xyz_p95_mm"], .1)
            self.assertTrue(report["do_not_deploy"])
            self.assertFalse(report["independent_new_trajectory_verified"])
            with self.assertRaisesRegex(ValueError, "has not passed"):
                apply(report, [.1, .2, .3])

    def test_rejects_mismatched_calibration_provenance(self):
        with tempfile.TemporaryDirectory() as tmp:
            capture, rows = self.make_capture(Path(tmp))
            rows[-1]["source_calibration_sha256"] = "different"
            capture.write_text("\n".join(json.dumps(r) for r in rows)+"\n")
            with self.assertRaisesRegex(ValueError, "Mixed calibration"):
                fit(capture, MDH)

    def test_metric_is_true_3d_norm(self):
        error = metrics(np.array([[.0003, .0004, 0.]]))
        self.assertAlmostEqual(error["xyz_mean_mm"], .5, places=9)


if __name__ == "__main__":
    unittest.main()
